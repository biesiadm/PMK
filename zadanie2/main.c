#include <delay.h>
#include <gpio.h>
#include <stm32.h>
#include <string.h>


// Leds

#define RED_LED_GPIO    GPIOA
#define GREEN_LED_GPIO  GPIOA
#define BLUE_LED_GPIO   GPIOB
#define GREEN2_LED_GPIO GPIOA

#define RED_LED_PIN    6
#define GREEN_LED_PIN  7
#define BLUE_LED_PIN   0
#define GREEN2_LED_PIN 5


// Buttons

#define USER_BUTTON_GPIO      GPIOC
#define JOYSTICK_GPIO         GPIOB
#define AT_MODE_BUTTON_GPIO   GPIOA

#define USER_BUTTON_PIN       13
#define JOYSTICK_LEFT_PIN     3
#define JOYSTICK_RIGHT_PIN    4
#define JOYSTICK_UP_PIN       5
#define JOYSTICK_DOWN_PIN     6
#define JOYSTICK_ACTION_PIN   10
#define AT_MODE_BUTTON_PIN    0


// CR1

#define USART_Mode_Rx_Tx (USART_CR1_RE | USART_CR1_TE)
#define USART_Enable      USART_CR1_UE

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No   0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd  (USART_CR1_PCE | USART_CR1_PS)

// CR2

#define USART_StopBits_1   0x0000
#define USART_StopBits_0_5 0x1000
#define USART_StopBits_2   0x2000
#define USART_StopBits_1_5 0x3000

// CR3

#define USART_FlowControl_None 0x0000
#define USART_FlowControl_RTS  USART_CR3_RTSE
#define USART_FlowControl_CTS  USART_CR3_CTSE

// BRR

// Po włączeniu mikrokontroler STM32F411 jest taktowany wewnętrznym
// generatorem RC HSI (ang.High SpeedInternal) o częstotliwości 16 MHz
#define HSI_HZ 16000000U

// UkładUART2 jest taktowany zegarem PCLK1, który powłączeniu mikrokontrolera jest zegarem HSI
#define PCLK1_HZ HSI_HZ

// Bufory

#define BUFF_SIZE 1024
static const int BUFF_START = 0;

volatile char cyclic_buffer[BUFF_SIZE];
volatile int cyclic_buffer_start = BUFF_START;
volatile int cyclic_buffer_end = BUFF_START;

volatile char to_send_buffer[BUFF_SIZE];
volatile int to_send_len = BUFF_START;

static void RedLEDon() {
  RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16);
}

static void RedLEDoff() {
  RED_LED_GPIO->BSRR = 1 << RED_LED_PIN;
}

static void GreenLEDon() {
  GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16);
}

static void GreenLEDoff() {
  GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN;
}

static void BlueLEDon() {
  BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16);
}

static void BlueLEDoff() {
  BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN;
}

static void Green2LEDon() {
  GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN;
}

static void Green2LEDoff() {
  GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16);
}

static int AtModeButtonCheck() {
  return (AT_MODE_BUTTON_GPIO->IDR & (1 << AT_MODE_BUTTON_PIN));
}

static int UserButtonCheck() {
  return !(USER_BUTTON_GPIO->IDR & (1 << USER_BUTTON_PIN));
}

static int JoystickDownCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_DOWN_PIN));
}

static int JoystickUpCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_UP_PIN));
}

static int JoystickLeftCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_LEFT_PIN));
}

static int JoystickRightCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_RIGHT_PIN));
}

static int JoystickActionCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_ACTION_PIN));
}

static void set_clock() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN |
                  RCC_AHB1ENR_DMA1EN;

  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

static void basic_configuration() {
  // Przykładowa konfiguracja (układ pozostaje nieaktywny – nie ustawiamy bitu USART_Enable)
  USART2->CR1 = USART_Mode_Rx_Tx |
      USART_WordLength_8b |
      USART_Parity_No;

  USART2->CR2 = USART_StopBits_1;

  uint32_t const baudrate = 9600U;
  USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) /
      baudrate;

  // USART2->CR3 = USART_FlowControl_None;
  USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

  // Konfigruacja DMA
  DMA1_Stream6->CR = 4U << 25 |
      DMA_SxCR_PL_1 |
      DMA_SxCR_MINC |
      DMA_SxCR_DIR_0 |
      DMA_SxCR_TCIE;

  DMA1_Stream6->PAR = (uint32_t) & USART2->DR;

  DMA1_Stream5->CR = 4U << 25 |
      DMA_SxCR_PL_1 |
      DMA_SxCR_MINC |
      DMA_SxCR_TCIE;

  DMA1_Stream5->PAR = (uint32_t) & USART2->DR;

  DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  USART2->CR1 |= USART_Enable;

  // Konfiguracja linii TXD
  GPIOafConfigure(GPIOA,
                  2,
                  GPIO_OType_PP,
                  GPIO_Fast_Speed,
                  GPIO_PuPd_NOPULL,
                  GPIO_AF_USART2);

  // Konfiguracja linii RXD
  GPIOafConfigure(GPIOA,
                  3,
                  GPIO_OType_PP,
                  GPIO_Fast_Speed,
                  GPIO_PuPd_UP,
                  GPIO_AF_USART2);
}

static void all_leds_off() {
  RedLEDoff();
  GreenLEDoff();
  BlueLEDoff();
  Green2LEDoff();
}

static void configurate_leds() {
  GPIOoutConfigure(RED_LED_GPIO,
                   RED_LED_PIN,
                   GPIO_OType_PP,
                   GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  GPIOoutConfigure(GREEN_LED_GPIO,
                   GREEN_LED_PIN,
                   GPIO_OType_PP,
                   GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  GPIOoutConfigure(BLUE_LED_GPIO,
                   BLUE_LED_PIN,
                   GPIO_OType_PP,
                   GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);

  GPIOoutConfigure(GREEN2_LED_GPIO,
                   GREEN2_LED_PIN,
                   GPIO_OType_PP,
                   GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);
}

static void configurate_buttons() {
  GPIOinConfigure(AT_MODE_BUTTON_GPIO,
                  AT_MODE_BUTTON_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

  NVIC_EnableIRQ(EXTI0_IRQn);

///////////////////////////////////////////////////////////////////////////////

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_LEFT_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;

  NVIC_EnableIRQ(EXTI3_IRQn);

///////////////////////////////////////////////////////////////////////////////

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_RIGHT_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;

  NVIC_EnableIRQ(EXTI4_IRQn);

///////////////////////////////////////////////////////////////////////////////

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_UP_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;

///////////////////////////////////////////////////////////////////////////////

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_DOWN_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB;

  NVIC_EnableIRQ(EXTI9_5_IRQn);

///////////////////////////////////////////////////////////////////////////////

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_ACTION_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;

///////////////////////////////////////////////////////////////////////////////

  GPIOinConfigure(USER_BUTTON_GPIO,
                  USER_BUTTON_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

  NVIC_EnableIRQ(EXTI15_10_IRQn);
}



static const char INITIALISE = ' ';

static const int CMD_PREFIX_POS = 0;
static const int CMD_COLOR_POS = 1;
static const int CMD_STATE_POS = 2;
static const int CMD_COMPLETE_POS = 3;

static const char CMD_PREFIX = 'L';

static const char CMD_GREEN = 'G';
static const char CMD_RED = 'R';
static const char CMD_BLUE = 'B';
static const char CMD_GREEN2 = 'g';

static const char CMD_STATE_ON = '1';
static const char CMD_STATE_OFF = '0';
static const char CMD_STATE_TOGGLE = 'T';

static const int NUMBER_OF_COLORS = 4;
static const int GREEN_STATE_POS = 0;
static const int RED_STATE_POS = 1;
static const int BLUE_STATE_POS = 2;
static const int GREEN2_STATE_POS = 3;

static const int STATE_ON = 1;
static const int STATE_OFF = 0;

static void check_command(char *color, char *state, char curr, int *pos) {
  switch (*pos) {
    case CMD_PREFIX_POS:
      if (curr == CMD_PREFIX) {
        (*pos)++;
      }

      break;
    case CMD_COLOR_POS:
      if (curr == CMD_GREEN ||
          curr == CMD_RED ||
          curr == CMD_BLUE ||
          curr == CMD_GREEN2) {
        *color = curr;
        (*pos)++;
      } else {
        *pos = CMD_PREFIX_POS;
      }

      break;
    case CMD_STATE_POS:
      if (curr == CMD_STATE_ON ||
          curr == CMD_STATE_OFF ||
          curr == CMD_STATE_TOGGLE) {
        *state = curr;
        (*pos)++;
      } else {
        *pos = CMD_PREFIX_POS;
      }

      break;
  }
}

static void execute_cmd_color(char state,
                              int *states,
                              int state_pos,
                              void (*led_on)(),
                              void (*led_off)()) {
  switch (state) {
    case CMD_STATE_ON:
      (*led_on)();
      states[state_pos] = STATE_ON;

      break;
    case CMD_STATE_OFF:
      (*led_off)();
      states[state_pos] = STATE_OFF;

      break;
    case CMD_STATE_TOGGLE:
      if (states[state_pos]) {
        (*led_off)();
        states[state_pos] = STATE_OFF;
      } else {
        (*led_on)();
        states[state_pos] = STATE_ON;
      }

      break;
  }
}

static void execute_command(char color, char state, int *states) {
  switch (color) {
    case CMD_GREEN:
      execute_cmd_color(state, states, GREEN_STATE_POS, GreenLEDon, GreenLEDoff);
      break;
    case CMD_RED:
      execute_cmd_color(state, states, RED_STATE_POS, RedLEDon, RedLEDoff);
      break;
    case CMD_BLUE:
      execute_cmd_color(state, states, BLUE_STATE_POS, BlueLEDon, BlueLEDoff);
      break;
    case CMD_GREEN2:
      execute_cmd_color(state, states, GREEN2_STATE_POS, Green2LEDon, Green2LEDoff);
      break;
  }
}


static const int BUTTONS_NUMBER = 7;

static const int RELEASED = 0;

static const int USER_BUTTON_POS = 0;
static const int AT_MODE_BUTTON_POS = 1;
static const int JOYSTICK_LEFT_POS = 2;
static const int JOYSTICK_RIGHT_POS = 3;
static const int JOYSTICK_UP_POS = 4;
static const int JOYSTICK_DOWN_POS = 5;
static const int JOYSTICK_ACTION_POS = 6;

static const char *LEFT_PRESSED = "LEFT PRESSED\r\n";
static const char *LEFT_RELEASED = "LEFT RELEASED\r\n";

static const char *RIGHT_PRESSED = "RIGHT PRESSED\r\n";
static const char *RIGHT_RELEASED = "RIGHT RELEASED\r\n";

static const char *UP_PRESSED = "UP PRESSED\r\n";
static const char *UP_RELEASED = "UP RELEASED\r\n";

static const char *DOWN_PRESSED = "DOWN PRESSED\r\n";
static const char *DOWN_RELEASED = "DOWN RELEASED\r\n";

static const char *FIRE_PRESSED = "FIRE PRESSED\r\n";
static const char *FIRE_RELEASED = "FIRE RELEASED\r\n";

static const char *USER_PRESSED = "USER PRESSED\r\n";
static const char *USER_RELEASED = "USER RELEASED\r\n";

static const char *MODE_PRESSED = "MODE PRESSED\r\n";
static const char *MODE_RELEASED = "MODE RELEASED\r\n";


static void activate_send_stream() {
  DMA1_Stream6->M0AR = (uint32_t) to_send_buffer;
  DMA1_Stream6->NDTR = to_send_len;
  DMA1_Stream6->CR |= DMA_SxCR_EN;
}

static void try_to_send_msg() {
  if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
      (DMA1->HISR & DMA_HISR_TCIF6) == 0 &&
      cyclic_buffer_start != cyclic_buffer_end) {
    int i = 0;

    while (cyclic_buffer_start != cyclic_buffer_end) {
      to_send_buffer[i] = cyclic_buffer[cyclic_buffer_start++];
      i++;

      if (cyclic_buffer_start == BUFF_SIZE) {
        cyclic_buffer_start = 0;
      }
    }

    to_send_len = i;

    activate_send_stream();
  }
}

static void fill_buffer(const char *msg) {
  for (int i = 0; msg[i] != 0; i++) {
    cyclic_buffer[cyclic_buffer_end++] = msg[i];

    if (cyclic_buffer_end == BUFF_SIZE) {
      cyclic_buffer_end = BUFF_START;
    }
  }

  try_to_send_msg();
}

static void check_button(int *buttons_states, int (*button_state)(),
                         int button_pos, const char *pressed_msg, const char *relased_msg) {
  int curr_state = RELEASED;

  if ((curr_state = (*button_state)()) != buttons_states[button_pos]) {
    buttons_states[button_pos] = curr_state;

    if (curr_state) {
      fill_buffer(pressed_msg);
    } else {
      fill_buffer(relased_msg);
    }
  }
}

static void check_buttons(int *buttons_states) {
  check_button(buttons_states, JoystickLeftCheck,
               JOYSTICK_LEFT_POS, LEFT_PRESSED, LEFT_RELEASED);

  check_button(buttons_states, JoystickRightCheck,
               JOYSTICK_RIGHT_POS, RIGHT_PRESSED, RIGHT_RELEASED);

  check_button(buttons_states, JoystickUpCheck,
               JOYSTICK_UP_POS, UP_PRESSED, UP_RELEASED);

  check_button(buttons_states, JoystickDownCheck,
               JOYSTICK_DOWN_POS, DOWN_PRESSED, DOWN_RELEASED);

  check_button(buttons_states, JoystickActionCheck,
               JOYSTICK_ACTION_POS, FIRE_PRESSED, FIRE_RELEASED);

  check_button(buttons_states, UserButtonCheck,
               USER_BUTTON_POS, USER_PRESSED, USER_RELEASED);

  check_button(buttons_states, AtModeButtonCheck,
               AT_MODE_BUTTON_POS, MODE_PRESSED, MODE_RELEASED);
}

static void check_for_input(int *pos, char *color, char *state, int *states) {
  char c = INITIALISE;

  if (USART2->SR & USART_SR_RXNE) {
    c = USART2->DR;
    check_command(color, state, c, pos);

    if ((*pos) == CMD_COMPLETE_POS) {
      execute_command(*color, *state, states);
      *pos = CMD_PREFIX_POS;
    }
  }
}

void EXTI0_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR0) {

    if (AtModeButtonCheck()) {
      fill_buffer(MODE_PRESSED);
    } else {
      fill_buffer(MODE_RELEASED);
    }
    EXTI->PR |= EXTI_PR_PR0;
//    fill_buffer("atmode\r\n");
  }
}

void EXTI3_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR3) {
    EXTI->PR |= EXTI_PR_PR3;

    fill_buffer("left\r\n");
  }
}

void EXTI4_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR4) {
    EXTI->PR |= EXTI_PR_PR4;

    fill_buffer("right\r\n");
  }
}

void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR5) {
    EXTI->PR |= EXTI_PR_PR5;

    fill_buffer("up\r\n");
  }

  if (EXTI->PR & EXTI_PR_PR6) {
    EXTI->PR |= EXTI_PR_PR6;

    fill_buffer("down\r\n");
  }
}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR10) {
    EXTI->PR |= EXTI_PR_PR10;

    fill_buffer("fire\r\n");
  }

  if (EXTI->PR & EXTI_PR_PR13) {
    EXTI->PR |= EXTI_PR_PR13;

    fill_buffer("user\r\n");
  }
}

void DMA1_Stream6_IRQHandler() {
  /* Odczytaj zgłoszone przerwania DMA1. */
  uint32_t isr = DMA1->HISR;

  if (isr & DMA_HISR_TCIF6) {
    /* Obsłuż zakończenie transferu w strumieniu 6. */
    DMA1->HIFCR = DMA_HIFCR_CTCIF6;

    try_to_send_msg();
  }
}

void DMA1_Stream5_IRQHandler() {
  /* Odczytaj zgłoszone przerwania DMA1. */
  uint32_t isr = DMA1->HISR;

  if (isr & DMA_HISR_TCIF5) {
    /* Obsłuż zakończenie transferuw strumieniu 5. */
    DMA1->HIFCR = DMA_HIFCR_CTCIF5;
    /* Ponownie uaktywnij odbieranie. */
  }
}

int main() {
  set_clock();
  basic_configuration();

  all_leds_off();
  configurate_leds();
  configurate_buttons();

  char color = INITIALISE;
  char state = INITIALISE;
  int pos = CMD_PREFIX_POS;

  int states[NUMBER_OF_COLORS];

  for (int i = 0; i < NUMBER_OF_COLORS; i++) {
    states[i] = STATE_OFF;
  }

  int buttons_states[BUTTONS_NUMBER];

  for (int i = 0; i < BUTTONS_NUMBER; i++) {
    buttons_states[i] = RELEASED;
  }

  for (;;) {
    check_for_input(&pos, &color, &state, states);
//    check_buttons(buttons_states);
  }
}
