#include <gpio.h>
#include <stm32.h>

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

#define CLEAR_EXTI_PR 0x7FFFF

// Bufory

#define BUFF_SIZE 1024
static const int BUFF_START = 0;

char cyclic_buffer[BUFF_SIZE];
int cyclic_buffer_start = BUFF_START;
int cyclic_buffer_end = BUFF_START;

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
  USART2->CR1 = USART_Mode_Rx_Tx |
      USART_WordLength_8b |
      USART_Parity_No;

  USART2->CR2 = USART_StopBits_1;

  uint32_t const baudrate = 9600U;
  USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) /
      baudrate;

  USART2->CR3 = USART_CR3_DMAT;

  DMA1_Stream6->CR = 4U << 25 |
      DMA_SxCR_PL_1 |
      DMA_SxCR_MINC |
      DMA_SxCR_DIR_0 |
      DMA_SxCR_TCIE;

  DMA1_Stream6->PAR = (uint32_t) & USART2->DR;

  DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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

static void configurate_buttons() {
  GPIOinConfigure(AT_MODE_BUTTON_GPIO,
                  AT_MODE_BUTTON_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_LEFT_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_RIGHT_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_UP_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_DOWN_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  GPIOinConfigure(JOYSTICK_GPIO,
                  JOYSTICK_ACTION_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  GPIOinConfigure(USER_BUTTON_GPIO,
                  USER_BUTTON_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Interrupt,
                  EXTI_Trigger_Rising_Falling);

  EXTI->PR = CLEAR_EXTI_PR;

  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

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

static void activate_send_stream(int to_send_len) {
  DMA1_Stream6->M0AR = (uint32_t)(cyclic_buffer + cyclic_buffer_start);
  DMA1_Stream6->NDTR = to_send_len;
  DMA1_Stream6->CR |= DMA_SxCR_EN;
}

static void try_to_send_msg() {
  if ((DMA1_Stream6->CR & DMA_SxCR_EN) == 0 &&
      (DMA1->HISR & DMA_HISR_TCIF6) == 0 &&
      cyclic_buffer_start != cyclic_buffer_end) {

    int to_send_len = 0, new_start = 0;

    if (cyclic_buffer_start < cyclic_buffer_end) {
      to_send_len = cyclic_buffer_end - cyclic_buffer_start;
      new_start = cyclic_buffer_end;
    } else {
      to_send_len = BUFF_SIZE - cyclic_buffer_start;
      new_start = BUFF_START;
    }

    activate_send_stream(to_send_len);

    cyclic_buffer_start = new_start;
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

static void log_event(const char *pressed_log, const char *released_log, int button_state) {
  if (button_state) {
    fill_buffer(pressed_log);
  } else {
    fill_buffer(released_log);
  }
}

void EXTI0_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR0) {
    log_event(MODE_PRESSED, MODE_RELEASED, AtModeButtonCheck());

    EXTI->PR |= EXTI_PR_PR0;
  }
}

void EXTI3_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR3) {
    log_event(LEFT_PRESSED, LEFT_RELEASED, JoystickLeftCheck());

    EXTI->PR |= EXTI_PR_PR3;
  }
}

void EXTI4_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR4) {
    log_event(RIGHT_PRESSED, RIGHT_RELEASED, JoystickRightCheck());

    EXTI->PR |= EXTI_PR_PR4;
  }
}

void EXTI9_5_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR5) {
    log_event(UP_PRESSED, UP_RELEASED, JoystickUpCheck());

    EXTI->PR |= EXTI_PR_PR5;
  }

  if (EXTI->PR & EXTI_PR_PR6) {
    log_event(DOWN_PRESSED, DOWN_RELEASED, JoystickDownCheck());

    EXTI->PR |= EXTI_PR_PR6;
  }
}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR10) {
    log_event(FIRE_PRESSED, FIRE_RELEASED, JoystickActionCheck());

    EXTI->PR |= EXTI_PR_PR10;
  }

  if (EXTI->PR & EXTI_PR_PR13) {
    log_event(USER_PRESSED, USER_RELEASED, UserButtonCheck());

    EXTI->PR |= EXTI_PR_PR13;
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

int main() {
  set_clock();
  basic_configuration();

  configurate_buttons();

  for (;;) {}
}
