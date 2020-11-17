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

#define USART_Mode_Rx_Tx (USART_CR1_RE | \
                          USART_CR1_TE)
#define USART_Enable      USART_CR1_UE

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No   0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd  (USART_CR1_PCE | \
                           USART_CR1_PS)

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

static inline void RedLEDon() {
  RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16);
}

static inline void RedLEDoff() {
  RED_LED_GPIO->BSRR = 1 << RED_LED_PIN;
}

static inline void GreenLEDon() {
  GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16);
}

static inline void GreenLEDoff() {
  GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN;
}

static inline void BlueLEDon() {
  BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16);
}

static inline void BlueLEDoff() {
  BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN;
}

static inline void Green2LEDon() {
  GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN;
}

static inline void Green2LEDoff() {
  GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16);
}

static inline int AtModeButtonCheck() {
  return (AT_MODE_BUTTON_GPIO->IDR & (1 << AT_MODE_BUTTON_PIN));
}

static inline int UserButtonCheck() {
  return !(USER_BUTTON_GPIO->IDR & (1 << USER_BUTTON_PIN));
}

static inline int JoystickDownCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_DOWN_PIN));
}

static inline int JoystickUpCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_UP_PIN));
}

static inline int JoystickLeftCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_LEFT_PIN));
}

static inline int JoystickRightCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_RIGHT_PIN));
}

static inline int JoystickActionCheck() {
  return !(JOYSTICK_GPIO->IDR & (1 << JOYSTICK_ACTION_PIN));
}

static void set_clock() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
      RCC_AHB1ENR_GPIOBEN |
      RCC_AHB1ENR_GPIOCEN;

  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}

static void basic_configuration() {
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

  // Przykładowa konfiguracja (układ pozostaje nieaktywny – nie ustawiamy bitu USART_Enable)
  USART2->CR1 = USART_Mode_Rx_Tx |
      USART_WordLength_8b |
      USART_Parity_No;

  USART2->CR2 = USART_StopBits_1;

  USART2->CR3 = USART_FlowControl_None;

  uint32_t const baudrate = 9600U;
  USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) /
      baudrate;

  USART2->CR1 |= USART_Enable;
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

static const int BUFF_SIZE = 1024;
static const int BUFF_START = 0;
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

static void fill_buffer(char *buffer, int *end, const char *msg) {
  for (int i = 0; msg[i] != 0; i++) {
    buffer[(*end)++] = msg[i];

    if ((*end) == BUFF_SIZE) {
      *end = BUFF_START;
    }
  }
}

static void check_button(char *buffer, int *end, int *buttons_states,
                         int (*button_state)(), int button_pos,
                         const char *pressed_msg, const char *relased_msg) {
  int curr_state = RELEASED;

  if ((curr_state = (*button_state)()) != buttons_states[button_pos]) {
    buttons_states[button_pos] = curr_state;

    if (curr_state) {
      fill_buffer(buffer, end, pressed_msg);
    } else {
      fill_buffer(buffer, end, relased_msg);
    }
  }
}

static void check_buttons(char *buffer, int *end, int *buttons_states) {
  check_button(buffer, end, buttons_states, JoystickLeftCheck,
               JOYSTICK_LEFT_POS, LEFT_PRESSED, LEFT_RELEASED);

  check_button(buffer, end, buttons_states, JoystickRightCheck,
               JOYSTICK_RIGHT_POS, RIGHT_PRESSED, RIGHT_RELEASED);

  check_button(buffer, end, buttons_states, JoystickUpCheck,
               JOYSTICK_UP_POS, UP_PRESSED, UP_RELEASED);

  check_button(buffer, end, buttons_states, JoystickDownCheck,
               JOYSTICK_DOWN_POS, DOWN_PRESSED, DOWN_RELEASED);

  check_button(buffer, end, buttons_states, JoystickActionCheck,
               JOYSTICK_ACTION_POS, FIRE_PRESSED, FIRE_RELEASED);

  check_button(buffer, end, buttons_states, UserButtonCheck,
               USER_BUTTON_POS, USER_PRESSED, USER_RELEASED);

  check_button(buffer, end, buttons_states, AtModeButtonCheck,
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

static void send_to_output(char *to_send, int *to_send_start, int *to_send_end) {
  if (USART2->SR & USART_SR_TXE) {
    if ((*to_send_start) != (*to_send_end)) {
      USART2->DR = to_send[(*to_send_start)++];

      if ((*to_send_start) == BUFF_SIZE) {
        *to_send_start = BUFF_START;
      }
    }
  }
}

int main() {
  set_clock();
  basic_configuration();

  all_leds_off();
  configurate_leds();

  char color = INITIALISE;
  char state = INITIALISE;
  int pos = CMD_PREFIX_POS;

  int states[NUMBER_OF_COLORS];

  for (int i = 0; i < NUMBER_OF_COLORS; i++) {
    states[i] = STATE_OFF;
  }

  char to_send[BUFF_SIZE];
  int to_send_start = BUFF_START;
  int to_send_end = BUFF_START;

  int buttons_states[BUTTONS_NUMBER];

  for (int i = 0; i < BUTTONS_NUMBER; i++) {
    buttons_states[i] = RELEASED;
  }

  for (;;) {
    check_for_input(&pos, &color, &state, states);
    check_buttons(to_send, &to_send_end, buttons_states);
    send_to_output(to_send, &to_send_start, &to_send_end);
  }
}
