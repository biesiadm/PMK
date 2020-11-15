#include <delay.h>
#include <gpio.h>
#include <stm32.h>


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

  // Tryb pracy
#define USART_Mode_Rx_Tx (USART_CR1_RE | \
                          USART_CR1_TE)
#define USART_Enable     USART_CR1_UE

  // Przesyłane słowo to dane łącznie z ewentualnym bitem parzystości
#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

  // Bit parzystości
#define USART_Parity_No   0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd  (USART_CR1_PCE | \
                           USART_CR1_PS)

// CR2

  // Bity stopu
#define USART_StopBits_1   0x0000
#define USART_StopBits_0_5 0x1000
#define USART_StopBits_2   0x2000
#define USART_StopBits_1_5 0x3000

// CR3

  // Sterowanie przepływem
#define USART_FlowControl_None 0x0000
#define USART_FlowControl_RTS  USART_CR3_RTSE
#define USART_FlowControl_CTS  USART_CR3_CTSE

// BRR

  // Po włączeniu mikrokontroler STM32F411 jest taktowany wewnętrznym generatorem RC HSI (ang.High SpeedInternal) o częstotliwości 16 MHz
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

static inline int UserButtonCheck() {
  return (AT_MODE_BUTTON_GPIO->IDR & (1 << AT_MODE_BUTTON_PIN));
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

void check_command(char *color, char *state, char curr, int *pos) {
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
      }
      else {
        *pos = CMD_PREFIX_POS;
      }

      break;
    case CMD_STATE_POS:
      if (curr == CMD_STATE_ON ||
          curr == CMD_STATE_OFF ||
          curr == CMD_STATE_TOGGLE) {
        *state = curr;
        (*pos)++;
      }
      else {
        *pos = CMD_PREFIX_POS;
      }

      break;
  }
}

void execute_cmd_color(char state, int *states, int state_pos, void (*led_on)(), void (*led_off)()) {
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
      }
      else {
        (*led_on)();
        states[state_pos] = STATE_ON;
      }

      break;
  }
}

void execute_command(char color, char state, int *states) {
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


int main() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOBEN;

//********************************************

  // Włączamy taktowanie odpowiednich układów peryferyjnych
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

//********************************************

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


  // Ustawiamy bit UE w rejestrze CR1
  USART2->CR1 |= USART_Enable;


  __NOP();

  RedLEDoff();
  GreenLEDoff();
  BlueLEDoff();
  Green2LEDoff();

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

  // GPIOinConfigure(USER_BUTTON_GPIO,
  //                 USER_BUTTON_PIN,
  //                 GPIO_PuPd_DOWN,
  //                 EXTI_Mode_Event,
  //                 EXTI_Trigger_Irrelevant);

  // GPIOoutConfigure(AT_MODE_BUTTON_GPIO,
  //                  AT_MODE_BUTTON_PIN,
  //                  GPIO_OType_PP,
  //                  GPIO_Fast_Speed,
  //                  GPIO_PuPd_DOWN);


  GPIOinConfigure(AT_MODE_BUTTON_GPIO,
                  AT_MODE_BUTTON_PIN,
                  GPIO_PuPd_DOWN,
                  EXTI_Mode_Event,
                  EXTI_Trigger_Irrelevant);

  char color = INITIALISE;
  char state = INITIALISE;
  int states[NUMBER_OF_COLORS];
  int pos = CMD_PREFIX_POS;

  for (int i = 0; i < NUMBER_OF_COLORS; i++) {
    states[i] = STATE_OFF;
  }

  char c = INITIALISE;

  for (;;) {
     if (USART2->SR & USART_SR_RXNE) {
         c = USART2->DR;
         check_command(&color, &state, c, &pos);

         if (pos == CMD_COMPLETE_POS) {
           execute_command(color, state, states);
           pos = CMD_PREFIX_POS;
         }

     }
     if (USART2->SR & USART_SR_TXE) {
       if (UserButtonCheck()) {
         USART2->DR = (char) c + 1;
       }
     }
  }

  // for (;;) {
  //   RedLEDon();
  //   Delay(4000000);
  //   RedLEDoff();
  //   GreenLEDon();
  //   Delay(4000000);
  //   GreenLEDoff();
  //   BlueLEDon();
  //   Delay(4000000);
  //   BlueLEDoff();
  //   Green2LEDon();
  //   Delay(4000000);
  //   Green2LEDoff();
  // }
}


////////////////////////////////////////////
/*              PRINTF START              */
////////////////////////////////////////////

// #include <errno.h>
// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <sys/stat.h>
//
//
//   /* Function used by C memory allocator */
// caddr_t _sbrk(int incr) {
//   register char *stack_ptr asm("sp"); /* Just stack pointer register name */
//   extern char end; /* First free memory location, defined in linker script */
//   static char *heap_end;
//   char *prev_heap_end;
//
//   if (heap_end == NULL)
//     heap_end = &end;
//
//   if (heap_end + incr > stack_ptr) {
//     /* Some of the libstdc++-v3 tests rely upon detecting
//        out of memory errors, so do not abort here.  */
//     errno = ENOMEM;
//     return (caddr_t)-1;
//   }
//
//   prev_heap_end = heap_end;
//   heap_end += incr;
//
//   errno = 0;
//   return (caddr_t)prev_heap_end;
// }
//
//
// long _read(int fd, char *ptr, long len) {
//   if (ptr == 0 || len <= 0) {
//     errno = EINVAL; return -1;
//   }
//   else if (fd == STDIN_FILENO) {
//     while (!(USART2->SR & USART_SR_RXNE));
//       *ptr = USART2->DR;
//       errno = 0; return 1;
//   }
//   else {
//     errno = EBADF; return -1;
//   }
// }
//
// long _write(int fd, char const *ptr, long len) {
//   long tmp = len;
//   if (ptr == 0 || len < 0) {
//     errno = EINVAL; return -1;
//   }
//   else if (fd == STDOUT_FILENO ||fd == STDERR_FILENO) {
//     while (tmp--) {
//       while (!(USART2->SR & USART_SR_TXE));
//       USART2->DR = *ptr++;
//     }
//
//     errno = 0; return len;
//   }
//   else {
//     errno = EBADF; return -1;
//   }
// }
//
// int _fstat(int fd, struct stat *st) {
//   if (st == 0) {
//     errno = EINVAL; return -1;
//   }
//   else if (fd == STDIN_FILENO ||
//            fd == STDOUT_FILENO ||
//            fd == STDERR_FILENO) {
//     memset(st, 0, sizeof(struct stat));
//     st->st_mode = S_IFCHR | 0666;
//     errno = 0; return 0;
//   }
//   else {
//     errno = EBADF; return -1;
//   }
// }
//
// long _lseek(int fd, long offset, int whence) {
//   errno = 0; return 0;
// }
//
// int _isatty(int fd) {
//   errno = 0;
//   return fd == STDIN_FILENO  ||
//   fd == STDOUT_FILENO ||
//   fd == STDERR_FILENO;
// }
//
// int _close(int fd) {
//   errno = 0; return 0;
// }
//
// int _open(const char *path, int flags, ...) {
//   if (strncmp(path, "tty", 3) == 0) {
//     init_usart();
//     errno = 0; return STDOUT_FILENO;
//   }
//   else {
//     errno = EACCES; return -1;
//   }
// }
//
//
// int main() {
//   fopen("tty", "w");
//   printf("Hello world\n");
//
//   for(;;);
// }

//////////////////////////////////////////
/*              PRINTF END              */
//////////////////////////////////////////

// #define RedLEDon()     \
//   RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
// #define RedLEDoff()    \
//   RED_LED_GPIO->BSRR = 1 << RED_LED_PIN
//
// #define GreenLEDon()   \
//   GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
// #define GreenLEDoff()  \
//   GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN
//
// #define BlueLEDon()    \
//   BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
// #define BlueLEDoff()   \
//   BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN
//
// #define Green2LEDon()  \
//   GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
// #define Green2LEDoff() \
//   GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)


// void execute_cmd_green(char state, int *states) {
//   switch (state) {
//     case CMD_STATE_ON:
//       GreenLEDon();
//       states[GREEN_STATE_POS] = STATE_ON;
//       break;
//     case CMD_STATE_OFF:
//       GreenLEDoff();
//       states[GREEN_STATE_POS] = STATE_OFF;
//       break;
//     case CMD_STATE_TOGGLE:
//       if (states[GREEN_STATE_POS]) {
//         GreenLEDoff();
//         states[GREEN_STATE_POS] = STATE_OFF;
//       }
//       else {
//         GreenLEDon();
//         states[GREEN_STATE_POS] = STATE_ON;
//       }
//   }
// }
//
// void execute_cmd_red(char state, int *states) {
//   switch (state) {
//     case CMD_STATE_ON:
//       RedLEDon();
//       states[RED_STATE_POS] = STATE_ON;
//       break;
//     case CMD_STATE_OFF:
//       RedLEDoff();
//       states[RED_STATE_POS] = STATE_OFF;
//       break;
//     case CMD_STATE_TOGGLE:
//       if (states[RED_STATE_POS]) {
//         RedLEDoff();
//         states[RED_STATE_POS] = STATE_OFF;
//       }
//       else {
//         RedLEDon();
//         states[RED_STATE_POS] = STATE_ON;
//       }
//   }
// }
//
// void execute_cmd_blue(char state, int *states) {
//   switch (state) {
//     case CMD_STATE_ON:
//       BlueLEDon();
//       states[BLUE_STATE_POS] = STATE_ON;
//       break;
//     case CMD_STATE_OFF:
//       BlueLEDoff();
//       states[BLUE_STATE_POS] = STATE_OFF;
//       break;
//     case CMD_STATE_TOGGLE:
//       if (states[BLUE_STATE_POS]) {
//         BlueLEDoff();
//         states[BLUE_STATE_POS] = STATE_OFF;
//       }
//       else {
//         BlueLEDon();
//         states[BLUE_STATE_POS] = STATE_ON;
//       }
//   }
// }
//
// void execute_cmd_green2(char state, int *states) {
//   switch (state) {
//     case CMD_STATE_ON:
//       Green2LEDon();
//       states[GREEN2_STATE_POS] = STATE_ON;
//       break;
//     case CMD_STATE_OFF:
//       Green2LEDoff();
//       states[GREEN2_STATE_POS] = STATE_OFF;
//       break;
//     case CMD_STATE_TOGGLE:
//       if (states[GREEN2_STATE_POS]) {
//         Green2LEDoff();
//         states[GREEN2_STATE_POS] = STATE_OFF;
//       }
//       else {
//         Green2LEDon();
//         states[GREEN2_STATE_POS] = STATE_ON;
//       }
//   }
// }


// void execute_command(char color, char state, int *states) {
//   switch (color) {
//     case CMD_GREEN:
//       execute_cmd_green(state, states);
//       break;
//     case CMD_RED:
//       execute_cmd_red(state, states);
//       break;
//     case CMD_BLUE:
//       execute_cmd_blue(state, states);
//       break;
//     case CMD_GREEN2:
//       execute_cmd_green2(state, states);
//       break;
//   }
// }
