#include <gpio.h>
#include <stm32.h>
#include <math.h>
#include "leds.h"
#include "print_dma.h"

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

// I2C
#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16

static const int LED_OFF = 0;

static void set_clock() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
      RCC_AHB1ENR_GPIOBEN |
      RCC_AHB1ENR_GPIOCEN |
      RCC_AHB1ENR_DMA1EN;

  RCC->APB1ENR |= RCC_APB1ENR_USART2EN |
      RCC_APB1ENR_I2C1EN |
      RCC_APB1ENR_TIM3EN;
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

  DMA1_Stream6->PAR = (uint32_t) &USART2->DR;

  DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  USART2->CR1 |= USART_Enable;

  // Konfiguracja linii TXD
  GPIOafConfigure(GPIOA, 2, GPIO_OType_PP,
                  GPIO_Fast_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_USART2);

  // Konfiguracja linii RXD
  GPIOafConfigure(GPIOA, 3, GPIO_OType_PP,
                  GPIO_Fast_Speed, GPIO_PuPd_UP,
                  GPIO_AF_USART2);

  // Konfiguracja SCL na PB8
  GPIOafConfigure(GPIOB, 8, GPIO_OType_OD,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);

  // Konfiguracja SDA na PB9
  GPIOafConfigure(GPIOB, 9, GPIO_OType_OD,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);

  I2C1->CR1 = 0;
  I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
  I2C1->CR2 = PCLK1_MHZ;
  I2C1->TRISE = PCLK1_MHZ + 1;

  I2C1->CR1 |= I2C_CR1_PE;
}

// Source: https://forbot.pl/blog/kurs-stm32-f1-hal-liczniki-timery-w-praktyce-pwm-id24334
float calc_pwm(float val) {
  const float k = 0.1f;
  const float x0 = 60.0f;
  return TIM3->ARR / (1.0f + expf(-k * (val - x0)));
}

void config_timer_leds() {
  GPIOafConfigure(GPIOA, 6, GPIO_OType_PP,
                  GPIO_Low_Speed,GPIO_PuPd_NOPULL,
                  GPIO_AF_TIM3);
  GPIOafConfigure(GPIOA, 7, GPIO_OType_PP,
                  GPIO_Low_Speed,GPIO_PuPd_NOPULL,
                  GPIO_AF_TIM3);
  GPIOafConfigure(GPIOB, 0, GPIO_OType_PP,
                  GPIO_Low_Speed,GPIO_PuPd_NOPULL,
                  GPIO_AF_TIM3);
}

void configurate_timer() {
//  TIM3->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF);
//  TIM3->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE;
//  NVIC_EnableIRQ(TIM3_IRQn);

  TIM3->PSC = 1;
  TIM3->ARR = 16000;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;

  TIM3->CCMR1 =
      TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
      TIM_CCMR1_OC1PE  |
      TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 |
//      TIM_CCMR1_OC2M_0 |
      TIM_CCMR1_OC2PE;

  // Blue
  TIM3->CCR3 = 0;

  TIM3->CCMR2 =
      TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 |
      TIM_CCMR2_OC3PE;

  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
      TIM_CCER_CC2E | TIM_CCER_CC2P |
      TIM_CCER_CC3E | TIM_CCER_CC3P;

  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void TIM3_IRQHandler(void) {
  uint32_t it_status = TIM3->SR & TIM3->DIER;

  if (it_status & TIM_SR_UIF) {
    TIM3->SR = ~TIM_SR_UIF;
    log_event("TIM_SR_UIF\r\n");
  }

  if (it_status & TIM_SR_CC1IF) {
    TIM3->SR = ~TIM_SR_CC1IF;
    log_event("TIM_SR_CC1IF\r\n");
    BlueLEDon();
  }

  if (it_status & TIM_SR_CC2IF) {
    TIM3->SR = ~TIM_SR_CC2IF;
    log_event("TIM_SR_CC2IF\r\n");
    BlueLEDoff();
  }
}

void my_sleep(int milis) {
#define STEPS_PER_MILI 4000
  for (int ms = 0; ms < milis; ms++) {
    for (int i = 0; i < STEPS_PER_MILI; i++) {
      asm("nop");
    }
  }
}

void setRedLEDPower(int power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) TIM3->CCR1 = calc_pwm(power_percent);
  else TIM3->CCR1 = LED_OFF;
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void setGreenLEDPower(int power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) TIM3->CCR2 = calc_pwm(power_percent);
  else TIM3->CCR2 = LED_OFF;
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void setBlueLEDPower(int power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) TIM3->CCR3 = calc_pwm(power_percent);
  else TIM3->CCR3 = LED_OFF;
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

int main() {
  set_clock();
  basic_configuration();
  configurate_leds();

  config_timer_leds();
  configurate_timer();

  all_leds_off();

  for (;;) {}
}
