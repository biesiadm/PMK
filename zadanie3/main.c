#include <gpio.h>
#include <stm32.h>
#include "leds.h"
#include "dma.h"
#include "timer.h"
#include "i2c.h"

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
#define HSI_HZ 16000000U

// Układ UART2 jest taktowany zegarem PCLK1, który powłączeniu mikrokontrolera jest zegarem HSI
#define PCLK1_HZ HSI_HZ

#define CLEAR_EXTI_PR 0x7FFFF

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
}

void TIM3_IRQHandler(void) {
  uint32_t it_status = TIM3->SR & TIM3->DIER;

  if (it_status & TIM_SR_UIF) {
    TIM3->SR = ~TIM_SR_UIF;
    Green2LEDoff();
    update_leds_by_acc();
    Green2LEDon();
  }
}


int main() {
  set_clock();
  basic_configuration();
  configurate_leds();     // FIXME is necessary?

  configurate_timer();
  configurate_i2c();

  all_leds_off();
  Green2LEDon();

  for (;;) {}
}
