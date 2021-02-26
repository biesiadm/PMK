#include <stm32.h>
#include <gpio.h>
#include "dma.h"

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


#define BUFF_SIZE 1024
static const int BUFF_START = 0;

static char cyclic_buffer[BUFF_SIZE];
static int cyclic_buffer_start = BUFF_START;
static int cyclic_buffer_end = BUFF_START;

static void activate_send_stream(int to_send_len);
static void try_to_send_msg();
static void fill_buffer(const char *msg);


void configurate_dma() {
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

void log_event(const char *message) {
  fill_buffer(message);
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

void activate_send_stream(int to_send_len) {
  DMA1_Stream6->M0AR = (uint32_t) (cyclic_buffer + cyclic_buffer_start);
  DMA1_Stream6->NDTR = to_send_len;
  DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void try_to_send_msg() {
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

void fill_buffer(const char *msg) {
  for (int i = 0; msg[i] != 0; i++) {
    cyclic_buffer[cyclic_buffer_end++] = msg[i];

    if (cyclic_buffer_end == BUFF_SIZE) {
      cyclic_buffer_end = BUFF_START;
    }
  }

  try_to_send_msg();
}
