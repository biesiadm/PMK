#include <stm32.h>
#include "print_dma.h"

// Bufory

#define BUFF_SIZE 1024
static const int BUFF_START = 0;

char cyclic_buffer[BUFF_SIZE];
int cyclic_buffer_start = BUFF_START;
int cyclic_buffer_end = BUFF_START;


static void activate_send_stream(int to_send_len) {
  DMA1_Stream6->M0AR = (uint32_t) (cyclic_buffer + cyclic_buffer_start);
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