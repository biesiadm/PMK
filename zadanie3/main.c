#include <stm32.h>
#include "leds.h"
#include "dma.h"
#include "timer.h"
#include "i2c.h"

static void set_clock();
static void configuration();

void set_clock() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
      RCC_AHB1ENR_GPIOBEN |
      RCC_AHB1ENR_GPIOCEN |
      RCC_AHB1ENR_DMA1EN;

  RCC->APB1ENR |= RCC_APB1ENR_USART2EN |
      RCC_APB1ENR_I2C1EN |
      RCC_APB1ENR_TIM3EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void configuration() {
  set_clock();
  configurate_dma();
  configurate_leds();

  configurate_i2c();
  configurate_timer();
}

int main() {
  configuration();
  Green2LEDon();

  while (1) {
    if (check_accelerometer_configurated()) {
      check_updates();
    }
  }
}
