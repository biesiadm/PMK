#include <stm32.h>
#include "leds.h"
#include "timer.h"
#include "i2c.h"
#include "lis35de.h"

static void set_clock();
static void configuration();

void set_clock() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
      RCC_AHB1ENR_GPIOBEN |
      RCC_AHB1ENR_GPIOCEN;

  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_TIM3EN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void configuration() {
  set_clock();
  config_leds();

  config_i2c();
  config_accelerometer();
  config_timer();
}

int main() {
  configuration();
  Green2LEDon();

  while (1) {
    check_updates();
  }
}
