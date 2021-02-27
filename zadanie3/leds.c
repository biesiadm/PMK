#include <gpio.h>
#include "leds.h"

#define GREEN2_LED_GPIO GPIOA
#define GREEN2_LED_PIN 5

void Green2LEDon() {
  GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN;
}

void Green2LEDoff() {
  GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16);
}

void configurate_leds() {
  GPIOoutConfigure(GREEN2_LED_GPIO,
                   GREEN2_LED_PIN,
                   GPIO_OType_PP,
                   GPIO_Low_Speed,
                   GPIO_PuPd_NOPULL);
}
