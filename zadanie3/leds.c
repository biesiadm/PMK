#include <gpio.h>
#include "leds.h"

#define RED_LED_GPIO    GPIOA
#define GREEN_LED_GPIO  GPIOA
#define BLUE_LED_GPIO   GPIOB
#define GREEN2_LED_GPIO GPIOA

#define RED_LED_PIN    6
#define GREEN_LED_PIN  7
#define BLUE_LED_PIN   0
#define GREEN2_LED_PIN 5


void RedLEDon() {
  RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16);
}

void RedLEDoff() {
  RED_LED_GPIO->BSRR = 1 << RED_LED_PIN;
}

void GreenLEDon() {
  GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16);
}

void GreenLEDoff() {
  GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN;
}

void BlueLEDon() {
  BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16);
}

void BlueLEDoff() {
  BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN;
}

void Green2LEDon() {
  GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN;
}

void Green2LEDoff() {
  GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16);
}

void configurate_leds() {
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

void all_leds_off() {
  RedLEDoff();
  GreenLEDoff();
  BlueLEDoff();
  Green2LEDoff();
}