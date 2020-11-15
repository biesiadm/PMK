#include <delay.h>
#include <gpio.h>
#include <stm32.h>

#define RED_LED_GPIO    GPIOA
#define GREEN_LED_GPIO  GPIOA
#define BLUE_LED_GPIO   GPIOB
#define GREEN2_LED_GPIO GPIOA

#define RED_LED_PIN    6
#define GREEN_LED_PIN  7
#define BLUE_LED_PIN   0
#define GREEN2_LED_PIN 5


#define RedLEDon()     \
  RED_LED_GPIO->BSRR = 1 << (RED_LED_PIN + 16)
#define RedLEDoff()    \
  RED_LED_GPIO->BSRR = 1 << RED_LED_PIN

#define GreenLEDon()   \
  GREEN_LED_GPIO->BSRR = 1 << (GREEN_LED_PIN + 16)
#define GreenLEDoff()  \
  GREEN_LED_GPIO->BSRR = 1 << GREEN_LED_PIN

#define BlueLEDon()    \
  BLUE_LED_GPIO->BSRR = 1 << (BLUE_LED_PIN + 16)
#define BlueLEDoff()   \
  BLUE_LED_GPIO->BSRR = 1 << BLUE_LED_PIN

#define Green2LEDon()  \
  GREEN2_LED_GPIO->BSRR = 1 << GREEN2_LED_PIN
#define Green2LEDoff() \
  GREEN2_LED_GPIO->BSRR = 1 << (GREEN2_LED_PIN + 16)


int main() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOBEN;

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

  for (;;) {
    RedLEDon();
    Delay(4000000);
    RedLEDoff();
    GreenLEDon();
    Delay(4000000);
    GreenLEDoff();
    BlueLEDon();
    Delay(4000000);
    BlueLEDoff();
    Green2LEDon();
    Delay(4000000);
    Green2LEDoff();
  }
}
