#include <stm32.h>
#include <gpio.h>
#include <math.h>
#include "timer.h"

#define RED_LED_GPIO    GPIOA
#define GREEN_LED_GPIO  GPIOA
#define BLUE_LED_GPIO   GPIOB

#define RED_LED_PIN    6
#define GREEN_LED_PIN  7
#define BLUE_LED_PIN   0

static const int LED_OFF = 0;
static const int ACC_CHECK = 100;

static void config_timer_leds();
static float calc_pwm(float val);
static void enable_interrupts();

void configurate_timer() {
  config_timer_leds();
  enable_interrupts();

  TIM3->PSC = 1;
  TIM3->ARR = 16000;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CCR1 = 0;   // Red
  TIM3->CCR2 = 0;   // Green
  TIM3->CCR3 = 0;   // Blue
  TIM3->CCR4 = ACC_CHECK; // acc check

  TIM3->CCMR1 =
      TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
          TIM_CCMR1_OC1PE |
          TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 |
          TIM_CCMR1_OC2PE;

  TIM3->CCMR2 =
      TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 |
          TIM_CCMR2_OC3PE |
          TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 |
          TIM_CCMR2_OC4PE;

  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
      TIM_CCER_CC2E | TIM_CCER_CC2P |
      TIM_CCER_CC3E | TIM_CCER_CC3P |
      TIM_CCER_CC4E | TIM_CCER_CC4P;

  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void setRedLEDPower(int power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) {
    TIM3->CCR1 = calc_pwm(power_percent);
  } else {
    TIM3->CCR1 = LED_OFF;
  }
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void setGreenLEDPower(int power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) {
    TIM3->CCR2 = calc_pwm(power_percent);
  } else {
    TIM3->CCR2 = LED_OFF;
  }
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void setBlueLEDPower(int power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) {
    TIM3->CCR3 = calc_pwm(power_percent);
  } else {
    TIM3->CCR3 = LED_OFF;
  }
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

// Source: https://forbot.pl/blog/kurs-stm32-f1-hal-liczniki-timery-w-praktyce-pwm-id24334
float calc_pwm(float val) {
  const float k = 0.1f;
  const float x0 = 60.0f;
  return TIM3->ARR / (1.0f + expf(-k * (val - x0)));
}

void config_timer_leds() {
  GPIOafConfigure(RED_LED_GPIO, RED_LED_PIN, GPIO_OType_PP,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_TIM3);
  GPIOafConfigure(GREEN_LED_GPIO, GREEN_LED_PIN, GPIO_OType_PP,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_TIM3);
  GPIOafConfigure(BLUE_LED_GPIO, BLUE_LED_PIN, GPIO_OType_PP,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_TIM3);
}

void enable_interrupts() {
  TIM3->SR = ~(TIM_SR_CC4IF);
  TIM3->DIER = TIM_DIER_CC4IE;
  NVIC_EnableIRQ(TIM3_IRQn);
}