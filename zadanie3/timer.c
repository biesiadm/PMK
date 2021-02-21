#include <stm32.h>
#include <gpio.h>
#include <math.h>
#include "timer.h"
#include "leds.h"
#include "i2c.h"

#define RED_LED_GPIO    GPIOA
#define GREEN_LED_GPIO  GPIOA
#define BLUE_LED_GPIO   GPIOB

#define RED_LED_PIN    6
#define GREEN_LED_PIN  7
#define BLUE_LED_PIN   0

static const int LED_OFF = 0;
static int UPDATE_LEDS = 0;

static void config_timer_leds();
static float calc_pwm(float val);
static void enable_interrupts();
static void set_update_flag();

void configurate_timer() {
  config_timer_leds();
  enable_interrupts();

  TIM3->PSC = 1;
  TIM3->ARR = 16000;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CCR1 = 0;   // Red
  TIM3->CCR2 = 0;   // Green
  TIM3->CCR3 = 0;   // Blue

  TIM3->CCMR1 =
      TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 |
          TIM_CCMR1_OC1PE |
          TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 |
          TIM_CCMR1_OC2PE;

  TIM3->CCMR2 =
      TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 |
          TIM_CCMR2_OC3PE;

  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P |
      TIM_CCER_CC2E | TIM_CCER_CC2P |
      TIM_CCER_CC3E | TIM_CCER_CC3P;

  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void setRedLEDPower(unsigned power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) {
    TIM3->CCR1 = calc_pwm(power_percent);
  } else {
    TIM3->CCR1 = LED_OFF;
  }
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void setGreenLEDPower(unsigned power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) {
    TIM3->CCR2 = calc_pwm(power_percent);
  } else {
    TIM3->CCR2 = LED_OFF;
  }
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void setBlueLEDPower(unsigned power_percent) {
  TIM3->CR1 |= TIM_CR1_UDIS;
  if (power_percent) {
    TIM3->CCR3 = calc_pwm(power_percent);
  } else {
    TIM3->CCR3 = LED_OFF;
  }
  TIM3->CR1 &= ~TIM_CR1_UDIS;
}

void TIM3_IRQHandler(void) {
  uint32_t it_status = TIM3->SR & TIM3->DIER;

  if (it_status & TIM_SR_UIF) {
    TIM3->SR = ~TIM_SR_UIF;
    Green2LEDoff();
//    update_leds_by_acc();
    set_update_flag();
    Green2LEDon();
  }
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
  TIM3->SR = ~(TIM_SR_UIF);
  TIM3->DIER = TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM3_IRQn);
}

void set_update_flag() {
  UPDATE_LEDS = 1;
}

void reset_update_flag() {
  UPDATE_LEDS = 0;
}

void check_updates() {
  if (UPDATE_LEDS) {
    reset_update_flag();
    update_leds_by_acc();
  }
}