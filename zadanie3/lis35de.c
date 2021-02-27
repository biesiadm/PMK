#include <gpio.h>
#include "lis35de.h"
#include "i2c.h"
#include "timer.h"

#define LIS35_REG_CR1 0x20
#define LIS35_REG_CR1_XEN 0x1
#define LIS35_REG_CR1_YEN 0x2
#define LIS35_REG_CR1_ZEN 0x4
#define LIS35_REG_CR1_ACTIVE 0x40

#define LIS35DE_ADDR 0x1C
#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0x2D

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

static uint8_t acc_axis_values[3] = {0};
static volatile int acc_config_enqueued = 0;

static unsigned calculate_acc_percent(int acc_axis);
static unsigned calculate_int8_percent(int8_t value);

static void update_red_by_acc();
static void update_green_by_acc();
static void update_blue_by_acc();

void config_accelerometer() {
  uint8_t to_send[2] = {
      LIS35_REG_CR1,
      LIS35_REG_CR1_ACTIVE |
          LIS35_REG_CR1_XEN |
          LIS35_REG_CR1_YEN |
          LIS35_REG_CR1_ZEN
  };

  enqueue_command(LIS35DE_ADDR, to_send, 2, 0, 0);
  acc_config_enqueued = 1;
}

void update_leds_by_acc() {
  update_red_by_acc();
  update_green_by_acc();
  update_blue_by_acc();
}

void enqueue_read(uint8_t acc_axis_addr, uint8_t *to_recv) {
  if (!acc_config_enqueued) { return; }
  uint8_t to_send[1] = {acc_axis_addr};

  enqueue_command(LIS35DE_ADDR, to_send, 1, to_recv, 1);
}

void update_red_by_acc() {
  enqueue_read(OUT_X, &acc_axis_values[X_AXIS]);
  unsigned acc_percent = calculate_acc_percent(X_AXIS);
  setRedLEDPower(acc_percent);
}

void update_green_by_acc() {
  enqueue_read(OUT_Y, &acc_axis_values[Y_AXIS]);
  unsigned acc_percent = calculate_acc_percent(Y_AXIS);
  setGreenLEDPower(acc_percent);
}

void update_blue_by_acc() {
  enqueue_read(OUT_Z, &acc_axis_values[Z_AXIS]);
  unsigned acc_percent = calculate_acc_percent(Z_AXIS);
  setBlueLEDPower(acc_percent);
}

unsigned calculate_acc_percent(int acc_axis) {
  int8_t acc_value = acc_axis_values[acc_axis];
  return calculate_int8_percent(acc_value);
}

unsigned calculate_int8_percent(int8_t value) {
  if (value < 0) {
    value = -value;
  }

  return ((((unsigned) value) * 100) / INT8_MAX) % 100;
}