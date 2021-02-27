#include <gpio.h>
#include "lis35de.h"
#include "i2c.h"
#include "timer.h"

#define LIS35_REG_CR1 0x20
#define LIS35_REG_CR1_XEN 0x1
#define LIS35_REG_CR1_YEN 0x2
#define LIS35_REG_CR1_ZEN 0x4
#define LIS35_REG_CR1_DR_400HZ 0x80
#define LIS35_REG_CR1_ACTIVE 0x40
#define LIS35_REG_CR1_FULL_SCALE 0x20

#define LIS35DE_ADDR 0x1C
#define OUT_X 0x29
#define OUT_Y 0x2B
#define OUT_Z 0x2D

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

static uint8_t acc_axis_values[3] = {0};

static unsigned calculate_acc_percent(uint8_t slave_register);
static unsigned calculate_int8_percent(int8_t value);

static void update_red_by_acc(uint8_t slave_register);
static void update_green_by_acc(uint8_t slave_register);
static void update_blue_by_acc(uint8_t slave_register);

void config_accelerometer() {
  uint8_t to_send[2] = {
      LIS35_REG_CR1,
      LIS35_REG_CR1_ACTIVE |
          LIS35_REG_CR1_XEN |
          LIS35_REG_CR1_YEN |
          LIS35_REG_CR1_ZEN
  };

  enqueue_command(LIS35DE_ADDR, to_send, 2, 0, 0);
}

int8_t read_from_accelerometer(uint8_t slave_register) {
  uint8_t to_send[1] = {slave_register};
  uint8_t *axis_value = 0;
  switch (slave_register) {
    case OUT_X:
      axis_value = &acc_axis_values[X_AXIS];
      break;
    case OUT_Y:
      axis_value = &acc_axis_values[Y_AXIS];
      break;
    case OUT_Z:
      axis_value = &acc_axis_values[Z_AXIS];
      break;
    default:
      return 0;
  }

  enqueue_command(LIS35DE_ADDR, to_send, 1, axis_value, 1);

  return axis_value[0];
}

void update_leds_by_acc() {
  update_red_by_acc(OUT_X);
  update_green_by_acc(OUT_Y);
  update_blue_by_acc(OUT_Z);
}

void update_red_by_acc(uint8_t slave_register) {
  unsigned acc_percent = calculate_acc_percent(slave_register);
  setRedLEDPower(acc_percent);
}

void update_green_by_acc(uint8_t slave_register) {
  unsigned acc_percent = calculate_acc_percent(slave_register);
  setGreenLEDPower(acc_percent);
}

void update_blue_by_acc(uint8_t slave_register) {
  unsigned acc_percent = calculate_acc_percent(slave_register);
  setBlueLEDPower(acc_percent);
}


unsigned calculate_acc_percent(uint8_t slave_register) {
  int8_t acc_value = read_from_accelerometer(slave_register);
  return calculate_int8_percent(acc_value);
}

unsigned calculate_int8_percent(int8_t value) {
  if (value < 0) {
    value = -value;
  }

  return ((((unsigned) value) * 100) / INT8_MAX) % 100;
}