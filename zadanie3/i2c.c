#include <gpio.h>
#include "i2c.h"
#include "timer.h"

#define I2C_SPEED_HZ 100000
#define PCLK1_MHZ 16

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

static const int TRIES_LIMIT = I2C_SPEED_HZ / 10;

static void update_red_by_acc(uint8_t slave_register);
static void update_green_by_acc(uint8_t slave_register);
static void update_blue_by_acc(uint8_t slave_register);

void config_accelerometer();
static int8_t read_from_accelerometer(uint8_t slave_register);
static void i2c_start(uint8_t slave_register);
static void i2c_repeated_start();
static int8_t i2c_stop();
static unsigned calculate_acc_percent(uint8_t slave_register);
static unsigned calculate_int8_percent(int8_t value);

void configurate_i2c() {
  // Konfiguracja SCL na PB8
  GPIOafConfigure(GPIOB, 8, GPIO_OType_OD,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);

  // Konfiguracja SDA na PB9
  GPIOafConfigure(GPIOB, 9, GPIO_OType_OD,
                  GPIO_Low_Speed, GPIO_PuPd_NOPULL,
                  GPIO_AF_I2C1);

  I2C1->CR1 = 0;
  I2C1->CCR = (PCLK1_MHZ * 1000000) / (I2C_SPEED_HZ << 1);
  I2C1->CR2 = PCLK1_MHZ;
  I2C1->TRISE = PCLK1_MHZ + 1;

  I2C1->CR1 |= I2C_CR1_PE;

  config_accelerometer();
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

void config_accelerometer() {
  int tries = 0;

  I2C1->CR1 |= I2C_CR1_START;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_SB) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }

  I2C1->DR = LIS35DE_ADDR << 1;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_ADDR) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }
  I2C1->SR2;

  I2C1->DR = LIS35_REG_CR1;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_TXE) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }

  I2C1->DR = LIS35_REG_CR1_ACTIVE |
      LIS35_REG_CR1_XEN |
      LIS35_REG_CR1_YEN |
      LIS35_REG_CR1_ZEN;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_BTF) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }

  I2C1->CR1 |= I2C_CR1_STOP;
}

int8_t read_from_accelerometer(uint8_t slave_register) {
  i2c_start(slave_register);
  i2c_repeated_start();

  return i2c_stop();
}

void i2c_start(uint8_t slave_register) {
  int tries = 0;

  I2C1->CR1 |= I2C_CR1_START;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_SB) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }

  I2C1->DR = LIS35DE_ADDR << 1;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_ADDR) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }
  I2C1->SR2;

  I2C1->DR = slave_register;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_BTF) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }
}

void i2c_repeated_start() {
  int tries = 0;

  I2C1->CR1 |= I2C_CR1_START;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_SB) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }

  I2C1->DR = LIS35DE_ADDR << 1 | 1;
  I2C1->CR1 &= ~I2C_CR1_ACK;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_ADDR) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return;
  }
  I2C1->SR2;
}

int8_t i2c_stop() {
  int tries = 0;

  I2C1->CR1 |= I2C_CR1_STOP;
  for (tries = 0; !(I2C1->SR1 & I2C_SR1_RXNE) && tries < TRIES_LIMIT; tries++) {}
  if (tries == TRIES_LIMIT) {
    return 0;
  }

  return I2C1->DR;
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