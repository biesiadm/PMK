#include <gpio.h>
#include <stdbool.h>
#include "i2c.h"
#include "timer.h"
#include "dma.h"

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

volatile int FLAG = 0;

static const int TRIES_LIMIT = I2C_SPEED_HZ / 10;
static const int READ = 1;
static const int WRITE = 0;

static void update_red_by_acc(uint8_t slave_register);
static void update_green_by_acc(uint8_t slave_register);
static void update_blue_by_acc(uint8_t slave_register);

static void enable_interrupts();
static void config_accelerometer();
static int8_t read_from_accelerometer(uint8_t slave_register);
static bool i2c_write_read(uint8_t slave_addr, uint8_t *to_send, int n, uint8_t *to_receive, int m);
static bool i2c_try_to_write(uint8_t slave_addr, uint8_t *to_send, int n);
static void i2c_send_addr(uint8_t slave_addr, int mode);
static bool i2c_send_data(uint8_t *to_send, int n);
static bool i2c_try_to_read(uint8_t slave_addr, uint8_t *to_receive, int m);
static bool i2c_read_data(uint8_t *to_receive, int m);
static bool i2c_send_start();
static unsigned calculate_acc_percent(uint8_t slave_register);
static unsigned calculate_int8_percent(int8_t value);
static bool i2c_wait_for_bit_sr1(uint32_t bit);

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

  enable_interrupts();
  I2C1->CR1 |= I2C_CR1_START;
//  config_accelerometer();
}

void update_leds_by_acc() {
  update_red_by_acc(OUT_X);
  update_green_by_acc(OUT_Y);
  update_blue_by_acc(OUT_Z);
}

int flag_true() {
  return FLAG;
}

void I2C1_EV_IRQHandler(void) {
  uint16_t it_status = I2C1->SR1;

  if (it_status & I2C_SR1_SB) {
    log_event("EV_SB\r\n");
    i2c_send_addr(LIS35DE_ADDR, WRITE);
  } else if (it_status & I2C_SR1_ADDR) {
    log_event("EV_ADDR\r\n");
    I2C1->SR2;
    I2C1->DR = LIS35_REG_CR1;
    I2C1->CR2 |= I2C_CR2_ITBUFEN;
  } else if (it_status & I2C_SR1_BTF) {
    log_event("EV_BTF\r\n");
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR2 &= ~(I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
    FLAG = 1;
  } else if (it_status & I2C_SR1_TXE) {
    log_event("EV_TXE\r\n");
    I2C1->DR = LIS35_REG_CR1_ACTIVE |
        LIS35_REG_CR1_XEN |
        LIS35_REG_CR1_YEN |
        LIS35_REG_CR1_ZEN;
    I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
  } else if (it_status & I2C_SR1_RXNE) {
    log_event("EV_RXNE\r\n");
  }
}

void I2C1_ER_IRQHandler(void) {
  uint32_t it_status = I2C1->SR1;
  log_event("I2c_ER\r\n");

  if (it_status & I2C_SR1_SB) {
    log_event("ER_SB\r\n");
  } else if (it_status & I2C_SR1_ADDR) {
    log_event("ER_ADDR\r\n");
  } else if (it_status & I2C_SR1_BTF) {
    log_event("ER_BTF\r\n");
  } else if (it_status & I2C_SR1_TXE) {
    log_event("ER_TXE\r\n");
  } else if (it_status & I2C_SR1_RXNE) {
    log_event("ER_RXNE\r\n");
  }
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
  uint8_t to_send[2] = {
      LIS35_REG_CR1,
      LIS35_REG_CR1_ACTIVE |
          LIS35_REG_CR1_XEN |
          LIS35_REG_CR1_YEN |
          LIS35_REG_CR1_ZEN
  };

  i2c_write_read(LIS35DE_ADDR, to_send, 2, 0, 0);
}

bool i2c_write_read(uint8_t slave_addr, uint8_t *to_send, int n, uint8_t *to_receive, int m) {
  if (!i2c_try_to_write(slave_addr, to_send, n)) { return false; }

  if (to_receive && m) {
    if (!i2c_try_to_read(slave_addr, to_receive, m)) { return false; }
  } else {
    I2C1->CR1 |= I2C_CR1_STOP;
  }

  return true;
}

bool i2c_try_to_write(uint8_t slave_addr, uint8_t *to_send, int n) {
  if (to_send && n) {
    if (!i2c_send_start()) { return false; }

    i2c_send_addr(slave_addr, WRITE);
    if (!i2c_wait_for_bit_sr1(I2C_SR1_ADDR)) { return false; }
    I2C1->SR2;

    if (!i2c_send_data(to_send, n)) { return false; }
  }

  return true;
}

bool i2c_send_data(uint8_t *to_send, int n) {
  for (int i = 0; i < n - 1; i++) {
    I2C1->DR = to_send[i];
    if (!i2c_wait_for_bit_sr1(I2C_SR1_TXE)) { return false; }
  }

  I2C1->DR = to_send[n - 1];
  if (!i2c_wait_for_bit_sr1(I2C_SR1_BTF)) { return false; }

  return true;
}

bool i2c_try_to_read(uint8_t slave_addr, uint8_t *to_receive, int m) {
  if (to_receive && m) {
    if (!i2c_send_start()) { return false; }

    i2c_send_addr(slave_addr, READ);
    I2C1->CR1 |= I2C_CR1_ACK;
    if (!i2c_wait_for_bit_sr1(I2C_SR1_ADDR)) { return false; }
    I2C1->SR2;

    if (!i2c_read_data(to_receive, m)) { return false; }
  }

  return true;
}

bool i2c_read_data(uint8_t *to_receive, int m) {
  for (int i = 0; i < m - 1; i++) {
    if (!i2c_wait_for_bit_sr1(I2C_SR1_RXNE)) { return false; }

    to_receive[i] = I2C1->DR;
  }

  I2C1->CR1 &= ~I2C_CR1_ACK;
  I2C1->CR1 |= I2C_CR1_STOP;

  if (!i2c_wait_for_bit_sr1(I2C_SR1_RXNE)) { return false; }

  to_receive[m - 1] = I2C1->DR;
  return true;
}

bool i2c_send_start() {
  I2C1->CR1 |= I2C_CR1_START;
  if (!i2c_wait_for_bit_sr1(I2C_SR1_SB)) { return false; }
  return true;
}

void i2c_send_addr(uint8_t slave_addr, int mode) {
  I2C1->DR = slave_addr << 1 | (mode == READ);
}

bool i2c_wait_for_bit_sr1(uint32_t bit) {
  int tries = 0;
  while (!(I2C1->SR1 & bit) && tries < TRIES_LIMIT) {
    tries++;
  }

  if (tries == TRIES_LIMIT) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  return true;
}

int8_t read_from_accelerometer(uint8_t slave_register) {
  uint8_t to_send[1] = {slave_register};
  uint8_t to_recive[1] = {0};
  if (!i2c_write_read(LIS35DE_ADDR, to_send, 1, to_recive, 1)) { to_recive[0] = 0; }

  return to_recive[0];
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

void enable_interrupts() {
//  I2C1->CR2 = I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
  I2C1->CR2 = I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
//  I2C1->CR2 = I2C_CR2_ITERREN;
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}