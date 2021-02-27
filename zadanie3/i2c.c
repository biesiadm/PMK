#include <gpio.h>
#include <stdbool.h>
#include "i2c.h"
#include "timer.h"
#include "leds.h"
#include "i2c_command.h"

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

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define MAX_COMM_SIZE 16

static const int READ = 1;
static const int WRITE = 0;

static volatile int ACCELEROMETER_CONFIGURATED = 0;
static bool using_i2c = false;

static uint8_t acc_axis_values[3] = {0};

static void update_red_by_acc(uint8_t slave_register);
static void update_green_by_acc(uint8_t slave_register);
static void update_blue_by_acc(uint8_t slave_register);

static void enable_interrupts();
static void config_accelerometer();
static int8_t read_from_accelerometer(uint8_t slave_register);
static void try_to_send_addr(i2c_command_t *curr_command);
static void try_to_send_bytes(i2c_command_t *curr_command);
static void send_bytes(i2c_command_t *curr_command);
static void read_bytes(i2c_command_t *curr_command);
static void try_to_read_or_stop(i2c_command_t *curr_command);
static void i2c_send_addr(uint8_t slave_addr, int mode);
static void i2c_send_start();
static void i2c_send_stop();
static void i2c_send_ack();
static void i2c_send_nack();
static void i2c_enable_interrupts(uint32_t interrupts);
static void i2c_disable_interrupts(uint32_t interrupts);
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

  enable_interrupts();
  config_accelerometer();
}

void update_leds_by_acc() {
  update_red_by_acc(OUT_X);
  update_green_by_acc(OUT_Y);
  update_blue_by_acc(OUT_Z);
}

int check_accelerometer_configurated() {
  return ACCELEROMETER_CONFIGURATED;
}

void try_to_send_addr(i2c_command_t *curr_command) {
  if (get_already_sent(curr_command) == get_send_size(curr_command)) {
    i2c_send_addr(get_slave_addr(curr_command), READ);
    i2c_send_ack();
  } else if (!check_if_after_send(curr_command)) {
    set_after_send(curr_command);
    i2c_send_addr(get_slave_addr(curr_command), WRITE);
  }
}

void try_to_send_bytes(i2c_command_t *curr_command) {
  if (get_already_sent(curr_command) == get_send_size(curr_command)) {
    i2c_enable_interrupts(I2C_CR2_ITBUFEN);
    return;
  }

  if (get_already_sent(curr_command) < get_send_size(curr_command)) {
    I2C1->DR = get_next_to_send(curr_command);
    inc_already_sent(curr_command);

    if (get_already_sent(curr_command) == get_send_size(curr_command)) {
      i2c_disable_interrupts(I2C_CR2_ITBUFEN);
    } else {
      i2c_enable_interrupts(I2C_CR2_ITBUFEN);
    }
  }
}

void send_bytes(i2c_command_t *curr_command) {
  if (get_already_sent(curr_command) < get_send_size(curr_command)) {
    I2C1->DR = get_next_to_send(curr_command);
    inc_already_sent(curr_command);

    if (get_already_sent(curr_command) == get_send_size(curr_command)) {
      i2c_disable_interrupts(I2C_CR2_ITBUFEN);
    }
  }
}

void read_bytes(i2c_command_t *curr_command) {
  if (get_already_recv(curr_command) < get_recv_size(curr_command) - 1) {
    set_next_to_receive(curr_command, I2C1->DR);
  } else if (get_already_recv(curr_command) < get_recv_size(curr_command)) {
    inc_already_recv(curr_command);
    i2c_send_nack();
    i2c_send_stop();
  } else {
    set_last_to_receive(curr_command, I2C1->DR);
    set_finished(curr_command);
    i2c_disable_interrupts(I2C_CR2_ITBUFEN);
    using_i2c = false;
  }
}

void try_to_read_or_stop(i2c_command_t *curr_command) {
  if (get_recv_size(curr_command) == 0) {
    set_finished(curr_command);
    using_i2c = false;
    i2c_send_stop();
    ACCELEROMETER_CONFIGURATED = 1;
  } else {
    i2c_send_start();
  }
}

void I2C1_EV_IRQHandler(void) {
  Green2LEDoff();

  uint16_t it_status = I2C1->SR1;
  i2c_command_t *curr_command = get_curr_command();

  if (!curr_command) {
    using_i2c = false;
    return;
  }

  if (it_status & I2C_SR1_SB) {
    try_to_send_addr(curr_command);
  } else if (it_status & I2C_SR1_ADDR) {
    I2C1->SR2;
    try_to_send_bytes(curr_command);
  } else if (it_status & I2C_SR1_BTF) {
    try_to_read_or_stop(curr_command);
  } else if (it_status & I2C_SR1_TXE) {
    send_bytes(curr_command);
  } else if (it_status & I2C_SR1_RXNE) {
    read_bytes(curr_command);
  }

  Green2LEDon();
}

void I2C1_ER_IRQHandler(void) {
  Green2LEDoff();

  I2C1->SR1;

  using_i2c = false;
  i2c_command_t *curr_command = get_curr_command();
  set_finished(curr_command);

  Green2LEDon();
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

  add_to_command_buffer(LIS35DE_ADDR, to_send, 2, 0, 0);
  using_i2c = true;
  i2c_send_start();
}

void i2c_send_addr(uint8_t slave_addr, int mode) {
  I2C1->DR = slave_addr << 1 | (mode == READ);
}

void i2c_send_start() {
  I2C1->CR1 |= I2C_CR1_START;
}

void i2c_send_stop() {
  I2C1->CR1 |= I2C_CR1_STOP;
}

void i2c_send_ack() {
  I2C1->CR1 |= I2C_CR1_ACK;
}

void i2c_send_nack() {
  I2C1->CR1 &= ~I2C_CR1_ACK;
}

void i2c_enable_interrupts(uint32_t interrupts) {
  I2C1->CR2 |= interrupts;
}

void i2c_disable_interrupts(uint32_t interrupts) {
  I2C1->CR2 &= ~(interrupts);
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
  }

  add_to_command_buffer(LIS35DE_ADDR, to_send, 1, axis_value, 1);

  if (!using_i2c) {
    using_i2c = true;
    i2c_send_start();
  }

  return axis_value[0];
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
  i2c_enable_interrupts(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_EnableIRQ(I2C1_ER_IRQn);
}