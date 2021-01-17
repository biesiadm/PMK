#include <gpio.h>
#include "i2c.h"

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
}

void write_to_accelerometer() {
  I2C1->CR1 |= I2C_CR1_START;
  while (!(I2C1->SR1 & I2C_SR1_SB)) {}

  I2C1->DR = LIS35DE_ADDR << 1;
  while (!(I2C1->SR1 & I2C_SR1_ADDR)) {}
  I2C1->SR2;

  I2C1->DR = LIS35_REG_CR1;
  while (!(I2C1->SR1 & I2C_SR1_TXE)) {}

  I2C1->DR = LIS35_REG_CR1_ACTIVE |
      LIS35_REG_CR1_XEN |
      LIS35_REG_CR1_YEN |
      LIS35_REG_CR1_ZEN;
  while (!(I2C1->SR1 & I2C_SR1_BTF)) {}

  I2C1->CR1 |= I2C_CR1_STOP;
}
