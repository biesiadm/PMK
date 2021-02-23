#ifndef ZADANIE3__I2C_H_
#define ZADANIE3__I2C_H_

void configurate_i2c();
void update_leds_by_acc();
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

#endif //ZADANIE3__I2C_H_
