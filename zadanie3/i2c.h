#ifndef ZADANIE3__I2C_H_
#define ZADANIE3__I2C_H_

void configurate_i2c();
void enqueue_command(uint8_t slave_addr, uint8_t *to_send, int send_size,
                   uint8_t *to_receive, int recv_size);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

#endif //ZADANIE3__I2C_H_
