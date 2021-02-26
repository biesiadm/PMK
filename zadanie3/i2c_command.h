#ifndef ZADANIE3__I2C_COMMAND_H_
#define ZADANIE3__I2C_COMMAND_H_

typedef struct i2c_command i2c_command_t;

i2c_command_t *get_curr_command();
void add_to_command_buffer(uint8_t slave_addr, uint8_t *to_send, int send_size,
                           uint8_t *to_receive, int recv_size);

//uint8_t *get_to_send(i2c_command_t *command);
uint8_t get_next_to_send(i2c_command_t *command);
uint8_t *get_to_receive(i2c_command_t *command);
uint8_t get_slave_addr(i2c_command_t *command);
int get_send_size(i2c_command_t *command);
int get_recv_size(i2c_command_t *command);
int get_already_sent(i2c_command_t *command);
int get_already_recv(i2c_command_t *command);
bool check_if_initialised(i2c_command_t *command);
bool check_if_after_send(i2c_command_t *command);
bool check_if_finished(i2c_command_t *command);

void set_after_send(i2c_command_t *command);
void inc_already_sent(i2c_command_t *command);
void set_next_to_receive(i2c_command_t *command, uint8_t value);
void inc_already_recv(i2c_command_t *command);
void set_finished(i2c_command_t *command);
void set_last_to_receive(i2c_command_t *command, uint8_t value);

#endif //ZADANIE3__I2C_COMMAND_H_
