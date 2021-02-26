#include <stdbool.h>
#include <gpio.h>
#include "i2c_command.h"

#define MAX_COMM_SIZE 128

typedef struct i2c_command {
  uint8_t to_send[MAX_COMM_SIZE];
  uint8_t *to_receive;
  uint8_t slave_addr;
  int send_size;
  int recv_size;
  int already_sent;
  int already_recv;
  bool initialised;
  bool after_send;
  bool finished;
} i2c_command_t;

static i2c_command_t command_buffer[MAX_COMM_SIZE] = {0};
static int cyclic_buffer_start = 0;
static int cyclic_buffer_end = 0;

static i2c_command_t *get_next_free_command();

i2c_command_t *get_curr_command() {
  i2c_command_t *curr_command = &command_buffer[cyclic_buffer_start];

  if (curr_command->finished) {
    cyclic_buffer_start++;
  }

  if (cyclic_buffer_start == MAX_COMM_SIZE) {
    cyclic_buffer_start = 0;
  }

  curr_command = &command_buffer[cyclic_buffer_start];

  if (!curr_command->initialised) {
    curr_command = 0;
  }

  return curr_command;
}

void add_to_command_buffer(uint8_t slave_addr, uint8_t *to_send,
                           int send_size, uint8_t *to_receive,
                           int recv_size) {
  i2c_command_t *c = get_next_free_command();
  if (!c) {
    return;
  }

  c->slave_addr = slave_addr;

  c->send_size = send_size;
  for (int i = 0; i < send_size; i++) {
    c->to_send[i] = to_send[i];
  }

  c->to_receive = to_receive;
  c->recv_size = recv_size;

  c->already_sent = 0;
  c->already_recv = 0;
  c->initialised = true;
  c->after_send = false;
  c->finished = false;
}

//uint8_t *get_to_send(i2c_command_t *command) {
//  if (!command) { return 0; }
//  return command->to_send;
//}

uint8_t get_next_to_send(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->to_send[command->already_sent];
}

uint8_t *get_to_receive(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->to_receive;
}

uint8_t get_slave_addr(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->slave_addr;
}

int get_send_size(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->send_size;
}

int get_recv_size(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->recv_size;
}

int get_already_sent(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->already_sent;
}

int get_already_recv(i2c_command_t *command) {
  if (!command) { return 0; }
  return command->already_recv;
}

bool check_if_initialised(i2c_command_t *command) {
  if (!command) { return false; }
  return command->initialised;
}

bool check_if_after_send(i2c_command_t *command) {
  if (!command) { return false; }
  return command->after_send;
}

bool check_if_finished(i2c_command_t *command) {
  if (!command) { return false; }
  return command->finished;
}

void set_after_send(i2c_command_t *command) {
  if (!command) { return; }
  command->after_send = true;
}

void inc_already_sent(i2c_command_t *command) {
  if (!command) { return; }
  command->already_sent++;
}

void set_next_to_receive(i2c_command_t *command, uint8_t value) {
  if (!command) { return; }
  command->to_receive[command->already_recv] = value;
  command->already_recv++;
}

void inc_already_recv(i2c_command_t *command) {
  if (!command) { return; }
  command->already_recv++;
}

void set_finished(i2c_command_t *command) {
  if (!command) { return; }
  command->finished = true;
}

void set_last_to_receive(i2c_command_t *command, uint8_t value) {
  if (!command) { return; }
  command->to_receive[command->recv_size - 1] = value;
}

i2c_command_t *get_next_free_command() {
  i2c_command_t *next_free = &command_buffer[cyclic_buffer_end];
  if (!next_free->initialised || next_free->finished) {
    cyclic_buffer_end++;
  } else {
    return 0;
  }

  if (cyclic_buffer_end == MAX_COMM_SIZE) {
    cyclic_buffer_end = 0;
  }

  return next_free;
}
