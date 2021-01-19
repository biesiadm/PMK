#include "utils.h"

void my_sleep(int milis) {
#define STEPS_PER_MILI 4000
  for (int ms = 0; ms < milis; ms++) {
    for (int i = 0; i < STEPS_PER_MILI; i++) {
      asm("nop");
    }
  }
}