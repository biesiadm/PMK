#ifndef ZADANIE3__TIMER_H_
#define ZADANIE3__TIMER_H_

void config_timer();
void setRedLEDPower(unsigned power_percent);
void setGreenLEDPower(unsigned power_percent);
void setBlueLEDPower(unsigned power_percent);
void check_updates();
void TIM3_IRQHandler(void);

#endif //ZADANIE3__TIMER_H_
