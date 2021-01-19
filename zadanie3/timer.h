#ifndef ZADANIE3__TIMER_H_
#define ZADANIE3__TIMER_H_

void configurate_timer();
void setRedLEDPower(unsigned power_percent);
void setGreenLEDPower(unsigned power_percent);
void setBlueLEDPower(unsigned power_percent);
void TIM3_IRQHandler(void);


#endif //ZADANIE3__TIMER_H_
