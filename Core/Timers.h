#ifndef _TIMERS_H_
#define _TIMERS_H_

#define EXCOUNT(T,LIM) (((T = (T+1)%LIM) == 0) ? 1 : 0)   

//TODO
void Timer1_Init(void);

void Timer2_Init(void);

void Timer4_Init(void);

void ETR1_Init(void);

uint16_t GetCounter2(void);

uint16_t GetCounter3(void);

void SysTick_Init(void);

#endif
