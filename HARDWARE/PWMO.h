#ifndef _PWMO_H_
#define _PWMO_H_

void SetTIM2CH1ARR(uint16_t t);

void ServoPWM_Init(void);

void SetTIM2CH2ARR(uint16_t t);

void SetServoRotation(float Angle);

void SetTIM2CH3ARR(uint16_t t);

void SetTIM2CH4ARR(uint16_t t);

#endif
