#ifndef _PWMO_H_
#define _PWMO_H_

void SetTIM2CH1ARR(uint16_t t);

void ServoPWM_Init(void);

void SetTIM2CH2ARR(uint16_t t);

void SetServoRotation(float Angle);

uint8_t ServoPWM_IsHealthy(void);

float ServoPWM_GetLastAngle(void);

uint16_t ServoPWM_GetPulseUs(void);

uint16_t ServoPWM_GetPsc(void);

uint16_t ServoPWM_GetArr(void);

uint16_t ServoPWM_GetCcr2(void);

uint16_t ServoPWM_GetCcer(void);

void SetTIM2CH3ARR(uint16_t t);

void SetTIM2CH4ARR(uint16_t t);

#endif
