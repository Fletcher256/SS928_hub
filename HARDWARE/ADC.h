#ifndef _ADC_H_
#define _ADC_H_

#define SAMPLE_DIV 20

void ADC1_Init(void);

uint16_t GetADC1Value(void);

uint16_t GetADC1ChxValue(uint8_t ADC_Channel_x);

void ADC1SCAN_Init(void);

void GetADC1SCANValue(void);

uint16_t * GetADBuffer(void);

#endif
