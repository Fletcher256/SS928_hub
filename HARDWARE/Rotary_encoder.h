#ifndef _ROTARY_ENCODER_H_
#define _ROTARY_ENCODER_H_

//原始的延时,消抖。。。

void RotaryEncoder_Init(void);

uint8_t RotaryEncoder_CheckInput(uint8_t KEYNum);

int GetRotaryEncoder(void);

int GetEncoderSpeed(void);

#endif
