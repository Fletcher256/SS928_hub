#ifndef _LED_H_
#define _LED_H_

typedef enum
{
	LED_STATE_YELLOW = 0,
	LED_STATE_RED,
	LED_STATE_GREEN
} LED_STATE;

extern LED_STATE CurrentLedState;

void LED_Init();

void LED_SetState(LED_STATE state);

#endif
