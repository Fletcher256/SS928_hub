#ifndef _LED_H_
#define _LED_H_

#include <stdint.h>

typedef enum
{
	LED_STATE_YELLOW = 0,
	LED_STATE_RED,
	LED_STATE_GREEN
} LED_STATE;

extern LED_STATE CurrentLedState;

void LED_Init();

void LED_SetState(LED_STATE state);

/* Blink the currently selected state.  LED_SetState may still change the
 * base colour while blinking (green standby -> yellow parking). */
void LED_SetBlink(uint8_t enabled);
uint8_t LED_IsBlinking(void);
void LED_BlinkService1ms(void);

#endif
