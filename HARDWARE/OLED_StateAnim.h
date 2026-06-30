#ifndef _OLED_STATE_ANIM_H_
#define _OLED_STATE_ANIM_H_

#include "CarControl.h"
#include <stdint.h>

typedef enum OLED_ActionVisual
{
	OLED_ACTION_IDLE = 0,
	OLED_ACTION_FORWARD,
	OLED_ACTION_REVERSE,
	OLED_ACTION_STRAIGHT,
	OLED_ACTION_TURN_LEFT,
	OLED_ACTION_TURN_RIGHT,
	OLED_ACTION_ARC_LEFT,
	OLED_ACTION_ARC_RIGHT,
	OLED_ACTION_AUTO,
	OLED_ACTION_PARKING,
	OLED_ACTION_STOP,
	OLED_ACTION_ERROR
} OLED_ActionVisual_t;

void OLED_StateAnim_Init(RS state);
void OLED_StateAnim_OnTransition(RS fromState, RS toState, uint32_t nowTicks);
void OLED_StateAnim_ShowAction(OLED_ActionVisual_t action, uint32_t nowTicks);
void OLED_StateAnim_Service(uint32_t nowTicks);

#endif
