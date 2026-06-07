#ifndef _CAR_PROTOCOL_H_
#define _CAR_PROTOCOL_H_

#include <stdint.h>

void CarProtocol_HandleTextCommand(char *buffer);

uint8_t CarProtocol_HasActiveMotion(void);
uint8_t CarProtocol_IsQuiet(void);
uint8_t CarProtocol_IsTelemetryEnabled(void);

void CarProtocol_FinishActiveMotionOk(const char *extra);
void CarProtocol_FinishActiveMotionErr(const char *code);

#endif
