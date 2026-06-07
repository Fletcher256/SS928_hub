#ifndef _COMMAND_PARSER_H_
#define _COMMAND_PARSER_H_

#include <stdint.h>

// Small text helpers used by the serial command protocol.
uint8_t CommandParser_IsDigit(char c);
uint8_t CommandParser_IsSpace(char c);
uint8_t CommandParser_ParseValue(const char *text, float *value, uint8_t scaledHundredths);
uint8_t CommandParser_Tokenize(char *buffer, char *tokens[], uint8_t maxTokens);
uint8_t CommandParser_IsUnsignedInteger(const char *token);
uint8_t CommandParser_ParseSeq(const char *token, uint16_t *seq);
const char *CommandParser_FindKeyValue(char *tokens[], uint8_t count, const char *key);
uint8_t CommandParser_GetFloatArg(char *tokens[], uint8_t count, const char *key, float *value);

#endif
