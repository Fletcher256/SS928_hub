#include "CommandParser.h"

#include "string.h"

uint8_t CommandParser_IsDigit(char c)
{
	return (c >= '0' && c <= '9');
}

uint8_t CommandParser_IsSpace(char c)
{
	return (c == ' ' || c == '\t');
}

static const char *SkipValuePrefix(const char *s)
{
	while(*s == ' ' || *s == '=' || *s == ':')
	{
		s++;
	}
	return s;
}

uint8_t CommandParser_ParseValue(const char *text, float *value, uint8_t scaledHundredths)
{
	float integer = 0.0f;
	float fraction = 0.0f;
	float scale = 1.0f;
	int8_t sign = 1;
	uint8_t hasDigit = 0;
	uint8_t hasDot = 0;

	text = SkipValuePrefix(text);

	if(*text == '-')
	{
		sign = -1;
		text++;
	}
	else if(*text == '+')
	{
		text++;
	}

	while(CommandParser_IsDigit(*text))
	{
		integer = integer * 10.0f + (float)(*text - '0');
		hasDigit = 1;
		text++;
	}

	if(*text == '.')
	{
		hasDot = 1;
		text++;
		while(CommandParser_IsDigit(*text))
		{
			scale *= 10.0f;
			fraction += (float)(*text - '0') / scale;
			hasDigit = 1;
			text++;
		}
	}

	text = SkipValuePrefix(text);
	if(!hasDigit || *text != '\0')
	{
		return 0;
	}

	*value = (integer + fraction) * (float)sign;
	if(scaledHundredths && !hasDot)
	{
		*value *= 0.01f;
	}
	return 1;
}

uint8_t CommandParser_Tokenize(char *buffer, char *tokens[], uint8_t maxTokens)
{
	uint8_t count = 0;
	char *p = buffer;

	while(*p != '\0' && count < maxTokens)
	{
		while(CommandParser_IsSpace(*p))
		{
			p++;
		}
		if(*p == '\0')
		{
			break;
		}
		tokens[count++] = p;
		while(*p != '\0' && !CommandParser_IsSpace(*p))
		{
			p++;
		}
		if(*p != '\0')
		{
			*p = '\0';
			p++;
		}
	}
	return count;
}

uint8_t CommandParser_IsUnsignedInteger(const char *token)
{
	if(token == 0 || *token == '\0')
	{
		return 0;
	}
	while(*token != '\0')
	{
		if(!CommandParser_IsDigit(*token))
		{
			return 0;
		}
		token++;
	}
	return 1;
}

uint8_t CommandParser_ParseSeq(const char *token, uint16_t *seq)
{
	uint32_t value = 0;

	if(!CommandParser_IsUnsignedInteger(token))
	{
		return 0;
	}
	while(*token != '\0')
	{
		value = value * 10U + (uint32_t)(*token - '0');
		if(value > 65535U)
		{
			return 0;
		}
		token++;
	}
	*seq = (uint16_t)value;
	return 1;
}

const char *CommandParser_FindKeyValue(char *tokens[], uint8_t count, const char *key)
{
	uint8_t i;
	uint16_t keyLen = (uint16_t)strlen(key);

	for(i = 0; i < count; i++)
	{
		if(strncmp(tokens[i], key, keyLen) == 0 && tokens[i][keyLen] == '=')
		{
			return &tokens[i][keyLen + 1U];
		}
	}
	return 0;
}

uint8_t CommandParser_GetFloatArg(char *tokens[], uint8_t count, const char *key, float *value)
{
	const char *text = CommandParser_FindKeyValue(tokens, count, key);

	if(text == 0)
	{
		return 0;
	}
	return CommandParser_ParseValue(text, value, 0);
}
