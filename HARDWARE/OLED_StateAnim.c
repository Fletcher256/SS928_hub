#include "OLED_StateAnim.h"
#include "OLED.h"

#define OLED_LABEL_LINE 4U

typedef struct
{
	uint8_t ready;
	OLED_ActionVisual_t currentAction;
	RS currentState;
} OLED_StateAnim_t;

static OLED_StateAnim_t OledAnim;
static uint8_t OledCanvas[OLED_IMAGE_SIZE];

static void CanvasClear(void)
{
	uint16_t i;

	for(i = 0; i < OLED_IMAGE_SIZE; i++)
	{
		OledCanvas[i] = 0x00;
	}
}

static void CanvasPixel(int16_t x, int16_t y, uint8_t on)
{
	uint16_t index;
	uint8_t mask;

	if(x < 0 || x >= (int16_t)OLED_WIDTH || y < 0 || y >= (int16_t)OLED_HEIGHT)
	{
		return;
	}

	index = (uint16_t)(y / 8) * OLED_WIDTH + (uint16_t)x;
	mask = (uint8_t)(1U << (y & 0x07));
	if(on)
	{
		OledCanvas[index] |= mask;
	}
	else
	{
		OledCanvas[index] &= (uint8_t)~mask;
	}
}

static void CanvasHLine(int16_t x0, int16_t x1, int16_t y)
{
	int16_t x;
	int16_t tmp;

	if(x0 > x1)
	{
		tmp = x0;
		x0 = x1;
		x1 = tmp;
	}
	for(x = x0; x <= x1; x++)
	{
		CanvasPixel(x, y, 1U);
	}
}

static void CanvasVLine(int16_t x, int16_t y0, int16_t y1)
{
	int16_t y;
	int16_t tmp;

	if(y0 > y1)
	{
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}
	for(y = y0; y <= y1; y++)
	{
		CanvasPixel(x, y, 1U);
	}
}

static void CanvasFillRect(int16_t x, int16_t y, int16_t w, int16_t h)
{
	int16_t yy;

	for(yy = y; yy < y + h; yy++)
	{
		CanvasHLine(x, x + w - 1, yy);
	}
}

static void CanvasRect(int16_t x, int16_t y, int16_t w, int16_t h)
{
	CanvasHLine(x, x + w - 1, y);
	CanvasHLine(x, x + w - 1, y + h - 1);
	CanvasVLine(x, y, y + h - 1);
	CanvasVLine(x + w - 1, y, y + h - 1);
}

static int16_t Abs16(int16_t value)
{
	return value < 0 ? (int16_t)-value : value;
}

static void CanvasLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
	int16_t dx = Abs16((int16_t)(x1 - x0));
	int16_t sx = x0 < x1 ? 1 : -1;
	int16_t dy = (int16_t)-Abs16((int16_t)(y1 - y0));
	int16_t sy = y0 < y1 ? 1 : -1;
	int16_t err = (int16_t)(dx + dy);
	int16_t e2;

	while(1)
	{
		CanvasPixel(x0, y0, 1U);
		if(x0 == x1 && y0 == y1)
		{
			break;
		}
		e2 = (int16_t)(2 * err);
		if(e2 >= dy)
		{
			err = (int16_t)(err + dy);
			x0 = (int16_t)(x0 + sx);
		}
		if(e2 <= dx)
		{
			err = (int16_t)(err + dx);
			y0 = (int16_t)(y0 + sy);
		}
	}
}

static void CanvasThickLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
	CanvasLine(x0, y0, x1, y1);
	CanvasLine((int16_t)(x0 + 1), y0, (int16_t)(x1 + 1), y1);
	CanvasLine(x0, (int16_t)(y0 + 1), x1, (int16_t)(y1 + 1));
}

static void DrawArrowUp(void)
{
	int16_t i;

	for(i = 0; i < 18; i++)
	{
		CanvasHLine((int16_t)(64 - i), (int16_t)(64 + i), (int16_t)(8 + i));
	}
	CanvasFillRect(56, 26, 17, 22);
}

static void DrawArrowDown(void)
{
	int16_t i;

	CanvasFillRect(56, 8, 17, 22);
	for(i = 0; i < 18; i++)
	{
		CanvasHLine((int16_t)(46 + i), (int16_t)(82 - i), (int16_t)(30 + i));
	}
}

static void DrawArrowLeft(void)
{
	int16_t i;

	CanvasFillRect(35, 22, 72, 11);
	for(i = 0; i < 18; i++)
	{
		CanvasVLine((int16_t)(35 - i), (int16_t)(14 + i), (int16_t)(40 - i));
	}
}

static void DrawArrowRight(void)
{
	int16_t i;

	CanvasFillRect(21, 22, 72, 11);
	for(i = 0; i < 18; i++)
	{
		CanvasVLine((int16_t)(93 + i), (int16_t)(14 + i), (int16_t)(40 - i));
	}
}

static void DrawLane(void)
{
	CanvasThickLine(30, 8, 18, 48);
	CanvasThickLine(98, 8, 110, 48);
}

static void DrawArcLeft(void)
{
	CanvasThickLine(92, 42, 76, 22);
	CanvasThickLine(76, 22, 48, 16);
	CanvasFillRect(40, 12, 10, 9);
	CanvasThickLine(42, 16, 54, 6);
	CanvasThickLine(42, 16, 54, 28);
}

static void DrawArcRight(void)
{
	CanvasThickLine(36, 42, 52, 22);
	CanvasThickLine(52, 22, 80, 16);
	CanvasFillRect(78, 12, 10, 9);
	CanvasThickLine(86, 16, 74, 6);
	CanvasThickLine(86, 16, 74, 28);
}

static void DrawAutoRoute(void)
{
	CanvasRect(18, 10, 22, 22);
	CanvasRect(88, 10, 22, 22);
	CanvasRect(53, 32, 22, 16);
	CanvasThickLine(40, 21, 53, 40);
	CanvasThickLine(75, 40, 88, 21);
	CanvasFillRect(23, 17, 12, 8);
	CanvasFillRect(93, 17, 12, 8);
}

static void DrawStop(void)
{
	CanvasRect(38, 8, 52, 40);
	CanvasThickLine(48, 16, 80, 40);
	CanvasThickLine(80, 16, 48, 40);
}

static void DrawParking(void)
{
	CanvasRect(42, 8, 44, 40);
	CanvasFillRect(52, 17, 8, 23);
	CanvasFillRect(60, 17, 15, 8);
	CanvasFillRect(75, 20, 5, 9);
	CanvasFillRect(60, 29, 15, 7);
}

static void DrawReady(void)
{
	CanvasRect(28, 8, 72, 40);
	CanvasRect(32, 12, 64, 32);
	CanvasThickLine(45, 29, 58, 39);
	CanvasThickLine(58, 39, 84, 16);
}

static void DrawError(void)
{
	CanvasRect(18, 6, 92, 44);
	CanvasThickLine(34, 14, 94, 42);
	CanvasThickLine(94, 14, 34, 42);
}

static void ShowLabel(const char *label)
{
	uint8_t len = 0;
	uint8_t column;

	while(label[len] != '\0' && len < 16U)
	{
		len++;
	}
	column = (uint8_t)((16U - len) / 2U + 1U);
	OLED_ClearLine(OLED_LABEL_LINE);
	OLED_ShowString(OLED_LABEL_LINE, column, (char *)label);
}

static OLED_ActionVisual_t ActionFromState(RS state)
{
	switch(state)
	{
	case STANDBY: return OLED_ACTION_IDLE;
	case PARKING: return OLED_ACTION_PARKING;
	case HITTED: return OLED_ACTION_ERROR;
	default: return OLED_ACTION_ERROR;
	}
}

static const char *ActionLabel(OLED_ActionVisual_t action)
{
	switch(action)
	{
	case OLED_ACTION_IDLE: return "READY";
	case OLED_ACTION_FORWARD: return "FORWARD";
	case OLED_ACTION_REVERSE: return "REVERSE";
	case OLED_ACTION_STRAIGHT: return "STRAIGHT";
	case OLED_ACTION_TURN_LEFT: return "TURN LEFT";
	case OLED_ACTION_TURN_RIGHT: return "TURN RIGHT";
	case OLED_ACTION_ARC_LEFT: return "ARC LEFT";
	case OLED_ACTION_ARC_RIGHT: return "ARC RIGHT";
	case OLED_ACTION_AUTO: return "AUTO";
	case OLED_ACTION_PARKING: return "PARKING";
	case OLED_ACTION_STOP: return "STOP";
	case OLED_ACTION_ERROR: return "ERROR";
	default: return "UNKNOWN";
	}
}

static void RenderAction(OLED_ActionVisual_t action)
{
	uint8_t showLabel = 1U;

	CanvasClear();

	switch(action)
	{
	case OLED_ACTION_FORWARD:
		DrawArrowUp();
		break;
	case OLED_ACTION_REVERSE:
		DrawArrowDown();
		break;
	case OLED_ACTION_STRAIGHT:
		DrawLane();
		DrawArrowUp();
		break;
	case OLED_ACTION_TURN_LEFT:
		DrawArrowLeft();
		break;
	case OLED_ACTION_TURN_RIGHT:
		DrawArrowRight();
		break;
	case OLED_ACTION_ARC_LEFT:
		DrawArcLeft();
		break;
	case OLED_ACTION_ARC_RIGHT:
		DrawArcRight();
		break;
	case OLED_ACTION_AUTO:
		DrawAutoRoute();
		break;
	case OLED_ACTION_PARKING:
		DrawParking();
		break;
	case OLED_ACTION_STOP:
		DrawStop();
		break;
	case OLED_ACTION_ERROR:
		DrawError();
		break;
	case OLED_ACTION_IDLE:
	default:
		DrawReady();
		break;
	}

	if(showLabel)
	{
		OLED_DrawBitmap128x64(OledCanvas);
		ShowLabel(ActionLabel(action));
	}
}

void OLED_StateAnim_Init(RS state)
{
	OledAnim.ready = 1U;
	OledAnim.currentAction = OLED_ACTION_ERROR;
	OledAnim.currentState = state;
	RenderAction(ActionFromState(state));
	OledAnim.currentAction = ActionFromState(state);
}

void OLED_StateAnim_OnTransition(RS fromState, RS toState, uint32_t nowTicks)
{
	(void)fromState;
	(void)nowTicks;

	if(!OledAnim.ready)
	{
		return;
	}

	OledAnim.currentState = toState;
	OLED_StateAnim_ShowAction(ActionFromState(toState), nowTicks);
}

void OLED_StateAnim_ShowAction(OLED_ActionVisual_t action, uint32_t nowTicks)
{
	(void)nowTicks;

	if(!OledAnim.ready)
	{
		return;
	}

	if(OledAnim.currentAction == action)
	{
		return;
	}

	RenderAction(action);
	OledAnim.currentAction = action;
}

void OLED_StateAnim_Service(uint32_t nowTicks)
{
	(void)nowTicks;
}
