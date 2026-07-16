#include "OLED_StateAnim.h"
#include "OLED.h"

#define OLED_LABEL_LINE                    4U
#define OLED_STATUS_LABEL_CHARS             12U
#define OLED_CONFIDENCE_COLUMN              13U
#define OLED_DETECT_TIMEOUT_MS              1000U
#define OLED_CONFIDENCE_RENDER_INTERVAL_MS  200U

typedef struct
{
	uint8_t ready;
	OLED_ActionVisual_t requestedAction;
	OLED_ActionVisual_t displayedAction;
	RS currentState;
	OLED_DetectState_t detectState;
	uint8_t confidencePct;
	uint8_t confidenceValid;
	uint8_t renderedConfidencePct;
	uint8_t renderedConfidenceValid;
	uint32_t lastDetectTick;
	uint32_t lastConfidenceRenderTick;
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
	CanvasThickLine(30, 8, 18, 46);
	CanvasThickLine(98, 8, 110, 46);
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

static void DrawReverseArcLeft(void)
{
	CanvasThickLine(80, 8, 66, 16);
	CanvasThickLine(66, 16, 52, 28);
	CanvasThickLine(52, 28, 44, 36);
	CanvasFillRect(38, 34, 13, 8);
	CanvasThickLine(40, 40, 30, 29);
	CanvasThickLine(49, 40, 59, 29);
}

static void DrawReverseArcRight(void)
{
	CanvasThickLine(48, 8, 62, 16);
	CanvasThickLine(62, 16, 76, 28);
	CanvasThickLine(76, 28, 84, 36);
	CanvasFillRect(78, 34, 13, 8);
	CanvasThickLine(80, 40, 70, 29);
	CanvasThickLine(89, 40, 99, 29);
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
	CanvasRect(18, 6, 92, 42);
	CanvasThickLine(34, 14, 94, 42);
	CanvasThickLine(94, 14, 34, 42);
}

static void DrawNoDetection(void)
{
	CanvasRect(24, 6, 80, 38);
	CanvasThickLine(34, 13, 94, 37);
	CanvasThickLine(94, 13, 34, 37);
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
	case OLED_ACTION_REVERSE_ARC_LEFT: return "REV ARC LEFT";
	case OLED_ACTION_REVERSE_ARC_RIGHT: return "REV ARC R";
	case OLED_ACTION_AUTO: return "AUTO";
	case OLED_ACTION_PARKING: return "PARKING";
	case OLED_ACTION_STOP: return "STOP";
	case OLED_ACTION_NO_DETECTION: return "NO DETECT";
	case OLED_ACTION_ERROR: return "ERROR";
	default: return "UNKNOWN";
	}
}

static uint8_t IsMotionAction(OLED_ActionVisual_t action)
{
	switch(action)
	{
	case OLED_ACTION_FORWARD:
	case OLED_ACTION_REVERSE:
	case OLED_ACTION_STRAIGHT:
	case OLED_ACTION_TURN_LEFT:
	case OLED_ACTION_TURN_RIGHT:
	case OLED_ACTION_ARC_LEFT:
	case OLED_ACTION_ARC_RIGHT:
	case OLED_ACTION_REVERSE_ARC_LEFT:
	case OLED_ACTION_REVERSE_ARC_RIGHT:
	case OLED_ACTION_AUTO:
		return 1U;
	default:
		return 0U;
	}
}

static OLED_ActionVisual_t ResolveDisplayAction(void)
{
	if(OledAnim.currentState == HITTED)
	{
		return OLED_ACTION_ERROR;
	}

	if(OledAnim.detectState == OLED_DETECT_MISSING)
	{
		return OLED_ACTION_NO_DETECTION;
	}

	if(IsMotionAction(OledAnim.requestedAction))
	{
		return OledAnim.requestedAction;
	}

	if(OledAnim.detectState == OLED_DETECT_PRESENT)
	{
		return OLED_ACTION_IDLE;
	}

	return ActionFromState(OledAnim.currentState);
}

static void FormatConfidence(char text[5])
{
	text[4] = '\0';
	text[3] = '%';

	if(!OledAnim.confidenceValid)
	{
		text[0] = ' ';
		text[1] = '-';
		text[2] = '-';
		return;
	}

	if(OledAnim.confidencePct >= 100U)
	{
		text[0] = '1';
		text[1] = '0';
		text[2] = '0';
		return;
	}

	text[0] = ' ';
	if(OledAnim.confidencePct >= 10U)
	{
		text[1] = (char)('0' + OledAnim.confidencePct / 10U);
	}
	else
	{
		text[1] = ' ';
	}
	text[2] = (char)('0' + OledAnim.confidencePct % 10U);
}

static void RenderConfidenceRegion(uint32_t nowTicks)
{
	char confidenceText[5];

	FormatConfidence(confidenceText);
	OLED_ShowString(OLED_LABEL_LINE, OLED_CONFIDENCE_COLUMN, confidenceText);
	OledAnim.renderedConfidencePct = OledAnim.confidencePct;
	OledAnim.renderedConfidenceValid = OledAnim.confidenceValid;
	OledAnim.lastConfidenceRenderTick = nowTicks;
}

static void ShowStatusLine(OLED_ActionVisual_t action, uint32_t nowTicks)
{
	const char *label = ActionLabel(action);
	char labelText[OLED_STATUS_LABEL_CHARS + 1U];
	uint8_t labelLength = 0U;
	uint8_t labelStart;
	uint8_t i;

	for(i = 0U; i < OLED_STATUS_LABEL_CHARS; i++)
	{
		labelText[i] = ' ';
	}
	labelText[OLED_STATUS_LABEL_CHARS] = '\0';

	while(label[labelLength] != '\0' && labelLength < OLED_STATUS_LABEL_CHARS)
	{
		labelLength++;
	}
	labelStart = (uint8_t)((OLED_STATUS_LABEL_CHARS - labelLength) / 2U);
	for(i = 0U; i < labelLength; i++)
	{
		labelText[labelStart + i] = label[i];
	}

	OLED_ShowString(OLED_LABEL_LINE, 1U, labelText);
	RenderConfidenceRegion(nowTicks);
}

static void RenderAction(OLED_ActionVisual_t action, uint32_t nowTicks)
{

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
	case OLED_ACTION_REVERSE_ARC_LEFT:
		DrawReverseArcLeft();
		break;
	case OLED_ACTION_REVERSE_ARC_RIGHT:
		DrawReverseArcRight();
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
	case OLED_ACTION_NO_DETECTION:
		DrawNoDetection();
		break;
	case OLED_ACTION_ERROR:
		DrawError();
		break;
	case OLED_ACTION_IDLE:
	default:
		DrawReady();
		break;
	}

	OLED_DrawBitmap128x64(OledCanvas);
	ShowStatusLine(action, nowTicks);
}

static uint8_t ConfidenceRenderPending(void)
{
	if(OledAnim.renderedConfidenceValid != OledAnim.confidenceValid)
	{
		return 1U;
	}

	return OledAnim.confidenceValid &&
	       OledAnim.renderedConfidencePct != OledAnim.confidencePct;
}

static void UpdateDisplay(uint32_t nowTicks, uint8_t forceFullRender)
{
	OLED_ActionVisual_t resolvedAction;

	if(!OledAnim.ready)
	{
		return;
	}

	resolvedAction = ResolveDisplayAction();
	if(forceFullRender || OledAnim.displayedAction != resolvedAction)
	{
		RenderAction(resolvedAction, nowTicks);
		OledAnim.displayedAction = resolvedAction;
		return;
	}

	if(ConfidenceRenderPending() &&
	   (uint32_t)(nowTicks - OledAnim.lastConfidenceRenderTick) >=
	       OLED_CONFIDENCE_RENDER_INTERVAL_MS)
	{
		RenderConfidenceRegion(nowTicks);
	}
}

void OLED_StateAnim_Init(RS state)
{
	OledAnim.ready = 0U;
	OledAnim.requestedAction = ActionFromState(state);
	OledAnim.displayedAction = OLED_ACTION_ERROR;
	OledAnim.currentState = state;
	OledAnim.detectState = OLED_DETECT_UNKNOWN;
	OledAnim.confidencePct = 0U;
	OledAnim.confidenceValid = 0U;
	OledAnim.renderedConfidencePct = 0U;
	OledAnim.renderedConfidenceValid = 0U;
	OledAnim.lastDetectTick = 0U;
	OledAnim.lastConfidenceRenderTick = 0U;
	OledAnim.ready = 1U;
	UpdateDisplay(0U, 1U);
}

void OLED_StateAnim_OnTransition(RS fromState, RS toState, uint32_t nowTicks)
{
	(void)fromState;

	if(!OledAnim.ready)
	{
		return;
	}

	OledAnim.currentState = toState;
	OLED_StateAnim_ShowAction(ActionFromState(toState), nowTicks);
}

void OLED_StateAnim_ShowAction(OLED_ActionVisual_t action, uint32_t nowTicks)
{
	if(!OledAnim.ready)
	{
		return;
	}

	OledAnim.requestedAction = action;
	UpdateDisplay(nowTicks, 0U);
}

void OLED_StateAnim_SetDetection(uint8_t detected, uint8_t confidencePct, uint32_t nowTicks)
{
	OLED_DetectState_t previousDetectState = OledAnim.detectState;

	OledAnim.lastDetectTick = nowTicks;
	if(detected)
	{
		OledAnim.detectState = OLED_DETECT_PRESENT;
		OledAnim.confidencePct = confidencePct > 100U ? 100U : confidencePct;
		OledAnim.confidenceValid = 1U;
	}
	else
	{
		OledAnim.detectState = OLED_DETECT_MISSING;
		OledAnim.confidencePct = 0U;
		OledAnim.confidenceValid = 0U;
	}

	UpdateDisplay(nowTicks, previousDetectState != OledAnim.detectState);
}

void OLED_StateAnim_Service(uint32_t nowTicks)
{
	if(!OledAnim.ready)
	{
		return;
	}

	if(OledAnim.detectState == OLED_DETECT_PRESENT &&
	   (uint32_t)(nowTicks - OledAnim.lastDetectTick) >= OLED_DETECT_TIMEOUT_MS)
	{
		OledAnim.detectState = OLED_DETECT_MISSING;
		OledAnim.confidencePct = 0U;
		OledAnim.confidenceValid = 0U;
		UpdateDisplay(nowTicks, 1U);
		return;
	}

	UpdateDisplay(nowTicks, 0U);
}
