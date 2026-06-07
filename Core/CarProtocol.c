#include "stm32f10x.h"
#include "CarProtocol.h"
#include "CarControl.h"
#include "CommandParser.h"
#include "Motors.h"
#include "PWMO.h"
#include "USART.h"
#include "LED.h"

#include "string.h"

static uint8_t TelemetryEnabled = 0;
static uint8_t ProtocolQuiet = 0;
static uint8_t ActiveMotionValid = 0;
static uint16_t ActiveMotionSeq = 0;
static char ActiveMotionName[8] = "";

static void CopySmallText(char *dest, const char *src, uint8_t destSize)
{
	uint8_t i = 0;

	if(destSize == 0)
	{
		return;
	}

	while(i < (uint8_t)(destSize - 1) && src[i] != '\0')
	{
		dest[i] = src[i];
		i++;
	}
	dest[i] = '\0';
}

static void ReplyAck(uint16_t seq, const char *cmd)
{
	if(seq > 0)
	{
		USART3_printf("ACK %u %s\r\n", seq, cmd);
	}
	else
	{
		USART3_printf("ACK %s\r\n", cmd);
	}
}

static void ReplyDone(uint16_t seq, const char *cmd, const char *extra)
{
	if(extra == 0 || extra[0] == '\0')
	{
		if(seq > 0)
		{
			USART3_printf("DONE %u %s\r\n", seq, cmd);
		}
		else
		{
			USART3_printf("DONE %s\r\n", cmd);
		}
	}
	else if(seq > 0)
	{
		USART3_printf("DONE %u %s %s\r\n", seq, cmd, extra);
	}
	else
	{
		USART3_printf("DONE %s %s\r\n", cmd, extra);
	}
}

static void ReplyErr(uint16_t seq, const char *code)
{
	if(seq > 0)
	{
		USART3_printf("ERR %u CODE=%s\r\n", seq, code);
	}
	else
	{
		USART3_printf("ERR CODE=%s\r\n", code);
	}
}

uint8_t CarProtocol_HasActiveMotion(void)
{
	return ActiveMotionValid;
}

uint8_t CarProtocol_IsQuiet(void)
{
	return ProtocolQuiet;
}

uint8_t CarProtocol_IsTelemetryEnabled(void)
{
	return TelemetryEnabled;
}

static void TrackActiveMotion(uint16_t seq, const char *cmd)
{
	ActiveMotionValid = 1;
	ActiveMotionSeq = seq;
	CopySmallText(ActiveMotionName, cmd, sizeof(ActiveMotionName));
}

void CarProtocol_FinishActiveMotionOk(const char *extra)
{
	if(ActiveMotionValid)
	{
		ReplyDone(ActiveMotionSeq, ActiveMotionName, extra);
		ActiveMotionValid = 0;
		ActiveMotionSeq = 0;
		ActiveMotionName[0] = '\0';
	}
}

void CarProtocol_FinishActiveMotionErr(const char *code)
{
	if(ActiveMotionValid)
	{
		ReplyErr(ActiveMotionSeq, code);
		ActiveMotionValid = 0;
		ActiveMotionSeq = 0;
		ActiveMotionName[0] = '\0';
	}
}

static uint8_t GetSpeedArg(char *tokens[], uint8_t count, uint8_t *speed)
{
	float value = 0.0f;

	if(!CommandParser_GetFloatArg(tokens, count, "V", &value))
	{
		*speed = (AutoSpeedLevel > SpeedLimitLevel) ? SpeedLimitLevel : AutoSpeedLevel;
		return 1;
	}
	if(value < 0.0f || value > (float)SpeedLimitLevel)
	{
		return 0;
	}
	*speed = (uint8_t)value;
	return 1;
}

static void PrintStatusV2(uint16_t seq)
{
	Odometry_t snapshot;
	uint8_t dropped;

	Odometry_GetSnapshot(&snapshot);
	dropped = USART3_GetDroppedTextCount();
	if(seq > 0)
	{
		USART3_printf("STAT %u MODE=%s RUN=%s DIR=%d SPD=%d ANG=%.1f YAW=%.1f X=%.1f Y=%.1f D=%.1f VEL=%.1f DROP=%u\r\n",
		              seq, ControlModeName(), RunStateName(), is_up, SpeedRank, Angle,
		              GetReportedYaw(), snapshot.x, snapshot.y, snapshot.distance, aveSpeed, dropped);
	}
	else
	{
		USART3_printf("STAT MODE=%s RUN=%s DIR=%d SPD=%d ANG=%.1f YAW=%.1f X=%.1f Y=%.1f D=%.1f VEL=%.1f DROP=%u\r\n",
		              ControlModeName(), RunStateName(), is_up, SpeedRank, Angle,
		              GetReportedYaw(), snapshot.x, snapshot.y, snapshot.distance, aveSpeed, dropped);
	}
}

static void PrintPwmStatusV2(uint16_t seq)
{
	if(seq > 0)
	{
		USART3_printf("PWM %u HEALTH=%u ANG=%.1f PULSE=%u PSC=%u ARR=%u CCR2=%u CCER=0x%04x RECOV=%u\r\n",
		              seq, ServoPWM_IsHealthy(), ServoPWM_GetLastAngle(), ServoPWM_GetPulseUs(),
		              ServoPWM_GetPsc(), ServoPWM_GetArr(), ServoPWM_GetCcr2(), ServoPWM_GetCcer(),
		              ServoPWM_GetRecoverCount());
	}
	else
	{
		USART3_printf("PWM HEALTH=%u ANG=%.1f PULSE=%u PSC=%u ARR=%u CCR2=%u CCER=0x%04x RECOV=%u\r\n",
		              ServoPWM_IsHealthy(), ServoPWM_GetLastAngle(), ServoPWM_GetPulseUs(),
		              ServoPWM_GetPsc(), ServoPWM_GetArr(), ServoPWM_GetCcr2(), ServoPWM_GetCcer(),
		              ServoPWM_GetRecoverCount());
	}
}

static void PrintParamV2(uint16_t seq, const char *param)
{
	if(strcmp(param, "HEADING") == 0)
	{
		if(seq > 0)
		{
			USART3_printf("PARAM %u HEADING KP=%.4f KI=%.4f KD=%.4f MAXI=%.2f MAXOUT=%.2f DEAD=%.2f CROSS=%.2f\r\n",
			              seq, headingPID.Kp, headingPID.Ki, headingPID.Kd, headingPID.MaxI,
			              headingPID.MaxOut, headingPID.Deadband, headingPID.CrossTrackKp);
		}
		else
		{
			USART3_printf("PARAM HEADING KP=%.4f KI=%.4f KD=%.4f MAXI=%.2f MAXOUT=%.2f DEAD=%.2f CROSS=%.2f\r\n",
			              headingPID.Kp, headingPID.Ki, headingPID.Kd, headingPID.MaxI,
			              headingPID.MaxOut, headingPID.Deadband, headingPID.CrossTrackKp);
		}
	}
	else if(strcmp(param, "MOTOR") == 0)
	{
		if(seq > 0)
		{
			USART3_printf("PARAM %u MOTOR RKP=%.4f RKI=%.4f RKD=%.4f LKP=%.4f LKI=%.4f LKD=%.4f\r\n",
			              seq, rSpeed_PID.Kp, rSpeed_PID.Ki, rSpeed_PID.Kd,
			              lSpeed_PID.Kp, lSpeed_PID.Ki, lSpeed_PID.Kd);
		}
		else
		{
			USART3_printf("PARAM MOTOR RKP=%.4f RKI=%.4f RKD=%.4f LKP=%.4f LKI=%.4f LKD=%.4f\r\n",
			              rSpeed_PID.Kp, rSpeed_PID.Ki, rSpeed_PID.Kd,
			              lSpeed_PID.Kp, lSpeed_PID.Ki, lSpeed_PID.Kd);
		}
	}
	else if(strcmp(param, "LIMIT") == 0 || strcmp(param, "SERVO") == 0)
	{
		if(seq > 0)
		{
			USART3_printf("PARAM %u LIMIT STE_MIN=%.1f STE_MAX=%.1f SPEED_MAX=%u\r\n",
			              seq, SteerMinAngle, SteerMaxAngle, SpeedLimitLevel);
		}
		else
		{
			USART3_printf("PARAM LIMIT STE_MIN=%.1f STE_MAX=%.1f SPEED_MAX=%u\r\n",
			              SteerMinAngle, SteerMaxAngle, SpeedLimitLevel);
		}
	}
	else
	{
		ReplyErr(seq, "BAD_PARAM");
	}
}

static uint8_t ApplyHeadingParams(char *tokens[], uint8_t count)
{
	float value;
	uint8_t changed = 0;

	if(CommandParser_GetFloatArg(tokens, count, "KP", &value)) { headingPID.Kp = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "KI", &value)) { headingPID.Ki = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "KD", &value)) { headingPID.Kd = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "MAXI", &value)) { headingPID.MaxI = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "MAXOUT", &value)) { headingPID.MaxOut = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "DEAD", &value)) { headingPID.Deadband = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "D_ALPHA", &value)) { headingPID.D_Alpha = ClampFloat(value, 0.0f, 1.0f); changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "SMOOTH", &value)) { headingPID.SmoothAlpha = ClampFloat(value, 0.0f, 1.0f); changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "CROSS", &value)) { headingPID.CrossTrackKp = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "CROSS_EN", &value)) { headingPID.CrossTrackEnable = (value != 0.0f); changed = 1; }
	if(changed)
	{
		HeadingPID_Reset(&headingPID);
	}
	return changed;
}

static uint8_t ApplyMotorParams(char *tokens[], uint8_t count)
{
	float value;
	uint8_t changed = 0;

	if(CommandParser_GetFloatArg(tokens, count, "KP", &value))
	{
		rSpeed_PID.Kp = value;
		lSpeed_PID.Kp = value;
		changed = 1;
	}
	if(CommandParser_GetFloatArg(tokens, count, "KI", &value))
	{
		rSpeed_PID.Ki = value;
		lSpeed_PID.Ki = value;
		changed = 1;
	}
	if(CommandParser_GetFloatArg(tokens, count, "KD", &value))
	{
		rSpeed_PID.Kd = value;
		lSpeed_PID.Kd = value;
		changed = 1;
	}
	if(CommandParser_GetFloatArg(tokens, count, "RKP", &value)) { rSpeed_PID.Kp = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "RKI", &value)) { rSpeed_PID.Ki = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "RKD", &value)) { rSpeed_PID.Kd = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "LKP", &value)) { lSpeed_PID.Kp = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "LKI", &value)) { lSpeed_PID.Ki = value; changed = 1; }
	if(CommandParser_GetFloatArg(tokens, count, "LKD", &value)) { lSpeed_PID.Kd = value; changed = 1; }
	if(changed)
	{
		InitAll();
	}
	return changed;
}

static uint8_t ApplyLimitParams(char *tokens[], uint8_t count)
{
	float value;
	float newMin = SteerMinAngle;
	float newMax = SteerMaxAngle;
	uint8_t changed = 0;

	if(CommandParser_GetFloatArg(tokens, count, "STE_MIN", &value))
	{
		newMin = ClampFloat(value, 0.0f, 180.0f);
		changed = 1;
	}
	if(CommandParser_GetFloatArg(tokens, count, "STE_MAX", &value))
	{
		newMax = ClampFloat(value, 0.0f, 180.0f);
		changed = 1;
	}
	if(newMin >= newMax)
	{
		return 0;
	}
	SteerMinAngle = newMin;
	SteerMaxAngle = newMax;
	if(CommandParser_GetFloatArg(tokens, count, "SPEED_MAX", &value))
	{
		if(value < 0.0f || value > (float)AUTO_MAX_SPEED)
		{
			return 0;
		}
		SpeedLimitLevel = (uint8_t)value;
		if(AutoSpeedLevel > SpeedLimitLevel)
		{
			AutoSpeedLevel = SpeedLimitLevel;
		}
		if(ABS(SpeedRank) > (int16_t)(SpeedLimitLevel * SPEEDSTEP))
		{
			SpeedRank = ABSTRACT(SpeedRank) * (int16_t)(SpeedLimitLevel * SPEEDSTEP);
		}
		changed = 1;
	}
	SetSteeringAngle(Angle);
	return changed;
}

static void RestoreDefaultRuntimeConfig(void)
{
	headingPID.Kp = 2.5f;
	headingPID.Ki = 0.01f;
	headingPID.Kd = 0.18f;
	headingPID.MaxI = 5.0f;
	headingPID.MaxOut = 8.0f;
	headingPID.Deadband = 2.0f;
	headingPID.D_Alpha = 0.7f;
	headingPID.SmoothAlpha = 0.4f;
	headingPID.CrossTrackKp = 2.0f;
	rSpeed_PID.Kp = KP;
	rSpeed_PID.Ki = KI;
	rSpeed_PID.Kd = KD;
	lSpeed_PID.Kp = KP;
	lSpeed_PID.Ki = KI;
	lSpeed_PID.Kd = KD;
	SpeedLimitLevel = AUTO_MAX_SPEED;
	AutoSpeedLevel = AUTO_DEFAULT_SPEED;
	SteerMinAngle = 0.0f;
	SteerMaxAngle = 180.0f;
	InitAll();
	HeadingPID_Reset(&headingPID);
	SetSteeringAngle(90.0f);
}

static uint8_t IsV2CommandName(const char *token)
{
	return (strcmp(token, "PING") == 0 ||
	        strcmp(token, "VER") == 0 ||
	        strcmp(token, "STAT") == 0 ||
	        strcmp(token, "PWM_STAT") == 0 ||
	        strcmp(token, "TEL") == 0 ||
	        strcmp(token, "STOP") == 0 ||
	        strcmp(token, "CANCEL") == 0 ||
	        strcmp(token, "MODE") == 0 ||
	        strcmp(token, "SERVO") == 0 ||
	        strcmp(token, "MOVE") == 0 ||
	        strcmp(token, "TURN") == 0 ||
	        strcmp(token, "ARC") == 0 ||
	        strcmp(token, "AUTO") == 0 ||
	        strcmp(token, "ZERO_ODOM") == 0 ||
	        strcmp(token, "ZERO_YAW") == 0 ||
	        strcmp(token, "ZERO_ALL") == 0 ||
	        strcmp(token, "GET") == 0 ||
	        strcmp(token, "SET") == 0 ||
	        strcmp(token, "SAVE_CFG") == 0 ||
	        strcmp(token, "LOAD_CFG") == 0 ||
	        strcmp(token, "DEFAULT_CFG") == 0);
}

static uint8_t IsV2Candidate(const char *buffer)
{
	char token[16];
	uint8_t i = 0;

	while(CommandParser_IsSpace(*buffer))
	{
		buffer++;
	}
	if(CommandParser_IsDigit(*buffer))
	{
		return 1;
	}
	while(buffer[i] != '\0' && !CommandParser_IsSpace(buffer[i]) && i < (uint8_t)(sizeof(token) - 1U))
	{
		token[i] = buffer[i];
		i++;
	}
	token[i] = '\0';
	return IsV2CommandName(token);
}

static void PrintFirmwareV2(uint16_t seq)
{
	if(seq > 0)
	{
		USART3_printf("VER %u FW=%s BAUD=9600 PROTO=2\r\n", seq, FIRMWARE_VERSION);
	}
	else
	{
		USART3_printf("VER FW=%s BAUD=9600 PROTO=2\r\n", FIRMWARE_VERSION);
	}
}

static void BeginProtocolMotion(uint8_t speed)
{
	AutoSpeedLevel = speed;
	SpeedRank = 0;
	ProtocolQuiet = 1;
}

static void FinishProtocolMotion(uint8_t ok, uint16_t seq, const char *cmd, const char *errorCode)
{
	ProtocolQuiet = 0;
	if(ok)
	{
		TrackActiveMotion(seq, cmd);
		ReplyAck(seq, cmd);
	}
	else
	{
		ReplyErr(seq, errorCode);
	}
}

static void HandleV2Telemetry(char *tokens[], uint8_t count, uint8_t cmdIndex, uint16_t seq)
{
	if(count <= (uint8_t)(cmdIndex + 1U))
	{
		ReplyErr(seq, "BAD_ARG");
	}
	else if(strcmp(tokens[cmdIndex + 1U], "ON") == 0 || strcmp(tokens[cmdIndex + 1U], "1") == 0)
	{
		TelemetryEnabled = 1;
		ReplyDone(seq, "TEL", "ON");
	}
	else if(strcmp(tokens[cmdIndex + 1U], "OFF") == 0 || strcmp(tokens[cmdIndex + 1U], "0") == 0)
	{
		TelemetryEnabled = 0;
		ReplyDone(seq, "TEL", "OFF");
	}
	else
	{
		ReplyErr(seq, "BAD_ARG");
	}
}

static void HandleV2Mode(char *tokens[], uint8_t count, uint16_t seq)
{
	const char *mode = CommandParser_FindKeyValue(tokens, count, "M");

	if(mode == 0)
	{
		ReplyErr(seq, "BAD_ARG");
	}
	else if(strcmp(mode, "MANUAL") == 0)
	{
		SetManualMode();
		CarProtocol_FinishActiveMotionErr("CANCELED");
		ReplyDone(seq, "MODE", "M=MANUAL");
	}
	else if(strcmp(mode, "IDLE") == 0 || strcmp(mode, "STANDBY") == 0)
	{
		SetStandbyMode();
		CarProtocol_FinishActiveMotionErr("CANCELED");
		ReplyDone(seq, "MODE", "M=IDLE");
	}
	else
	{
		ReplyErr(seq, "BAD_MODE");
	}
}

static void HandleV2Servo(char *tokens[], uint8_t count, uint16_t seq)
{
	float value;

	if(!CommandParser_GetFloatArg(tokens, count, "A", &value) &&
	   !CommandParser_GetFloatArg(tokens, count, "ANG", &value))
	{
		ReplyErr(seq, "BAD_ARG");
	}
	else
	{
		SetManualMode();
		CarProtocol_FinishActiveMotionErr("CANCELED");
		SetSteeringAngle(value);
		ReplyDone(seq, "SERVO", "");
	}
}

static void HandleV2Move(char *tokens[], uint8_t count, uint16_t seq)
{
	float distance;
	uint8_t speed;

	if(!CommandParser_GetFloatArg(tokens, count, "D", &distance) ||
	   !GetSpeedArg(tokens, count, &speed))
	{
		ReplyErr(seq, "BAD_ARG");
		return;
	}

	BeginProtocolMotion(speed);
	FinishProtocolMotion(StartDistanceDrive(distance), seq, "MOVE", "BAD_ARG");
}

static void HandleV2Turn(char *tokens[], uint8_t count, uint16_t seq)
{
	float angle;
	uint8_t speed;

	if(!CommandParser_GetFloatArg(tokens, count, "A", &angle) ||
	   !GetSpeedArg(tokens, count, &speed))
	{
		ReplyErr(seq, "BAD_ARG");
		return;
	}

	BeginProtocolMotion(speed);
	FinishProtocolMotion(StartYawTurn(angle), seq, "TURN", "BAD_ARG");
}

static void HandleV2Arc(char *tokens[], uint8_t count, uint16_t seq)
{
	float distance;
	float steer;
	uint8_t speed;

	if(!CommandParser_GetFloatArg(tokens, count, "D", &distance) ||
	   !CommandParser_GetFloatArg(tokens, count, "STE", &steer) ||
	   !GetSpeedArg(tokens, count, &speed))
	{
		ReplyErr(seq, "BAD_ARG");
		return;
	}

	BeginProtocolMotion(speed);
	FinishProtocolMotion(StartArcDrive(distance, steer), seq, "ARC", "BAD_ARG");
}

static void HandleV2Auto(uint16_t seq)
{
	BeginProtocolMotion(AUTO_DEFAULT_SPEED);
	FinishProtocolMotion(StartAutoRoute(), seq, "AUTO", "AUTO_FAIL");
}

static void HandleV2Zero(const char *cmd, uint16_t seq)
{
	if(strcmp(cmd, "ZERO_ODOM") == 0 || strcmp(cmd, "ZERO_ALL") == 0)
	{
		Odometry_Reset();
	}
	if(strcmp(cmd, "ZERO_YAW") == 0 || strcmp(cmd, "ZERO_ALL") == 0)
	{
		YawReportOffset = New_Yaw;
		Org_Yaw = New_Yaw;
		TargetYaw = New_Yaw;
	}
	ReplyDone(seq, cmd, "");
}

static void HandleV2Get(char *tokens[], uint8_t count, uint16_t seq)
{
	const char *param = CommandParser_FindKeyValue(tokens, count, "PARAM");

	if(param == 0)
	{
		ReplyErr(seq, "BAD_ARG");
	}
	else
	{
		PrintParamV2(seq, param);
	}
}

static void HandleV2Set(char *tokens[], uint8_t count, uint16_t seq)
{
	const char *param = CommandParser_FindKeyValue(tokens, count, "PARAM");

	if(param == 0)
	{
		ReplyErr(seq, "BAD_ARG");
	}
	else if(strcmp(param, "HEADING") == 0)
	{
		if(ApplyHeadingParams(tokens, count)) ReplyDone(seq, "SET", "PARAM=HEADING");
		else ReplyErr(seq, "BAD_ARG");
	}
	else if(strcmp(param, "MOTOR") == 0)
	{
		if(ApplyMotorParams(tokens, count)) ReplyDone(seq, "SET", "PARAM=MOTOR");
		else ReplyErr(seq, "BAD_ARG");
	}
	else if(strcmp(param, "LIMIT") == 0 || strcmp(param, "SERVO") == 0)
	{
		if(ApplyLimitParams(tokens, count)) ReplyDone(seq, "SET", "PARAM=LIMIT");
		else ReplyErr(seq, "BAD_ARG");
	}
	else
	{
		ReplyErr(seq, "BAD_PARAM");
	}
}

static uint8_t HandleV2Command(char *pBuffer)
{
	char *tokens[16];
	uint8_t count;
	uint8_t cmdIndex = 0;
	uint16_t seq = 0;
	const char *cmd;

	count = CommandParser_Tokenize(pBuffer, tokens, 16);
	if(count == 0)
	{
		return 1;
	}

	if(CommandParser_IsUnsignedInteger(tokens[0]))
	{
		if(!CommandParser_ParseSeq(tokens[0], &seq) || count < 2)
		{
			ReplyErr(seq, "BAD_SEQ");
			return 1;
		}
		cmdIndex = 1;
	}

	cmd = tokens[cmdIndex];
	if(!IsV2CommandName(cmd))
	{
		ReplyErr(seq, "BAD_CMD");
		return 1;
	}

	if(strcmp(cmd, "PING") == 0)
	{
		ReplyDone(seq, "PING", "PONG");
	}
	else if(strcmp(cmd, "VER") == 0)
	{
		PrintFirmwareV2(seq);
	}
	else if(strcmp(cmd, "STAT") == 0)
	{
		PrintStatusV2(seq);
	}
	else if(strcmp(cmd, "PWM_STAT") == 0)
	{
		PrintPwmStatusV2(seq);
	}
	else if(strcmp(cmd, "TEL") == 0)
	{
		HandleV2Telemetry(tokens, count, cmdIndex, seq);
	}
	else if(strcmp(cmd, "STOP") == 0 || strcmp(cmd, "CANCEL") == 0)
	{
		SetStandbyMode();
		CarProtocol_FinishActiveMotionErr("CANCELED");
		ReplyDone(seq, cmd, "");
	}
	else if(strcmp(cmd, "MODE") == 0)
	{
		HandleV2Mode(tokens, count, seq);
	}
	else if(strcmp(cmd, "SERVO") == 0)
	{
		HandleV2Servo(tokens, count, seq);
	}
	else if(strcmp(cmd, "MOVE") == 0)
	{
		HandleV2Move(tokens, count, seq);
	}
	else if(strcmp(cmd, "TURN") == 0)
	{
		HandleV2Turn(tokens, count, seq);
	}
	else if(strcmp(cmd, "ARC") == 0)
	{
		HandleV2Arc(tokens, count, seq);
	}
	else if(strcmp(cmd, "AUTO") == 0)
	{
		HandleV2Auto(seq);
	}
	else if(strcmp(cmd, "ZERO_ODOM") == 0 || strcmp(cmd, "ZERO_YAW") == 0 || strcmp(cmd, "ZERO_ALL") == 0)
	{
		HandleV2Zero(cmd, seq);
	}
	else if(strcmp(cmd, "GET") == 0)
	{
		HandleV2Get(tokens, count, seq);
	}
	else if(strcmp(cmd, "SET") == 0)
	{
		HandleV2Set(tokens, count, seq);
	}
	else if(strcmp(cmd, "DEFAULT_CFG") == 0)
	{
		RestoreDefaultRuntimeConfig();
		ReplyDone(seq, "DEFAULT_CFG", "");
	}
	else if(strcmp(cmd, "SAVE_CFG") == 0 || strcmp(cmd, "LOAD_CFG") == 0)
	{
		ReplyErr(seq, "UNSUPPORTED");
	}
	else
	{
		ReplyErr(seq, "BAD_CMD");
	}

	RefreshCommandWatchdog();
	return 1;
}

typedef enum CommandResult
{
	CMD_NOT_MATCHED = 0,
	CMD_HANDLED,
	CMD_REJECTED
} CommandResult_t;

static void EnterManualSpeedMode(void)
{
	if(IsAutoMotionMode())
	{
		SetManualMode();
	}
	SetManualModeIfIdle();
}

static CommandResult_t HandleRcSimpleCommand(char *pBuffer)
{
	const char *command = (const char *)pBuffer;

	if(strcmp(command, "RC_HB") == 0)
	{
		USART3_printf("OK\r\n");
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_STOP") == 0 || strcmp(command, "AU_STOP") == 0)
	{
		USART3_printf("Stop!\r\n");
		SetStandbyMode();
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_MAN") == 0)
	{
		SetManualMode();
		USART3_printf("Manual mode\r\n");
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_TEL0") == 0)
	{
		TelemetryEnabled = 0;
		USART3_printf("Telemetry off\r\n");
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_TEL1") == 0)
	{
		TelemetryEnabled = 1;
		USART3_printf("Telemetry on\r\n");
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_STAT") == 0)
	{
		PrintTelemetry();
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_STR") == 0)
	{
		PrepareStraightHold();
		USART3_printf("Straight hold mode\r\n");
		return CMD_HANDLED;
	}
	if(strcmp(command, "RC_AUTO") == 0 || strcmp(command, "AU_RUN") == 0)
	{
		return StartAutoRoute() ? CMD_HANDLED : CMD_REJECTED;
	}

	return CMD_NOT_MATCHED;
}

static CommandResult_t HandleRcValueCommand(char *pBuffer)
{
	const char *command = (const char *)pBuffer;
	float value = 0.0f;

	if(strncmp(command, "RC_DST", 6) == 0)
	{
		if(CommandParser_ParseValue(&pBuffer[6], &value, 0))
		{
			return StartDistanceDrive(value) ? CMD_HANDLED : CMD_REJECTED;
		}
		USART3_printf("Invalid distance value!\r\n");
		return CMD_REJECTED;
	}
	if(strncmp(command, "RC_YAW", 6) == 0)
	{
		if(CommandParser_ParseValue(&pBuffer[6], &value, 0))
		{
			return StartYawTurn(value) ? CMD_HANDLED : CMD_REJECTED;
		}
		USART3_printf("Invalid yaw value!\r\n");
		return CMD_REJECTED;
	}
	if(strncmp(command, "RC_SPD", 6) == 0)
	{
		if(CommandParser_ParseValue(&pBuffer[6], &value, 0) && value >= 0.0f && value <= 6.0f)
		{
			EnterManualSpeedMode();
			SetSpeedRank((int8_t)value);
			USART3_printf("SET %d Rank!\r\n", SpeedRank);
			return CMD_HANDLED;
		}
		USART3_printf("Invalid speed rank!\r\n");
		return CMD_REJECTED;
	}
	if(strncmp(command, "RC_STE", 6) == 0)
	{
		if(CommandParser_ParseValue(&pBuffer[6], &value, 0))
		{
			SetManualMode();
			SetSteeringAngle(value);
			USART3_printf("Servo to %f deg!\r\n", Angle);
			return CMD_HANDLED;
		}
		USART3_printf("Invalid servo value!\r\n");
		return CMD_REJECTED;
	}

	return CMD_NOT_MATCHED;
}

static CommandResult_t HandleLegacySimpleCommand(char *pBuffer)
{
	const char *command = (const char *)pBuffer;

	if(strcmp(command, COMMANDS[7]) == 0)
	{
		USART3_printf("Reset!\r\n");
		SoftReset();
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[6]) == 0)
	{
		SetManualMode();
		USART3_printf("Down!\r\n");
		if(is_up == 1) ExDirect(0);
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[5]) == 0)
	{
		SetManualMode();
		USART3_printf("Up!\r\n");
		if(is_up == -1) ExDirect(1);
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[12]) == 0)
	{
		USART3_printf("Stand by!\r\n");
		SetStandbyMode();
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[10]) == 0)
	{
		USART3_printf("Parking auto!\r\n");
		return StartAutoRoute() ? CMD_HANDLED : CMD_REJECTED;
	}
	if(strcmp(command, COMMANDS[11]) == 0)
	{
		USART3_printf("Hitted!\r\n");
		SetStandbyMode();
		rS = HITTED;
		SetLEDs(GPIO_Pin_13);
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[8]) == 0)
	{
		USART3_printf("Straight!\r\n");
		PrepareStraightHold();
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[9]) == 0)
	{
		SetManualMode();
		is_turn = 1;
		USART3_printf("Turn manual mode!\r\n");
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[0]) == 0)
	{
		EnterManualSpeedMode();
		USART3_printf("SpeedRank add!\r\n");
		SpeedAcc();
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[1]) == 0)
	{
		EnterManualSpeedMode();
		USART3_printf("SpeedRank decline!\r\n");
		SpeedSlowDown();
		return CMD_HANDLED;
	}
	if(strcmp(command, COMMANDS[3]) == 0)
	{
		SetManualMode();
		USART3_printf("SpeedRank stop!\r\n");
		SpeedRank = 0;
		rSetSpeed(0);
		lSetSpeed(0);
		return CMD_HANDLED;
	}

	return CMD_NOT_MATCHED;
}

static CommandResult_t HandleLegacyValueCommand(char *pBuffer)
{
	const char *command = (const char *)pBuffer;

	if(strncmp(command, COMMANDS[13], 4) == 0)
	{
		float t = 0.0f;

		if((pBuffer[4] == 'P' || pBuffer[4] == 'I' || pBuffer[4] == 'D') &&
		   CommandParser_ParseValue(&pBuffer[5], &t, 1))
		{
			switch(pBuffer[4])
			{
			case 'P':
				headingPID.Kp = t;
				USART3_printf("Set heading Kp to %f!\r\n", headingPID.Kp);
				return CMD_HANDLED;
			case 'I':
				headingPID.Ki = t;
				USART3_printf("Set heading Ki to %f!\r\n", headingPID.Ki);
				return CMD_HANDLED;
			case 'D':
				headingPID.Kd = t;
				USART3_printf("Set heading Kd to %f!\r\n", headingPID.Kd);
				return CMD_HANDLED;
			default:
				USART3_printf("Unknown heading PID command!\r\n");
				return CMD_REJECTED;
			}
		}
		USART3_printf("Invalid heading PID value!\r\n");
		return CMD_REJECTED;
	}

	if(strncmp(command, COMMANDS[4], 5) == 0)
	{
		float rotateCmd = 0.0f;

		if(!CommandParser_ParseValue(&pBuffer[5], &rotateCmd, 0))
		{
			USART3_printf("Invalid rotate value!\r\n");
			return CMD_REJECTED;
		}
		SetManualMode();
		SetSteeringAngle(180.0f - rotateCmd);
		USART3_printf("Rotate to %f deg!\r\n", Angle);
		SetSteeringAngle(Angle);
		return CMD_HANDLED;
	}

	if(strncmp(command, COMMANDS[2], 6) == 0)
	{
		if(pBuffer[6] >= '0' && pBuffer[6] <= '6')
		{
			EnterManualSpeedMode();
			SetSpeedRank(pBuffer[6] - '0');
			USART3_printf("SET %d Rank!\r\n", SpeedRank);
			return CMD_HANDLED;
		}
		USART3_printf("Invalid speed rank!\r\n");
		return CMD_REJECTED;
	}

	return CMD_NOT_MATCHED;
}

void CarProtocol_HandleTextCommand(char *pBuffer)
{
	CommandResult_t result;

	if(IsV2Candidate(pBuffer))
	{
		HandleV2Command(pBuffer);
		return;
	}

	result = HandleRcSimpleCommand(pBuffer);
	if(result == CMD_NOT_MATCHED) result = HandleRcValueCommand(pBuffer);
	if(result == CMD_NOT_MATCHED) result = HandleLegacySimpleCommand(pBuffer);
	if(result == CMD_NOT_MATCHED) result = HandleLegacyValueCommand(pBuffer);

	if(result == CMD_NOT_MATCHED)
	{
		USART3_printf("Unknown command!\r\n");
	}

	if(result == CMD_HANDLED)
	{
		RefreshCommandWatchdog();
	}
}
