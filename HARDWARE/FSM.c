#include "FSM.h"
#include "USART.h"
#include "PWMO.h"
#include "LED.h"
#include "Steering.h"
#include <string.h>

// ========== FSM 拥有的全局状态变量 ==========
RS rS = STANDBY;
uint8_t is_Pause = 1;

// ========== FSM 内部静态变量 ==========
static ControlMode_t ControlMode = CTRL_IDLE;
static AutoStep_t AutoStep = AUTO_IDLE;
static uint32_t LastCommandTick = 0;
static uint32_t ActionStartTick = 0;
static float TargetDistanceCm = 0.0f;
static float TargetYaw = 0.0f;
static uint8_t AutoSpeedLevel = AUTO_DEFAULT_SPEED;

// ========== 工具函数 ==========

static uint8_t IsDigitChar(char c)
{
    return (c >= '0' && c <= '9');
}

static const char *SkipValuePrefix(const char *s)
{
    while(*s == ' ' || *s == '=' || *s == ':')
    {
        s++;
    }
    return s;
}

uint8_t ParseCommandValue(const char *s, float *value, uint8_t scaledHundredths)
{
    float integer = 0.0f;
    float fraction = 0.0f;
    float scale = 1.0f;
    int8_t sign = 1;
    uint8_t hasDigit = 0;
    uint8_t hasDot = 0;

    s = SkipValuePrefix(s);

    if(*s == '-')
    {
        sign = -1;
        s++;
    }
    else if(*s == '+')
    {
        s++;
    }

    while(IsDigitChar(*s))
    {
        integer = integer * 10.0f + (float)(*s - '0');
        hasDigit = 1;
        s++;
    }

    if(*s == '.')
    {
        hasDot = 1;
        s++;
        while(IsDigitChar(*s))
        {
            scale *= 10.0f;
            fraction += (float)(*s - '0') / scale;
            hasDigit = 1;
            s++;
        }
    }

    s = SkipValuePrefix(s);
    if(!hasDigit || *s != '\0')
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

float ClampFloat(float value, float minValue, float maxValue)
{
    if(value < minValue) return minValue;
    if(value > maxValue) return maxValue;
    return value;
}

float AbsFloat(float value)
{
    return value < 0.0f ? -value : value;
}

float NormalizeYaw(float yaw)
{
    while(yaw > 180.0f) yaw -= 360.0f;
    while(yaw < -180.0f) yaw += 360.0f;
    return yaw;
}

float GetYawError(float target, float current)
{
    return NormalizeYaw(target - current);
}

// ========== 系统复位 ==========

void SoftReset(void)
{
    __set_FAULTMASK(1);
    for(int i = 0; i < 10000; i++);
    NVIC_SystemReset();
}

// ========== 模式切换 ==========

void RefreshCommandWatchdog(void)
{
    LastCommandTick = ControlTicks;
}

void CenterSteering(void)
{
    Steering_Center();
}

void HardStopMotion(void)
{
    SpeedRank = 0;
    rSetSpeed(0);
    lSetSpeed(0);
    InitAll();
    Steering_StopOpenLoopTurn();
    Steering_CancelYawRate();
    CenterSteering();
    is_straight = 0;
    is_turn = 0;
    AutoStep = AUTO_IDLE;
    ControlMode = CTRL_IDLE;
}

void SetStandbyMode(void)
{
    HardStopMotion();
    rS = STANDBY;
    SetLEDs(GPIO_Pin_14);
}

void SetManualMode(void)
{
    if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
    {
        AutoStep = AUTO_IDLE;
    }
    ControlMode = CTRL_MANUAL;
    is_straight = 0;
    is_turn = 0;
    headingPID.CrossTrackEnable = 0;
    Steering_StopOpenLoopTurn();
}

void SetManualModeIfIdle(void)
{
    if(ControlMode == CTRL_IDLE)
    {
        SetManualMode();
    }
}

void EnsureAutoSpeed(void)
{
    if(SpeedRank == 0)
    {
        SetSpeedRank(AutoSpeedLevel);
    }
}

void PrepareStraightHold(void)
{
    Set_Straight();
    ControlMode = CTRL_STRAIGHT;
    AutoStep = AUTO_IDLE;
}

// ========== 速度与方向 ==========

void SpeedAcc()
{
    int16_t rank = ABS(SpeedRank);
    if(rank < 720)
    {
        rank += SPEEDSTEP;
        if(rank > 720) rank = 720;
        SpeedRank = ABSTRACT(is_up) * rank;
    }
}

void SpeedSlowDown()
{
    int16_t rank = ABS(SpeedRank);
    if(rank > 0)
    {
        rank -= SPEEDSTEP;
        if(rank < 0) rank = 0;
        SpeedRank = ABSTRACT(is_up) * rank;
    }
}

void ExDirect(uint8_t Rot)
{
    if(Rot)
    {
        is_up = 1;
        SpeedRank = ABS(SpeedRank);
    }
    else
    {
        is_up = -1;
        SpeedRank = -ABS(SpeedRank);
    }
    InitAll();
    HeadingPID_Reset(&headingPID);
    Odometry_Reset();
    if(is_straight)
    {
        headingPID.CrossTrackEnable = 1;
    }
    is_Switch = 1;
}

void SetSpeedRank(int8_t level)
{
    if(level < 7 && level > -1)
    {
        SpeedRank = is_up * level * SPEEDSTEP;
    }
}

// ========== 自动驾驶 ==========

uint8_t PrepareDistanceDrive(float distanceCm)
{
    if(AbsFloat(distanceCm) < 1.0f)
    {
        return 0;
    }

    if(distanceCm < 0.0f)
    {
        if(is_up == 1)
        {
            ExDirect(0);
        }
        TargetDistanceCm = -distanceCm;
    }
    else
    {
        if(is_up == -1)
        {
            ExDirect(1);
        }
        TargetDistanceCm = distanceCm;
    }

    Set_Straight();
    Odometry_Reset();
    ActionStartTick = ControlTicks;
    EnsureAutoSpeed();
    return 1;
}

void StartDistanceDrive(float distanceCm)
{
    if(PrepareDistanceDrive(distanceCm))
    {
        ControlMode = CTRL_DISTANCE;
        AutoStep = AUTO_IDLE;
        USART3_printf("Distance drive %.1f cm\r\n", TargetDistanceCm);
    }
    else
    {
        USART3_printf("Invalid distance target!\r\n");
    }
}

uint8_t PrepareYawTurn(float relativeYawDeg)
{
    if(AbsFloat(relativeYawDeg) < TURN_DONE_DEG)
    {
        return 0;
    }

    TargetYaw = NormalizeYaw(New_Yaw + relativeYawDeg);
    is_straight = 0;
    is_turn = 1;
    headingPID.CrossTrackEnable = 0;
    ActionStartTick = ControlTicks;
    EnsureAutoSpeed();
    return 1;
}

void StartYawTurn(float relativeYawDeg)
{
    if(PrepareYawTurn(relativeYawDeg))
    {
        ControlMode = CTRL_TURN_YAW;
        AutoStep = AUTO_IDLE;
        USART3_printf("Yaw turn %.1f deg\r\n", relativeYawDeg);
    }
    else
    {
        USART3_printf("Invalid yaw target!\r\n");
    }
}

void StartAutoRoute(void)
{
    rS = PARKING;
    SetLEDs(GPIO_Pin_12);
    AutoSpeedLevel = AUTO_DEFAULT_SPEED;
    ControlMode = CTRL_AUTO_ROUTE;
    AutoStep = AUTO_FORWARD1;
    if(PrepareDistanceDrive(AUTO_FORWARD1_CM))
    {
        USART3_printf("Auto route start\r\n");
    }
    else
    {
        SetStandbyMode();
        USART3_printf("Auto route failed\r\n");
    }
}

uint8_t UpdateDistanceDrive(void)
{
    if(TargetDistanceCm <= 0.0f)
    {
        return 1;
    }
    if((TargetDistanceCm - odom.distance) <= DISTANCE_DONE_CM)
    {
        return 1;
    }
    return 0;
}

uint8_t UpdateYawTurn(void)
{
    float error = GetYawError(TargetYaw, New_Yaw);

    if(AbsFloat(error) <= TURN_DONE_DEG)
    {
        SpeedRank = 0;
        CenterSteering();
        is_turn = 0;
        return 1;
    }

    Steering_SetYawTurnCorrection(error, is_up);
    EnsureAutoSpeed();
    return 0;
}

void UpdateControlTask(void)
{
    if((ControlMode == CTRL_MANUAL || ControlMode == CTRL_STRAIGHT) &&
       SpeedRank != 0 &&
       (uint32_t)(ControlTicks - LastCommandTick) > REMOTE_TIMEOUT_MS)
    {
        SetStandbyMode();
        USART3_printf("Remote timeout stop!\r\n");
        return;
    }

    if((ControlMode == CTRL_DISTANCE ||
       (ControlMode == CTRL_AUTO_ROUTE && (AutoStep == AUTO_FORWARD1 || AutoStep == AUTO_FORWARD2))) &&
       (uint32_t)(ControlTicks - ActionStartTick) > DISTANCE_TIMEOUT_MS)
    {
        SetStandbyMode();
        USART3_printf("Distance timeout stop!\r\n");
        return;
    }

    if((ControlMode == CTRL_TURN_YAW ||
       (ControlMode == CTRL_AUTO_ROUTE && AutoStep == AUTO_TURN1)) &&
       (uint32_t)(ControlTicks - ActionStartTick) > TURN_TIMEOUT_MS)
    {
        SetStandbyMode();
        USART3_printf("Turn timeout stop!\r\n");
        return;
    }

    if(ControlMode == CTRL_DISTANCE)
    {
        if(UpdateDistanceDrive())
        {
            SetStandbyMode();
            USART3_printf("Distance done\r\n");
        }
    }
    else if(ControlMode == CTRL_TURN_YAW)
    {
        if(UpdateYawTurn())
        {
            SetStandbyMode();
            USART3_printf("Yaw turn done\r\n");
        }
    }
    else if(ControlMode == CTRL_AUTO_ROUTE)
    {
        if(AutoStep == AUTO_FORWARD1)
        {
            if(UpdateDistanceDrive())
            {
                SpeedRank = 0;
                if(PrepareYawTurn(AUTO_TURN_DEG))
                {
                    AutoStep = AUTO_TURN1;
                }
                else
                {
                    SetStandbyMode();
                    USART3_printf("Auto turn failed\r\n");
                }
            }
        }
        else if(AutoStep == AUTO_TURN1)
        {
            if(UpdateYawTurn())
            {
                if(PrepareDistanceDrive(AUTO_FORWARD2_CM))
                {
                    AutoStep = AUTO_FORWARD2;
                }
                else
                {
                    SetStandbyMode();
                    USART3_printf("Auto forward failed\r\n");
                }
            }
        }
        else if(AutoStep == AUTO_FORWARD2)
        {
            if(UpdateDistanceDrive())
            {
                SetStandbyMode();
                rS = PARKING;
                SetLEDs(GPIO_Pin_12);
                USART3_printf("Auto route done\r\n");
            }
        }
    }
}

// ========== 命令处理 (唯一入口) ==========

void HandleTextCommand(char *pBuffer)
{
    float value = 0.0f;

    RefreshCommandWatchdog();

    if(strcmp((const char *)pBuffer, COMMANDS[19]) == 0)
    {
        USART3_printf("OK\r\n");
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[20]) == 0 || strcmp((const char *)pBuffer, COMMANDS[21]) == 0)
    {
        USART3_printf("Stop!\r\n");
        SetStandbyMode();
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[22]) == 0)
    {
        SetManualMode();
        USART3_printf("Manual mode\r\n");
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[23]) == 0)
    {
        PrepareStraightHold();
        USART3_printf("Straight hold mode\r\n");
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[24]) == 0 || strcmp((const char *)pBuffer, COMMANDS[25]) == 0)
    {
        StartAutoRoute();
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[26], 6) == 0)
    {
        if(ParseCommandValue(&pBuffer[6], &value, 0))
        {
            StartDistanceDrive(value);
        }
        else
        {
            USART3_printf("Invalid distance value!\r\n");
        }
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[27], 6) == 0)
    {
        if(ParseCommandValue(&pBuffer[6], &value, 0))
        {
            StartYawTurn(value);
        }
        else
        {
            USART3_printf("Invalid yaw value!\r\n");
        }
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[28], 6) == 0)
    {
        if(ParseCommandValue(&pBuffer[6], &value, 0) && value >= 0.0f && value <= 6.0f)
        {
            if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
            {
                SetManualMode();
            }
            SetManualModeIfIdle();
            SetSpeedRank((int8_t)value);
            USART3_printf("SET %d Rank!\r\n", SpeedRank);
        }
        else
        {
            USART3_printf("Invalid speed rank!\r\n");
        }
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[29], 6) == 0)
    {
        if(ParseCommandValue(&pBuffer[6], &value, 0))
        {
            SetManualMode();
            Steering_SetAngle(value);
            USART3_printf("Servo to %f deg!\r\n", (double)Steering_GetAngle());
        }
        else
        {
            USART3_printf("Invalid servo value!\r\n");
        }
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[7]) == 0)
    {
        USART3_printf("Reset!\r\n");
        SoftReset();
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[6]) == 0)
    {
        SetManualMode();
        USART3_printf("Down!\r\n");
        if(is_up == 1) ExDirect(0);
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[5]) == 0)
    {
        SetManualMode();
        USART3_printf("Up!\r\n");
        if(is_up == -1) ExDirect(1);
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[12]) == 0)
    {
        USART3_printf("Stand by!\r\n");
        SetStandbyMode();
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[10]) == 0)
    {
        USART3_printf("Parking auto!\r\n");
        StartAutoRoute();
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[11]) == 0)
    {
        USART3_printf("Hitted!\r\n");
        SetStandbyMode();
        rS = HITTED;
        SetLEDs(GPIO_Pin_13);
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[8]) == 0)
    {
        USART3_printf("Straight!\r\n");
        PrepareStraightHold();
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[9], 6) == 0)
    {
        SetManualMode();
        is_turn = 1;
        if(pBuffer[6] != '\0')
        {
            float rate_dps = 0.0f;
            if(ParseCommandValue(&pBuffer[6], &rate_dps, 0))
            {
                Steering_SetOpenLoopTurnRate(rate_dps);
            }
            else
            {
                USART3_printf("Invalid turn rate!\r\n");
            }
        }
        else
        {
            Steering_StopOpenLoopTurn();
            USART3_printf("Turn manual mode!\r\n");
        }
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[0]) == 0)
    {
        if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
        {
            SetManualMode();
        }
        SetManualModeIfIdle();
        USART3_printf("SpeedRank add!\r\n");
        SpeedAcc();
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[1]) == 0)
    {
        if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
        {
            SetManualMode();
        }
        SetManualModeIfIdle();
        USART3_printf("SpeedRank decline!\r\n");
        SpeedSlowDown();
    }
    else if(strcmp((const char *)pBuffer, COMMANDS[3]) == 0)
    {
        SetManualMode();
        USART3_printf("SpeedRank stop!\r\n");
        SpeedRank = 0;
        rSetSpeed(0);
        lSetSpeed(0);
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[13], 4) == 0)
    {
        float t = 0.0f;
        if((pBuffer[4] == 'P' || pBuffer[4] == 'I' || pBuffer[4] == 'D') &&
           ParseCommandValue(&pBuffer[5], &t, 1))
        {
            switch(pBuffer[4])
            {
            case 'P':
                headingPID.Kp = t;
                USART3_printf("Set heading Kp to %f!\r\n", headingPID.Kp);
                break;
            case 'I':
                headingPID.Ki = t;
                USART3_printf("Set heading Ki to %f!\r\n", headingPID.Ki);
                break;
            case 'D':
                headingPID.Kd = t;
                USART3_printf("Set heading Kd to %f!\r\n", headingPID.Kd);
                break;
            default:
                USART3_printf("Unknown heading PID command!\r\n");
                break;
            }
        }
        else
        {
            USART3_printf("Invalid heading PID value!\r\n");
        }
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[16], 4) == 0)
    {
        float t = 0.0f;
        if((pBuffer[4] == 'P' || pBuffer[4] == 'I' || pBuffer[4] == 'D') &&
           ParseCommandValue(&pBuffer[5], &t, 1))
        {
            switch(pBuffer[4])
            {
            case 'P':
                rSpeed_PID.Kp = t;  lSpeed_PID.Kp = t;
                USART3_printf("Set speed Kp to %.4f!\r\n", (double)rSpeed_PID.Kp);
                break;
            case 'I':
                rSpeed_PID.Ki = t;  lSpeed_PID.Ki = t;
                USART3_printf("Set speed Ki to %.4f!\r\n", (double)rSpeed_PID.Ki);
                break;
            case 'D':
                rSpeed_PID.Kd = t;  lSpeed_PID.Kd = t;
                USART3_printf("Set speed Kd to %.4f!\r\n", (double)rSpeed_PID.Kd);
                break;
            default:
                USART3_printf("Unknown speed PID command!\r\n");
                break;
            }
        }
        else
        {
            USART3_printf("Invalid speed PID value!\r\n");
        }
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[4], 5) == 0)
    {
        float rotateCmd = 0.0f;
        if(ParseCommandValue(&pBuffer[5], &rotateCmd, 0))
        {
            SetManualMode();
            Steering_SetAngle(180.0f - rotateCmd);
        }
        else
        {
            USART3_printf("Invalid rotate value!\r\n");
            return;
        }
        USART3_printf("Rotate to %f deg!\r\n", (double)Steering_GetAngle());
    }
    else if(strncmp((const char *)pBuffer, COMMANDS[2], 6) == 0)
    {
        if(pBuffer[6] >= '0' && pBuffer[6] <= '6')
        {
            if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
            {
                SetManualMode();
            }
            SetManualModeIfIdle();
            SetSpeedRank(pBuffer[6] - '0');
            USART3_printf("SET %d Rank!\r\n", SpeedRank);
        }
        else
        {
            USART3_printf("Invalid speed rank!\r\n");
        }
    }
    else
    {
        USART3_printf("Unknown command!\r\n");
    }
}
