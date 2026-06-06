#include "host_stubs.h"

#include <string.h>

#define HOST_LOG_SIZE 8192

const char *COMMANDS[] = {
	"SR_ACC",
	"SR_DEC",
	"SR_SET",
	"SR_PAU",
	"RT_TO",
	"DT_1",
	"DT_0",
	"RST",
	"DT_STA",
	"DT_TUR",
	"ST_PK",
	"ST_ER",
	"ST_SB",
	"ST_KP",
	"ST_KI",
	"ST_KD"
};

int16_t SpeedRank = 0;
int16_t is_Switch = 0;
SPEED_t rSpeed = {0};
SPEED_t lSpeed = {0};
PID_t rSpeed_PID = {.Kp = 0.1f, .Ki = 0.05f, .Kd = 0.1f, .OutMax = 720.0f, .OutMin = -720.0f};
PID_t lSpeed_PID = {.Kp = 0.1f, .Ki = 0.05f, .Kd = 0.1f, .OutMax = 720.0f, .OutMin = -720.0f};
PID_t Turn_PID = {.Kp = 0.1f, .Ki = 0.05f, .Kd = 0.1f};
HeadingPID_t headingPID = {
	.Kp = 2.5f,
	.Ki = 0.01f,
	.Kd = 0.18f,
	.MaxI = 5.0f,
	.MaxOut = 8.0f,
	.Deadband = 2.0f,
	.D_Alpha = 0.7f,
	.SmoothAlpha = 0.4f,
	.CrossTrackKp = 2.0f,
	.CrossTrackEnable = 1,
	.SmoothedAngle = 90.0f,
	.FirstRun = 1
};
float aveSpeed = 0.0f;
float diffSpeed = 0.0f;
Odometry_t odom = {0};

static char host_log[HOST_LOG_SIZE];
static size_t host_log_len = 0;
static float host_servo_angle = 90.0f;
static int16_t host_left_pwm = 0;
static int16_t host_right_pwm = 0;
static uint16_t host_led_mask = 0;
static int host_soft_reset_requested = 0;

static void host_append(const char *text)
{
	size_t remaining = HOST_LOG_SIZE - host_log_len;
	if(remaining <= 1)
	{
		return;
	}
	int written = snprintf(host_log + host_log_len, remaining, "%s", text);
	if(written > 0)
	{
		size_t count = (size_t)written;
		if(count >= remaining)
		{
			host_log_len = HOST_LOG_SIZE - 1;
		}
		else
		{
			host_log_len += count;
		}
	}
}

void SS928_HostSoftReset(void)
{
	host_soft_reset_requested = 1;
	host_append("[soft reset requested]\n");
}

void SS928_HostClearLog(void)
{
	host_log[0] = '\0';
	host_log_len = 0;
}

const char *SS928_HostLog(void)
{
	return host_log;
}

float SS928_HostServoAngle(void)
{
	return host_servo_angle;
}

int16_t SS928_HostLeftPwm(void)
{
	return host_left_pwm;
}

int16_t SS928_HostRightPwm(void)
{
	return host_right_pwm;
}

uint16_t SS928_HostLedMask(void)
{
	return host_led_mask;
}

void USART3_printf(const char *format, ...)
{
	char line[512];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	host_append(line);
}

static void pid_init(PID_t *p)
{
	p->Target = 0.0f;
	p->Actual = 0.0f;
	p->Out = 0.0f;
	p->Error0 = 0.0f;
	p->Error1 = 0.0f;
	p->ErrorInt = 0.0f;
	p->pV = 0.0f;
	p->iV = 0.0f;
	p->dV = 0.0f;
}

static void speed_init(SPEED_t *sp)
{
	sp->Speed = 0.0f;
	sp->tSpeed = 0;
	sp->SpeedCnt = 0;
}

void InitAll(void)
{
	pid_init(&rSpeed_PID);
	pid_init(&lSpeed_PID);
	pid_init(&Turn_PID);
	speed_init(&rSpeed);
	speed_init(&lSpeed);
	aveSpeed = 0.0f;
	diffSpeed = 0.0f;
}

void lSetSpeed(int16_t speed)
{
	host_left_pwm = speed;
}

void rSetSpeed(int16_t speed)
{
	host_right_pwm = speed;
}

void HeadingPID_Init(HeadingPID_t *p)
{
	p->Kp = 2.5f;
	p->Ki = 0.01f;
	p->Kd = 0.18f;
	p->MaxI = 5.0f;
	p->MaxOut = 8.0f;
	p->Deadband = 2.0f;
	p->D_Alpha = 0.7f;
	p->SmoothAlpha = 0.4f;
	p->CrossTrackKp = 2.0f;
	p->CrossTrackEnable = 1;
	HeadingPID_Reset(p);
}

void HeadingPID_Reset(HeadingPID_t *p)
{
	p->Integral = 0.0f;
	p->LastError = 0.0f;
	p->dV = 0.0f;
	p->SmoothedAngle = 90.0f;
	p->FirstRun = 1;
}

void Odometry_Reset(void)
{
	odom.x = 0.0f;
	odom.y = 0.0f;
	odom.theta = 0.0f;
	odom.distance = 0.0f;
}

void Odometry_Update(float left_speed_cms, float right_speed_cms)
{
	float dl = left_speed_cms * 0.02f;
	float dr = right_speed_cms * 0.02f;
	float dc = (dl + dr) * 0.5f;
	float raw_dtheta = (dr - dl) / WHEEL_TRACK_CM;
	float dtheta = dc >= 0.0f ? raw_dtheta : -raw_dtheta;
	float mid_theta = odom.theta + dtheta * 0.5f;
	odom.x -= dc * (float)sin((double)mid_theta);
	odom.y += dc * (float)cos((double)mid_theta);
	odom.theta += dtheta;
	while(odom.theta > 3.1415926f) odom.theta -= 2.0f * 3.1415926f;
	while(odom.theta < -3.1415926f) odom.theta += 2.0f * 3.1415926f;
	odom.distance += (float)fabs((double)dc);
}

void SetServoRotation(float angle)
{
	if(angle < 0.0f)
	{
		angle = 0.0f;
	}
	else if(angle > 180.0f)
	{
		angle = 180.0f;
	}
	host_servo_angle = angle;
}

void SetLEDs(uint16_t gpio_pin)
{
	host_led_mask = gpio_pin;
}

void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float x, float p)
{
	kf->q = q;
	kf->r = r;
	kf->x = x;
	kf->p = p;
	kf->k = 0.0f;
}

float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
	kf->p += kf->q;
	kf->k = kf->p / (kf->p + kf->r);
	kf->x += kf->k * (measurement - kf->x);
	kf->p = (1.0f - kf->k) * kf->p;
	return kf->x;
}
