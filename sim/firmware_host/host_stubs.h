#ifndef SS928_HOST_STUBS_H
#define SS928_HOST_STUBS_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#define GPIO_Pin_0  0x0001U
#define GPIO_Pin_1  0x0002U
#define GPIO_Pin_12 0x1000U
#define GPIO_Pin_13 0x2000U
#define GPIO_Pin_14 0x4000U
#define GPIOB ((void *)0)

#define ABS(v) ((v) < 0 ? -(v) : (v))
#define ABSTRACT(t) ((t) < 0 ? -1 : 1)
#define SPEEDSTEP 120
#define RSPEEDSTEP 15
#define WHEEL_TRACK_CM 14.5f
#define EXCOUNT(T, LIM) ((((T) = ((T) + 1) % (LIM)) == 0) ? 1 : 0)

#define __disable_irq() ((void)0)
#define NVIC_SystemReset() SS928_HostSoftReset()

typedef struct MPU6050 {
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
	int16_t rawTemp;
	float yaw;
	float roll;
	float pitch;
	float temp;
	float q0;
	float q1;
	float q2;
	float q3;
} MPU6050;

typedef struct {
	float q;
	float r;
	float x;
	float p;
	float k;
} KalmanFilter;

typedef struct {
	float x;
	float y;
	float theta;
	float distance;
} Odometry_t;

typedef struct {
	float Target;
	float Actual;
	float Out;
	float Kp;
	float Ki;
	float Kd;
	float Error0;
	float Error1;
	float ErrorInt;
	float dV;
	float pV;
	float iV;
	float OutMax;
	float OutMin;
} PID_t;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float MaxI;
	float MaxOut;
	float Deadband;
	float D_Alpha;
	float SmoothAlpha;
	float CrossTrackKp;
	uint8_t CrossTrackEnable;
	float Integral;
	float LastError;
	float dV;
	float SmoothedAngle;
	uint8_t FirstRun;
} HeadingPID_t;

typedef struct {
	float Speed;
	int32_t tSpeed;
	int32_t SpeedCnt;
} SPEED_t;

extern const char *COMMANDS[];
extern int16_t SpeedRank;
extern int16_t is_Switch;
extern SPEED_t rSpeed;
extern SPEED_t lSpeed;
extern PID_t rSpeed_PID;
extern PID_t lSpeed_PID;
extern PID_t Turn_PID;
extern HeadingPID_t headingPID;
extern float aveSpeed;
extern float diffSpeed;
extern Odometry_t odom;

void SS928_HostSoftReset(void);
void SS928_HostClearLog(void);
const char *SS928_HostLog(void);
float SS928_HostServoAngle(void);
int16_t SS928_HostLeftPwm(void);
int16_t SS928_HostRightPwm(void);
uint16_t SS928_HostLedMask(void);

void USART3_printf(const char *format, ...);
void InitAll(void);
void lSetSpeed(int16_t speed);
void rSetSpeed(int16_t speed);
void HeadingPID_Init(HeadingPID_t *p);
void HeadingPID_Reset(HeadingPID_t *p);
void Odometry_Reset(void);
void Odometry_Update(float left_speed_cms, float right_speed_cms);
void SetServoRotation(float angle);
void SetLEDs(uint16_t gpio_pin);
void KalmanFilter_Init(KalmanFilter *kf, float q, float r, float x, float p);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif
