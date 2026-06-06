#ifndef _MOTORS_H_
#define _MOTORS_H_

#define TIM1_LMOTOR_PSC 1

#define TIM1_LMOTOR_ARR 720

#define TIM1_RMOTOR_PSC 1

#define TIM1_RMOTOR_ARR 720

#define ABS(v) (v < 0 ? -v : v) 

#define ABSTRACT(t) (t < 0 ? -1 : 1)

#define SPEEDSTEP 120

#define RSPEEDSTEP 15

#define AD_NUM 2

// ========== 里程计参数 ==========
#define WHEEL_TRACK_CM   14.5f   // 后轮轮距(cm),两后轮中心间距,需实测标定

// ========== 里程计数据结构 ==========
typedef struct {
	float x;          // X坐标(cm), 右方为正
	float y;          // Y坐标(cm), 前方为正
	float theta;      // 航向角(rad), 基于编码器差分,左转为正
	float distance;   // 累计行驶里程(cm)
} Odometry_t;

//标志位弃用以防止反方向直接诶正反馈超调。
//unsigned char is_right = 1;

// 46/72 == 0.63,以下数值大概在这个范围内。其实呢还是要看情况来着

//注意。若总线上的负载情况变化会对真个电路的电压分配情况造成影响,这时电机输出又会变化,PID可能需要重新调整。

//比如去掉OLED,这时少一个负载电机的分流就变大,驱动力增大,PID又要重新调整以适应新的负载情况

#define KP  0.1
//0.05

#define KI  0.05
//0.05

#define KD  0.1

//滤波系数。这个对dV的改善效果很是明显啊。
#define alpha 0.7

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

// ========== 舵机航向保持PID结构体 ==========
// 封装航向PID的全部可调参数与内部状态,支持串口在线修改Kp/Ki/Kd
typedef struct {
	// ---- 可调参数 (可通过串口 ST_KP/ST_KI/ST_KD 在线修改) ----
	float Kp;          // 比例增益 (度/度)
	float Ki;          // 积分增益
	float Kd;          // 微分增益
	float MaxI;        // 积分限幅 (度)
	float MaxOut;      // 输出总限幅 (度) - 舵机偏离中位的最大角度
	float Deadband;    // 死区 (度)
	float D_Alpha;     // D项低通滤波系数
	float SmoothAlpha; // 输出平滑系数 (0~1, 越小越平滑)
	float CrossTrackKp;   // 横向偏差增益 (度/cm)
	uint8_t CrossTrackEnable; // 横向修正使能 (1=开启, 0=关闭)

	// ---- 内部状态 (由 keep_straight() 维护, 换向/重设时复位) ----
	float Integral;
	float LastError;
	float dV;
	float SmoothedAngle;
	uint8_t FirstRun;
} HeadingPID_t;

typedef struct{
    float Speed;
    int32_t tSpeed;
    int32_t SpeedCnt;
}SPEED_t;

extern int16_t SpeedRank;

extern SPEED_t rSpeed;

extern SPEED_t lSpeed;

extern int16_t is_Switch;

extern PID_t rSpeed_PID;

extern PID_t lSpeed_PID;

extern PID_t Turn_PID;

extern HeadingPID_t headingPID;    // 舵机航向PID实例

extern float aveSpeed;

extern float diffSpeed;

extern Odometry_t odom;

void InitAll(void);

void Motor_Init(void);

void lSetSpeed(int16_t speed);

void rSetSpeed(int16_t speed);

void MotorEnCoder_Init(void);

void lSpeedCycle_Frame(void);

void rSpeedCycle_Frame(void);

void PID_Speed(PID_t *p,SPEED_t *sp,int8_t is_right);

void HeadingPID_Init(HeadingPID_t *p);     // 初始化航向PID (设为默认值)

void HeadingPID_Reset(HeadingPID_t *p);    // 复位航向PID内部状态 (保留Kp/Ki/Kd设置)

void Odometry_Reset(void);

void Odometry_Update(float left_speed_cms, float right_speed_cms);

#endif
