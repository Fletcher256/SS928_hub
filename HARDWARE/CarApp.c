#include "stm32f10x.h"
#include "CarApp.h"
#include "CarControl.h"
#include "CarProtocol.h"
#include "Timers.h"
#include "Motors.h"
#include "PWMO.h"
#include "YH8.h"
//#include "OLED.h"
#include "USART.h"
#include "BMI270/bmi270_driver.h"
#include "_MyI2C_.h"
#include "filter.h"
#include "LED.h"

static volatile uint16_t MpuTaskElapsedMs = 0;
static volatile uint8_t StraightTaskReady = 0;
static volatile uint8_t ServoTaskReady = 0;

// Main loop tasks -----------------------------------------------------------

static uint8_t TakeTaskFlag(volatile uint8_t *flag)
{
	uint8_t ready;

	__disable_irq();
	ready = *flag;
	*flag = 0;
	__enable_irq();

	return ready;
}

static uint16_t TakeElapsedMs(volatile uint16_t *elapsedMs)
{
	uint16_t elapsed;

	__disable_irq();
	elapsed = *elapsedMs;
	*elapsedMs = 0;
	__enable_irq();

	return elapsed;
}

static void ServiceMpuTask(void)
{
	uint16_t elapsedMs = TakeElapsedMs(&MpuTaskElapsedMs);

	if(elapsedMs > 0)
	{
		BMI270_Get_AngleDt(&MM, (float)elapsedMs * 0.001f);
		New_Pitch = KalmanFilter_Update(&Kal_Pitch,MM.pitch);
		New_Roll = KalmanFilter_Update(&Kal_Roll,MM.roll);
		New_Yaw = KalmanFilter_Update(&Kal_Yaw,MM.yaw);
	}
}

static void ServiceStraightTask(void)
{
	if(TakeTaskFlag(&StraightTaskReady) && is_straight)
	{
		keep_straight();
	}
}

static void ServiceServoTask(void)
{
	if(TakeTaskFlag(&ServoTaskReady))
	{
		ServoPWM_Service();
	}
}

void CarApp_Run(void)
{
	KalmanFilter_Init(&Kal_Yaw,0.5,0.1,1,100);   // q=0.5: 绋虫€佸鐩妦83%,蹇€熻窡韪獃aw鍙樺寲(鍘?.01澶參浠呭惛鏀?%)
	KalmanFilter_Init(&Kal_Roll,0.01,0.1,1,100);
	KalmanFilter_Init(&Kal_Pitch,0.01,0.1,1,100);
	YH8_Init();
	LED_Init();
	USART3_Init();
	/* I2C diag: scan + try reading Chip ID directly */
	{
		i2cbus_struct scan_bus;
		uint8_t addr;
		uint16_t chip_id_raw;

		/* Try with longer delay (10 instead of 5) */
		MyI2C_Init(&scan_bus, GPIOB, GPIO_Pin_0, GPIOB, GPIO_Pin_1, 0x69, 10);
		addr = MYI2C_Add_Scan(&scan_bus);
		USART3_printf("[SCAN] I2C device at 0x%02X\r\n", addr);

		/* Direct Chip ID read, no soft-reset first */
		chip_id_raw = MYI2C_Read_Reg(&scan_bus, 0x00);
		USART3_printf("[DIAG] Raw Chip ID read: 0x%04X → 0x%02X\r\n",
		              chip_id_raw, (uint8_t)(chip_id_raw & 0xFF));
	}
	//aMPU6050_Init();
	BMI270_init(GPIOB, GPIO_Pin_0, GPIO_Pin_1);
	//涔熶篃璁告垜浠渶瑕佸MPU6050杩涜涓€涓潤鎬佹牎鍑嗐€?
	//MPU6050_Calibration();
	//mpu_dmp_init(GPIOB,GPIO_Pin_1,GPIO_Pin_0);
	MotorEnCoder_Init();

	//寮€濮嬩娇鑳界粰0,涓嶈兘婊¤冻涓?鏃堕棿瓒冲闀垮洜姝ゆ棤娉曡緭鍑恒€?
	//鎵€浠T4950涔熼渶瑕佷竴涓垵濮嬪寲,灏辨槸涓婄數鍏堟妸瀹冨敜閱掋€傘€傘€?
	//OLED_Init();
	SysTick_Init();
	ServoPWM_Init();
	SetSteeringAngle(90.0f);

	Motor_Init();
	SetStandbyMode();
	RefreshCommandWatchdog();
	char commandBuffer[128];
	SetYH8(1); // RS0102YH8: 1 for car mode, 0 for programming mode. Set to car mode.

	//鏍￠獙MPU6050鏄惁鎴愬姛璇诲埌鏁版嵁銆?
	USART3_printf("Everything is ready!\r\n");
	while(1)
	{
		//mpu_dmp_get_data(&MM.pitch,&MM.roll,&MM.yaw);
		//璇诲彇鏍囧織浣嶅氨缁€?
		if(USART3_ReadText(commandBuffer, sizeof(commandBuffer)) == 1)
		{
			CarProtocol_HandleTextCommand(commandBuffer);
		}

		ServiceMpuTask();
		ServiceStraightTask();
		ServiceServoTask();
		UpdateControlTask();


		//USART3_printf("%d,%d,%d,%d,%d,%d\r\n",MD.xAcc,MD.yAcc,MD.zAcc,MD.xGyro,MD.yGyro,MD.zGyro);

		//atan2:鍙互璁＄畻-180deg鍒?80deg,绗竴涓舰鍙傛槸鍒嗗瓙銆?
		//USART3_printf("%f,%f,%f,%f,%f,%f,%f\r\n",EA.MPU6050_Yaw,EA.MPU6050_Roll,EA.MPU6050_Pitch,MD.zAcc*G*16/(0X7FFF),atan2(MD.xAcc,MD.yAcc)/PI*180,atan2(MD.yAcc,MD.zAcc)/PI*180,atan2(MD.xAcc,MD.zAcc)/PI*180);
		 //USART3_printf("%.3f,%.3f,%.3f\r\n", MM.roll, MM.pitch, MM.yaw);
		 //USART3_printf("%f,%f,%f,%d,%f,%f\r\n",rSpeed.Speed,lSpeed.Speed,aveSpeed,SpeedRank,rSpeed_PID.Out,lSpeed_PID.Out);
		 if(TakeTaskFlag(&TelemetryReady) && CarProtocol_IsTelemetryEnabled())
		 {
			 PrintTelemetry();
		 }
	}
}

void SysTick_Handler(void)
{
	ControlTicks++;
	//GetALLData(&MD);
	//CalEulerAngleHandler(&MD);
	//ComplementaryFilter(&MD);

	static uint16_t SwitchCnt = 0;
	static uint16_t MPU6050Cnt = 0;
	static uint16_t StraightCnt = 0;
	static uint16_t ServoCnt = 0;
	static uint16_t TelemetryCnt = 0;

	if(EXCOUNT(MPU6050Cnt,5) == 1)
	{
		if(MpuTaskElapsedMs <= 995U)
		{
			MpuTaskElapsedMs += 5U;
		}

		//杩欓噷鍋氫竴涓鍙栦涪鍖呮娴嬨€傚鏋滀竴涓暟鎹秴杩?娆℃病鏈変换浣曞彉鍖栭偅涔堣涓篗PU6050涓㈠寘,鐩存帴閲嶆柊璇诲彇銆?
	}
	if(is_Switch)
	{
		//鎹㈠悜鏃跺仠鎽?0ms
		if(EXCOUNT(SwitchCnt,20) == 1)
		{
			is_Switch = 0;
		}
	}
	else
	{
		//AccContrllor();
	//鍥犱负AT4950鐨勭壒鎬?鍞ら啋),鎵€浠ユ垜浠拡瀵瑰畠鏉ュ杩欎釜PID鐜繘琛屾敼杩?
	//鍒濆淇濇寔涓よ矾PWM涓嶉渶瑕佹媴蹇冩棤娉曞敜閱?


		PID_Speed(&rSpeed_PID,&rSpeed,1);
		PID_Speed(&lSpeed_PID,&lSpeed,0);
		//杩欎釜浼氬鑷存甯歌皟鐢垫満娌℃硶姝ｅ父杞悜,璋冭埖鏈虹殑鏃跺€欐敞鎰忓厛鎶婂畠鍏虫帀鍟娿€傘€傘€?
		if(is_straight && (EXCOUNT(StraightCnt,20) == 1))
		{
			StraightTaskReady = 1;
		}

		aveSpeed = (rSpeed.Speed + lSpeed.Speed)*0.5f;
	}

	if(EXCOUNT(TelemetryCnt,100) == 1)
	{
		TelemetryReady = 1;
	}
	if(EXCOUNT(ServoCnt,50) == 1)
	{
		ServoTaskReady = 1;
	}
}
