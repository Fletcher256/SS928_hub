#ifndef _SENSOR_FUSION_H_
#define _SENSOR_FUSION_H_

#include <stdint.h>

#define SENSOR_FUSION_FAULT_NONE              0x0000U
#define SENSOR_FUSION_FAULT_FLOW_NOT_READY    0x0001U
#define SENSOR_FUSION_FAULT_FLOW_DT           0x0002U
#define SENSOR_FUSION_FAULT_FLOW_QUALITY      0x0004U
#define SENSOR_FUSION_FAULT_FLOW_MOTION       0x0008U
#define SENSOR_FUSION_FAULT_FLOW_SPEED        0x0010U
#define SENSOR_FUSION_FAULT_FLOW_JUMP         0x0020U
#define SENSOR_FUSION_FAULT_FLOW_HEIGHT       0x0040U
#define SENSOR_FUSION_FAULT_FLOW_SHUTTER      0x0080U
#define SENSOR_FUSION_FAULT_ENCODER_RANGE     0x0100U
#define SENSOR_FUSION_FAULT_IMU_INVALID       0x0200U
#define SENSOR_FUSION_FAULT_IMU_YAW_JUMP      0x0400U
#define SENSOR_FUSION_FAULT_FLOW_COMP_OMEGA   0x0800U

#define SENSOR_FUSION_FALLBACK_NONE           0x00U
#define SENSOR_FUSION_FALLBACK_FLOW           0x01U
#define SENSOR_FUSION_FALLBACK_IMU            0x02U
#define SENSOR_FUSION_FALLBACK_FLOW_COMP      0x04U

#define SENSOR_FUSION_DEFAULT_FLOW_MOUNT_X_CM 0.0f
#define SENSOR_FUSION_DEFAULT_FLOW_MOUNT_Y_CM 22.1f
#define SENSOR_FUSION_DEFAULT_FLOW_COMP_ENABLE 1U

typedef struct SensorFusionConfig
{
	float min_dt_s;
	float max_dt_s;
	float min_height_cm;
	float max_height_cm;
	float max_encoder_speed_cms;
	float max_flow_speed_cms;
	float max_flow_accel_cms2;
	float stationary_flow_speed_cms;
	float max_flow_weight;
	float flow_weight_alpha;
	float wheel_track_cm;
	float flow_mount_x_cm;
	float flow_mount_y_cm;
	float max_gyro_rps;
	float max_imu_yaw_jump_deg;
	float max_yaw_gyro_error_deg;
	uint8_t squal_min;
	uint8_t squal_good;
	uint8_t stationary_confirm_samples;
	uint8_t enable_flow_lever_compensation;
	uint16_t shutter_good_max;
	uint16_t shutter_invalid_min;
} SensorFusionConfig_t;

typedef struct SensorFusionInput
{
	uint32_t timestamp_ms;
	float dt_s;
	float left_speed_cms;
	float right_speed_cms;
	float flow_right_cm;
	float flow_forward_cm;
	float flow_height_cm;
	float imu_yaw_deg;
	float imu_gyro_z_rps;
	uint16_t flow_shutter;
	uint8_t flow_squal;
	uint8_t flow_motion;
	uint8_t flow_ready;
	uint8_t imu_valid;
} SensorFusionInput_t;

typedef struct SensorFusionOutput
{
	uint32_t timestamp_ms;
	float dt_s;
	float encoder_forward_cms;
	float raw_flow_right_cms;
	float raw_flow_forward_cms;
	float compensated_flow_right_cms;
	float compensated_flow_forward_cms;
	float compensation_omega_rps;
	float flow_mount_x_cm;
	float flow_mount_y_cm;
	/* Backward-compatible aliases for compensated flow at the rear axle. */
	float flow_right_cms;
	float flow_forward_cms;
	float fused_right_cms;
	float fused_forward_cms;
	float world_vx_cms;
	float world_vy_cms;
	float x_cm;
	float y_cm;
	float distance_cm;
	float yaw_deg;
	float flow_quality;
	float flow_weight;
	uint16_t fault_flags;
	uint8_t fallback_flags;
	uint8_t flow_valid;
	uint8_t compensation_valid;
} SensorFusionOutput_t;

void SensorFusion_GetDefaultConfig(SensorFusionConfig_t *config);
void SensorFusion_Init(const SensorFusionConfig_t *config);
void SensorFusion_ResetPose(void);
void SensorFusion_SetFlowMountPosition(float right_cm, float forward_cm);
void SensorFusion_EnableFlowLeverCompensation(uint8_t enable);
void SensorFusion_Update(const SensorFusionInput_t *input);
void SensorFusion_GetOutput(SensorFusionOutput_t *output);

#endif
