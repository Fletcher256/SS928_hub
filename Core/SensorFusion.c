#include "SensorFusion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SENSOR_FUSION_PI             3.1415926f
#define SENSOR_FUSION_DEG_TO_RAD     0.01745329252f
#define SENSOR_FUSION_RAD_TO_DEG     57.2957795f
#define SENSOR_FUSION_MOTION_BIT     0x80U
#define SENSOR_FUSION_DELTA_EPS_CM   0.00001f

static SensorFusionConfig_t FusionConfig;
static SensorFusionOutput_t FusionOutput;
static float FusionYawRad;
static float FusionYawOriginDeg;
static float PreviousFlowRightCms;
static float PreviousFlowForwardCms;
static uint8_t FusionInitialized;
static uint8_t FusionYawOriginValid;
static uint8_t PreviousFlowValid;
static uint8_t PreviousImuValid;
static uint8_t StationaryFlowCount;

static float SensorFusion_Abs(float value)
{
	return value < 0.0f ? -value : value;
}

static float SensorFusion_Clamp(float value, float min_value, float max_value)
{
	if(value < min_value) return min_value;
	if(value > max_value) return max_value;
	return value;
}

static float SensorFusion_WrapRad(float value)
{
	while(value > SENSOR_FUSION_PI) value -= 2.0f * SENSOR_FUSION_PI;
	while(value < -SENSOR_FUSION_PI) value += 2.0f * SENSOR_FUSION_PI;
	return value;
}

static float SensorFusion_WrapDeg(float value)
{
	while(value > 180.0f) value -= 360.0f;
	while(value < -180.0f) value += 360.0f;
	return value;
}

static uint8_t SensorFusion_IsFinite(float value)
{
	return (value == value && value < 3.4e38f && value > -3.4e38f) ? 1U : 0U;
}

void SensorFusion_GetDefaultConfig(SensorFusionConfig_t *config)
{
	if(config == NULL)
	{
		return;
	}

	config->min_dt_s = 0.005f;
	config->max_dt_s = 0.050f;
	config->min_height_cm = 1.0f;
	config->max_height_cm = 100.0f;
	config->max_encoder_speed_cms = 180.0f;
	config->max_flow_speed_cms = 200.0f;
	config->max_flow_accel_cms2 = 12000.0f;
	config->stationary_flow_speed_cms = 1.2f;
	config->max_flow_weight = 0.95f;
	config->flow_weight_alpha = 0.25f;
	config->wheel_track_cm = 14.5f;
	config->flow_mount_x_cm = SENSOR_FUSION_DEFAULT_FLOW_MOUNT_X_CM;
	config->flow_mount_y_cm = SENSOR_FUSION_DEFAULT_FLOW_MOUNT_Y_CM;
	config->max_gyro_rps = 20.0f;
	config->max_imu_yaw_jump_deg = 45.0f;
	config->max_yaw_gyro_error_deg = 20.0f;
	config->squal_min = 15U;
	config->squal_good = 80U;
	config->stationary_confirm_samples = 5U;
	config->enable_flow_lever_compensation = SENSOR_FUSION_DEFAULT_FLOW_COMP_ENABLE;
	config->shutter_good_max = 12000U;
	config->shutter_invalid_min = 60000U;
}

void SensorFusion_Init(const SensorFusionConfig_t *config)
{
	if(config == NULL)
	{
		SensorFusion_GetDefaultConfig(&FusionConfig);
	}
	else
	{
		FusionConfig = *config;
	}

	memset(&FusionOutput, 0, sizeof(FusionOutput));
	FusionYawRad = 0.0f;
	FusionYawOriginDeg = 0.0f;
	PreviousFlowRightCms = 0.0f;
	PreviousFlowForwardCms = 0.0f;
	FusionYawOriginValid = 0U;
	PreviousFlowValid = 0U;
	PreviousImuValid = 0U;
	StationaryFlowCount = 0U;
	FusionOutput.flow_mount_x_cm = FusionConfig.flow_mount_x_cm;
	FusionOutput.flow_mount_y_cm = FusionConfig.flow_mount_y_cm;
	FusionInitialized = 1U;
}

void SensorFusion_ResetPose(void)
{
	FusionOutput.x_cm = 0.0f;
	FusionOutput.y_cm = 0.0f;
	FusionOutput.distance_cm = 0.0f;
	FusionYawRad = 0.0f;
	FusionOutput.yaw_deg = 0.0f;
	FusionYawOriginValid = 0U;
	PreviousImuValid = 0U;
}

static void SensorFusion_ResetFlowHistory(void)
{
	PreviousFlowRightCms = 0.0f;
	PreviousFlowForwardCms = 0.0f;
	PreviousFlowValid = 0U;
	StationaryFlowCount = 0U;
}

void SensorFusion_SetFlowMountPosition(float right_cm, float forward_cm)
{
	if(!SensorFusion_IsFinite(right_cm) || !SensorFusion_IsFinite(forward_cm))
	{
		return;
	}
	if(!FusionInitialized)
	{
		SensorFusion_Init(NULL);
	}

	FusionConfig.flow_mount_x_cm = right_cm;
	FusionConfig.flow_mount_y_cm = forward_cm;
	FusionOutput.flow_mount_x_cm = right_cm;
	FusionOutput.flow_mount_y_cm = forward_cm;
	SensorFusion_ResetFlowHistory();
}

void SensorFusion_EnableFlowLeverCompensation(uint8_t enable)
{
	if(!FusionInitialized)
	{
		SensorFusion_Init(NULL);
	}

	FusionConfig.enable_flow_lever_compensation = enable ? 1U : 0U;
	FusionOutput.compensation_valid = 0U;
	FusionOutput.compensation_omega_rps = 0.0f;
	SensorFusion_ResetFlowHistory();
}

static uint8_t SensorFusion_GetCompensationOmega(const SensorFusionInput_t *input,
	                                              float left_speed_cms,
	                                              float right_speed_cms,
	                                              uint8_t encoder_valid,
	                                              float *omega_rps,
	                                              uint16_t *fault_flags,
	                                              uint8_t *fallback_flags)
{
	float wheel_omega_rps;

	*omega_rps = 0.0f;
	if(!FusionConfig.enable_flow_lever_compensation)
	{
		return 0U;
	}

	if(input->imu_valid && SensorFusion_IsFinite(input->imu_gyro_z_rps) &&
	   SensorFusion_Abs(input->imu_gyro_z_rps) <= FusionConfig.max_gyro_rps)
	{
		*omega_rps = input->imu_gyro_z_rps;
		return 1U;
	}

	if(encoder_valid && SensorFusion_IsFinite(FusionConfig.wheel_track_cm) &&
	   SensorFusion_Abs(FusionConfig.wheel_track_cm) > SENSOR_FUSION_DELTA_EPS_CM)
	{
		wheel_omega_rps = (right_speed_cms - left_speed_cms) / FusionConfig.wheel_track_cm;
		if(SensorFusion_IsFinite(wheel_omega_rps) &&
		   SensorFusion_Abs(wheel_omega_rps) <= FusionConfig.max_gyro_rps)
		{
			*omega_rps = wheel_omega_rps;
			*fallback_flags |= SENSOR_FUSION_FALLBACK_IMU;
			return 1U;
		}
	}

	*fault_flags |= SENSOR_FUSION_FAULT_FLOW_COMP_OMEGA;
	*fallback_flags |= SENSOR_FUSION_FALLBACK_FLOW_COMP;
	return 0U;
}

static float SensorFusion_EvaluateFlow(const SensorFusionInput_t *input,
	                                   float flow_right_cms,
	                                   float flow_forward_cms,
	                                   uint16_t *fault_flags)
{
	float squal_score;
	float shutter_score = 1.0f;
	float max_step;
	uint8_t delta_present;
	uint8_t basic_speed_valid = 0U;

	if(!input->flow_ready)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_NOT_READY;
	}
	if(input->dt_s < FusionConfig.min_dt_s || input->dt_s > FusionConfig.max_dt_s)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_DT;
	}
	if(input->flow_height_cm < FusionConfig.min_height_cm ||
	   input->flow_height_cm > FusionConfig.max_height_cm)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_HEIGHT;
	}
	if(input->flow_squal <= FusionConfig.squal_min)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_QUALITY;
	}
	if(input->flow_shutter >= FusionConfig.shutter_invalid_min)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_SHUTTER;
	}

	delta_present = (SensorFusion_Abs(input->flow_right_cm) > SENSOR_FUSION_DELTA_EPS_CM ||
	                 SensorFusion_Abs(input->flow_forward_cm) > SENSOR_FUSION_DELTA_EPS_CM) ? 1U : 0U;
	if(delta_present && (input->flow_motion & SENSOR_FUSION_MOTION_BIT) == 0U)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_MOTION;
	}

	if(!SensorFusion_IsFinite(flow_right_cms) || !SensorFusion_IsFinite(flow_forward_cms) ||
	   SensorFusion_Abs(flow_right_cms) > FusionConfig.max_flow_speed_cms ||
	   SensorFusion_Abs(flow_forward_cms) > FusionConfig.max_flow_speed_cms)
	{
		*fault_flags |= SENSOR_FUSION_FAULT_FLOW_SPEED;
	}
	else
	{
		basic_speed_valid = 1U;
	}

	if(basic_speed_valid && PreviousFlowValid &&
	   input->dt_s >= FusionConfig.min_dt_s && input->dt_s <= FusionConfig.max_dt_s)
	{
		max_step = FusionConfig.max_flow_accel_cms2 * input->dt_s;
		if(SensorFusion_Abs(flow_right_cms - PreviousFlowRightCms) > max_step ||
		   SensorFusion_Abs(flow_forward_cms - PreviousFlowForwardCms) > max_step)
		{
			*fault_flags |= SENSOR_FUSION_FAULT_FLOW_JUMP;
		}
	}

	if(basic_speed_valid)
	{
		PreviousFlowRightCms = flow_right_cms;
		PreviousFlowForwardCms = flow_forward_cms;
		PreviousFlowValid = 1U;
	}
	else
	{
		PreviousFlowValid = 0U;
	}

	if((*fault_flags & (SENSOR_FUSION_FAULT_FLOW_NOT_READY |
	                    SENSOR_FUSION_FAULT_FLOW_DT |
	                    SENSOR_FUSION_FAULT_FLOW_QUALITY |
	                    SENSOR_FUSION_FAULT_FLOW_MOTION |
	                    SENSOR_FUSION_FAULT_FLOW_SPEED |
	                    SENSOR_FUSION_FAULT_FLOW_JUMP |
	                    SENSOR_FUSION_FAULT_FLOW_HEIGHT |
	                    SENSOR_FUSION_FAULT_FLOW_SHUTTER)) != 0U)
	{
		return 0.0f;
	}

	if(FusionConfig.squal_good <= FusionConfig.squal_min)
	{
		squal_score = 1.0f;
	}
	else
	{
		squal_score = ((float)input->flow_squal - (float)FusionConfig.squal_min) /
		              ((float)FusionConfig.squal_good - (float)FusionConfig.squal_min);
		squal_score = SensorFusion_Clamp(squal_score, 0.0f, 1.0f);
	}

	if(input->flow_shutter > FusionConfig.shutter_good_max &&
	   FusionConfig.shutter_invalid_min > FusionConfig.shutter_good_max)
	{
		shutter_score = ((float)FusionConfig.shutter_invalid_min - (float)input->flow_shutter) /
		                ((float)FusionConfig.shutter_invalid_min - (float)FusionConfig.shutter_good_max);
		shutter_score = SensorFusion_Clamp(shutter_score, 0.0f, 1.0f);
	}

	return squal_score * shutter_score;
}

static float SensorFusion_UpdateYaw(const SensorFusionInput_t *input,
	                                float left_speed_cms,
	                                float right_speed_cms,
	                                float dt_s,
	                                uint16_t *fault_flags,
	                                uint8_t *fallback_flags)
{
	float dtheta;
	float target_yaw_rad;
	float gyro_delta_rad;
	float yaw_gyro_error_deg;
	float mid_yaw_rad;
	uint8_t gyro_valid;

	gyro_valid = (SensorFusion_IsFinite(input->imu_gyro_z_rps) &&
	              SensorFusion_Abs(input->imu_gyro_z_rps) <= FusionConfig.max_gyro_rps) ? 1U : 0U;

	if(input->imu_valid && SensorFusion_IsFinite(input->imu_yaw_deg))
	{
		if(!FusionYawOriginValid || !PreviousImuValid)
		{
			FusionYawOriginDeg = input->imu_yaw_deg - FusionYawRad * SENSOR_FUSION_RAD_TO_DEG;
			FusionYawOriginValid = 1U;
		}

		target_yaw_rad = SensorFusion_WrapDeg(input->imu_yaw_deg - FusionYawOriginDeg) *
		                 SENSOR_FUSION_DEG_TO_RAD;
		dtheta = SensorFusion_WrapRad(target_yaw_rad - FusionYawRad);
		gyro_delta_rad = gyro_valid ? input->imu_gyro_z_rps * dt_s : dtheta;
		yaw_gyro_error_deg = SensorFusion_Abs(SensorFusion_WrapRad(dtheta - gyro_delta_rad)) *
		                     SENSOR_FUSION_RAD_TO_DEG;

		if(SensorFusion_Abs(dtheta) * SENSOR_FUSION_RAD_TO_DEG > FusionConfig.max_imu_yaw_jump_deg ||
		   (gyro_valid && yaw_gyro_error_deg > FusionConfig.max_yaw_gyro_error_deg))
		{
			dtheta = gyro_valid ? gyro_delta_rad : 0.0f;
			*fault_flags |= SENSOR_FUSION_FAULT_IMU_YAW_JUMP;
			*fallback_flags |= SENSOR_FUSION_FALLBACK_IMU;
		}
		PreviousImuValid = 1U;
	}
	else
	{
		dtheta = (right_speed_cms - left_speed_cms) / FusionConfig.wheel_track_cm * dt_s;
		if((left_speed_cms + right_speed_cms) < 0.0f)
		{
			dtheta = -dtheta;
		}
		*fault_flags |= SENSOR_FUSION_FAULT_IMU_INVALID;
		*fallback_flags |= SENSOR_FUSION_FALLBACK_IMU;
		PreviousImuValid = 0U;
	}

	mid_yaw_rad = SensorFusion_WrapRad(FusionYawRad + 0.5f * dtheta);
	FusionYawRad = SensorFusion_WrapRad(FusionYawRad + dtheta);
	return mid_yaw_rad;
}

void SensorFusion_Update(const SensorFusionInput_t *input)
{
	float dt_s;
	float left_speed_cms;
	float right_speed_cms;
	float encoder_forward_cms;
	float raw_flow_right_cms = 0.0f;
	float raw_flow_forward_cms = 0.0f;
	float compensated_flow_right_cms = 0.0f;
	float compensated_flow_forward_cms = 0.0f;
	float fusion_flow_right_cms = 0.0f;
	float fusion_flow_forward_cms = 0.0f;
	float compensation_omega_rps = 0.0f;
	float flow_quality;
	float target_weight;
	float flow_speed_magnitude;
	float mid_yaw_rad;
	float sin_yaw;
	float cos_yaw;
	float distance_speed;
	uint16_t fault_flags = SENSOR_FUSION_FAULT_NONE;
	uint8_t fallback_flags = SENSOR_FUSION_FALLBACK_NONE;
	uint8_t flow_valid;
	uint8_t dt_valid;
	uint8_t encoder_valid = 1U;
	uint8_t compensation_omega_valid;
	uint8_t compensation_valid = 0U;

	if(input == NULL)
	{
		return;
	}
	if(!FusionInitialized)
	{
		SensorFusion_Init(NULL);
	}

	dt_s = input->dt_s;
	dt_valid = (dt_s >= FusionConfig.min_dt_s && dt_s <= FusionConfig.max_dt_s) ? 1U : 0U;
	left_speed_cms = input->left_speed_cms;
	right_speed_cms = input->right_speed_cms;

	if(!SensorFusion_IsFinite(left_speed_cms))
	{
		left_speed_cms = 0.0f;
		encoder_valid = 0U;
		fault_flags |= SENSOR_FUSION_FAULT_ENCODER_RANGE;
	}
	else if(SensorFusion_Abs(left_speed_cms) > FusionConfig.max_encoder_speed_cms)
	{
		encoder_valid = 0U;
		left_speed_cms = SensorFusion_Clamp(left_speed_cms,
		                                  -FusionConfig.max_encoder_speed_cms,
		                                   FusionConfig.max_encoder_speed_cms);
		fault_flags |= SENSOR_FUSION_FAULT_ENCODER_RANGE;
	}
	if(!SensorFusion_IsFinite(right_speed_cms))
	{
		right_speed_cms = 0.0f;
		encoder_valid = 0U;
		fault_flags |= SENSOR_FUSION_FAULT_ENCODER_RANGE;
	}
	else if(SensorFusion_Abs(right_speed_cms) > FusionConfig.max_encoder_speed_cms)
	{
		encoder_valid = 0U;
		right_speed_cms = SensorFusion_Clamp(right_speed_cms,
		                                   -FusionConfig.max_encoder_speed_cms,
		                                    FusionConfig.max_encoder_speed_cms);
		fault_flags |= SENSOR_FUSION_FAULT_ENCODER_RANGE;
	}
	encoder_forward_cms = 0.5f * (left_speed_cms + right_speed_cms);

	if(dt_valid)
	{
		raw_flow_right_cms = input->flow_right_cm / dt_s;
		raw_flow_forward_cms = input->flow_forward_cm / dt_s;
	}
	compensated_flow_right_cms = raw_flow_right_cms;
	compensated_flow_forward_cms = raw_flow_forward_cms;
	compensation_omega_valid = SensorFusion_GetCompensationOmega(input,
	                                                            left_speed_cms,
	                                                            right_speed_cms,
	                                                            encoder_valid,
	                                                            &compensation_omega_rps,
	                                                            &fault_flags,
	                                                            &fallback_flags);
	if(dt_valid && input->flow_ready && FusionConfig.enable_flow_lever_compensation &&
	   compensation_omega_valid)
	{
		/* x is right, y is forward, and positive omega is a left turn. */
		compensated_flow_right_cms += compensation_omega_rps * FusionConfig.flow_mount_y_cm;
		compensated_flow_forward_cms -= compensation_omega_rps * FusionConfig.flow_mount_x_cm;
		compensation_valid = 1U;
	}
	flow_quality = SensorFusion_EvaluateFlow(input,
	                                         compensated_flow_right_cms,
	                                         compensated_flow_forward_cms,
	                                         &fault_flags);
	flow_valid = flow_quality > 0.0f ? 1U : 0U;
	fusion_flow_right_cms = compensated_flow_right_cms;
	fusion_flow_forward_cms = compensated_flow_forward_cms;

	if(flow_valid)
	{
		flow_speed_magnitude = sqrtf(compensated_flow_right_cms * compensated_flow_right_cms +
		                             compensated_flow_forward_cms * compensated_flow_forward_cms);
		if(flow_speed_magnitude <= FusionConfig.stationary_flow_speed_cms)
		{
			if(StationaryFlowCount < 255U) StationaryFlowCount++;
		}
		else
		{
			StationaryFlowCount = 0U;
		}

		target_weight = flow_quality * FusionConfig.max_flow_weight;
		if((fault_flags & SENSOR_FUSION_FAULT_ENCODER_RANGE) != 0U)
		{
			target_weight = FusionConfig.max_flow_weight;
		}
		if(StationaryFlowCount >= FusionConfig.stationary_confirm_samples)
		{
			fusion_flow_right_cms = 0.0f;
			fusion_flow_forward_cms = 0.0f;
			FusionOutput.flow_weight = 1.0f;
		}
		else
		{
			FusionOutput.flow_weight += FusionConfig.flow_weight_alpha *
			                                  (target_weight - FusionOutput.flow_weight);
		}
	}
	else
	{
		StationaryFlowCount = 0U;
		FusionOutput.flow_weight = 0.0f;
		fallback_flags |= SENSOR_FUSION_FALLBACK_FLOW;
	}

	FusionOutput.flow_weight = SensorFusion_Clamp(FusionOutput.flow_weight, 0.0f, 1.0f);
	FusionOutput.fused_forward_cms =
		(1.0f - FusionOutput.flow_weight) * encoder_forward_cms +
		FusionOutput.flow_weight * fusion_flow_forward_cms;
	FusionOutput.fused_right_cms = FusionOutput.flow_weight * fusion_flow_right_cms;

	if(dt_valid)
	{
		mid_yaw_rad = SensorFusion_UpdateYaw(input, left_speed_cms, right_speed_cms,
		                                      dt_s, &fault_flags, &fallback_flags);
		cos_yaw = cosf(mid_yaw_rad);
		sin_yaw = sinf(mid_yaw_rad);
		FusionOutput.world_vx_cms = FusionOutput.fused_right_cms * cos_yaw -
		                            FusionOutput.fused_forward_cms * sin_yaw;
		FusionOutput.world_vy_cms = FusionOutput.fused_right_cms * sin_yaw +
		                            FusionOutput.fused_forward_cms * cos_yaw;
		FusionOutput.x_cm += FusionOutput.world_vx_cms * dt_s;
		FusionOutput.y_cm += FusionOutput.world_vy_cms * dt_s;
		distance_speed = sqrtf(FusionOutput.fused_right_cms * FusionOutput.fused_right_cms +
		                       FusionOutput.fused_forward_cms * FusionOutput.fused_forward_cms);
		FusionOutput.distance_cm += distance_speed * dt_s;
	}
	else
	{
		FusionOutput.world_vx_cms = 0.0f;
		FusionOutput.world_vy_cms = 0.0f;
		fault_flags |= SENSOR_FUSION_FAULT_FLOW_DT;
		fallback_flags |= SENSOR_FUSION_FALLBACK_FLOW;
	}

	FusionOutput.timestamp_ms = input->timestamp_ms;
	FusionOutput.dt_s = input->dt_s;
	FusionOutput.encoder_forward_cms = encoder_forward_cms;
	FusionOutput.raw_flow_right_cms = raw_flow_right_cms;
	FusionOutput.raw_flow_forward_cms = raw_flow_forward_cms;
	FusionOutput.compensated_flow_right_cms = compensated_flow_right_cms;
	FusionOutput.compensated_flow_forward_cms = compensated_flow_forward_cms;
	FusionOutput.compensation_omega_rps = compensation_omega_rps;
	FusionOutput.flow_mount_x_cm = FusionConfig.flow_mount_x_cm;
	FusionOutput.flow_mount_y_cm = FusionConfig.flow_mount_y_cm;
	FusionOutput.flow_right_cms = fusion_flow_right_cms;
	FusionOutput.flow_forward_cms = fusion_flow_forward_cms;
	FusionOutput.yaw_deg = FusionYawRad * SENSOR_FUSION_RAD_TO_DEG;
	FusionOutput.flow_quality = flow_quality;
	FusionOutput.flow_valid = flow_valid;
	FusionOutput.fault_flags = fault_flags;
	FusionOutput.fallback_flags = fallback_flags;
	FusionOutput.compensation_valid = compensation_valid;
}

void SensorFusion_GetOutput(SensorFusionOutput_t *output)
{
	if(output != NULL)
	{
		*output = FusionOutput;
	}
}
