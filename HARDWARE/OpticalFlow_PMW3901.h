#ifndef _OPTICAL_FLOW_PMW3901_H_
#define _OPTICAL_FLOW_PMW3901_H_

#include <stdint.h>

#define PMW3901_EXPECTED_PRODUCT_ID          0x49U
#define PMW3901_EXPECTED_INVERSE_PRODUCT_ID  0xB6U
#define PMW3901_RESOLUTION_CM_PER_PIXEL_1M   0.2131946f
#define PMW3901_DEFAULT_HEIGHT_CM            3.8f

typedef struct PMW3901_MotionBurst
{
	uint8_t motion;
	uint8_t observation;
	int16_t delta_x;
	int16_t delta_y;
	uint8_t squal;
	uint8_t raw_data_sum;
	uint8_t max_raw_data;
	uint8_t min_raw_data;
	uint16_t shutter;
} PMW3901_MotionBurst_t;

uint8_t OpticalFlow_PMW3901_Init(void);
uint8_t OpticalFlow_PMW3901_IsReady(void);
uint8_t OpticalFlow_PMW3901_ReadMotion(PMW3901_MotionBurst_t *motion);
uint8_t OpticalFlow_PMW3901_ConvertDeltaToCm(const PMW3901_MotionBurst_t *motion,
	                                           float *delta_x_cm,
	                                           float *delta_y_cm);
void OpticalFlow_PMW3901_SetHeightCm(float height_cm);
float OpticalFlow_PMW3901_GetHeightCm(void);
uint8_t OpticalFlow_PMW3901_GetProductId(void);
uint8_t OpticalFlow_PMW3901_GetInverseProductId(void);

#endif
