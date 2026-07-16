#include "OpticalFlow_PMW3901.h"
#include "PMW3901_SoftSPI.h"
#include "generic.h"

#include <stddef.h>

#define PMW3901_REG_PRODUCT_ID           0x00U
#define PMW3901_REG_MOTION               0x02U
#define PMW3901_REG_DELTA_X_L            0x03U
#define PMW3901_REG_DELTA_X_H            0x04U
#define PMW3901_REG_DELTA_Y_L            0x05U
#define PMW3901_REG_DELTA_Y_H            0x06U
#define PMW3901_REG_MOTION_BURST         0x16U
#define PMW3901_REG_POWER_UP_RESET       0x3AU
#define PMW3901_REG_INVERSE_PRODUCT_ID   0x5FU

#define PMW3901_SPI_READ_DELAY_US        40U
#define PMW3901_SPI_WRITE_GAP_US         50U
#define PMW3901_SPI_READ_GAP_US          25U
#define PMW3901_ID_READ_RETRY_COUNT      3U

typedef struct PMW3901_RegInit
{
	uint8_t reg;
	uint8_t value;
	uint16_t delay_ms_after;
} PMW3901_RegInit_t;

static uint8_t Pmw3901Ready = 0;
static uint8_t Pmw3901ProductId = 0;
static uint8_t Pmw3901InverseProductId = 0;
static float Pmw3901HeightCm = PMW3901_DEFAULT_HEIGHT_CM;

static uint8_t PMW3901_ReadReg(uint8_t reg);

static uint8_t PMW3901_ReadProductIdWithRetry(uint8_t reg)
{
	uint8_t i;
	uint8_t value = 0U;

	for(i = 0U; i < PMW3901_ID_READ_RETRY_COUNT; i++)
	{
		value = PMW3901_ReadReg(reg);
		if(value != 0x00U && value != 0xFFU)
		{
			break;
		}
		Delay_ms(2U);
	}
	return value;
}

static void PMW3901_WriteReg(uint8_t reg, uint8_t value)
{
	PMW3901_SoftSPI_Select();
	PMW3901_SoftSPI_Transfer(reg | 0x80U);
	PMW3901_SoftSPI_Transfer(value);
	PMW3901_SoftSPI_Deselect();
	Delay_us(PMW3901_SPI_WRITE_GAP_US);
}

static uint8_t PMW3901_ReadReg(uint8_t reg)
{
	uint8_t value;

	PMW3901_SoftSPI_Select();
	PMW3901_SoftSPI_Transfer(reg & 0x7FU);
	Delay_us(PMW3901_SPI_READ_DELAY_US);
	value = PMW3901_SoftSPI_Transfer(0x00U);
	PMW3901_SoftSPI_Deselect();
	Delay_us(PMW3901_SPI_READ_GAP_US);

	return value;
}

static void PMW3901_ApplyPerformanceOptimization(void)
{
	uint16_t i;
	static const PMW3901_RegInit_t initSequence[] =
	{
		{0x7FU, 0x00U, 0U},
		{0x61U, 0xADU, 0U},
		{0x7FU, 0x03U, 0U},
		{0x40U, 0x00U, 0U},
		{0x7FU, 0x05U, 0U},
		{0x41U, 0xB3U, 0U},
		{0x43U, 0xF1U, 0U},
		{0x45U, 0x14U, 0U},
		{0x5BU, 0x32U, 0U},
		{0x5FU, 0x34U, 0U},
		{0x7BU, 0x08U, 0U},
		{0x7FU, 0x06U, 0U},
		{0x44U, 0x1BU, 0U},
		{0x40U, 0xBFU, 0U},
		{0x4EU, 0x3FU, 0U},
		{0x7FU, 0x08U, 0U},
		{0x65U, 0x20U, 0U},
		{0x6AU, 0x18U, 0U},
		{0x7FU, 0x09U, 0U},
		{0x4FU, 0xAFU, 0U},
		{0x5FU, 0x40U, 0U},
		{0x48U, 0x80U, 0U},
		{0x49U, 0x80U, 0U},
		{0x57U, 0x77U, 0U},
		{0x60U, 0x78U, 0U},
		{0x61U, 0x78U, 0U},
		{0x62U, 0x08U, 0U},
		{0x63U, 0x50U, 0U},
		{0x7FU, 0x0AU, 0U},
		{0x45U, 0x60U, 0U},
		{0x7FU, 0x00U, 0U},
		{0x4DU, 0x11U, 0U},
		{0x55U, 0x80U, 0U},
		{0x74U, 0x1FU, 0U},
		{0x75U, 0x1FU, 0U},
		{0x4AU, 0x78U, 0U},
		{0x4BU, 0x78U, 0U},
		{0x44U, 0x08U, 0U},
		{0x45U, 0x50U, 0U},
		{0x64U, 0xFFU, 0U},
		{0x65U, 0x1FU, 0U},
		{0x7FU, 0x14U, 0U},
		{0x65U, 0x67U, 0U},
		{0x66U, 0x08U, 0U},
		{0x63U, 0x70U, 0U},
		{0x7FU, 0x15U, 0U},
		{0x48U, 0x48U, 0U},
		{0x7FU, 0x07U, 0U},
		{0x41U, 0x0DU, 0U},
		{0x43U, 0x14U, 0U},
		{0x4BU, 0x0EU, 0U},
		{0x45U, 0x0FU, 0U},
		{0x44U, 0x42U, 0U},
		{0x4CU, 0x80U, 0U},
		{0x7FU, 0x10U, 0U},
		{0x5BU, 0x02U, 0U},
		{0x7FU, 0x07U, 0U},
		{0x40U, 0x41U, 0U},
		{0x70U, 0x00U, 10U},
		{0x32U, 0x44U, 0U},
		{0x7FU, 0x07U, 0U},
		{0x40U, 0x40U, 0U},
		{0x7FU, 0x06U, 0U},
		{0x62U, 0xF0U, 0U},
		{0x63U, 0x00U, 0U},
		{0x7FU, 0x0DU, 0U},
		{0x48U, 0xC0U, 0U},
		{0x6FU, 0xD5U, 0U},
		{0x7FU, 0x00U, 0U},
		{0x5BU, 0xA0U, 0U},
		{0x4EU, 0xA8U, 0U},
		{0x5AU, 0x50U, 0U},
		{0x40U, 0x80U, 240U},
		{0x7FU, 0x14U, 0U},
		{0x6FU, 0x1CU, 0U},
		{0x7FU, 0x00U, 0U},
		{0x5BU, 0xA0U, 0U},
		{0x4EU, 0xA8U, 0U},
		{0x5AU, 0x50U, 0U},
		{0x40U, 0x00U, 0U}
	};

	for(i = 0; i < (uint16_t)(sizeof(initSequence) / sizeof(initSequence[0])); i++)
	{
		PMW3901_WriteReg(initSequence[i].reg, initSequence[i].value);
		if(initSequence[i].delay_ms_after != 0U)
		{
			Delay_ms(initSequence[i].delay_ms_after);
		}
	}
}

uint8_t OpticalFlow_PMW3901_Init(void)
{
	Pmw3901Ready = 0;
	Pmw3901ProductId = 0;
	Pmw3901InverseProductId = 0;

	PMW3901_SoftSPI_Init();
	Delay_ms(50U);

	PMW3901_SoftSPI_Deselect();
	Delay_us(5U);
	PMW3901_SoftSPI_Select();
	Delay_us(5U);
	PMW3901_SoftSPI_Deselect();
	Delay_us(5U);

	Pmw3901ProductId = PMW3901_ReadProductIdWithRetry(PMW3901_REG_PRODUCT_ID);
	Pmw3901InverseProductId = PMW3901_ReadProductIdWithRetry(PMW3901_REG_INVERSE_PRODUCT_ID);

	PMW3901_WriteReg(PMW3901_REG_POWER_UP_RESET, 0x5AU);
	Delay_ms(5U);

	(void)PMW3901_ReadReg(PMW3901_REG_MOTION);
	(void)PMW3901_ReadReg(PMW3901_REG_DELTA_X_L);
	(void)PMW3901_ReadReg(PMW3901_REG_DELTA_X_H);
	(void)PMW3901_ReadReg(PMW3901_REG_DELTA_Y_L);
	(void)PMW3901_ReadReg(PMW3901_REG_DELTA_Y_H);

	PMW3901_ApplyPerformanceOptimization();

	Pmw3901ProductId = PMW3901_ReadProductIdWithRetry(PMW3901_REG_PRODUCT_ID);
	Pmw3901InverseProductId = PMW3901_ReadProductIdWithRetry(PMW3901_REG_INVERSE_PRODUCT_ID);
	if(Pmw3901ProductId == PMW3901_EXPECTED_PRODUCT_ID &&
	   Pmw3901InverseProductId == PMW3901_EXPECTED_INVERSE_PRODUCT_ID)
	{
		Pmw3901Ready = 1;
	}

	return Pmw3901Ready;
}

uint8_t OpticalFlow_PMW3901_IsReady(void)
{
	return Pmw3901Ready;
}

uint8_t OpticalFlow_PMW3901_ReadMotion(PMW3901_MotionBurst_t *motion)
{
	uint8_t data[12];
	uint8_t i;

	if(motion == NULL || Pmw3901Ready == 0U)
	{
		return 0U;
	}

	PMW3901_SoftSPI_Select();
	PMW3901_SoftSPI_Transfer(PMW3901_REG_MOTION_BURST & 0x7FU);
	Delay_us(PMW3901_SPI_READ_DELAY_US);
	for(i = 0; i < (uint8_t)sizeof(data); i++)
	{
		data[i] = PMW3901_SoftSPI_Transfer(0x00U);
	}
	PMW3901_SoftSPI_Deselect();
	Delay_us(1U);

	motion->motion = data[0];
	motion->observation = data[1];
	motion->delta_x = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
	motion->delta_y = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
	motion->squal = data[6];
	motion->raw_data_sum = data[7];
	motion->max_raw_data = data[8];
	motion->min_raw_data = data[9];
	motion->shutter = (uint16_t)data[10] | ((uint16_t)data[11] << 8);

	return 1U;
}

uint8_t OpticalFlow_PMW3901_ConvertDeltaToCm(const PMW3901_MotionBurst_t *motion,
	                                           float *delta_x_cm,
	                                           float *delta_y_cm)
{
	float cm_per_pixel;

	if(motion == NULL || delta_x_cm == NULL || delta_y_cm == NULL || Pmw3901HeightCm <= 0.0f)
	{
		return 0U;
	}

	/* The manual's 0.2131946 cm/pixel value is specified at 1 m. */
	cm_per_pixel = PMW3901_RESOLUTION_CM_PER_PIXEL_1M * (Pmw3901HeightCm / 100.0f);
	*delta_x_cm = (float)motion->delta_x * cm_per_pixel;
	*delta_y_cm = (float)motion->delta_y * cm_per_pixel;

	return 1U;
}

void OpticalFlow_PMW3901_SetHeightCm(float height_cm)
{
	if(height_cm > 0.0f)
	{
		Pmw3901HeightCm = height_cm;
	}
}

float OpticalFlow_PMW3901_GetHeightCm(void)
{
	return Pmw3901HeightCm;
}

uint8_t OpticalFlow_PMW3901_GetProductId(void)
{
	return Pmw3901ProductId;
}

uint8_t OpticalFlow_PMW3901_GetInverseProductId(void)
{
	return Pmw3901InverseProductId;
}
