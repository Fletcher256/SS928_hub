#ifndef _PMW3901_SOFTSPI_H_
#define _PMW3901_SOFTSPI_H_

#include <stdint.h>

void PMW3901_SoftSPI_Init(void);
void PMW3901_SoftSPI_Select(void);
void PMW3901_SoftSPI_Deselect(void);
uint8_t PMW3901_SoftSPI_Transfer(uint8_t txData);
uint8_t PMW3901_SoftSPI_ReadMisoLevel(void);

#endif
