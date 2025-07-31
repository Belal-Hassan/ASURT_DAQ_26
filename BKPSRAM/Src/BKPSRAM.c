/*
 * BKPSRAM.c
 *
 *  Created on: Jul 29, 2025
 *      Author: Belal
 */

#include "BKPSRAM.h"
#include "BKPSRAM_Private.h"

void BKPSRAM_Init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_BKPSRAM_CLK_ENABLE();
}

void BKPSRAM_Write(uint32_t data)
{
	//BKPSRAM_BASE_ADDR[0] = data;
}
