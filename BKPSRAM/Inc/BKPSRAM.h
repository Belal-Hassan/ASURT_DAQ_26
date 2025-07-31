/*
 * BKPSRAM.h
 *
 *  Created on: Jul 29, 2025
 *      Author: Belal
 */

#ifndef BKPSRAM_INC_BKPSRAM_H_
#define BKPSRAM_INC_BKPSRAM_H_

#include "main.h"

typedef struct {
	uint32_t data;
	uint32_t position;
}bkpsram_data_t;

void BKPSRAM_Init(void);
void BKPSRAM_Write(uint32_t data);


#endif /* BKPSRAM_INC_BKPSRAM_H_ */
