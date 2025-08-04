/*
 * DAQ.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx.h"
#include "wwdg.h"
#include "DAQ.h"

/* =========================================== PRIVATE VARIABLES =========================================== */
QueueHandle_t daq_can_queue;
CAN_HandleTypeDef daq_can_handle;
uint32_t tx_mailbox = 0;
CAN_TxHeaderTypeDef* ptr_tx_header;
uint8_t queue_size;
extern TaskHandle_t task_handles[DAQ_NO_OF_TASKS];
extern daq_fault_record_t g_fault_record;
/* =========================================== PRIVATE VARIABLES END =========================================== */
void DAQ_BKPSRAM_Read(void* readTo, daq_bkpsram_read_type_t type)
{
	switch(type)
	{
		case DAQ_READ_PREVIOUS_LOG:
			memcpy(readTo, (void*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_word_t) + sizeof(fault_log_t)), sizeof(fault_log_t));
			break;
		case DAQ_READ_CURRENT_LOG:
			memcpy(readTo, (void*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_word_t)), sizeof(fault_log_t));
			break;
		case DAQ_READ_STATUS_WORDS:
			memcpy(readTo, (void*)DAQ_BKPSRAM_BASE_ADDR, sizeof(daq_status_word_t));
			break;
		default:
			break;
	}
}
void DAQ_BKPSRAM_Write(void* toWrite, daq_bkpsram_write_type_t type)
{
	daq_status_word_t* status = (daq_status_word_t*) toWrite;
	switch(type)
	{
		case DAQ_WRITE_LOG:
			memcpy((void*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_word_t)), toWrite, sizeof(fault_log_t));
			memcpy((void*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_word_t) + sizeof(fault_log_t)), toWrite, sizeof(fault_log_t));
			break;
		case DAQ_CLEAR_CURRENT_LOG:
			memset((void*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_status_word_t)), 0, sizeof(fault_log_t));
			break;
		case DAQ_WRITE_BKPSRAM_STATE:
			memcpy((void*)(DAQ_BKPSRAM_BASE_ADDR), &(status->bkpsram_state), sizeof(daq_bkpsram_state_t));
			break;
		case DAQ_WRITE_LOG_STATUS:
			memcpy((void*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(daq_bkpsram_state_t)), &(status->log_status), sizeof(daq_log_status_t));
			break;
		default:
			break;
	}
}
void DAQ_CAN_Init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header)
{
	daq_can_queue = xQueueCreate(10, sizeof(daq_can_msg_t));
    daq_can_handle = *can_handle;
    can_tx_header->IDE = CAN_ID_STD;
    can_tx_header->RTR = CAN_RTR_DATA;
    can_tx_header->TransmitGlobalTime = DISABLE;
    ptr_tx_header = can_tx_header;
}
void DAQ_FaultLog_Init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_BKPSRAM_CLK_ENABLE();
	LL_PWR_EnableBkUpRegulator();
	while ((PWR->CSR & PWR_CSR_BRR) == 0);
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
		       |  SCB_SHCSR_BUSFAULTENA_Msk
		       |  SCB_SHCSR_USGFAULTENA_Msk;
	daq_status_word_t status = {.bkpsram_state = DAQ_BKPSRAM_INITIALIZED};
	DAQ_BKPSRAM_Write(&status, DAQ_WRITE_BKPSRAM_STATE);
}

void DAQ_CAN_Msg_Enqueue(daq_can_msg_t* message)
{
    xQueueSend(daq_can_queue, message, 2);
}
daq_can_msg_t DAQ_CAN_Msg_Dequeue(void)
{
	daq_can_msg_t message = {};
    xQueueReceive(daq_can_queue, &message, portMAX_DELAY);
    return message;
}
daq_fault_log_t DAQ_FaultLog_Read(void)
{
	daq_fault_log_t log = {0};
	daq_status_word_t status = {0};
	DAQ_BKPSRAM_Read(&log.prev, DAQ_READ_PREVIOUS_LOG);
	DAQ_BKPSRAM_Read(&status, DAQ_READ_STATUS_WORDS);
	if(status.bkpsram_state == DAQ_BKPSRAM_INITIALIZED && status.log_status == DAQ_FAULT_LOGGED)
	{
		DAQ_BKPSRAM_Read(&log.current, DAQ_READ_CURRENT_LOG);
		DAQ_BKPSRAM_Write(NULL, DAQ_CLEAR_CURRENT_LOG);
		status.log_status = DAQ_FAULT_READ;
		DAQ_BKPSRAM_Write(&status, DAQ_WRITE_LOG_STATUS);
	}
	return log;
}
void DAQ_FaultLog_Write(fault_log_t log)
{
	daq_status_word_t status = {0};
	DAQ_BKPSRAM_Read(&status, DAQ_READ_STATUS_WORDS);
	if(status.bkpsram_state == DAQ_BKPSRAM_INITIALIZED)
	{
		DAQ_BKPSRAM_Write(&log, DAQ_WRITE_LOG);
		status.bkpsram_state = DAQ_BKPSRAM_UNINITIALIZED;
		status.log_status = DAQ_FAULT_LOGGED;
		DAQ_BKPSRAM_Write(&status, DAQ_WRITE_BKPSRAM_STATE);
		DAQ_BKPSRAM_Write(&status, DAQ_WRITE_LOG_STATUS);
	}
}

void DAQ_CAN_Task(void *pvParameters)
{
	daq_can_msg_t can_msg = {0};
	while (1)
	{
		vTaskDelay(4);
		queue_size = uxQueueMessagesWaiting(daq_can_queue);
		while(queue_size--)
		{
			can_msg = DAQ_CAN_Msg_Dequeue();
			ptr_tx_header->DLC = can_msg.size;
			ptr_tx_header->StdId = can_msg.id;
			//if(HAL_CAN_GetTxMailboxesFreeLevel(&daq_can_handle) > 0)
			{
				if (HAL_CAN_AddTxMessage(&daq_can_handle, ptr_tx_header, &can_msg.data, &tx_mailbox) == HAL_ERROR)
				{
					//Error_Handler();
				}
			}
		}
	}
}
