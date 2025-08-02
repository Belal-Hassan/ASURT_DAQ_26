/*
 * DAQ.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */
#include "wwdg.h"
#include "DAQ.h"

/* =========================================== PRIVATE VARIABLES =========================================== */
QueueHandle_t daq_can_queue;
CAN_HandleTypeDef daq_can_handle;
uint32_t tx_mailbox = 0;
CAN_TxHeaderTypeDef* ptr_tx_header;
uint8_t queue_size;
extern TaskHandle_t task_handles[DAQ_NO_OF_TASKS];
extern daq_task_entry_count_t g_task_entry_count;
/* =========================================== PRIVATE VARIABLES END =========================================== */
bool DAQ_Tasks_Valid(BaseType_t returns[DAQ_NO_OF_TASKS])
{
	for(uint8_t i = 0; i < DAQ_NO_OF_TASKS; i++)
		if(!returns[i])
			return false;
	return true;
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
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
		       |  SCB_SHCSR_BUSFAULTENA_Msk
		       |  SCB_SHCSR_USGFAULTENA_Msk;
	*((uint32_t*)DAQ_BKPSRAM_BASE_ADDR) = DAQ_FAULT_LOGGING_INITIALIZED;
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
	uint32_t sram_status[2] = {0,0};
	memcpy(&log.prev, (uint32_t*)(DAQ_BKPSRAM_BASE_ADDR + 2 * sizeof(uint32_t) + sizeof(fault_log_t)), sizeof(fault_log_t));
	memcpy(sram_status, (uint32_t*)DAQ_BKPSRAM_BASE_ADDR, sizeof(sram_status));
	if(sram_status[DAQ_BKPSRAM_INIT_WORD] == DAQ_FAULT_LOGGING_INITIALIZED && sram_status[DAQ_BKPSRAM_STATE_WORD] == DAQ_FAULT_LOGGED)
	{
		memcpy(&log.current, (uint32_t*)(DAQ_BKPSRAM_BASE_ADDR + 2 * sizeof(uint32_t)), sizeof(fault_log_t));
		memset((uint32_t*)(DAQ_BKPSRAM_BASE_ADDR + 2 * sizeof(uint32_t)), 0, sizeof(fault_log_t));
		*((uint32_t*)(DAQ_BKPSRAM_BASE_ADDR + sizeof(uint32_t))) = DAQ_FAULT_READ;
	}
	return log;
}
void DAQ_FaultLog_Write(fault_log_t log)
{
	uint32_t sram_status[2] = {0,0};
	memcpy(sram_status, (uint32_t*)DAQ_BKPSRAM_BASE_ADDR, sizeof(sram_status));
	if(sram_status[DAQ_BKPSRAM_INIT_WORD] == DAQ_FAULT_LOGGING_INITIALIZED)
	{
		memcpy((uint32_t*)(DAQ_BKPSRAM_BASE_ADDR + 2 * sizeof(uint32_t)), &log, sizeof(fault_log_t));
		memcpy((uint32_t*)(DAQ_BKPSRAM_BASE_ADDR + 2 * sizeof(uint32_t) + sizeof(fault_log_t)), &log, sizeof(fault_log_t));
		*((uint32_t*)(DAQ_BKPSRAM_BASE_ADDR)) = DAQ_FAULT_LOGGING_UNINITIALIZED;
		*((uint32_t*)(DAQ_BKPSRAM_BASE_ADDR +  sizeof(uint32_t))) = DAQ_FAULT_LOGGED;
	}
}

void DAQ_CAN_Task(void *pvParameters)
{
	daq_can_msg_t can_msg = {0};
	while (1)
	{
		vTaskDelay(1);
		queue_size = uxQueueMessagesWaiting(daq_can_queue);
		if(queue_size)
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
		g_task_entry_count.can++;
	}
}
