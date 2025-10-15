/*
 * @file DAQ.c
 * @brief ff
 *
 *
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx.h"
#include "wwdg.h"
#include "DAQ.h"
#include "DAQ_BKPSRAM_Private.h"

/* =========================================== PRIVATE VARIABLES =========================================== */
QueueHandle_t daq_can_queue;
CAN_HandleTypeDef daq_can_handle;
uint32_t tx_mailbox = 0;
CAN_TxHeaderTypeDef* ptr_tx_header;
uint8_t queue_size;
daq_fault_record_t g_daq_fault_record;
extern daq_timestamp_t g_timestamp;
extern TaskHandle_t task_handles[DAQ_NO_OF_TASKS];
/* =========================================== PRIVATE VARIABLES END =========================================== */
/** @addtogroup Fault_Module
 *  @{
 */

/**
 * @brief Reads data from the backup SRAM according to the chosen read type.
 * @param readTo Pointer to the variable in which the read data will be stored.
 * @param type Read operation type.
 */
void DAQ_BKPSRAM_Read(void* readTo, daq_bkpsram_read_type_t type)
{
	switch(type)
	{
		case DAQ_READ_PREVIOUS_LOG:
			memcpy(readTo, (void*)DAQ_BKPSRAM_PREVIOUS_LOG_ADDR, sizeof(fault_log_t));
			break;
		case DAQ_READ_CURRENT_LOG:
			memcpy(readTo, (void*)DAQ_BKPSRAM_CURRENT_LOG_ADDR, sizeof(fault_log_t));
			break;
		case DAQ_READ_STATUS_WORDS:
			memcpy(readTo, (void*)DAQ_BKPSRAM_BASE_ADDR, sizeof(daq_status_words_t));
			break;
		default:
			break;
	}
}
/**
 * @brief Writes data to the backup SRAM according to the chosen write type.
 * @param toWrite Pointer to the data to be written on the backup SRAM.
 * @param type Write operation type.
 */
void DAQ_BKPSRAM_Write(void* toWrite, daq_bkpsram_write_type_t type)
{
	daq_status_words_t* status = (daq_status_words_t*) toWrite;
	switch(type)
	{
		case DAQ_WRITE_LOG:
			memcpy((void*)DAQ_BKPSRAM_CURRENT_LOG_ADDR, toWrite, sizeof(fault_log_t));
			memcpy((void*)DAQ_BKPSRAM_PREVIOUS_LOG_ADDR, toWrite, sizeof(fault_log_t));
			break;
		case DAQ_CLEAR_CURRENT_LOG:
			memset((void*)DAQ_BKPSRAM_CURRENT_LOG_ADDR, 0, sizeof(fault_log_t));
			break;
		case DAQ_WRITE_BKPSRAM_STATE:
			memcpy((void*)DAQ_BKPSRAM_STATE_ADDR, &(status->bkpsram_state), sizeof(daq_bkpsram_state_t));
			break;
		case DAQ_WRITE_LOG_STATUS:
			memcpy((void*)DAQ_BKPSRAM_LOG_STATUS_ADDR, &(status->log_status), sizeof(daq_log_status_t));
			break;
		default:
			break;
	}
}
/** @} */
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
	while ((PWR->CSR & PWR_CSR_BRR) == 0); // Waits until the regulator works.
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk
		       |  SCB_SHCSR_BUSFAULTENA_Msk
		       |  SCB_SHCSR_USGFAULTENA_Msk; // Enables the fault handlers (Hard fault isn't there because it's enabled by default).
	daq_status_words_t status = {.bkpsram_state = DAQ_BKPSRAM_INITIALIZED};
	DAQ_BKPSRAM_Write(&status, DAQ_WRITE_BKPSRAM_STATE); // Writes 'DAQ_BKPSRAM_INITIALIZED' to the SRAM's state word.
}

void DAQ_CAN_Msg_Enqueue(daq_can_msg_t* message)
{
    xQueueSend(daq_can_queue, message, 2);
}
BaseType_t DAQ_CAN_Msg_Dequeue(daq_can_msg_t* msg)
{
    return xQueueReceive(daq_can_queue, msg, DAQ_CAN_MAX_WAIT_TICKS);
}
void DAQ_FaultLog_Read(daq_fault_log_buffer_t* log)
{
	daq_status_words_t status = {0};
	DAQ_BKPSRAM_Read(&(log->prev), DAQ_READ_PREVIOUS_LOG);
	DAQ_BKPSRAM_Read(&status, DAQ_READ_STATUS_WORDS);
	if(status.bkpsram_state == DAQ_BKPSRAM_INITIALIZED && status.log_status == DAQ_FAULT_LOGGED)
	{
		DAQ_BKPSRAM_Read(&(log->current), DAQ_READ_CURRENT_LOG);
		DAQ_BKPSRAM_Write(NULL, DAQ_CLEAR_CURRENT_LOG);
		status.log_status = DAQ_FAULT_READ;
		DAQ_BKPSRAM_Write(&status, DAQ_WRITE_LOG_STATUS);
	}
}
void DAQ_FaultLog_Write(fault_log_t* log)
{
	daq_status_words_t status = {0};
	DAQ_BKPSRAM_Read(&status, DAQ_READ_STATUS_WORDS);
	if(status.bkpsram_state == DAQ_BKPSRAM_INITIALIZED)
	{
		DAQ_BKPSRAM_Write(log, DAQ_WRITE_LOG);
		status.bkpsram_state = DAQ_BKPSRAM_UNINITIALIZED;
		status.log_status = DAQ_FAULT_LOGGED;
		DAQ_BKPSRAM_Write(&status, DAQ_WRITE_BKPSRAM_STATE);
		DAQ_BKPSRAM_Write(&status, DAQ_WRITE_LOG_STATUS);
	}
}
void DAQ_WWDG_Task(void *pvParameters)
{
	uint8_t initial = 3; // Determined by testing.
	bool max_runtime_error;
	TickType_t current_tick;
	for(;;)
	{
		vTaskDelay(pdMS_TO_TICKS(60));
		if(initial || ( (g_daq_fault_record.tasks[ADC_TASK].entry_count || g_daq_fault_record.tasks[ADC_TASK].task_demoted) &&
						(g_daq_fault_record.tasks[PROX_TASK].entry_count||g_daq_fault_record.tasks[PROX_TASK].task_demoted) &&
						(g_daq_fault_record.tasks[IMU_TASK].entry_count || g_daq_fault_record.tasks[IMU_TASK].task_demoted) &&
						(g_daq_fault_record.tasks[GPS_TASK].entry_count || g_daq_fault_record.tasks[GPS_TASK].task_demoted) &&
						(g_daq_fault_record.tasks[TEMP_TASK].entry_count||g_daq_fault_record.tasks[TEMP_TASK].task_demoted) ))
		{
			if(initial)
				initial--; // Decrement until 0 (system steady state has been reached).
			HAL_WWDG_Refresh(&hwwdg); // The main purpose of this task

			// Clears task stats.
			for(uint8_t i = 0; i < DAQ_NO_OF_READ_TASKS; i++)
			{
				g_daq_fault_record.tasks[i].entry_count = 0;
				g_daq_fault_record.tasks[i].runtime = 0;
			}
		}
		else
		{
			HAL_WWDG_Refresh(&hwwdg); // Refresh to give some time for fault detection and logging.
			max_runtime_error = 0;
			for(uint8_t i = 0; i < DAQ_NO_OF_READ_TASKS; i++)
			{
				// Check if max runtime exceeded.
				if(g_daq_fault_record.tasks[i].runtime > DAQ_MAX_RUNTIME_TICKS && !g_daq_fault_record.tasks[i].task_demoted)
				{
					max_runtime_error = 1;
					g_daq_fault_record.tasks[i].error_count++;
					break;
				}
			}
			// If runtimes are fine, the problem must be with a task that wasn't finished yet.
			if(!max_runtime_error)
			{
				current_tick = xTaskGetTickCount();
				for(uint8_t i = 0; i < DAQ_NO_OF_READ_TASKS; i++)
				{
					// Check if the last time the task was entered is more than the max ticks.
					if(current_tick - g_daq_fault_record.tasks[i].start_tick > DAQ_MAX_TOTAL_TICKS_ELAPSED && !g_daq_fault_record.tasks[i].task_demoted)
					{
						g_daq_fault_record.tasks[i].error_count++;
						break;
					}
				}
			}
			DAQ_Task_Fault_Handler();
		}
	}
}
void DAQ_Task_Fault_Handler(void)
{
	__disable_irq();
	HAL_WWDG_Refresh(&hwwdg);
	fault_log_t log = {0};
	log.reset_reason = DAQ_RESET_REASON_TASKFAULTHANDLER;
	log.task_records = g_daq_fault_record;
	log.timestamp = g_timestamp;
	DAQ_FaultLog_Write(&log);
	for(;;);
}
void DAQ_Fault_Blink(void *pvParameters)
{
    for(;;)
    {
    	vTaskDelay(20);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
void DAQ_CAN_Task(void *pvParameters)
{
	daq_can_msg_t can_msg = {0};
	while (1)
	{
		// Blocks if the Queue is empty.
		if(DAQ_CAN_Msg_Dequeue(&can_msg) == pdTRUE)
		{
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
