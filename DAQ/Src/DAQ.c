/*
 * DAQ.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Belal
 */
#include "DAQ.h"

/* =========================================== PRIVATE VARIABLES =========================================== */
QueueHandle_t daq_can_queue;
CAN_HandleTypeDef daq_can_handle;
uint32_t tx_mailbox = 0;
CAN_TxHeaderTypeDef* ptr_tx_header;
uint8_t queue_size;
/* =========================================== PRIVATE VARIABLES END =========================================== */

void DAQ_CAN_Init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header)
{
	daq_can_queue = xQueueCreate(10, sizeof(daq_can_msg_t));
    daq_can_handle = *can_handle;
    can_tx_header->IDE = CAN_ID_STD;
    can_tx_header->RTR = CAN_RTR_DATA;
    can_tx_header->TransmitGlobalTime = DISABLE;
    ptr_tx_header = can_tx_header;
}

void DAQ_CAN_Msg_Enqueue(daq_can_msg_t* message)
{
    xQueueSend(daq_can_queue, message, 2);
}

daq_can_msg_t DAQ_CAN_Msg_Dequeue(void){
	daq_can_msg_t message = {};
    xQueueReceive(daq_can_queue, &message, portMAX_DELAY);
    return message;
}

void DAQ_CAN_Task(void *pvParameters)
{
	daq_can_msg_t can_msg;
	while (1)
	{
		vTaskDelay(2);
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
	}
}

