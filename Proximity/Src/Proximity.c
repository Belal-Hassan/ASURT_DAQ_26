



#include "proximity.h"

uint16_t prox_dma_buffer[4][16];
prox_rpms_t prox_wheel_rpms;
static TIM_HandleTypeDef prox_timer_handle;
static DMA_HandleTypeDef prox_dma_handles[4];
static uint32_t timer_counters[4] = {&TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3, &TIM3->CCR4};


void Prox_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[4])
{
    prox_timer_handle = *htim;
    for (int i = 0; i < 4; i++)
        prox_dma_handles[i] = *hdma[i];
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_1, &prox_dma_buffer[FRONT_LEFT_BUFF], 16);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_2, &prox_dma_buffer[FRONT_RIGHT_BUFF], 16);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_3, &prox_dma_buffer[REAR_LEFT_BUFF], 16);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_4, &prox_dma_buffer[REAR_RIGHT_BUFF], 16);
}
void Prox_Task(void *pvParameters)
{
    uint8_t slow_counter[4] = {0, 0, 0, 0};
    while (1)
    {
        vTaskDelay(10);
        for (int wheel_no = 0; wheel_no < 4; wheel_no++)
        {
            int last_reading_index = 0;

            for (last_reading_index = 0; last_reading_index < 16; last_reading_index++)
                if (prox_dma_buffer[wheel_no][last_reading_index] == 0)
                    break;
            // If the wheel is moving fast.
            if (last_reading_index > 1)
            {
                slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter
                // Calculate the speed.
                uint16_t difference = prox_dma_buffer[wheel_no][last_reading_index - 1] - prox_dma_buffer[wheel_no][last_reading_index - 2];
                prox_wheel_rpms.current[wheel_no] = 0.25 * 60 * (PROX_TIMER_FREQ / difference);

                // Pause DMA.
                HAL_DMA_Abort(&prox_dma_handles[wheel_no]);

                // Erase the array.
                for (int i = 0; i < 16; i++)
                    prox_dma_buffer[wheel_no][i] = 0;

                // Reset the DMA pointer so that it starts at the beginning of the array.
                prox_dma_handles[wheel_no].Instance->M0AR = &prox_dma_buffer[wheel_no];
                prox_dma_handles[wheel_no].Instance->NDTR = 16;
                HAL_DMA_Start(&prox_dma_handles[wheel_no], timer_counters[wheel_no], &prox_dma_buffer[wheel_no], 16);
            }
            else
            {
                if (slow_counter[wheel_no] <= 14) // The car is slow
                    slow_counter[wheel_no]++; // Increment the corresponding slow_counter
                else
                { // The car is at rest.
                    prox_wheel_rpms.current[wheel_no] = 0;
                    slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter

                    // Pause DMA.
                    HAL_DMA_Abort(&prox_dma_handles[wheel_no]);

                    // Erase the array.
                    for (int i = 0; i < 16; i++)
                        prox_dma_buffer[wheel_no][i] = 0;

                    // Reset the DMA pointer so that it starts at the beginning of the array.
                    prox_dma_handles[wheel_no].Instance->M0AR = &prox_dma_buffer[wheel_no];
                    prox_dma_handles[wheel_no].Instance->NDTR = 16;
                    HAL_DMA_Start(&prox_dma_handles[wheel_no], timer_counters[wheel_no], &prox_dma_buffer[wheel_no], 16);
                }
            }
        }
        if(DAQ_CheckChange(prox_wheel_rpms.current[FRONT_LEFT_BUFF], prox_wheel_rpms.prev[FRONT_LEFT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
           DAQ_CheckChange(prox_wheel_rpms.current[FRONT_RIGHT_BUFF], prox_wheel_rpms.prev[FRONT_RIGHT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
		   DAQ_CheckChange(prox_wheel_rpms.current[REAR_LEFT_BUFF], prox_wheel_rpms.prev[REAR_LEFT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
		   DAQ_CheckChange(prox_wheel_rpms.current[REAR_RIGHT_BUFF], prox_wheel_rpms.prev[REAR_RIGHT_BUFF], DAQ_MIN_CHANGE_PROX_RPM))
        {
        	daq_can_msg_t can_msg_prox = {};
        	daq_can_msg_prox_t encoder_msg_prox = {};
        	can_msg_prox.id = DAQ_CAN_ID_PROX_ENCODER;
        	can_msg_prox.size = 8;
        	encoder_msg_prox.rpm_front_left  = (uint64_t)prox_wheel_rpms.current[FRONT_LEFT_BUFF];
        	encoder_msg_prox.rpm_front_right = (uint64_t)prox_wheel_rpms.current[FRONT_RIGHT_BUFF];
        	encoder_msg_prox.rpm_rear_left   = (uint64_t)prox_wheel_rpms.current[REAR_LEFT_BUFF];
        	encoder_msg_prox.rpm_rear_right  = (uint64_t)prox_wheel_rpms.current[REAR_RIGHT_BUFF];
        	//encoder_msg_prox.ENCODER_angle = (uint64_t)ENCODER_get_angle();
        	encoder_msg_prox.Speedkmh = (uint64_t) PROX_CALCULATE_SPEED(prox_wheel_rpms.current[FRONT_LEFT_BUFF], prox_wheel_rpms.current[FRONT_RIGHT_BUFF]);
        	can_msg_prox.data = *((uint64_t*)&encoder_msg_prox);
        	DAQ_CAN_Msg_Enqueue(&can_msg_prox);
        	prox_wheel_rpms.prev[FRONT_LEFT_BUFF]  = prox_wheel_rpms.current[FRONT_LEFT_BUFF];
        	prox_wheel_rpms.prev[FRONT_RIGHT_BUFF] = prox_wheel_rpms.current[FRONT_RIGHT_BUFF];
        	prox_wheel_rpms.prev[REAR_LEFT_BUFF]   = prox_wheel_rpms.current[REAR_LEFT_BUFF];
        	prox_wheel_rpms.prev[REAR_RIGHT_BUFF]  = prox_wheel_rpms.current[REAR_RIGHT_BUFF];
        }
    }
}
