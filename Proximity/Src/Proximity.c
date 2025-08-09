



#include "proximity.h"

uint16_t prox_dma_buffer[PROX_NO_OF_WHEELS][PROX_DMA_WHEEL_BUFFER_SIZE];
prox_rpm_buffer_t prox_rpm_buffer;
static TIM_HandleTypeDef prox_timer_handle;
static DMA_HandleTypeDef prox_dma_handles[PROX_NO_OF_WHEELS];
static uint32_t timer_counters[PROX_NO_OF_WHEELS] = {&TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3, &TIM3->CCR4};
extern daq_fault_record_t g_fault_record;

static inline float Prox_GetTimerFreq(void)
{
    float pclk1 = (float)HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        pclk1 *= 2.0f;
    return pclk1 / ((float)TIM3->PSC + 1.0f);
}
void Prox_Init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[PROX_NO_OF_WHEELS])
{
    prox_timer_handle = *htim;
    for (uint8_t i = 0; i < PROX_NO_OF_WHEELS; i++)
        prox_dma_handles[i] = *hdma[i];
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_1, prox_dma_buffer[FRONT_LEFT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_2, prox_dma_buffer[FRONT_RIGHT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_3, prox_dma_buffer[REAR_LEFT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_4, prox_dma_buffer[REAR_RIGHT_BUFF], PROX_DMA_WHEEL_BUFFER_SIZE);
}
void Prox_Task(void *pvParameters)
{
    uint8_t slow_counter[PROX_NO_OF_WHEELS] = {0, 0, 0, 0};
    while (1)
    {
        vTaskDelay(7);
        g_fault_record.tasks[PROX_TASK].start_tick = xTaskGetTickCount();
        for (uint8_t wheel_no = 0; wheel_no < PROX_NO_OF_WHEELS; wheel_no++)
        {
            uint8_t last_reading_index = 0;

            for (last_reading_index = 0; last_reading_index < PROX_DMA_WHEEL_BUFFER_SIZE; last_reading_index++)
                if (prox_dma_buffer[wheel_no][last_reading_index] == 0)
                    break;
            // If the wheel is moving fast.
            if (last_reading_index > 1)
            {
                slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter
                // Calculate the speed.
                uint16_t difference = prox_dma_buffer[wheel_no][last_reading_index - 1] - prox_dma_buffer[wheel_no][last_reading_index - 2];
                prox_rpm_buffer.current[wheel_no] = 0.25 * 60 * (Prox_GetTimerFreq() / difference);

                // Pause DMA.
                HAL_DMA_Abort(&prox_dma_handles[wheel_no]);

                // Erase the array.
                memset(prox_dma_buffer[wheel_no], 0, PROX_DMA_WHEEL_BUFFER_SIZE * sizeof(uint16_t));
                /*for (uint8_t i = 0; i < PROX_DMA_WHEEL_BUFFER_SIZE; i++)
                    prox_dma_buffer[wheel_no][i] = 0;*/

                // Reset the DMA pointer so that it starts at the beginning of the array.
                prox_dma_handles[wheel_no].Instance->M0AR = prox_dma_buffer[wheel_no];
                prox_dma_handles[wheel_no].Instance->NDTR = PROX_DMA_WHEEL_BUFFER_SIZE;
                HAL_DMA_Start(&prox_dma_handles[wheel_no], timer_counters[wheel_no], prox_dma_buffer[wheel_no], PROX_DMA_WHEEL_BUFFER_SIZE);
            }
            else
            {
                if (slow_counter[wheel_no] <= 20) // The car is slow
                    slow_counter[wheel_no]++; // Increment the corresponding slow_counter
                else
                { // The car is at rest.
                    prox_rpm_buffer.current[wheel_no] = 0;
                    slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter

                    // Pause DMA.
                    HAL_DMA_Abort(&prox_dma_handles[wheel_no]);

                    // Erase the array.
                    memset(prox_dma_buffer[wheel_no], 0, PROX_DMA_WHEEL_BUFFER_SIZE * sizeof(uint16_t));
                    /*for (uint8_t i = 0; i < PROX_DMA_WHEEL_BUFFER_SIZE; i++)
                        prox_dma_buffer[wheel_no][i] = 0;*/

                    // Reset the DMA pointer so that it starts at the beginning of the array.
                    prox_dma_handles[wheel_no].Instance->M0AR = prox_dma_buffer[wheel_no];
                    prox_dma_handles[wheel_no].Instance->NDTR = PROX_DMA_WHEEL_BUFFER_SIZE;
                    HAL_DMA_Start(&prox_dma_handles[wheel_no], timer_counters[wheel_no], prox_dma_buffer[wheel_no], PROX_DMA_WHEEL_BUFFER_SIZE);
                }
            }
        }
        if(DAQ_CheckChange(prox_rpm_buffer.current[FRONT_LEFT_BUFF], prox_rpm_buffer.prev[FRONT_LEFT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
           DAQ_CheckChange(prox_rpm_buffer.current[FRONT_RIGHT_BUFF], prox_rpm_buffer.prev[FRONT_RIGHT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
		   DAQ_CheckChange(prox_rpm_buffer.current[REAR_LEFT_BUFF], prox_rpm_buffer.prev[REAR_LEFT_BUFF], DAQ_MIN_CHANGE_PROX_RPM) ||
		   DAQ_CheckChange(prox_rpm_buffer.current[REAR_RIGHT_BUFF], prox_rpm_buffer.prev[REAR_RIGHT_BUFF], DAQ_MIN_CHANGE_PROX_RPM))
        {
        	daq_can_msg_t can_msg_prox = {0};
        	daq_can_msg_prox_t encoder_msg_prox = {0};
        	can_msg_prox.id = DAQ_CAN_ID_PROX_ENCODER;
        	can_msg_prox.size = 8;
        	encoder_msg_prox.rpm_front_left  = (uint64_t)prox_rpm_buffer.current[FRONT_LEFT_BUFF];
        	encoder_msg_prox.rpm_front_right = (uint64_t)prox_rpm_buffer.current[FRONT_RIGHT_BUFF];
        	encoder_msg_prox.rpm_rear_left   = (uint64_t)prox_rpm_buffer.current[REAR_LEFT_BUFF];
        	encoder_msg_prox.rpm_rear_right  = (uint64_t)prox_rpm_buffer.current[REAR_RIGHT_BUFF];
        	//encoder_msg_prox.ENCODER_angle = (uint64_t)ENCODER_get_angle();
        	encoder_msg_prox.Speedkmh = (uint64_t) PROX_CALCULATE_SPEED(prox_rpm_buffer.current[FRONT_LEFT_BUFF], prox_rpm_buffer.current[FRONT_RIGHT_BUFF]);
        	can_msg_prox.data = *((uint64_t*)&encoder_msg_prox);
        	DAQ_CAN_Msg_Enqueue(&can_msg_prox);
        	for(uint8_t i = 0; i < PROX_NO_OF_WHEELS; i++)
        		prox_rpm_buffer.prev[i] = prox_rpm_buffer.current[i];
        }
        g_fault_record.tasks[PROX_TASK].entry_count++;
        if(g_fault_record.tasks[PROX_TASK].runtime == 0)
        {
        	g_fault_record.tasks[PROX_TASK].runtime = xTaskGetTickCount();
        	g_fault_record.tasks[PROX_TASK].runtime -= g_fault_record.tasks[PROX_TASK].start_tick;
        }
    }
}
