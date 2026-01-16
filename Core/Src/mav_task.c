#include "ins_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "main.h"
#include "mav_task.h"

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern FusionEuler euler;
extern CRC_HandleTypeDef hcrc;
extern ICM42688P_Data_t imudata;

// Double buffer design for DMA transmission
IMU_Frame_t tx_buffers[2];
uint8_t active_buf_idx = 0;
volatile uint8_t dma_ready = 1;
uint32_t frame_tick = 0;
uint16_t delta_us = 0;
uint16_t delta_us2 = 0;
/**
 * @brief 1kHz telemetry packet send interface
 */
void Telemetry_Send_1kHz(ICM42688P_Data_t* imudata, FusionEuler* euler) {
    // 1. If DMA is busy, skip this frame without blocking the CPU
    if (dma_ready == 0) {
        return;
    }

    // 2. Point to the buffer currently being filled
    IMU_Frame_t *p_pkt = &tx_buffers[active_buf_idx];

    // 3. Fill packet with sensor data
    p_pkt->header = 0x55AA;
    p_pkt->msg_id = 0x01;
    p_pkt->data_len = 44;
    p_pkt->counter = frame_tick++;
    p_pkt->dt = delta_us;
    p_pkt->ax = imudata->accel_x;
    p_pkt->ay = imudata->accel_y;
    p_pkt->az = imudata->accel_z;
    p_pkt->gx = imudata->gyro_x;
    p_pkt->gy = imudata->gyro_y;
    p_pkt->gz = imudata->gyro_z;
    p_pkt->roll = euler->angle.roll;
    p_pkt->pitch = euler->angle.pitch;
    p_pkt->yaw = euler->angle.yaw;

    // 4. Manual CRC calculation
    __HAL_CRC_DR_RESET(&hcrc);
    p_pkt->crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)p_pkt, 7);

    // 5. Prepare to send: set flag to busy
    dma_ready = 0;

    // 6. Start transmission and check return value
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)p_pkt, sizeof(IMU_Frame_t));

    if (status != HAL_OK) {
        // If transmission fails (e.g., HAL reports busy), must set the flag back to 1, otherwise the system can deadlock
        dma_ready = 1;
    } else {
        // On success, switch to the other buffer for the next frame
        active_buf_idx = !active_buf_idx;
    }
}

/**
 * @brief DMA transfer complete callback (should be called from UART interrupt)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        dma_ready = 1;
    }
}



uint16_t last_count2 = 0;

void mavlinkTask(void *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(2); // 1ms = 1kHz

	for(;;) {
        // Precise periodic trigger
	    vTaskDelayUntil(&xLastWakeTime, xFrequency);
       // Assume data has been prepared in another sampling task
       Telemetry_Send_1kHz(&imudata, &euler);
       // 1. Read current counter value immediately
       uint16_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
       // 2. Compute difference (handle 16-bit counter overflow/wraparound)
       if (current_count >= last_count2) {
       	delta_us = current_count - last_count2;
       } else {
           // Overflow/wraparound occurred: (max - last) + current + 1
           delta_us = (0xFFFF - last_count2) + current_count + 1;
       }
       if(delta_us<1000){
	   delta_us2 = delta_us;}
       // 3. Update last value for next use
       last_count2 = current_count;
    }

}

// UART receive callback (interrupt-based)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

