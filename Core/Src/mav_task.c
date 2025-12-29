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

//// 宏定义：将变量放入 CCMRAM 区域
//#define ATTRIBUTE_CCMRAM __attribute__((section(".ccmram")))

// 双缓冲区设计 (Ping-Pong)
IMU_Frame_t tx_buffers[2];
uint8_t active_buf_idx = 0;
volatile uint8_t dma_ready = 1;


/**
 * @brief 1kHz 遥测打包发送接口
 */
void Telemetry_Send_1kHz(ICM42688P_Data_t* imudata, FusionEuler* euler) {
    // 1. 如果 DMA 忙，直接跳过这一帧，不阻塞 CPU
    if (dma_ready == 0) {
        return;
    }

    // 2. 指向当前准备填充的缓冲区
    IMU_Frame_t *p_pkt = &tx_buffers[active_buf_idx];

    // 3. 填充模拟/真实数据
    p_pkt->header = 0x55AA;
    p_pkt->msg_id = 0x01;
    p_pkt->data_len = 36;
    p_pkt->ax = imudata->accel_x;
    p_pkt->ay = imudata->accel_y;
    p_pkt->az = imudata->accel_z;
    p_pkt->gx = imudata->gyro_x;
    p_pkt->gy = imudata->gyro_y;
    p_pkt->gz = imudata->gyro_z;
    p_pkt->roll = euler->angle.roll;
    p_pkt->pitch = euler->angle.pitch;
    p_pkt->yaw = euler->angle.yaw;

    // 4. 手动 CRC 计算
    __HAL_CRC_DR_RESET(&hcrc);
    p_pkt->crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)p_pkt, 7);

    // 5. 准备发送：先置零标志位
    dma_ready = 0;

    // 6. 启动发送，并检查返回值
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)p_pkt, sizeof(IMU_Frame_t));

    if (status != HAL_OK) {
        // 如果发送失败（例如 HAL 报忙），必须把标志位拨回 1，否则系统会永久锁死
        dma_ready = 1;
    } else {
        // 发送成功，切换到另一个缓冲区准备下一帧
        active_buf_idx = !active_buf_idx;
    }
}

/**
 * @brief DMA 传输完成回调 (需在串口中断中被调用)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        dma_ready = 1;
    }
}

uint16_t delta_us = 0;
uint16_t delta_us2 = 0;

uint16_t last_count2 = 0;

void mavlinkTask(void *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms = 1kHz

	for(;;) {
	    // 精确等时触发
	    vTaskDelayUntil(&xLastWakeTime, xFrequency);
       // 假定数据已经在其它采样任务中准备好
       Telemetry_Send_1kHz(&imudata, &euler);
       // 1. 立即读取当前计数值
       uint16_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
       // 2. 计算差值（考虑 16 位计数器溢出回环的情况）
       if (current_count >= last_count2) {
       	delta_us = current_count - last_count2;
       } else {
           // 发生溢出回环：(最大值 - 上一次) + 当前值 + 1
           delta_us = (0xFFFF - last_count2) + current_count + 1;
       }
       if(delta_us<1000){
	   delta_us2 = delta_us;}
       // 3. 更新上一次的值，供下次使用
       last_count2 = current_count;
    }

}

// 串口中断接收方式
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

