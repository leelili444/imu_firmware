#ifndef __MAV_TASK_H__
#define __MAV_TASK_H__

#include "time.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

// 强制 4 字节对齐
#pragma pack(push, 4)
typedef struct __attribute__((packed)){
    uint16_t header;      // 0x55AA
    uint8_t  msg_id;      // 0x01
    uint8_t  data_len;    // 44 (负载长度)

    uint32_t counter;   // 【新增】硬件自增计数器 (偏移量 5-8)
    uint32_t dt; //us

    // 数据负载 (36 字节)
    float  ax;
    float  ay;
    float  az;

    float  gx;
    float  gy;
    float  gz;

    float    roll;
    float    pitch;
    float    yaw;

    uint32_t crc32;       // 硬件 CRC32 校验码
} IMU_Frame_t;
#pragma pack(pop)




void mavlinkTask(void *argument);
void mavlink_send_param_value(UART_HandleTypeDef *huart, int idx);
#endif /* __MAV_TASK_H__ */
