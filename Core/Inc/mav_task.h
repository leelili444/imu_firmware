#ifndef __MAV_TASK_H__
#define __MAV_TASK_H__

#include "time.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

// force 4-byte alignment
#pragma pack(push, 4)
typedef struct __attribute__((packed)){
    uint16_t header;      // 0x55AA
    uint8_t  msg_id;      // 0x01
    uint8_t  data_len;    // paylod length 44 bytes

    uint32_t counter;   // incremental counter
    uint32_t dt; //us

    // imu and euler data (36 bytes)
    float  ax;
    float  ay;
    float  az;

    float  gx;
    float  gy;
    float  gz;

    float    roll;
    float    pitch;
    float    yaw;

    uint32_t crc32;       // hardware CRC32
} IMU_Frame_t;
#pragma pack(pop)




void mavlinkTask(void *argument);
void mavlink_send_param_value(UART_HandleTypeDef *huart, int idx);
#endif /* __MAV_TASK_H__ */
