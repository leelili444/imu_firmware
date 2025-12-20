#ifndef __MAV_TASK_H__
#define __MAV_TASK_H__

#include <common/mavlink.h>
#include "time.h"
#include "stm32f4xx_hal.h"
void mavlink_scheduler_add_task(uint8_t msg_id, uint32_t interval_ms);
void mavlink_send_heartbeat(void);
void mavlink_send_attitude(void);
void mavlink_send_param(mavlink_message_t *const msg,int idx);

void SendToPC(UART_HandleTypeDef *huart);
void mavlinkTask(void *argument);
void mavlink_send_param_value(UART_HandleTypeDef *huart, int idx);
#endif /* __MAV_TASK_H__ */
