#include "mav_task.h"
#include "ins_task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "main.h"
uint8_t mavlink_tx_buffer[MAVLINK_MAX_PACKET_LEN];
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern FusionEuler euler;

// 参数结构体
typedef struct {
    char param_id[16];
    float param_value;
    uint8_t param_type;
} MavParam;

// 参数列表
#define PARAM_COUNT 2
static MavParam param_list[PARAM_COUNT] = {
    {"USER_PARAM", 0, MAV_PARAM_TYPE_INT32},
    {"GAIN", 1.0f, MAV_PARAM_TYPE_REAL32}
};


// MAVLink 消息任务结构体
typedef struct {
    uint8_t msg_id;            // MAVLink 消息 ID (e.g., MAVLINK_MSG_ID_HEARTBEAT)
    uint32_t interval_ms;      // 发送间隔，单位：毫秒
    uint64_t last_sent_time_ms;  // 上次发送时间，单位：毫秒
    bool is_active;            // 标记任务是否已添加
    uint32_t actual_interval_ms;      // 发送间隔，单位：毫秒
} MavlinkTask;

#define TX_BUF_SIZE 2048

typedef struct {
    uint8_t buffer[TX_BUF_SIZE];
    uint16_t head;  // 写入位置
    uint16_t tail;  // 读取位置
    uint16_t count; // 当前缓冲区内数据长度
} MAV_TxQueue;

MAV_TxQueue mav_queue = {0};

// 将数据压入队列
void Enqueue_MAV_Data(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        if (mav_queue.count < TX_BUF_SIZE) {
            mav_queue.buffer[mav_queue.head] = data[i];
            mav_queue.head = (mav_queue.head + 1) % TX_BUF_SIZE;
            mav_queue.count++;
        }
    }
}
// 串口接收相关静态变量
static uint8_t rx_byte;
static mavlink_message_t rx_msg;
static mavlink_status_t rx_status;


// ----------------------------------------------------
// 消息调度器
// ----------------------------------------------------

#define MAX_MAVLINK_TASKS 10 // 最大支持的消息任务数量

static MavlinkTask mavlink_tasks[MAX_MAVLINK_TASKS];
static int task_count = 0;

// 添加一个消息任务到调度器
void mavlink_scheduler_add_task(uint8_t msg_id, uint32_t interval_ms) {
    if (task_count >= MAX_MAVLINK_TASKS) {
        //printf("Error: Max number of MAVLink tasks reached.\n");
        return;
    }

    // 检查是否已存在相同 ID 的任务
    for (int i = 0; i < task_count; i++) {
        if (mavlink_tasks[i].msg_id == msg_id) {
            //printf("Warning: Task for message ID %d already exists. Skipping.\n", msg_id);
            return;
        }
    }

    mavlink_tasks[task_count].msg_id = msg_id;
    mavlink_tasks[task_count].interval_ms = interval_ms;
    mavlink_tasks[task_count].last_sent_time_ms = 0;
    mavlink_tasks[task_count].is_active = true;
    task_count++;

    //printf("Added task for message ID %d with interval %d ms\n", msg_id, interval_ms);
}

// 主循环中调用的更新函数
void mavlink_scheduler_update(UART_HandleTypeDef *huart) {
    uint64_t current_time_ms = 0;

    for (int i = 0; i < task_count; i++) {
        if (!mavlink_tasks[i].is_active) {
            continue;
        }
        current_time_ms = GetTimeUS_TIM();
        if (current_time_ms - mavlink_tasks[i].last_sent_time_ms >= mavlink_tasks[i].interval_ms) {
            mavlink_message_t msg;
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            mavlink_tasks[i].actual_interval_ms = current_time_ms - mavlink_tasks[i].last_sent_time_ms;

            // ----------------------------------------------------
            // 核心: 根据消息ID打包不同的消息
            // ----------------------------------------------------
            switch(mavlink_tasks[i].msg_id) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                	//mavlink_send_heartbeat(&msg);
                    break;
                case MAVLINK_MSG_ID_RAW_IMU:
                	//mavlink_send_imu();
                	break;
                case MAVLINK_MSG_ID_ATTITUDE:
                	//mavlink_send_attitude(&msg);
                	break;
                case MAVLINK_MSG_ID_PARAM_VALUE:
                	for (int i = 0; i < PARAM_COUNT; i++) {
                		mavlink_send_param(&msg, i);
                	}
                	break;

                // 在这里添加更多消息类型...
                default:
                    continue; // 不支持的消息ID，跳过
            }

            // 将消息打包成字节流并发送
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            if (HAL_UART_GetState(huart) == HAL_UART_STATE_READY) {
                // UART2 DMA可用，可以发送
                //HAL_UART_Transmit_DMA(huart, buf, len);
            }
            HAL_UART_Transmit(huart, buf, len, HAL_MAX_DELAY);

            // 更新上次发送时间
            mavlink_tasks[i].last_sent_time_ms = current_time_ms;
        }
    }
}


/* USER CODE BEGIN Header_StartDefaultTask */
void mavlink_send_heartbeat(void) {
    mavlink_heartbeat_t heartbeat;

    heartbeat.type = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = 3;

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Enqueue_MAV_Data(buf, len); // 压入队列
}

void mavlink_send_attitude(void){
	// 创建一个 Attitude 消息
	mavlink_attitude_t attitude;
    // 填充消息字段
    attitude.time_boot_ms = HAL_GetTick(); // 使用 HAL 库获取系统启动时间（毫秒）
    attitude.roll = euler.angle.roll;                   // 滚转角（弧度）
    attitude.pitch = euler.angle.pitch;                  // 俯仰角（弧度）
    attitude.yaw = euler.angle.yaw;                    // 偏航角（弧度）

    // 将 Attitude 消息打包成 Mavlink 消息
    // 参数: system_id, component_id, &msg, &attitude
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_attitude_encode(1, 1, &msg, &attitude);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Enqueue_MAV_Data(buf, len); // 压入队列
}
int32_t user_param = 0; // 用户参数

// 参数设置处理（支持多个参数）
void mavlink_handle_param_set(const mavlink_message_t* msg) {
    mavlink_param_set_t param_set;
    mavlink_msg_param_set_decode(msg, &param_set);
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(param_set.param_id, param_list[i].param_id) == 0) {
            param_list[i].param_value = param_set.param_value;
            mavlink_send_param_value(&huart1, i);
            break;
        }
    }
}
// 发送单个参数
void mavlink_send_param_value(UART_HandleTypeDef *huart, int idx)
{
    mavlink_message_t msg;
    mavlink_param_value_t param_value;
    strcpy(param_value.param_id, param_list[idx].param_id);
    param_value.param_value = param_list[idx].param_value;
    param_value.param_type = param_list[idx].param_type;
    param_value.param_count = PARAM_COUNT;
    param_value.param_index = idx;
    mavlink_msg_param_value_encode(1, 1, &msg, &param_value);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(huart, buf, len, HAL_MAX_DELAY);
}

// 发送全部参数
void mavlink_send_all_params(UART_HandleTypeDef *huart)
{
    uint8_t tryNumber = 10;
    while(tryNumber-- > 0)
    {
    	for (int i = 0; i < PARAM_COUNT; i++) {
    	    mavlink_send_param_value(huart, i);
    	}
    	osDelay(100);

    }
}
void mavlink_send_param(mavlink_message_t *const msg, int idx) {
		    mavlink_param_value_t param_value;
		    strcpy(param_value.param_id, param_list[idx].param_id);
		    param_value.param_value = param_list[idx].param_value;
		    param_value.param_type = param_list[idx].param_type;
		    param_value.param_count = PARAM_COUNT;
		    param_value.param_index = idx;
		    mavlink_msg_param_value_encode(1, 1, msg, &param_value);
}

void MAV_Check_And_Send_DMA(void) {
    // 检查 DMA 是否忙碌
    if (huart1.gState != HAL_UART_STATE_READY || mav_queue.count == 0) {
        return;
    }

    // 确定本次能发送的数据长度（处理环形缓冲区的绕回情况）
    uint16_t send_len;
    if (mav_queue.head > mav_queue.tail) {
        send_len = mav_queue.count;
    } else {
        send_len = TX_BUF_SIZE - mav_queue.tail;
    }

    // 启动 DMA 发送
    HAL_UART_Transmit_DMA(&huart1, &mav_queue.buffer[mav_queue.tail], send_len);

    // 更新队列索引
    mav_queue.tail = (mav_queue.tail + send_len) % TX_BUF_SIZE;
    mav_queue.count -= send_len;
}



void mavlinkTask(void *argument) {
	uint32_t tick = osKernelGetTickCount();
	uint32_t counter_500ms = 0;             // 500ms counter
	uint32_t period = 1;                   //  1ms base period
    while (1){
    	   tick =  osKernelGetTickCount();

        counter_500ms++;
    	// --- 逻辑 A: 每 1ms 执行一次 ---
	    mavlink_send_attitude();
	    // 2. 计数器累加，分频出 500ms 逻辑
	    if (counter_500ms >= 500)
	    {
	    // --- 逻辑 B: 每 500ms 执行一次 ---
	        mavlink_send_heartbeat();
	        counter_500ms = 0; // 计数器清零
	     }
	   MAV_Check_And_Send_DMA();
   	   osDelayUntil(tick +period); // 以 50Hz (20ms) 运行
	 }
}

// 串口中断接收方式
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &rx_msg, &rx_status))
        {
            if (rx_msg.msgid == MAVLINK_MSG_ID_PARAM_SET)
            {
                mavlink_handle_param_set(&rx_msg);
                mavlink_send_all_params(&huart1);
            }
            if (rx_msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST)
            {
                mavlink_send_all_params(&huart1);
            }
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

