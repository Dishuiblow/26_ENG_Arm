#include "read.h"
#include "string.h"
#include "stdio.h"

// 引用串口句柄
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart10;

extern uint8_t uart7_rx_buf[10];
extern uint8_t uart10_rx_buf[10];

// 全局变量定义
uint8_t uart_rx_buf[RX_BUF_SIZE];
uint8_t uart_rx_flag = 0;
uint16_t rx_index = 0;
static uint16_t rx_read_index = 0; // 环形缓冲读指针

typedef struct {
    uint16_t angle;
    uint8_t updated;
} ServoInfo;

ServoInfo servos[MAX_SERVO_NUM];

// ============================================================
//  1. 仿照官方 z_delay.c 实现的延时函数
// ============================================================
// 官方写法是 while(millis()!=(systick_ms_bak+t)); 
// 这种写法如果滴答计数器跳变超过1ms会导致死循环，
// 这里改为差值判断，既保留了原逻辑结构，又更安全。
void delay_ms(uint16_t t) {
    uint32_t systick_ms_bak = millis();
    while ((millis() - systick_ms_bak) < t) {
        // 等待
    }
}

// ============================================================
//  2. 仿照官方 z_uart.c 重构的发送函数 (关键!)
// ============================================================
// 官方逻辑：关中断 -> 发送 -> 开中断
// 这能有效防止半双工总线的回波干扰发送过程
void usart_send_str(uint8_t *str) {
    // 1. 关闭接收中断 (对应 uart1_close)
    __HAL_UART_DISABLE_IT(&huart10, UART_IT_RXNE);
    
    // 2. 发送数据 (阻塞式)
    HAL_UART_Transmit(&huart10, str, strlen((char *)str), 100);
    
    // 3. 清除可能产生的回波标志 (可选，但推荐)
    // 读取 SR 和 DR 可以清除 RXNE 标志，防止一开中断就进中断处理回波
    __HAL_UART_CLEAR_FLAG(&huart10, UART_FLAG_RXNE);
    
    // 4. 重新开启接收中断 (对应 uart1_open)
    __HAL_UART_ENABLE_IT(&huart10, UART_IT_RXNE);
}

// ============================================================
//  3. 业务逻辑 (使用新的发送和延时函数)
// ============================================================

// 夹爪释放（复位）
void servo_init(void) {
    uint8_t cmd[] = "#000P1500T2000!";
    usart_send_str(cmd); // 使用重构的发送函数
    delay_ms(10);        // 使用重构的延时函数
}

// 夹爪夹起
void servo_PickUp(void) {
    uint8_t cmd[] = "#000P0800T2000!";
    usart_send_str(cmd);
    delay_ms(100); 
}

// 请求指定ID舵机角度
void request_angle_id(uint8_t id) {
    if (id >= MAX_SERVO_NUM) return;
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPRAD!", id);
    usart_send_str((uint8_t *)cmd);
}

// 解析角度 (环形缓冲逻辑)
void parse_angle(void) {
    static uint8_t frame[ANGLE_RESPONSE_LEN];
    static uint8_t frame_pos = 0;

    while (rx_read_index != rx_index) {
        uint8_t byte = uart_rx_buf[rx_read_index];
        rx_read_index = (rx_read_index + 1) % RX_BUF_SIZE;

        if (byte == '#') {
            frame_pos = 0;
            frame[frame_pos++] = byte;
            continue;
        }

        if (frame_pos > 0 && frame_pos < ANGLE_RESPONSE_LEN) {
            frame[frame_pos++] = byte;
        }

        if (byte == '!' && frame_pos == ANGLE_RESPONSE_LEN) {
            if (frame[4] == 'P') {
                uint8_t id = (frame[1] - '0') * 100 + (frame[2] - '0') * 10 + (frame[3] - '0');
                if (id < MAX_SERVO_NUM) {
                    servos[id].angle = (frame[5] - '0') * 1000 +
                                       (frame[6] - '0') * 100 +
                                       (frame[7] - '0') * 10 +
                                       (frame[8] - '0');
                    servos[id].updated = 1;
                }
            }
            frame_pos = 0;
        }
    }
}

// 获取角度
uint8_t get_servo_angle(uint8_t id, uint16_t *angle) {
    if (id >= MAX_SERVO_NUM || angle == NULL) return 0;
    if (servos[id].updated) {
        if (servos[id].angle > 500 && servos[id].angle < 2500) {
            *angle = servos[id].angle;
        }
        servos[id].updated = 0;
        return 1;
    }
    return 0;
}

// 接收中断回调 (保持环形写入)
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//if(huart->Instance == UART7)
//    {
//        // 重新开启 UART7 接收
//        HAL_UART_Receive_IT(&huart7, uart7_rx_buf, 1);
//    }
//    else if(huart->Instance == USART10)
//    {
//        // 重新开启 USART10 DMA 接收
//        HAL_UART_Receive_DMA(&huart10, uart10_rx_buf, sizeof(uart10_rx_buf));
//    }
//}
