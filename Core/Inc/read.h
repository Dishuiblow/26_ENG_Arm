#ifndef __READ_H__
#define __READ_H__

#include "main.h"

// 宏定义：映射 millis() 到 HAL_GetTick()
#define millis() HAL_GetTick()

// 舵机数量和缓冲区大小
#define MAX_SERVO_NUM 1
#define RX_BUF_SIZE 255
#define ANGLE_RESPONSE_LEN 10 

// 外部变量声明
extern uint8_t uart_rx_buf[RX_BUF_SIZE];
extern uint16_t rx_index;
extern uint8_t uart_rx_flag;

// 函数声明
void delay_ms(uint16_t t); // [新增] 仿照官方的延时函数
void usart_send_str(uint8_t *str); // [新增] 带中断保护的发送函数

void servo_init(void);
void servo_PickUp(void);
void request_angle_id(uint8_t id);
void parse_angle(void);
uint8_t get_servo_angle(uint8_t id, uint16_t *angle);

#endif /* __READ_H__ */
