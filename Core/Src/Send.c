#include "send.h"
#include "string.h"
#include "CRC8_CRC16.h"

static uint8_t seq_num = 0; 

// 发送缓冲区
uint8_t send_buf[39] = {0}; 
extern UART_HandleTypeDef huart10; // 引用外部串口句柄

void Send_Servo_Angles(uint16_t angles[6], uint8_t mode)
{
    uint16_t cmd_id = 0x0302;
    uint16_t data_len = 30;

    // 帧头
    send_buf[0] = 0xA5;
    send_buf[1] = (uint8_t)(data_len);
    send_buf[2] = (uint8_t)(data_len >> 8);
    send_buf[3] = seq_num++;
    Append_CRC8_Check_Sum(send_buf, 5);

    // CMD ID
    send_buf[5] = (uint8_t)(cmd_id);
    send_buf[6] = (uint8_t)(cmd_id >> 8);

    // 数据段 (30 Bytes)
    send_buf[7] = mode; // [0] 模式
    
    // [1-12] 6个关节角度 (小端序)
    for(int i=0; i<6; i++) {
        send_buf[8 + i*2]     = (uint8_t)(angles[i]);
        send_buf[8 + i*2 + 1] = (uint8_t)(angles[i] >> 8);
    }
    
    // [13-29] 补零
    memset(&send_buf[20], 0, 17);

    // CRC16
    Append_CRC16_Check_Sum(send_buf, 39);

    // DMA 发送
    HAL_UART_Transmit_DMA(&huart10, send_buf, sizeof(send_buf));
}
