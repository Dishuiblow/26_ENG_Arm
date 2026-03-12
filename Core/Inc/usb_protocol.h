#ifndef __USB_PROTOCOL_H__
#define __USB_PROTOCOL_H__

#include <stdint.h>

// 1. 帧定义
#define FRAME_HEADER_H    0xAA
#define FRAME_HEADER_L    0x55
#define FRAME_TAIL_H      0x0D
#define FRAME_TAIL_L      0x0A

// 2. 功能码 (请与 Python 上位机保持一致)
#define CMD_STOP          0x00
#define CMD_HOME          0x01
#define CMD_GRAVITY_COMP  0x02
#define CMD_TEACH_MODE    0x03
#define CMD_TRAJ_DATA     0x04
// 【新增】
#define CMD_ENABLE        0x05 // 电机使能
#define CMD_LOCK_CURRENT  0x06 // 锁死当前位置

// 3. 函数声明

// 【修改】统一函数名，并修正参数类型为指针 (uint32_t *Len)
// 这样才能匹配 usbd_cdc_if.c 中的调用
void User_USB_Receive_Callback(uint8_t* Buf, uint32_t *Len);

// 协议解析主循环
void USB_Protocol_Process(void);

// 发送数据 (发送12个float)
void USB_Send_Motor_Status(float* data_array);

#endif
