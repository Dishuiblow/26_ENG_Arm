#include "usb_protocol.h"
#include "Arm_Task.h"
#include "string.h"
#include "usbd_cdc_if.h" // 必须包含这个才能调用 CDC_Transmit_HS

// --- 外部变量引用 ---
extern Arm_t my_arm;
extern float usb_target_pos[6];
extern USBD_HandleTypeDef hUsbDeviceHS; // 引用 USB 句柄

// --- 环形缓冲区定义 ---
#define RING_BUFFER_SIZE 512
static uint8_t rx_fifo[RING_BUFFER_SIZE];
static uint16_t fifo_head = 0;
static uint16_t fifo_tail = 0;

// --- 【修复1】函数声明 (解决 Warning #223, #159) ---
void Execute_Command(uint8_t cmd, uint8_t* data, uint8_t len);
void USB_Send_Motor_Status(float* data_array);

// --- USB 接收回调 ---
void User_USB_Receive_Callback(uint8_t* Buf, uint32_t *Len)
{
    uint32_t len = *Len;
    for(uint32_t i=0; i<len; i++) {
        rx_fifo[fifo_head] = Buf[i];
        fifo_head = (fifo_head + 1) % RING_BUFFER_SIZE;
    }
}

uint8_t Read_Byte_From_Fifo(uint8_t* byte) {
    if(fifo_head == fifo_tail) return 0;
    *byte = rx_fifo[fifo_tail];
    fifo_tail = (fifo_tail + 1) % RING_BUFFER_SIZE;
    return 1;
}

// --- 协议解析主循环 ---
void USB_Protocol_Process(void)
{
    static uint8_t state = 0;
    static uint8_t cmd_id = 0;
    static uint8_t data_len = 0;
    static uint8_t data_cnt = 0;
    static uint8_t rx_buf[64];
    static uint8_t checksum = 0;
    
    uint8_t byte = 0;
    
    while(Read_Byte_From_Fifo(&byte)) 
    {
        switch(state) {
            case 0: if(byte == 0xAA) state = 1; break;
            case 1: if(byte == 0x55) state = 2; else state = 0; break;
            case 2: cmd_id = byte; checksum = cmd_id; state = 3; break;
            case 3: 
                data_len = byte; checksum += data_len; data_cnt = 0;
                if(data_len == 0) state = 5; else state = 4; 
                break;
            case 4: 
                rx_buf[data_cnt++] = byte; checksum += byte;
                if(data_cnt >= data_len) state = 5; 
                break;
            case 5: 
                checksum &= 0xFF;
                if(byte == checksum) Execute_Command(cmd_id, rx_buf, data_len);
                state = 6; 
                break;
            case 6: if(byte == 0x0D) state = 7; else state = 0; break;
            case 7: state = 0; break; // 0x0A
            default: state = 0; break;
        }
    }
}

// --- 指令执行 ---
void Execute_Command(uint8_t cmd, uint8_t* data, uint8_t len) {
    switch(cmd) {
        case 0x00: my_arm.current_state = ARM_STATE_DISABLED; break;     // Stop
        case 0x01: my_arm.current_state = ARM_STATE_HOMING; break;       //Home
        case 0x02: my_arm.current_state = ARM_STATE_GRAVITY_COMP; break; // Gravity
        case 0x03: my_arm.current_state = ARM_STATE_GRAVITY_COMP; break; // Teach
        case 0x04: // Traj Data
            if(len == 24) {
                memcpy(usb_target_pos, data, 24);
                my_arm.current_state = ARM_STATE_REPLAY;
            }
            break;
        case 0x05: // Enable
            Arm_Init();
            my_arm.current_state = ARM_STATE_GRAVITY_COMP;
            break;
        case 0x06: // Lock Current
            for(int i=0; i<6; i++) usb_target_pos[i] = my_arm.joints[i].para.pos;
            my_arm.current_state = ARM_STATE_REPLAY;
            break;
    }
}

// --- 【修复2】补回发送函数 (解决 Error L6218E) ---
void USB_Send_Motor_Status(float* data_array) {
    static uint8_t tx_buf[64]; 
    uint8_t i, cs=0;
    
    // 协议头
    tx_buf[0] = 0xAA;
    tx_buf[1] = 0x55;
    tx_buf[2] = 0x10; // ID for Status
    tx_buf[3] = 48;   // Length: 12 floats * 4 bytes = 48
    
    // 拷贝数据 (12个float)
    cs = 0x10 + 48;
    memcpy(&tx_buf[4], data_array, 48);
    
    // 计算校验和
    for(i=0; i<48; i++) cs += tx_buf[4+i];
    
    // 协议尾
    tx_buf[4 + 48] = cs;
    tx_buf[4 + 48 + 1] = 0x0D;
    tx_buf[4 + 48 + 2] = 0x0A;
    
    // 发送 (总长 55 字节)
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
    if (hcdc->TxState == 0) {
        CDC_Transmit_HS(tx_buf, 55); 
    }
}
