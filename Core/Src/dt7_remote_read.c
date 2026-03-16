#include "main.h"
#include "dt7_remote_read.h"
#include "usart.h"
#include "dma.h"

/*使用uart3——dma*/

extern DMA_HandleTypeDef hdma_usart10_rx;
extern UART_HandleTypeDef huart1;

RC_ctrl_t dt7;

static void solve_dt7_data(volatile const uint8_t *sbus_buf, RC_ctrl_t *dt7); //进行解码
uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_NUM];  //缓冲数组

//void dt7_remote_read_init(void){
//    //Double_dma_init(&huart1,&hdma_usart1_rx,sbus_rx_buffer[0],sbus_rx_buffer[1],RC_FRAME_LENGTH);  //双缓冲加载
//}


const RC_ctrl_t *get_dt7_remote_control_point(void){
    return &dt7;
}


extern DMA_HandleTypeDef hdma_usart10_rx;

//串口中断
void USART10_IRQHandler(void){
    // 仅处理IDLE帧结束中断（一帧数据接收完成触发）
    if (__HAL_UART_GET_FLAG(&huart10, UART_FLAG_IDLE) != RESET){
        // 必须先清除IDLE标志，否则会反复进入中断
        __HAL_UART_CLEAR_IDLEFLAG(&huart10);
        
        // 计算本次接收到的帧长度
        uint16_t rx_len = SBUS_RX_BUF_NUM - ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->NDTR;
        
        // 帧长校验通过，执行解码
        if (rx_len == RC_FRAME_LENGTH){
            solve_dt7_data(sbus_rx_buffer[1], &dt7);
        }
    }

    // HAL库标准中断兜底，处理错误等其他事件
    HAL_UART_IRQHandler(&huart10);
}

static void  solve_dt7_data(volatile const uint8_t *sbus, RC_ctrl_t *dt7){
    //114  第0
    if(sbus==NULL||dt7==NULL){
        return;
    }
    else {
        dt7->rc.ch[0]=((sbus[0])|(sbus[1]<<8)) & 0x7ff;      //左边   ——>0
        dt7->rc.ch[1]=((sbus[1]>>3)|(sbus[2]<<5)) &0x7ff;    

        dt7->rc.ch[2]=((sbus[2]>>6)|(sbus[3]<<2)|(sbus[4]<<10)) &0x7ff;   //右边   ——>2
        dt7->rc.ch[3]=((sbus[4]>>1)|(sbus[5]<<7)) &0x7ff;

        dt7->rc.s[0]= ((sbus[5]>>4)) &0x0003;  //左边
        dt7->rc.s[1]=((sbus[5]>>6))  &0x0003;  //右边

        dt7->mouse.x=((sbus[6])|(sbus[7]<<8));
        dt7->mouse.y=((sbus[8])|(sbus[9]<<8));
        dt7->mouse.z=((sbus[10])|(sbus[11]<<8));
        dt7->mouse.press_l_last=dt7->mouse.press_l;
        dt7->mouse.press_r_last = dt7->mouse.press_r;
        dt7->mouse.press_l=sbus[12];
        dt7->mouse.press_r=sbus[13];
        dt7->key.v = sbus[14] | (sbus[15] << 8);                    //键盘通道
        dt7->rc.ch[4] = sbus[16] | (sbus[17] << 8);                 //波轮

        dt7->rc.ch[0] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[1] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[2] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[3] -= RC_CH_VALUE_OFFSET;
        dt7->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    }
}



