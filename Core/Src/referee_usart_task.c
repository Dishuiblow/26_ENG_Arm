/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  * Version    Date            Author          Modification
  * V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart10;

// 将原本的 usart6_buf 修改为 usart10_buf，保持命名一致性
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
uint8_t test; 

/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
int asdweq;
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    
    // 初始化 USART1 的双缓冲 DMA 接收
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

    while(1)
    {
        test = sizeof(ext_student_interactive_header_data_t);
        // 单次解包，每次一个字节
        common_data_send();
        referee_unpack_fifo_data();

        vTaskDelay(10);
      
        asdweq++;
        if(asdweq > 1000){asdweq = 0;}
    }
}


/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( Verify_CRC8_Check_Sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( Verify_CRC16_Check_Sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

// 串口10全局中断处理函数，处理空闲中断以实现不定长双缓冲接收
void USART10_IRQHandler(void)
{ 
    // H7芯片判定空闲中断使用的是 ISR 寄存器
    if(USART10->ISR & UART_FLAG_IDLE)
    {
        // H7 清除空闲中断标志位的宏
        __HAL_UART_CLEAR_IDLEFLAG(&huart10);

        static uint16_t this_time_rx_len = 0;

        // 获取 DMA 实例指针并强转，判断当前使用的是哪个缓冲区
        if ((((DMA_Stream_TypeDef *)huart10.hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart10.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart10.hdmarx);
            __HAL_DMA_SET_COUNTER(huart10.hdmarx, USART_RX_BUF_LENGHT);
            ((DMA_Stream_TypeDef *)huart10.hdmarx->Instance)->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart10.hdmarx);
            
            // =======================================================
            // 【极其致命：必须加上这句！】强制让 CPU 清除缓存，去物理内存读最新数据
            SCB_InvalidateDCache_by_Addr((uint32_t *)usart1_buf[0], USART_RX_BUF_LENGHT);
            // =======================================================
            
            // 写入 FIFO 时保证传递正确的地址
            fifo_s_puts(&referee_fifo, (char*)usart1_buf[0], this_time_rx_len);
            // 未进中断该函数就会停止计时，用以判断设备离线
            detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(huart10.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart10.hdmarx);
            __HAL_DMA_SET_COUNTER(huart10.hdmarx, USART_RX_BUF_LENGHT);
            ((DMA_Stream_TypeDef *)huart10.hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart10.hdmarx);
            
            // =======================================================
            // 【极其致命：必须加上这句！】强制让 CPU 清除缓存，去物理内存读最新数据
            SCB_InvalidateDCache_by_Addr((uint32_t *)usart1_buf[1], USART_RX_BUF_LENGHT);
            // =======================================================
            
            fifo_s_puts(&referee_fifo, (char*)usart1_buf[1], this_time_rx_len);
            // 未进中断该函数就会停止计时，用以判断设备离线
            detect_hook(REFEREE_TOE);
        }
    }
}
