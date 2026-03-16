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
#include "referee.h"
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
extern UART_HandleTypeDef huart1;

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
    //usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

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

///* 在 stm32h7xx_it.c 中 */
//void USART1_IRQHandler(void){
//  /* USER CODE BEGIN USART1_IRQn 0 */
//  if(USART1->ISR & UART_FLAG_IDLE){
//      __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//      
//      // 计算本次接收长度
//      uint16_t this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
//      
//      // 停止 DMA 以防止处理数据时新数据进来覆盖 (可选，视实时性要求)
//      __HAL_DMA_DISABLE(huart1.hdmarx);
//      // 刷 Cache (H7 必须)
//      SCB_InvalidateDCache_by_Addr((uint32_t *)usart1_buf, USART_RX_BUF_LENGHT);
//      
//      // 写入 FIFO
//      fifo_s_puts(&referee_fifo, (char*)usart1_buf, this_time_rx_len);
//      detect_hook(REFEREE_TOE);
//      
//      // 重新设置 DMA 计数并开启 (因为是 Circular 模式，其实只需 Enable 即可，NDTR会自动重载)
//      // 注意：如果不 Disable，这里甚至不需要重新 Enable，直接清标志走人也行，看你对数据完整性的要求
//      __HAL_DMA_ENABLE(huart1.hdmarx);
//  }

//  /* USER CODE END USART1_IRQn 0 */
//  
//  HAL_UART_IRQHandler(&huart1);

//  /* USER CODE BEGIN USART1_IRQn 1 */

//  /* USER CODE END USART1_IRQn 1 */
//}