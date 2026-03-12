#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1; // 声明我们的新主力 USART1

// 1. USART1 双缓冲接收 DMA 初始化
void usart1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能串口空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    // 【核心修复】：H7的 Instance 是 void*，必须强转为具体的 DMA 数据流指针
    DMA_Stream_TypeDef *dmarx_stream = (DMA_Stream_TypeDef *)huart1.hdmarx->Instance;

    // 清除并关闭 DMA 准备配置
    __HAL_DMA_DISABLE(huart1.hdmarx);
    while(dmarx_stream->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart1.hdmarx);
    }

    // 配置双缓冲地址
    dmarx_stream->PAR = (uint32_t) & (USART1->RDR);
    dmarx_stream->M0AR = (uint32_t)(rx1_buf);
    dmarx_stream->M1AR = (uint32_t)(rx2_buf);
    dmarx_stream->NDTR = dma_buf_num;
    
    SET_BIT(dmarx_stream->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(huart1.hdmarx);
}

// 2. USART1 发送 DMA (用于给裁判系统发自定义UI等)
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    // 【核心修复】：强转指针
    DMA_Stream_TypeDef *dmatx_stream = (DMA_Stream_TypeDef *)huart1.hdmatx->Instance;

    // 关闭发送 DMA
    __HAL_DMA_DISABLE(huart1.hdmatx);
    while(dmatx_stream->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(huart1.hdmatx);
    }

    // 装载数据并开启发送
    dmatx_stream->PAR = (uint32_t) & (USART1->TDR);
    dmatx_stream->M0AR = (uint32_t)(data);
    dmatx_stream->NDTR = len;
    
    __HAL_DMA_ENABLE(huart1.hdmatx);
}
