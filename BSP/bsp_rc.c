#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 板载 DBUS 接口（USART3）已内置反相器，不需要开 USART_CR2_RXINV；
    // 只有自己拿普通串口接接收机时，才需要软件反相

    // 使能 DMA 串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // 失效 DMA，并等 EN 真正清零（H7 写 NDTR 必须 EN=0，否则写入被忽略）
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while (((DMA_Stream_TypeDef *)hdma_usart3_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    // 外设地址：H7 的 USART 接收数据寄存器是 RDR（F4 是 DR）
    ((DMA_Stream_TypeDef *)hdma_usart3_rx.Instance)->PAR = (uint32_t)&(USART3->RDR);

    // 内存缓冲区 1
    ((DMA_Stream_TypeDef *)hdma_usart3_rx.Instance)->M0AR = (uint32_t)(rx1_buf);

    // 内存缓冲区 2
    ((DMA_Stream_TypeDef *)hdma_usart3_rx.Instance)->M1AR = (uint32_t)(rx2_buf);

    // 数据长度
    ((DMA_Stream_TypeDef *)hdma_usart3_rx.Instance)->NDTR = dma_buf_num;

    // 使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef *)hdma_usart3_rx.Instance)->CR, DMA_SxCR_DBM);

    // 使能 DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}
