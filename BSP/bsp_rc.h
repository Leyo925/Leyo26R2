#ifndef __BSP_RC_H
#define __BSP_RC_H

#include "main.h"   /* 提供 HAL 类型与寄存器宏 */

/*
 * DR16 遥控器接收：DMA 双缓冲 + IDLE 空闲中断（H7 版）
 *
 * 注意：
 *  - 该方案依赖 DMA 为 Circular（循环）模式 + DBM 双缓冲，两个内存区交替填充。
 *  - IDLE 中断在 USART3_IRQHandler 里处理（BSP_Sr.c 中实现）。
 */
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif /* __BSP_RC_H */
