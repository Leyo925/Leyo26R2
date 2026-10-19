#ifndef __BSP_FDCAN_H
#define __BSP_FDCAN_H

#include "main.h"

//参数调用声明：
extern FDCAN_HandleTypeDef hfdcan1;

//函数调用声明：
extern void can_filter_init(void);
extern HAL_StatusTypeDef can_send_data(FDCAN_HandleTypeDef *pcan, uint32_t cob_id, uint8_t *data, uint8_t len);
extern HAL_StatusTypeDef can_send_ext_data(FDCAN_HandleTypeDef *pcan, uint32_t ext_id, uint8_t *data, uint8_t len);

/*
 * CAN 接收处理句柄：由上层任务在初始化时赋值（如 g_fdcan_rx_handler = FDCAN_Rx_Dispatch）。
 * bsp_fdcan 在接收中断里只负责调用该句柄，不关心来源是 VESC 还是 AK/EL05，
 * 从而与具体协议完全解耦。
 */
typedef void (*FDCAN_Rx_Handler)(FDCAN_HandleTypeDef *hfdcan);
extern FDCAN_Rx_Handler g_fdcan_rx_handler;

#endif
