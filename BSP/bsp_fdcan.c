#include "bsp_fdcan.h"

uint8_t CAN_Rxbuf[8];
HAL_StatusTypeDef a;

//函数1：CAN过滤器（全局"全收"：标准帧和扩展帧都进 RX FIFO0）
void can_filter_init(void)
{
    /* 用全局过滤器"全收"：标准帧和扩展帧都直接进 RX FIFO0，
     * 不再依赖单独的过滤器元素，最简洁也最不容易出错。 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_ACCEPT_IN_RX_FIFO0,   /* 非匹配标准帧 → RX FIFO0 */
        FDCAN_ACCEPT_IN_RX_FIFO0,   /* 非匹配扩展帧 → RX FIFO0 */
        FDCAN_REJECT_REMOTE,        /* 拒收远程标准帧 */
        FDCAN_REJECT_REMOTE);       /* 拒收远程扩展帧 */

    HAL_FDCAN_Start(&hfdcan1);//使能can通信
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//使能接收完成中断
	//    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter_st);
	//    HAL_FDCAN_Start(&hfdcan2);
	//    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
//函数2：数据发送函数（标准帧）
HAL_StatusTypeDef can_send_data(FDCAN_HandleTypeDef *pcan, uint32_t cob_id, uint8_t *data, uint8_t len)
{
    if (pcan == NULL) {
        return HAL_ERROR;
    }
    
    if (len > 8) len = 8;
    if (len == 0) return HAL_ERROR;
    
    FDCAN_TxHeaderTypeDef txHeader;
    
    txHeader.Identifier          = cob_id;                    /* 标准ID */
    txHeader.IdType              = FDCAN_STANDARD_ID;         /* 标准帧 */
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;          /* 数据帧 */
    txHeader.DataLength          = len;                       /* 数据长度 */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;          /* 错误状态指示 */
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;             /* 波特率切换关闭 */
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;         /* 经典CAN模式 */
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;        /* 不存储发送事件 */
    txHeader.MessageMarker       = 0;                         /* 消息标记 */
    
    return HAL_FDCAN_AddMessageToTxFifoQ(pcan, &txHeader, data);
}
//函数3：数据发送函数（扩展帧）
HAL_StatusTypeDef can_send_ext_data(FDCAN_HandleTypeDef *pcan, uint32_t ext_id, uint8_t *data, uint8_t len)
{
    if (pcan == NULL) {
        return HAL_ERROR;
    }

    if (len > 8) len = 8;
    if (len == 0) return HAL_ERROR;

    FDCAN_TxHeaderTypeDef txHeader;

    txHeader.Identifier          = ext_id;                    /* 扩展ID */
    txHeader.IdType              = FDCAN_EXTENDED_ID;         /* 扩展帧 */
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;          /* 数据帧 */
    txHeader.DataLength          = len;                       /* 数据长度 */
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;          /* 错误状态指示 */
    txHeader.BitRateSwitch       = FDCAN_BRS_OFF;             /* 波特率切换关闭 */
    txHeader.FDFormat            = FDCAN_CLASSIC_CAN;         /* 经典CAN模式 */
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;        /* 不存储发送事件 */
    txHeader.MessageMarker       = 0;                         /* 消息标记 */

    return a=HAL_FDCAN_AddMessageToTxFifoQ(pcan, &txHeader, data);
}

/* CAN 接收处理句柄：由上层任务在初始化时赋值，接收中断里直接调用 */
FDCAN_Rx_Handler g_fdcan_rx_handler = NULL;

/* 重写 HAL weak 接收回调：只把中断转发给上层句柄，不关心具体协议 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (g_fdcan_rx_handler != NULL)
    {
        g_fdcan_rx_handler(hfdcan);
    }
}
