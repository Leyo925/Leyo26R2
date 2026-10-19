#include "bsp_vesc_can.h"

/************************************************************************************************************************************************
VESC 电调驱动（FDCAN 版）
使用步骤：
1. 定义电机（可定义在全局）：
       VESC_Motor motor1;
       VESC_Motor_Struct_Init(&motor1, &hfdcan2, 10);   // 绑定 FDCAN2 和电机 ID

2. CAN 接收初始化（在 MX_FDCAN2_Init() 之后调用一次）：
       VESC_Can_Init(&hfdcan2);

3. 控制（需要在任务/主循环中持续发送）：
       motor1.SetRpm(&motor1, 1000);        // 设置转速 1000 RPM
       Vesc_Motor_U8[1].SetRpm(&Vesc_Motor_U8[1], 1000); // 通过数组访问

4. 接收解析：本文件已实现 HAL_FDCAN_RxFifo0Callback()，中断触发后自动解析到对应电机的 RxData 中。
   读反馈：motor1.RxData.Rpm / .Total_Current / .Duty

注意：
- VESC 不应与大疆电机使用同一条 CAN 线。
- 极对数 VESC_PAIRS_OF_POLES 需要按实际电机修改。
************************************************************************************************************************************************/

/*
 * 4 个电调的电机数组，索引 1~4（索引 0 保留不用）
 */
VESC_Motor Vesc_Motor_U8[5];

/*
 * 发送调试计数（供调试器 Watch 窗口观察，索引 1~4 对应电机 FL/FR/RL/RR）
 */
volatile uint32_t vesc_tx_ok_cnt[5]    = {0};   /* 各电机发送成功次数 */
volatile uint32_t vesc_tx_fail_cnt[5]  = {0};   /* 各电机发送失败次数 */
volatile uint32_t vesc_tx_fail_code[5] = {0};   /* 各电机最近一次失败的错误码（HAL_StatusTypeDef） */

/*
 * 大端序 int32 解码
 */
static int32_t vesc_decode_int32(const uint8_t *buf)
{
	uint32_t v = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
	             ((uint32_t)buf[2] << 8)  |  (uint32_t)buf[3];
	return (int32_t)v;
}

/*
 * 大端序 int16 解码
 */
static int16_t vesc_decode_int16(const uint8_t *buf)
{
	return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/*
 * 通用发送函数：构造 VESC CAN 帧并发送
 * @return HAL_OK 表示成功写入 TxFIFO，其他值表示发送失败
 */
static HAL_StatusTypeDef vesc_send(FDCAN_HandleTypeDef *hcan, uint16_t id, uint16_t cmd, int32_t value)
{
	FDCAN_TxHeaderTypeDef tx_header = {0};
	uint8_t data[8] = {0};

	/* 构造扩展 ID（VESC CAN 协议位布局，29 位）：
	 *   bit[7:0]   = 电机 ID
	 *   bit[15:8]  = 指令类型（如 CAN_PACKET_SET_RPM）
	 *   bit[28:16] = 保留（恒为 0）
	 */
	tx_header.Identifier  = ((uint32_t)cmd << 8) | (uint32_t)id;
	tx_header.IdType      = FDCAN_EXTENDED_ID;   // 扩展帧
	tx_header.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
	tx_header.DataLength  = FDCAN_DLC_BYTES_8;   // 8 字节
	tx_header.FDFormat    = FDCAN_CLASSIC_CAN;   // 经典 CAN 格式

	// 大端序填充（高字节在前）
	data[0] = (uint8_t)((value >> 24) & 0xFF);
	data[1] = (uint8_t)((value >> 16) & 0xFF);
	data[2] = (uint8_t)((value >> 8)  & 0xFF);
	data[3] = (uint8_t)(value & 0xFF);

	return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, data);
}

/**
 * @brief   VESC_Can_Init：配置接收滤波并启动 FDCAN 接收
 * @param   hcan: FDCAN 外设句柄（&hfdcan2）
 */
void VESC_Can_Init(FDCAN_HandleTypeDef *hcan)
{
	FDCAN_FilterTypeDef filter = {0};

	// 配置扩展帧滤波：接受所有扩展帧（mask=0 表示所有位不关心）
	filter.IdType       = FDCAN_EXTENDED_ID;
	filter.FilterIndex  = 0;
	filter.FilterType   = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1    = 0;
	filter.FilterID2    = 0;
	HAL_FDCAN_ConfigFilter(hcan, &filter);

	// 使能 FIFO0 新消息中断
	HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	// 使能 BusOff 中断：总线错误时自动恢复（见 HAL_FDCAN_ErrorStatusCallback）
	HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_BUS_OFF, 0);

	// 启动 FDCAN
	HAL_FDCAN_Start(hcan);
}

/**
 * @brief   VESC_Motor_Struct_Init：电机结构体初始化
 * @param   vesc_motor: 目标电机结构体指针
 * @param   hcan: 绑定的 FDCAN 外设句柄
 * @param   id: 电机 CAN 通信 ID
 */
void VESC_Motor_Struct_Init(VESC_Motor *vesc_motor, FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	vesc_motor->Basic_Parameters.hcan = hcan;
	vesc_motor->Basic_Parameters.ID   = id;

	// 关联控制函数指针
	vesc_motor->SetCurrent = VESC_Set_Current_Struct;
	vesc_motor->SetRpm     = VESC_Set_Rpm_Struct;
}

/**
 * @brief   VESC_Motors_Init：一键初始化 4 个电机到数组
 * @param   hcan: 绑定的 FDCAN 外设句柄（&hfdcan2）
 * @note    数组索引 1~4 对应轮子位置 FL/FR/RL/RR，ID 来自 bsp_vesc_can.h 宏定义：
 *            索引 1 = 前左 FL（ID 103）
 *            索引 2 = 前右 FR（ID 91）
 *            索引 3 = 后左 RL（ID 121）
 *            索引 4 = 后右 RR（ID 80）
 *          调用前需先 VESC_Can_Init(hcan) 完成接收配置。
 */
void VESC_Motors_Init(FDCAN_HandleTypeDef *hcan)
{
	VESC_Motor_Struct_Init(&Vesc_Motor_U8[1], hcan, VESC_ID_FL);  // 前左
	VESC_Motor_Struct_Init(&Vesc_Motor_U8[2], hcan, VESC_ID_FR);  // 前右
	VESC_Motor_Struct_Init(&Vesc_Motor_U8[3], hcan, VESC_ID_RL);  // 后左
	VESC_Motor_Struct_Init(&Vesc_Motor_U8[4], hcan, VESC_ID_RR);  // 后右
}


/**
 * @brief   VESC_Set_Rpm_Struct：设置转速（结构体封装）
 * @param   vesc_motor: 目标电机结构体指针
 * @param   rpm: 目标机械转速（RPM），内部转换为电角速度
 */
void VESC_Set_Rpm_Struct(VESC_Motor *vesc_motor, int32_t rpm)
{
	int32_t erpm = rpm * VESC_PAIRS_OF_POLES;
	HAL_StatusTypeDef st = vesc_send(vesc_motor->Basic_Parameters.hcan,
	                                 vesc_motor->Basic_Parameters.ID,
	                                 CAN_PACKET_SET_RPM, erpm);

	/* 按电机记录发送结果，便于调试器观察 */
	int idx = (int)(vesc_motor - &Vesc_Motor_U8[0]);
	if (idx >= 1 && idx <= 4)
	{
		if (st == HAL_OK)
		{
			vesc_tx_ok_cnt[idx]++;
		}
		else
		{
			vesc_tx_fail_cnt[idx]++;
			vesc_tx_fail_code[idx] = (uint32_t)st;
		}
	}
}

///**
// * @brief   vesc_set_rpm：设置转速（底层函数，手动传入 CAN 和 ID）
// */
//void vesc_set_rpm(FDCAN_HandleTypeDef *hcan, uint8_t id, int32_t rpm)
//{
//	int32_t erpm = rpm * VESC_PAIRS_OF_POLES;
//	vesc_send(hcan, id, CAN_PACKET_SET_RPM, erpm);
//}

/**
 * @brief   VESC_Set_Current_Struct：设置电流（结构体封装）
 * @param   vesc_motor: 目标电机结构体指针
 * @param   current: 目标电流（mA），VESC 内部做电流闭环
 */
void VESC_Set_Current_Struct(VESC_Motor *vesc_motor, int32_t current)
{
	/* VESC CAN 协议 COMM_SET_CURRENT 的单位即 mA，直接发送，不再换算 */
	vesc_send(vesc_motor->Basic_Parameters.hcan,
	          vesc_motor->Basic_Parameters.ID,
	          CAN_PACKET_SET_CURRENT, current);
}

/**
 * @brief   VESC_Rx_Handler：解析一帧 VESC 反馈数据并分发到对应电机
 * @note    由 CAN 接收句柄（上层 FDCAN_Rx_Dispatch）调用，每调用一次读取并解析一帧。
 * @param   hcan: FDCAN 外设句柄
 */
void VESC_Rx_Handler(FDCAN_HandleTypeDef *hcan)
{
	FDCAN_RxHeaderTypeDef rx_header = {0};
	uint8_t rx_data[8] = {0};
	uint8_t id, cmd;
	VESC_Motor *motor = NULL;
	int i;

	// 从 FIFO0 读取一帧
	if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
	{
		return;
	}

	/* 解析扩展 ID 位布局（与发送端一致）：
	 *   bit[7:0]  = 电机 ID（低 8 位）
	 *   bit[15:8] = 指令类型（高 8 位）
	 */
	id  = (uint8_t)(rx_header.Identifier & 0xFF);          // 取出电机 ID
	cmd = (uint8_t)((rx_header.Identifier >> 8) & 0xFF);   // 取出指令类型

	// 根据 CAN 句柄和 ID 找到对应电机
	for (i = 1; i <= 4; i++)
	{
		if (Vesc_Motor_U8[i].Basic_Parameters.hcan == hcan &&
		    Vesc_Motor_U8[i].Basic_Parameters.ID == id)
		{
			motor = &Vesc_Motor_U8[i];
			break;
		}
	}
	if (motor == NULL)
	{
		return;
	}

	// 按指令解包
	switch (cmd)
	{
		case CAN_PACKET_STATUS:   // 状态报文：转速 / 电流 / 占空比
		{
			/* 数据布局（VESC COMM_GET_VALUES，大端序）：
			 *  rx_data[0..3] : 电转速 ERPM（int32，可直接为负）
			 *  rx_data[4..5] : 总电流（int16，原始值 ×10，除以 10 得 A）
			 *  rx_data[6..7] : 占空比（int16，原始值 ×1000，除以 1000 得 -1~1）
			 */
			motor->RxData.ERpm = (float)vesc_decode_int32(&rx_data[0]);            // 电转速（ERPM）
			motor->RxData.Rpm  = motor->RxData.ERpm / VESC_PAIRS_OF_POLES;         // 机械转速 = 电转速 / 极对数
			motor->RxData.Total_Current = vesc_decode_int16(&rx_data[4]) / 10.0f;  // 总电流（A）
			motor->RxData.Duty = vesc_decode_int16(&rx_data[6]) / 1000.0f;         // 占空比（-1 ~ 1）
			break;
		}
		case CAN_PACKET_STATUS_4: // 状态报文：PID 位置
		{
			/* 数据布局（大端序）：
			 *  rx_data[0..1] : MOSFET 温度（原始值 ×10，0.1℃）
			 *  rx_data[2..3] : 电机温度（原始值 ×10，0.1℃）
			 *  rx_data[4..5] : 输入电流（原始值 ×10，0.1A）
			 *  rx_data[6..7] : PID 位置（原始值 ×50，即角度×50，除以 50 得度）
			 */
			// 先保存上一次位置，用于跨零判断
			motor->RxData.PID_Position_Last = motor->RxData.PID_Position_Now;
			// 解析当前 PID 位置（°）
			motor->RxData.PID_Position_Now  = vesc_decode_int16(&rx_data[6]) / 50.0f;

			/* 处理 360° 跨零，累计圈数：
			 * 位置从 >180 跳到 <-180（如 270°→90°）视为正转一圈；反之反转一圈 */
			if (motor->RxData.PID_Position_Now - motor->RxData.PID_Position_Last < -180.0f)
			{
				motor->RxData.Cnt++;       // 正向跨零，圈数 +1
			}
			else if (motor->RxData.PID_Position_Now - motor->RxData.PID_Position_Last > 180.0f)
			{
				motor->RxData.Cnt--;       // 反向跨零，圈数 -1
			}
			// 累计总角度 = 圈数 × 360° + 当前角度
			motor->RxData.Total_Position = motor->RxData.Cnt * 360.0f + motor->RxData.PID_Position_Now;
			break;
		}
		default:
			break;
	}
}

/**
 * @brief   FDCAN 错误状态回调：BusOff 时自动恢复
 * @note    BusOff 后 FDCAN 会停止收发，这里 Stop/Start 一次，复位错误计数器并回到总线。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if ((ErrorStatusITs & FDCAN_FLAG_BUS_OFF) != 0U)
	{
		HAL_FDCAN_Stop(hfdcan);
		HAL_FDCAN_Start(hfdcan);
	}
}
