#ifndef __BSP_VESC_CAN_H
#define __BSP_VESC_CAN_H
#include "main.h"
#include "fdcan.h"
#include "bsp_structer.h"

/*
 * 电机极对数：机械转速 = 电转速 / 极对数
 * 24N28P 电机为 14 对极，请按实际电机修改。
 * 作用：发送时把机械 RPM 换算成电角速度 ERPM；接收时把 ERPM 换回机械 RPM。
 */
#define VESC_PAIRS_OF_POLES   14

/*
 * 四个电调的 CAN ID（对应四个轮子位置，俯视车头朝上）。
 * 每个 ID 在 VESC Tool 里配置，需与电调实际设置一致。
 */
#define VESC_ID_FL   103   /* 前左（左上角 -135°） */
#define VESC_ID_FR   91    /* 前右（右上角 135°） */
#define VESC_ID_RL   121   /* 后左（左下角 -45°） */
#define VESC_ID_RR   80    /* 后右（右下角 45°） */

/*
 * VESC CAN 指令枚举（与 VESC CAN 协议一致，仅保留本工程用到部分）。
 * 指令会放到扩展 ID 的高 8 位（bit[15:8]），低 8 位是电机 ID。
 */
typedef enum
{
	CAN_PACKET_SET_CURRENT = 1,   // 设置电机目标电流
	CAN_PACKET_SET_RPM = 3,       // 设置电机目标转速（电角速度）
	CAN_PACKET_STATUS = 9,        // 状态报文：转速 / 电流 / 占空比
	CAN_PACKET_STATUS_4 = 16      // 状态报文：PID 位置
} CAN_PACKET_ID;

/*
 * 外部电机数组：索引 1~4 对应 4 个电调（索引 0 保留未使用）。
 * 电机需初始化到该数组中，接收中断按 ID 自动查表分发。
 */
extern VESC_Motor Vesc_Motor_U8[5];

/*
 * 初始化：绑定 FDCAN 句柄和电机 ID，并关联控制函数指针。
 * 单个电机实例调用一次。
 */
void VESC_Motor_Struct_Init(VESC_Motor *vesc_motor, FDCAN_HandleTypeDef *hcan, uint16_t id);

/*
 * CAN 接收初始化：配置接收滤波 + 使能 FIFO0 接收中断 + 启动 FDCAN。
 * 在 MX_FDCAN2_Init() 之后调用一次即可（多电机共用一条总线也只需一次）。
 */
void VESC_Can_Init(FDCAN_HandleTypeDef *hcan);

/*
 * 一键初始化 4 个电机到 Vesc_Motor_U8[1..4]，按 FL/FR/RL/RR 顺序绑定 ID。
 * 注意：不包含 VESC_Can_Init()，需先调用 VESC_Can_Init(hcan) 配置接收。
 */
void VESC_Motors_Init(FDCAN_HandleTypeDef *hcan);

/*
 * 控制：设置转速。
 *  - VESC_Set_Rpm_Struct()：结构体封装接口，推荐使用。
 *  - vesc_set_rpm()：底层接口，需手动传入 CAN 句柄和电机 ID。
 * 入参 rpm 为机械转速（RPM），内部自动 ×极对数 换算为电角速度发送。
 */
void VESC_Set_Rpm_Struct(VESC_Motor *vesc_motor, int32_t rpm);
void vesc_set_rpm(FDCAN_HandleTypeDef *hcan, uint8_t id, int32_t rpm);

/*
 * 控制：设置电流（mA），内部自动 ×1000 换算为 uA 发送。
 */
void VESC_Set_Current_Struct(VESC_Motor *vesc_motor, int32_t current);

/*
 * 接收解析：由 CAN 接收句柄（上层 FDCAN_Rx_Dispatch）调用，读取一帧并按 ID 分发到对应电机。
 */
void VESC_Rx_Handler(FDCAN_HandleTypeDef *hcan);

#endif
