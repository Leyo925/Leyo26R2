#ifndef __Motor_AK80_H
#define __Motor_AK80_H

#include "main.h"
#include "bsp_structer.h"
#include "string.h"
//宏定义声明：
//需要确认
#define CMD_ENTER_MODE   0xFC
#define CMD_EXIT_MODE    0xFD   
#define CMD_SET_ORIGIN   0xFE  
#define CMD_READ_STATUS  0xFC
#define CLAMP(value, min, max)  (((value) < (min)) ? (min) : (((value) > (max)) ? (max) : (value)))

#define AK80_ID 0x01

/* ==================== 电机错误判定 ==================== */
/* 这里判断 error_code 是否代表"电机出错"。
 * ?? 注意：务必按你的电机手册确认！
 *   大多数宇树 AK 电机协议：data[7](error) = 0 表示正常，非 0 表示有错误。
 *   如果你确认"0 表示出错"，保持下面 == 0；如果是"非 0 出错"，改成 != 0。
 * 只改这一个宏即可，下面所有判断都会跟着变。 */
#define AK_IS_ERROR(code)  ((code) != 0)


/* 运控模式参数限制 (AK60-6规格) */
//需要确认
#define P_MIN   (-12.5f)
#define P_MAX   ( 12.5f)
#define V_MIN   (-30.0f)
#define V_MAX   ( 30.0f)
#define T_MIN   (-18.0f)
#define T_MAX   ( 18.0f)
#define KP_MIN  ( 0.0f)
#define KP_MAX  ( 500.0f)
#define KD_MIN  ( 0.0f)
#define KD_MAX  ( 5.0f)

//句柄实例声明（定义在 Joint_AK80.c）：
extern AK_Handle_t g_ak80;
extern AK_Handle_t g_ak45;

//函数外部调用声明：
extern void parse_motion_feedback(AK_Handle_t*ak,uint8_t *data);
extern HAL_StatusTypeDef AK_Motion_Init(AK_Handle_t*ak,FDCAN_HandleTypeDef *pcan, uint8_t motor_id);
extern HAL_StatusTypeDef AK_Motion_Enter(AK_Handle_t*ak);
extern HAL_StatusTypeDef AK_Motion_Exit(AK_Handle_t*ak);
extern HAL_StatusTypeDef AK_Motion_Control(AK_Handle_t*ak,float position, float speed, float kp, float kd, float torque);

/* 电机错误监测：检查单个 AK 电机的 error_code，
 * 一旦出错立即失能（退出运控模式），防止烧坏电机。
 * 建议在电机控制任务的每个周期都调用一次。 */
extern void AK_Error_Monitor(AK_Handle_t *ak);

#endif