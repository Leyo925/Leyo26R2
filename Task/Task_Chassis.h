#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H

#include "bsp_vesc_can.h"   /* VESC_Motor / Vesc_Motor_U8 电机实例与反馈 */
#include "Motor_vesc.h"     /* Chassis_TypeDef 底盘、速度分解/减速比等 */
#include "BSP_Sr.h"         /* DR16 遥控器（提供 Vx/Vy/w） */

/*
 * ============================================================================
 * 底盘控制任务（FreeRTOS 任务，VESC 转速直控）
 *
 * 任务函数：My_Chassis()，由 freertos.c 里 osThreadNew(My_Chassis,...) 创建，
 * 本文件提供同名强实现，覆盖 freertos.c 中的 __weak 空实现。
 *
 * 使用电机：bsp_vesc_can 中的 Vesc_Motor_U8[1..4] 代表四个电机（FL/FR/RL/RR）。
 * 控制方式：车体速度 → 4 轮转速(RPM) 直接下发 SetRpm()，
 *           转速闭环由 VESC 电调内部完成，STM32 侧不再做 PID。
 * ============================================================================
 */

/* ===== 遥控器 → 底盘速度映射参数（按需调整） ===== */
#define CHASSIS_VX_MAX      2.0f     /* 最大前后速度（m/s），对应摇杆满量程 */
#define CHASSIS_VY_MAX      2.0f     /* 最大左右速度（m/s） */
#define CHASSIS_WZ_MAX      6.28f    /* 最大自转角速度（rad/s），约 180°/s */
#define CHASSIS_DEADBAND    40       /* 摇杆死区（摇杆中位抖动抑制），中位仍抖就继续调大 */
/*
 * 底盘任务句柄：封装底盘（4 电机绑定 + 安装方向）
 */
 //结构体声明：
typedef struct
{
    Chassis_TypeDef chassis;              /* 底盘（4 电机绑定 + 安装方向） */
} ChassisTask_TypeDef;

/*
===============内部函数声明================
*/
/* 初始化：绑定 4 个电机（FL/FR/RL/RR） */
void ChassisTask_Init(ChassisTask_TypeDef *task);

/* 控制周期函数：车体速度 → 4 轮转速(RPM) 直接下发，每个控制周期调用一次 */
void ChassisTask_Control(ChassisTask_TypeDef *task, float vx, float vy, float wz);

/* FreeRTOS 底盘任务函数（覆盖 freertos.c 的 __weak My_Chassis） */
void My_Chassis(void *argument);
#endif /* __TASK_CHASSIS_H */
