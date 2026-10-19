#include "Task_Chassis.h"
#include "usart.h"       /* huart1（遥控器串口） */
#include "fdcan.h"       /* hfdcan2（VESC） */
#include "cmsis_os2.h"   /* osDelay */
float vx = 0.0f, vy = 0.0f, wz = 0.0f;
/* 底盘任务句柄（本文件内部使用） */
ChassisTask_TypeDef s_chassis;

/* 摇杆死区 + 重映射：
 *   |v| < deadband → 返回 0（中位彻底静止，吞掉噪声/漂移）
 *   其余 → 线性映射到 [-1,1]，且在死区边界连续，无跳变 */
static float rc_stick_map(float v, float deadband, float range)
{
    if (v > -deadband && v < deadband)
    {
        return 0.0f;
    }
    if (v >= deadband)
    {
        return (v - deadband) / (range - deadband);
    }
    return (v + deadband) / (range - deadband);
}

void Set_Speed()
{
	const RC_ctrl_t *rc = get_remote_control_point();

	/* 安全保护：遥控器失联（或上电后从未收到数据）时，速度强制归零 */
	if (RC_IsLost())
	{
		vx = 0.0f;
		vy = 0.0f;
		wz = 0.0f;
		return;
	}

	if(rc->rc.s[1] == RC_SW_MID)
	{
		/* 死区 + 重映射：中位噪声彻底归零，死区外线性映射无跳变 */
		float ch0 = rc_stick_map((float)rc->rc.ch[0], CHASSIS_DEADBAND, 660.0f);
		float ch2 = rc_stick_map((float)rc->rc.ch[2], CHASSIS_DEADBAND, 660.0f);
		float ch3 = rc_stick_map((float)rc->rc.ch[3], CHASSIS_DEADBAND, 660.0f);

		vx = -ch3 * CHASSIS_VX_MAX;
		vy = ch2 * CHASSIS_VY_MAX;
		wz = ch0 * CHASSIS_WZ_MAX;
	}
	else
	{
		/* 未使能时速度归零，防止拨杆离开后电机保持上一速度 */
		vx = 0.0f;
		vy = 0.0f;
		wz = 0.0f;
	}
}
/**
底盘任务初始化
 */
void ChassisTask_Init(ChassisTask_TypeDef *task)
{
    /* 绑定 4 个电机：Vesc_Motor_U8[1..4] → FL/FR/RL/RR */
    Chassis_Init(&task->chassis,
                 &Vesc_Motor_U8[1], &Vesc_Motor_U8[2],
                 &Vesc_Motor_U8[3], &Vesc_Motor_U8[4]);
}

/**
ChassisTask_Control：底盘控制周期函数
 */
void ChassisTask_Control(ChassisTask_TypeDef *task, float vx, float vy, float wz)
{
    /* 车体速度 → 4 轮转速(RPM) → SetRpm 直接下发（VESC 内部做转速闭环） */
    Chassis_Move(&task->chassis, vx, vy, wz);
}
/*
My_Chassis：FreeRTOS 底盘任务函数
 */
void My_Chassis(void *argument)
{
    /* ===== 一次性初始化 ===== */
    VESC_Can_Init(&hfdcan2);             /* VESC 接收滤波 + 中断 + 启动 */
    VESC_Motors_Init(&hfdcan2);          /* 4 电机绑定 ID（FL/FR/RL/RR） */
    remote_control_init();     /* 遥控器：DMA 双缓冲 + IDLE 中断 */
    ChassisTask_Init(&s_chassis);        /* 底盘：绑定 4 电机 */

    /* ===== 控制循环（1ms） ===== */
    for (;;)
    {
        /* ① 读遥控器并映射速度（数据由 IDLE 中断更新到 rc_ctrl） */
        Set_Speed();

        /* ② 底盘控制：逆解算 + 下发目标转速(RPM) */
        ChassisTask_Control(&s_chassis, vy, vx, wz);

        /* ③ 1ms 任务节拍 */
        osDelay(1);
    }
}
