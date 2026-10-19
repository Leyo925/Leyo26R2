#include "Task_Arm.h"
#include "Control_pos.h"      /* 正逆运动学、五次多项式轨迹、leg_motion 等 */
#include "Control_torque.h"   /* 重力补偿前馈力矩 Control_Torque_FeedForward */
#include "bsp_dwt.h"          /* DWT 高精度计时器 */
#include "bsp_fdcan.h"        /* can_filter_init / hfdcan1 */
#include "Motor_AK80.h"       /* AK_Motion_* / AK_Error_Monitor / AK80_ID */
#include "Motor_EL05.h"       /* EL05_Motion_* / EL05_Calc_Horizontal_Angle / EL05_ID */
#include "bsp_vesc_can.h"     /* VESC_Rx_Handler（底盘电调接收解析） */
#include "BSP_Sr.h"           /* 遥控器 RC_ctrl_t / get_remote_control_point / RC_IsLost */
#include "cmsis_os2.h"        /* osDelay */

/* CAN 总接收分发（定义在文件末尾，句柄 g_fdcan_rx_handler 指向它） */
void FDCAN_Rx_Dispatch(FDCAN_HandleTypeDef *hfdcan);

/* ==================== 控制周期定义 ==================== */
/* 固定 1kHz 控制频率。注意：CPU_CYCLE_PER_US 必须和 DWT_Init(240) 的主频一致 */
#define CONTROL_PERIOD_US  1000U   /* 控制周期 1000us = 1kHz */
#define CPU_CYCLE_PER   480    /* 时钟周期数：480MHz */

/* ==================== 遥控器控制参数 ==================== */
#define RC_DEADZONE       10      /* 摇杆死区（原始值，满偏约 660） */
#define RC_MAX_VEL        0.30f   /* 满杆时末端最大移动速度 (m/s)，越大越快 */
#define RC_REPLAN_DIST    0.005f  /* 目标移动超过该距离(m)才重新规划，防震荡 */
#define RC_X_MIN         -0.48f   /* 末端 x 工作空间下限 (m)，按实际机械臂调 */
#define RC_X_MAX          0.30f   /* 末端 x 工作空间上限 (m) */
#define RC_Z_MIN          0.0f   /* 末端 z 工作空间下限 (m) */
#define RC_Z_MAX          0.60f   /* 末端 z 工作空间上限 (m) */
#define RC_Z_LIMIT_XPOS   0.22f   /* X>0 时 Z 的下限（机械限位保护），低于此不执行 Z 命令 */

/* ==================== 全局控制变量定义 ==================== */

/* 当前控制模式：上电默认进"归零模式"，你可在 debug 里改成 1 切到位置控制 */
volatile control_mode_t g_ctrl_mode   = MODE_ZERO;
/* 位置规划触发标志：debug 里置 1，触发一次新的末端位置规划（触发后自动清零） */
volatile int            g_cmd_trigger = 0;
float tau_sh;   /* 大臂（肩）前馈力矩 */
float tau_el;   /* 小臂（肘）前馈力矩 */
float el05_target, dt;   /* EL05 目标角；dt 本周期真实耗时（秒） */
int ret;
float motor1_cmd, motor2_cmd, vel1_cmd, vel2_cmd,dz,dx;
/* 位置控制指令：debug 里改这四个字段 */
pos_cmd_t g_pos_cmd =
{
    .target_x_s = 0.22f,   /* 默认末端落点 x */
    .target_z_s = 0.23f,   /* 默认末端落点 z */
    .target_yaw = 0.0f,   /* 0 = 保持当前末端姿态 */
    .duration   = 0.5f,   /* 默认 0.5 秒走完 */
};

/* 电机位置环控制参数：debug 里可调，觉得软就加大 Kp，抖就加大 Kd */
float g_kp     = 30.0f;   /* 位置环 Kp */
float g_kd     = 1.5f;    /* 位置环 Kd */
float g_torque = 0.0f;    /* 前馈力矩，默认 0 */

float g_kp_EL     = 50.0f;   /* 位置环 Kp */
float g_kd_EL     = 1.0f;    /* 位置环 Kd */
float g_torque_EL = 0.0f;    /* 前馈力矩，默认 0 */

/* ==================== 遥控器控制状态 ==================== */
static float   g_rc_target_x    = 0.0f;  /* 遥控累积的目标末端 x (m) */
static float   g_rc_target_z    = 0.0f;  /* 遥控累积的目标末端 z (m) */
static float   g_last_planned_x = 0.0f;  /* 上次规划时的目标末端 x */
static float   g_last_planned_z = 0.0f;  /* 上次规划时的目标末端 z */
static uint8_t s_rc_active      = 0;     /* 是否处于遥控模式（首次进入同步目标） */

/* ==================== 电机初始化 ==================== */
static void Motor_Init(void)
{
    /* 先设 CAN 接收句柄，再使能接收中断，避免启动瞬间丢帧 */
    g_fdcan_rx_handler = FDCAN_Rx_Dispatch;

    /* 配置 CAN 接收过滤器，启动 CAN 并开启接收中断 */
    can_filter_init();

    /* 大臂电机 AK80：初始化 + 进入运控模式（进入后开始回传反馈） */
    AK_Motion_Init(&g_ak80, &hfdcan1, AK80_ID);
    AK_Motion_Enter(&g_ak80);

    /* 小臂电机 AK45：初始化 + 进入运控模式 */
    AK_Motion_Init(&g_ak45, &hfdcan1, AK45_ID);
    AK_Motion_Enter(&g_ak45);

    /* 腕部电机 EL05：先延时 10ms 等 EL05 启动，再初始化 + 进入运控模式 */
    osDelay(1);
    EL05_Motion_Init(&g_el05, &hfdcan1, EL05_ID);
    EL05_Motion_Enter(&g_el05);
}

/* ==================== 遥控器控制机械臂 ==================== */
/*
 * 右摇杆：上/下 → 末端 Z（上正下负）；左/右 → 末端 X（左正右负）。
 * 采用「目标跟踪 + 距离阈值重规划」：
 *   - 摇杆值按速度积分成虚拟目标末端位置；
 *   - 只有目标移动超过 RC_REPLAN_DIST 才重新 IK + jtraj_start，
 *     避免每个控制周期都重规划导致轨迹反复重启、震荡；
 *   - 重规划时以当前实际关节角为起点，保证与上一段轨迹无缝衔接。
 */
static void Arm_RC_Control(const RC_ctrl_t *rc, float q1, float q2, float q3, float abs_angle)
{
    /* 1. 读右摇杆：ch[3] 上下，ch[2] 左右（范围约 -660 ~ +660） */
    int16_t stick_z = rc->rc.ch[3];
    int16_t stick_x = rc->rc.ch[2];

    /* 2. 死区：小抖动归零 */
    if (stick_z > -RC_DEADZONE && stick_z < RC_DEADZONE) stick_z = 0;
    if (stick_x > -RC_DEADZONE && stick_x < RC_DEADZONE) stick_x = 0;

    /* 3. 摇杆 → 末端目标增量（上正下负；左正右负故 x 取反） */
    dz = ((float)stick_z / 660.0f) * RC_MAX_VEL * dt;
    dx = (-(float)stick_x / 660.0f) * RC_MAX_VEL * dt;
    g_rc_target_z += dz;
    g_rc_target_x += dx;

    /* 4. 工作空间限幅（X>0 时 Z 下限提高到 0.2，防止撞机械限位） */
    g_rc_target_x = clampf(g_rc_target_x, RC_X_MIN, RC_X_MAX);
    float z_min = (g_rc_target_x > 0.0f) ? RC_Z_LIMIT_XPOS : RC_Z_MIN;
    g_rc_target_z = clampf(g_rc_target_z, z_min, RC_Z_MAX);

    /* 5. 目标移动够远才重规划（防震荡） */
    float ddx = g_rc_target_x - g_last_planned_x;
    float ddz = g_rc_target_z - g_last_planned_z;
    if ((ddx * ddx + ddz * ddz) > (RC_REPLAN_DIST * RC_REPLAN_DIST))
    {
        leg_pos_t target = {0};
        target.x_s = g_rc_target_x;
        target.z_s = g_rc_target_z;
        target.yaw = EL05_HORIZONTAL_ABS_ANGLE;

        int ret = (target.x_s <= 0.0f)
                    ? Inverse_Kinematics_EE(&target, ELBOW_AUTO, q1, q2, q3, abs_angle)
                    : Inverse_Kinematics_EE(&target, ELBOW_UP,   q1, q2, q3, abs_angle);

        if (ret == 0)
        {
            float T = calc_motion_time(q1, q2, leg_motion.q1_target, leg_motion.q2_target);
            jtraj_start(q1, q2, leg_motion.q1_target, leg_motion.q2_target, T);

            g_last_planned_x = g_rc_target_x;
            g_last_planned_z = g_rc_target_z;
        }
    }

    /* 6. 每周期推进轨迹 */
    int traj_state = jtraj_update(&leg_motion);

    if (traj_state >= 0)
    {
        motor1_cmd = leg_motion.motor1_target_angle;
        motor2_cmd = leg_motion.motor2_target_angle;
        vel1_cmd   = AK80_DIR * leg_motion.joint1_speed_target;
        vel2_cmd   = AK45_DIR * leg_motion.joint2_speed_target;
    }
    else
    {
        motor1_cmd = g_ak80.status.position;
        motor2_cmd = g_ak45.status.position;
        vel1_cmd   = 0.0f;
        vel2_cmd   = 0.0f;
    }

    /* 7. 重力补偿 + 下发大小臂 */
    tau_sh = 0.0f;
    tau_el = 0.0f;
    Control_Torque_FeedForward(&tau_sh, &tau_el, 0, dt, q1, q2);
    AK_Motion_Control(&g_ak80, motor1_cmd, vel1_cmd, g_kp, g_kd, tau_sh);
    AK_Motion_Control(&g_ak45, motor2_cmd, vel2_cmd, g_kp, g_kd, tau_el);

    /* 8. 腕部 EL05：保持水平 + 左摇杆额外俯仰（握手效果） */
    el05_target = EL05_Calc_Horizontal_Angle(motor1_to_joint(g_ak80.status.position),
                                             motor2_to_joint(g_ak45.status.position));

    /* 左摇杆 ch[1]（上下）映射额外末端俯仰角：上=+2π/3，下=-3π/4，中位=0 */
    int16_t stick_pitch = rc->rc.ch[1];
    if (stick_pitch > -RC_DEADZONE && stick_pitch < RC_DEADZONE) stick_pitch = 0;
    float extra_angle;
    if (stick_pitch >= 0)
    {
        extra_angle = ((float)stick_pitch / 660.0f) * (2.0f * 3.14159265f / 3.0f);  /* 0 ~ 2π/3 */
    }
    else
    {
        extra_angle = ((float)stick_pitch / 660.0f) * (3.0f * 3.14159265f / 4.0f);  /* -3π/4 ~ 0 */
    }

    /* 末端绝对角由 π 偏转 extra_angle，反解到 EL05 目标角需乘 EL05_DIR */
    el05_target += EL05_DIR * extra_angle;

    osDelay(1);
    EL05_Motion_Control(&g_el05, el05_target, 0, g_kp_EL, g_kd_EL, 0.0f);
}

/*
 * 机械臂电机接收帧解析：按帧类型/ID 分发到对应电机。
 */
void FDCAN_Rx_Message_Handler(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rx_data)
{
    if (rxHeader->IdType == FDCAN_EXTENDED_ID) {
        /* 扩展帧 → EL05 */
        EL05_Parse_Feedback(&g_el05, rxHeader->Identifier, rx_data);
    } else {
        /* 标准帧 → AK 系列，按 CAN ID 分发 */
        if (rxHeader->Identifier == g_ak80.motor_id) {
            parse_motion_feedback(&g_ak80, rx_data);
        } else if (rxHeader->Identifier == g_ak45.motor_id) {
            parse_motion_feedback(&g_ak45, rx_data);
        }
    }
}
/*
 * CAN 总接收分发：bsp_fdcan 接收中断的句柄 g_fdcan_rx_handler 指向本函数。
 * 按 hfdcan 区分两路总线：
 *   - hfdcan2 → 底盘 VESC 电调
 *   - 其余（hfdcan1）→ 机械臂 AK80/AK45/EL05
 */
void FDCAN_Rx_Dispatch(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan2)
    {
        /* VESC：每次读一帧，循环读空 FIFO0，避免高频回传溢出 */
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
        {
            VESC_Rx_Handler(hfdcan);
        }
    }
    else
    {
        /* 机械臂：取帧后交 FDCAN_Rx_Message_Handler 解析 */
        FDCAN_RxHeaderTypeDef rxHeader;
        uint8_t rx_data[8];

        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
        {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rx_data) != HAL_OK)
            {
                break;
            }
            FDCAN_Rx_Message_Handler(&rxHeader, rx_data);
        }
    }
}
/* ==================== 任务主体 ==================== */
void Moto_Diver(void *argument)
{
		osDelay(1000);
    uint32_t last_cycle = 0;   /* 周期计时基准（电机初始化后赋值） */
		uint8_t rc_enable = 1;
    for (;;)
    {
        const RC_ctrl_t *rc = get_remote_control_point();
        /* ===== 电机使能判断：未使能成功 → 执行初始化；已使能成功 → 跳过初始化 ===== */
        if (!(g_ak80.is_entered && g_ak45.is_entered))
        {
            /* 未使能：拨杆不在最上方（或失联）时执行初始化（含电机使能） */
            if (RC_IsLost() || rc->rc.s[1] != RC_SW_UP)
            {                
                Motor_Init();               /* 初始化并使能三个电机 */
                DWT_Init(CPU_CYCLE_PER);    /* 初始化 DWT 计时器 */
                last_cycle = DWT->CYCCNT;   /* 记录计时基准 */
            }
            osDelay(1);
            continue;
        }
		if(rc->rc.s[1] == RC_SW_UP)
		{
        /* 记录本周期开始时刻（用于后面固定周期补齐） */
        uint32_t cycle_start = DWT->CYCCNT;

        /* ========== 第一步：用 DWT 测本周期真实耗时 dt（秒） ==========*/
        dt = DWT_GetDeltaT(&last_cycle);

        /* ========== 第二步：电机错误监测 ==========*/
        AK_Error_Monitor(&g_ak80);
        AK_Error_Monitor(&g_ak45);

        /* ========== 第三步：正运动学刷新 ==========*/
        float q1 = motor1_to_joint(g_ak80.status.position);
        float q2 = motor2_to_joint(g_ak45.status.position);
        float q3 = g_el05.status.position;
        float abs_angle = q1 + q2 + EL05_DIR * (q3 - EL05_OFFSET);
        Forward_Kinematics(NULL, q1, q2, abs_angle);

        /* ========== EL05 速度前馈：预测大小臂运动，让 EL05 提前跟上 ==========*/
        float el05_vel = AK80_DIR * g_ak80.status.speed + AK45_DIR * g_ak45.status.speed;

        /* ========== 第四步：遥控器控制 ==========*/
        /* 首次进入遥控模式：把目标同步到当前末端位置，避免跳变 */
        if (rc_enable==1)
        {
					g_rc_target_x    = 0.22f;
          g_rc_target_z    = 0.23f;
          g_last_planned_x = 0.22f;
          g_last_planned_z = 0.23f;
          rc_enable = 0;
         }
        Arm_RC_Control(rc, q1, q2, q3, abs_angle);
			}
    }
}
