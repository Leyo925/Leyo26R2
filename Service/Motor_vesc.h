#ifndef __MOTOR_VESC_H
#define __MOTOR_VESC_H

#include "bsp_vesc_can.h"   /* 使用 VESC_Motor 类型和 VESC 控制接口 */

/*
 * ============================================================================
 * 四全向轮底盘运动学（全向轮，正方形 O 型布局）
 *
 * 全向轮与麦轮的区别：
 *   - 全向轮辊子与轮轴垂直（90°），轮子沿轴向可自由滑动，主动驱动方向为
 *     轮子滚动切线方向。
 *   - 四个全向轮装在正方形四个顶点，轮轴指向车体中心（沿对角线），
 *     四个驱动方向互相垂直（各相差 90°），从而实现全向移动 + 自转。
 *
 * 车体坐标系定义（俯视）：
 *   x 轴：指向车体前进方向
 *   y 轴：指向车体左侧
 *   wz ：绕 z 轴旋转角速度，逆时针为正
 *
 * 轮子编号（俯视，车头朝上）：
 *     FL(前左) ── FR(前右)
 *     RL(后左) ── RR(后右)
 *
 * 用法示例：
 *     Chassis_TypeDef chassis;
 *     Chassis_Init(&chassis, &Vesc_Motor_U8[1], &Vesc_Motor_U8[2],
 *                           &Vesc_Motor_U8[3], &Vesc_Motor_U8[4]);
 *     // 在控制循环里持续调用：
 *     Chassis_Move(&chassis, 0.5f, 0.0f, 0.0f);   // 以 0.5 m/s 前进
 * ============================================================================
 */

/* 圆周率常量（不依赖 math.h） */
#define CHASSIS_PI          3.14159265358979f

/* ===== 底盘几何参数（按实际车体测量后修改）===== */
#define WHEEL_RADIUS        0.0745f               /* 全向轮半径（m），轮径 149mm */
#define WHEEL_CIRCUMFERENCE (2.0f * CHASSIS_PI * WHEEL_RADIUS)  /* 轮周长（m） */
#define CHASSIS_HALF_FACTOR 0.70710678f           /* 1/√2：正方形布局轮子驱动方向投影系数 */
#define CHASSIS_RADIUS      0.29212f              /* 轮子到车体中心距离（旋转半径，m） */

/* ===== 传动与电机参数 ===== */
#define GEAR_RATIO          (82.0f / 36.0f)       /* 减速比 82:36，电机转速 = 轮转速 × GEAR_RATIO */
#define GEAR_EFFICIENCY     0.85f                 /* 减速箱效率（估算 85%） */
#define MOTOR_KT_NM_PER_A   0.04244f              /* 电机扭矩常数 KT（Nm/A），MN705_KV225 */
#define WHEEL_KT_NM_PER_A   (MOTOR_KT_NM_PER_A * GEAR_RATIO * GEAR_EFFICIENCY) /* 轮上等效力矩系数（Nm/A） */
#define MOTOR_MAX_CURRENT_A 20.0f                 /* 电机最大电流（A） */
#define MOTOR_MAX_CURRENT_MA 20000                /* 电机最大电流（mA） */

/* ===== 转速限幅 ===== */
#define WHEEL_SPEED_MAX     1200.0f               /* 电机最大转速（RPM） */

/* 静止死区：目标转速绝对值低于该值视为"停止"，切电流 0（无转矩），
 * 避免转速闭环在零速无感状态下自激抖动（电机极小幅度来回空转） */
#define CHASSIS_STOP_RPM    30.0f

/* 主动刹车（软刹车）：目标转速进入死区后，用速度环（SetRpm）把目标转速缓降到 0，
 * 减速交给 VESC 内部速度环闭环，避免外部电流刹车刹过头过零震荡；
 * 实际转速连续低速一段时间判定停稳，切电流 0 保持静止，避免无感零速观测器漂移抖动 */
#define CHASSIS_BRAKE_DECEL         1200.0f  /* 刹车减速度（RPM/s），越大刹得越急 */
#define CHASSIS_BRAKE_STILL_ERPM    150.0f    /* 停稳判定阈值（ERPM）：实际转速低于此视为停 */
#define CHASSIS_BRAKE_STILL_CYCLES  50       /* 连续低速周期数（1ms/周期）：达到才切电流 0 */
#define CHASSIS_BRAKE_DT            0.001f   /* 控制周期（s），与底盘任务 1ms 节拍一致 */

/* 轮子索引 */
typedef enum
{
    WHEEL_FL = 0,   /* 前左 */
    WHEEL_FR,       /* 前右 */
    WHEEL_RL,       /* 后左 */
    WHEEL_RR,       /* 后右 */
    WHEEL_NUM       /* 轮子数量 = 4（用作数组大小） */
} Wheel_Index;

/* 底盘目标速度（车体坐标系） */
typedef struct
{
    float vx;   /* 前后线速度 m/s，向前为正 */
    float vy;   /* 左右线速度 m/s，向左为正 */
    float wz;   /* 旋转角速度 rad/s，逆时针为正 */
} Chassis_Velocity;

/* 底盘结构体：绑定四个电机 + 安装方向 + 刹车状态 */
typedef struct
{
    VESC_Motor *motor[WHEEL_NUM];   /* 四个电机指针，顺序 FL/FR/RL/RR */
    int8_t      dir[WHEEL_NUM];     /* 安装方向系数 +1/-1，用于修正电机正反转 */
    float       brake_target_rpm[WHEEL_NUM];  /* 刹车缓降目标转速（RPM） */
    uint16_t    brake_still_cnt[WHEEL_NUM];   /* 停稳连续计数（周期数） */
} Chassis_TypeDef;

/* 初始化底盘：绑定四个电机，方向系数默认全部 +1 */
void Chassis_Init(Chassis_TypeDef *chassis,
                  VESC_Motor *fl, VESC_Motor *fr,
                  VESC_Motor *rl, VESC_Motor *rr);

/* 轮子线速度(m/s) → 电机转速(RPM)，含减速比 */
float Chassis_WheelVel_To_Rpm(float v);

/* 电机转速(RPM) → 轮子线速度(m/s)，含减速比 */
float Chassis_Rpm_To_WheelVel(float rpm);

/* 逆运动学：车体速度 → 四个轮子线速度(m/s)，结果写入 wheel_vel[WHEEL_NUM] */
void Chassis_Speed_Decompose(const Chassis_Velocity *vel, float wheel_vel[WHEEL_NUM]);

/* 正运动学：四个轮子线速度 → 车体速度（用于里程计/调试） */
void Chassis_Speed_Compose(const float wheel_vel[WHEEL_NUM], Chassis_Velocity *vel);

/* 底盘运动：把车体速度分解并下发到四个电机（RPM 模式），需在控制循环中持续调用 */
void Chassis_Move(Chassis_TypeDef *chassis, float vx, float vy, float wz);

/////* 轮上扭矩(Nm) → 电机电流(A) */
////float Chassis_TorqueToCurrent(float torque_nm);

///* 电机电流(A) → 轮上扭矩(Nm) */
//float Chassis_CurrentToTorque(float current_a);

///* 动力学逆解算：车体期望力/力矩 → 四个轮子电流(A)，用于电流前馈，结果写入 current_a[WHEEL_NUM] */
//void Chassis_Force_Decompose(const Chassis_TypeDef *chassis, float Fx, float Fy, float Tz,
//                             float current_a[WHEEL_NUM]);

#endif /* __MOTOR_VESC_H */
