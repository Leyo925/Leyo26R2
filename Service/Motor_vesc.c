#include "Motor_vesc.h"

/*
 * 数值限幅：把 v 限制在 [lo, hi] 区间
 */
static float clampf(float v, float lo, float hi)
{
    if (v > hi)
    {
        return hi;
    }
    if (v < lo)
    {
        return lo;
    }
    return v;
}

/**
 * @brief   Chassis_Init：底盘初始化，绑定四个电机
 * @param   chassis  底盘结构体指针
 * @param   fl/fr/rl/rr  四个轮子对应的 VESC 电机指针（顺序 FL/FR/RL/RR）
 * @note    方向系数默认全部 +1；若某电机实测转向相反，把对应 dir[i] 改成 -1 即可
 */
void Chassis_Init(Chassis_TypeDef *chassis,
                  VESC_Motor *fl, VESC_Motor *fr,
                  VESC_Motor *rl, VESC_Motor *rr)
{
    chassis->motor[WHEEL_FL] = fl;
    chassis->motor[WHEEL_FR] = fr;
    chassis->motor[WHEEL_RL] = rl;
    chassis->motor[WHEEL_RR] = rr;

    /* 默认四个电机方向一致，刹车状态清零 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        chassis->dir[i] = 1;
        chassis->brake_target_rpm[i] = 0.0f;
        chassis->brake_still_cnt[i] = 0;
    }

//    /* RL（后左，ID 121）实测转向相反，方向取反 */
//    chassis->dir[WHEEL_RL] = -1;
}

/**
 * @brief   Chassis_WheelVel_To_Rpm：轮子线速度 → 电机转速
 * @param   v  轮子线速度（m/s）
 * @return  电机转速（RPM）
 *
 * 换算关系：
 *   轮子转速 n_wheel = v * 60 / 周长
 *   电机转速 n_motor = n_wheel * GEAR_RATIO（经减速箱，电机比轮子转得快）
 */
float Chassis_WheelVel_To_Rpm(float v)
{
    return v * 60.0f / WHEEL_CIRCUMFERENCE * GEAR_RATIO;
}

/**
 * @brief   Chassis_Rpm_To_WheelVel：电机转速 → 轮子线速度
 * @param   rpm  电机转速（RPM）
 * @return  轮子线速度（m/s）
 *
 * 为 Chassis_WheelVel_To_Rpm 的逆运算（里程计用）。
 */
float Chassis_Rpm_To_WheelVel(float rpm)
{
    return rpm / GEAR_RATIO * WHEEL_CIRCUMFERENCE / 60.0f;
}

/**
 * @brief   Chassis_Speed_Decompose：逆运动学（底盘速度分解）
 * @param   vel        目标车体速度（vx/vy/wz）
 * @param   wheel_vel  输出：四个轮子的线速度（m/s），顺序 FL/FR/RL/RR
 *
 * 全向轮逆运动学（正方形 O 型，轮轴指向中心，驱动方向沿切线）：
 *   设 a = 1/√2（投影系数），R = 轮子到中心距离
 *   V_fl = (-vx - vy)*a + wz*R
 *   V_fr = (-vx + vy)*a + wz*R
 *   V_rl = ( vx - vy)*a + wz*R
 *   V_rr = ( vx + vy)*a + wz*R
 */
void Chassis_Speed_Decompose(const Chassis_Velocity *vel, float wheel_vel[WHEEL_NUM])
{
    float a = CHASSIS_HALF_FACTOR;   /* 1/√2 */
    float R = CHASSIS_RADIUS;        /* 旋转半径 */

    wheel_vel[WHEEL_FL] = (-vel->vx - vel->vy) * a + vel->wz * R;   /* 前左 */
    wheel_vel[WHEEL_FR] = (-vel->vx + vel->vy) * a + vel->wz * R;   /* 前右 */
    wheel_vel[WHEEL_RL] = ( vel->vx - vel->vy) * a + vel->wz * R;   /* 后左 */
    wheel_vel[WHEEL_RR] = ( vel->vx + vel->vy) * a + vel->wz * R;   /* 后右 */
}

/**
 * @brief   Chassis_Speed_Compose：正运动学（轮速合成车体速度）
 * @param   wheel_vel  四个轮子的线速度（m/s），顺序 FL/FR/RL/RR
 * @param   vel        输出：车体速度（vx/vy/wz）
 *
 * 正运动学公式：
 *   vx = (V_rl + V_rr - V_fl - V_fr) / (4*a)
 *   vy = (V_fr + V_rr - V_fl - V_rl) / (4*a)
 *   wz = (V_fl + V_fr + V_rl + V_rr) / (4*R)
 */
void Chassis_Speed_Compose(const float wheel_vel[WHEEL_NUM], Chassis_Velocity *vel)
{
    float a = CHASSIS_HALF_FACTOR;
    float R = CHASSIS_RADIUS;

    vel->vx = (wheel_vel[WHEEL_RL] + wheel_vel[WHEEL_RR] -
               wheel_vel[WHEEL_FL] - wheel_vel[WHEEL_FR]) / (4.0f * a);
    vel->vy = (wheel_vel[WHEEL_FR] + wheel_vel[WHEEL_RR] -
               wheel_vel[WHEEL_FL] - wheel_vel[WHEEL_RL]) / (4.0f * a);
    vel->wz = (wheel_vel[WHEEL_FL] + wheel_vel[WHEEL_FR] +
               wheel_vel[WHEEL_RL] + wheel_vel[WHEEL_RR]) / (4.0f * R);
}

/**
 * @brief   Chassis_Move：底盘运动控制
 * @param   chassis  底盘结构体指针
 * @param   vx       前后线速度 m/s（前为正）
 * @param   vy       左右线速度 m/s（左为正）
 * @param   wz       旋转角速度 rad/s（逆时针为正）
 * @note    需在控制循环中持续调用（VESC 转速指令需不断刷新）
 */
void Chassis_Move(Chassis_TypeDef *chassis, float vx, float vy, float wz)
{
    Chassis_Velocity vel;
    float wheel_vel[WHEEL_NUM];

    vel.vx = vx;
    vel.vy = vy;
    vel.wz = wz;

    /* 1. 车体速度 → 四个轮子线速度 */
    Chassis_Speed_Decompose(&vel, wheel_vel);
    /* 2. 线速度 → 电机转速(RPM)，按安装方向修正符号，限幅后下发到各电机 */
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        float rpm = Chassis_WheelVel_To_Rpm(wheel_vel[i]) * (float)chassis->dir[i];
        rpm = clampf(rpm, -WHEEL_SPEED_MAX, WHEEL_SPEED_MAX);

        /* 空指针保护：未绑定的轮子跳过 */
        if (chassis->motor[i] != NULL)
        {
            if (rpm > -CHASSIS_STOP_RPM && rpm < CHASSIS_STOP_RPM)
            {
                /* ===== 软刹车：目标转速缓降到 0，交给 VESC 速度环闭环减速 ===== */
                float fb_erpm = chassis->motor[i]->RxData.ERpm;   /* 实际电转速反馈 */
                float target  = chassis->brake_target_rpm[i];      /* 缓降目标转速 */

                /* 每周期最多降 DECEL*DT，降到 0 为止 */
                float step = CHASSIS_BRAKE_DECEL * CHASSIS_BRAKE_DT;
                if (target >  step)      target -= step;
                else if (target < -step) target += step;
                else                     target  = 0.0f;
                chassis->brake_target_rpm[i] = target;

                /* 停稳判定：实际转速连续低速 STILL_CYCLES 个周期 → 切电流 0 保持 */
                if (fb_erpm > -CHASSIS_BRAKE_STILL_ERPM && fb_erpm < CHASSIS_BRAKE_STILL_ERPM)
                {
                    chassis->brake_still_cnt[i]++;
                }
                else
                {
                    chassis->brake_still_cnt[i] = 0;
                }

                if (chassis->brake_still_cnt[i] >= CHASSIS_BRAKE_STILL_CYCLES)
                {
                    /* 已停稳：断电保持静止，避免无感零速观测器漂移抖动 */
                    chassis->motor[i]->SetCurrent(chassis->motor[i], 0);
                }
                else
                {
                    /* 仍在减速：下发缓降目标转速，速度环平滑刹车 */
                    chassis->motor[i]->SetRpm(chassis->motor[i], (int32_t)target);
                }
            }
            else
            {
                /* 正常驱动：跟踪目标转速（供进入刹车时无缝缓降），清停稳计数 */
                chassis->brake_target_rpm[i] = rpm;
                chassis->brake_still_cnt[i] = 0;
                chassis->motor[i]->SetRpm(chassis->motor[i], (int32_t)rpm);
            }
        }
    }
}

///**
// * @brief   Chassis_TorqueToCurrent：轮上扭矩 → 电机电流
// * @param   torque_nm  轮上扭矩（Nm）
// * @return  电机电流（A）
// *
// *  电流 = 扭矩 / 轮上等效力矩系数（WHEEL_KT_NM_PER_A）
// */
//float Chassis_TorqueToCurrent(float torque_nm)
//{
//    return torque_nm / WHEEL_KT_NM_PER_A;
//}

///**
// * @brief   Chassis_CurrentToTorque：电机电流 → 轮上扭矩
// * @param   current_a  电机电流（A）
// * @return  轮上扭矩（Nm）
// *
// *  扭矩 = 电流 × 轮上等效力矩系数（WHEEL_KT_NM_PER_A）
// */
//float Chassis_CurrentToTorque(float current_a)
//{
//    return current_a * WHEEL_KT_NM_PER_A;
//}

///**
// * @brief   Chassis_Force_Decompose：动力学逆解算（力 → 电流前馈）
// * @param   chassis    底盘结构体指针
// * @param   Fx         车体期望合力 x 方向（N，前为正）
// * @param   Fy         车体期望合力 y 方向（N，左为正）
// * @param   Tz         车体期望力矩（Nm，逆时针为正）
// * @param   current_a  输出：四个轮子的目标电流（A），顺序 FL/FR/RL/RR
// *
// * 与速度分解对偶：把车体期望力/力矩按轮子驱动方向投影，
// * 再通过轮上力矩系数换算成电流，作为前馈量。
// *
// *   F_fl = (-Fx - Fy)*a + Tz/(4*R)
// *   F_fr = (-Fx + Fy)*a + Tz/(4*R)
// *   F_rl = ( Fx - Fy)*a + Tz/(4*R)
// *   F_rr = ( Fx + Fy)*a + Tz/(4*R)
// *   I_i  = F_i * WHEEL_RADIUS / WHEEL_KT_NM_PER_A
// */
//void Chassis_Force_Decompose(const Chassis_TypeDef *chassis, float Fx, float Fy, float Tz,
//                             float current_a[WHEEL_NUM])
//{
//    float a = CHASSIS_HALF_FACTOR;
//    float R = CHASSIS_RADIUS;
//    float wheel_force[WHEEL_NUM];

//    /* 1. 车体力/力矩 → 四个轮子的驱动力（N） */
//    wheel_force[WHEEL_FL] = (-Fx - Fy) * a + Tz / (4.0f * R);   /* 前左 */
//    wheel_force[WHEEL_FR] = (-Fx + Fy) * a + Tz / (4.0f * R);   /* 前右 */
//    wheel_force[WHEEL_RL] = ( Fx - Fy) * a + Tz / (4.0f * R);   /* 后左 */
//    wheel_force[WHEEL_RR] = ( Fx + Fy) * a + Tz / (4.0f * R);   /* 后右 */

//    /* 2. 驱动力 → 轮上扭矩 → 电流，并按安装方向修正符号 + 电流限幅 */
//    for (int i = 0; i < WHEEL_NUM; i++)
//    {
//        float torque = wheel_force[i] * WHEEL_RADIUS;          /* 力 × 轮半径 = 轮上扭矩 */
//        float current = Chassis_TorqueToCurrent(torque);       /* 扭矩 → 电流 */
//        current *= (float)chassis->dir[i];                     /* 方向修正 */
//        current_a[i] = clampf(current, -MOTOR_MAX_CURRENT_A, MOTOR_MAX_CURRENT_A);  /* 限幅 */
//    }
//}
