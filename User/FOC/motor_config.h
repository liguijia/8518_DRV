#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include <stdint.h>

/* ----------------- 电机基本参数 ----------------- */
#define MOTOR_POLE_PAIRS 14             // 电机极对数
#define MOTOR_PHASE_RESISTANCE 0.05f    // 相电阻 (Ohm)
#define MOTOR_PHASE_INDUCTANCE 0.00012f // 相电感 (H)
#define MOTOR_BACK_EMF_CONST 0.08f      // 反电动势常数 (V/rad/s)
#define MOTOR_TORQUE_CONST 0.08f        // 转矩常数 (Nm/A)
#define MOTOR_MAX_CURRENT 12.0f         // 最大电流 (A)
#define MOTOR_MAX_VOLTAGE 24.0f         // 最大电压 (V)

/* ----------------- FOC 控制参数 ----------------- */
#define FOC_DT_CURRENT 0.0000417f // 电流环 24 kHz
#define FOC_DT_SPEED 0.001f       // 速度环周期 1ms
#define FOC_DT_POSITION 0.01f     // 位置环周期 10ms

/* ----------------- 默认初始值 ----------------- */
#define MOTOR_DEFAULT_ZERO_ANGLE 0.0f // 机械零点角度

#endif /* __MOTOR_CONFIG_H__ */
