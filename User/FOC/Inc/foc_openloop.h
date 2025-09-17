#ifndef __FOC_OPENLOOP_H__
#define __FOC_OPENLOOP_H__

#include "foc_core.h"
#include <stdint.h>

/* ------------------ 配置结构体 ------------------ */
typedef struct {
  float voltage;        // 电压幅值 (V)
  float speed;          // 电角速度 (rad/s)
  float theta_e;        // 当前电角度
  float control_period; // 控制周期 (s)
  FOC_PWM_t pwm;        // PWM 占空比
} FOC_OpenLoop_t;

/* ------------------ 接口函数 ------------------ */
void FOC_OpenLoop_Init(FOC_OpenLoop_t *loop, float target_voltage, float speed);
void FOC_OpenLoop_Update(FOC_OpenLoop_t *loop, FOC_PWM_t *pwm, float dt);
void FOC_OpenLoop_SetSpeed(FOC_OpenLoop_t *loop, float speed);
void FOC_OpenLoop_SetVoltage(FOC_OpenLoop_t *loop, float voltage);

#endif
