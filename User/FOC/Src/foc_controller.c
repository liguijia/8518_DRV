#include "foc_controller.h"
#include "foc_core.h"
#include "motor_config.h"
#include <math.h>

#define TWO_PI 6.28318530718f
// constexpr float PI = 3.14159265359f;

/* ----------------- 初始化 ----------------- */
void FOC_Controller_Init(FOC_Controller_t *foc, KTH7823_HandleTypeDef *encoder,
                         float dt_current, float dt_speed, float dt_position) {
  if (!foc || !encoder)
    return;

  foc->encoder = encoder;
  foc->id_ref = 0.0f;
  foc->iq_ref = 0.0f;
  foc->speed_ref = 0.0f;
  foc->position_ref = 0.0f;
  foc->theta_e = 0.0f;
  foc->speed_mech = 0.0f;
  foc->position_mech = 0.0f;

  // 电流环 PID
  FOC_PID_Init(&foc->id_pid, FOC_PID_TYPE_PI, 0.2f, 0.01f, 0.0f, dt_current,
               10.0f, 12.0f, 0.001f);
  FOC_PID_Init(&foc->iq_pid, FOC_PID_TYPE_PI, 0.5f, 0.005f, 0.0f, dt_current,
               5.0f, 20.0f, 0.001f);

  // 速度环 PID
  FOC_PID_Init(&foc->speed_pid, FOC_PID_TYPE_PI, 0.5f, 0.01f, 0.0f, dt_speed,
               5.0f, 20.0f, 0.0001f);

  // 位置环 PID
  FOC_PID_Init(&foc->position_pid, FOC_PID_TYPE_PI, 0.5f, 0.05f, 0.0f,
               dt_position, 100.0f, 500.0f, 0.01f);
}

/* ----------------- 电流环 ----------------- */
void FOC_CurrentLoop_Update(FOC_Controller_t *foc,
                            const phase_current_t *i_abc) {
  if (!foc || !i_abc)
    return;

  float angle_deg;
  if (BSP_KTH7823_ReadAngle(foc->encoder, &angle_deg) != KTH7823_OK)
    return;

  // 转成弧度 (0..2π)
  float angle_rad = angle_deg * TWO_PI / 360.0f;

  // 电角度 = 机械角度 * 极对数
  foc->theta_e = fmodf(angle_rad * MOTOR_POLE_PAIRS, TWO_PI);

  // 更新 PWM
  FOC_UpdatePWM(i_abc, foc->theta_e, &foc->pwm, &foc->id_pid, foc->id_ref,
                &foc->iq_pid, foc->iq_ref);
}

/* ----------------- 速度环 ----------------- */
void FOC_SpeedLoop_Update(FOC_Controller_t *foc) {
  if (!foc)
    return;

  float pos_now_deg;
  if (BSP_KTH7823_ReadAngle(foc->encoder, &pos_now_deg) != KTH7823_OK)
    return;

  // 转换成弧度 (0 ~ 2π)
  float pos_now = pos_now_deg * TWO_PI / 360.0f;

  // 角度差 (弧度)
  float delta = pos_now - foc->position_mech;
  if (delta > M_PI)
    delta -= TWO_PI;
  else if (delta < -M_PI)
    delta += TWO_PI;

  // 计算机械速度 (rad/s)，差分法
  float raw_speed = delta / foc->speed_pid.dt;

  // 速度滤波（指数平均法）
  const float alpha = 0.1f; // 0~1，越小越平滑
  foc->speed_mech = (1.0f - alpha) * foc->speed_mech + alpha * raw_speed;

  // 保存最新位置
  foc->position_mech = pos_now;

  // 速度环 PID：注意参数顺序（假设 Compute(ref, feedback)）
  foc->iq_ref =
      FOC_PID_Compute(&foc->speed_pid, foc->speed_ref, foc->speed_mech);

  // 限幅 iq_ref，避免过大
  if (foc->iq_ref > MOTOR_MAX_CURRENT)
    foc->iq_ref = MOTOR_MAX_CURRENT;
  else if (foc->iq_ref < -MOTOR_MAX_CURRENT)
    foc->iq_ref = -MOTOR_MAX_CURRENT;
}

/* ----------------- 位置环 ----------------- */
void FOC_PositionLoop_Update(FOC_Controller_t *foc) {
  if (!foc)
    return;

  float pos_now;
  if (BSP_KTH7823_ReadAngle(foc->encoder, &pos_now) != KTH7823_OK)
    return;

  foc->position_mech = pos_now;

  // 位置环生成 speed_ref
  foc->speed_ref = FOC_PID_Compute(&foc->position_pid, foc->position_mech,
                                   foc->position_ref);
}

/* ----------------- 设置目标 ----------------- */
void FOC_Controller_SetIdIq(FOC_Controller_t *foc, float id_ref, float iq_ref) {
  if (!foc)
    return;
  foc->id_ref = id_ref;
  foc->iq_ref = iq_ref;
}

void FOC_Controller_SetSpeed(FOC_Controller_t *foc, float speed_ref) {
  if (!foc)
    return;
  foc->speed_ref = speed_ref;
}

void FOC_Controller_SetPosition(FOC_Controller_t *foc, float position_ref) {
  if (!foc)
    return;
  foc->position_ref = position_ref;
}
