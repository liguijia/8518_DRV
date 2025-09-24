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
  // 初始值
  foc->theta_e = 0.0f;
  foc->pos_now_deg = 0.0f;
  foc->speed_mech_rpm = 0.0f;
  foc->position_mech_deg = 0.0f;

  // 电流环 PID
  FOC_PID_Init(&foc->id_pid, FOC_PID_TYPE_PI, 0.2f, 0.01f, 0.0f, dt_current,
               10.0f, 12.0f, 0.001f);
  FOC_PID_Init(&foc->iq_pid, FOC_PID_TYPE_PI, 0.25f, 0.005f, 0.0f, dt_current,
               5.0f, 20.0f, 0.001f);

  PLL_Init(&foc->speed_pll, 20.0f, 10.0f, dt_speed);
  // 速度环 PID
  FOC_PID_Init(&foc->speed_pid, FOC_PID_TYPE_PI, 0.05f, 0.05f, 0.0f, dt_speed,
               10.0f, 20.0f, 0.001f);

  // 位置环 PID
  FOC_PID_Init(&foc->position_pid, FOC_PID_TYPE_PI, 0.05f, 0.005f, 0.0f,
               dt_position, 10.0f, 300.0f, 0.001f);
}

/* ----------------- 电流环 ----------------- */
void FOC_CurrentLoop_Update(FOC_Controller_t *foc,
                            const phase_current_t *i_abc) {
  if (!foc || !i_abc)
    return;

  if (BSP_KTH7823_ReadAngle(foc->encoder, &foc->pos_now_deg) != KTH7823_OK)
    return;

  // 转成弧度 (0..2π)
  float angle_rad = foc->pos_now_deg * TWO_PI / 360.0f;

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

  if (BSP_KTH7823_ReadAngle(foc->encoder, &foc->pos_now_deg) != KTH7823_OK)
    return;

  // --- 角度展开 ---
  static float last_deg = 0.0f;
  static int revolutions = 0;

  float delta_deg = foc->pos_now_deg - last_deg;
  if (delta_deg > 180.0f) {
    revolutions--;
  } else if (delta_deg < -180.0f) {
    revolutions++;
  }

  float pos_cont_deg = foc->pos_now_deg + 360.0f * revolutions;
  last_deg = foc->pos_now_deg;

  // --- 差分求速度 ---
  float delta_cont_deg = pos_cont_deg - foc->position_mech_deg;
  foc->position_mech_deg = pos_cont_deg;

  float raw_speed_dps = delta_cont_deg / foc->speed_pid.dt; // deg/s
  float raw_speed_rpm = raw_speed_dps * 60.0f / 360.0f;     // rpm

  // --- 滤波 ---
  const float alpha = 0.1f;
  foc->speed_mech_rpm =
      (1.0f - alpha) * foc->speed_mech_rpm + alpha * raw_speed_rpm;

  // --- PID 控制 ---
  foc->iq_ref =
      FOC_PID_Compute(&foc->speed_pid, foc->speed_ref, foc->speed_mech_rpm);

  // 限幅 iq_ref
  if (foc->iq_ref > MOTOR_MAX_CURRENT)
    foc->iq_ref = MOTOR_MAX_CURRENT;
  else if (foc->iq_ref < -MOTOR_MAX_CURRENT)
    foc->iq_ref = -MOTOR_MAX_CURRENT;
}

void FOC_PLLSpeedLoop_Update(FOC_Controller_t *foc) {
  if (!foc)
    return;

  if (BSP_KTH7823_ReadAngle(foc->encoder, &foc->pos_now_deg) != KTH7823_OK)
    return;

  // --- 角度转换为弧度 ---
  float pos_now_rad = foc->pos_now_deg * M_PI / 180.0f;

  // --- 使用 PLL 估算速度 ---
  float speed_rad_s = PLL_Update(&foc->speed_pll, pos_now_rad);

  // 转换为 rpm
  foc->speed_mech_rpm = speed_rad_s * 60.0f / (2.0f * M_PI);

  // --- PID 控制 ---
  foc->iq_ref =
      FOC_PID_Compute(&foc->speed_pid, foc->speed_ref, foc->speed_mech_rpm);

  // 限幅 iq_ref
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

  foc->position_mech_deg = pos_now;

  // 位置环生成 speed_ref
  foc->speed_ref = FOC_PID_Compute(&foc->position_pid, foc->position_mech_deg,
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
