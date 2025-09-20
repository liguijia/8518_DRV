#include "foc_controller.h"
#include "foc_core.h"
#include "motor_config.h"
#include <math.h>

#define TWO_PI 6.28318530718f

FOC_Controller_t foc;

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
  FOC_PID_Init(&foc->iq_pid, FOC_PID_TYPE_PI, 0.2f, 0.01f, 0.0f, dt_current,
               10.0f, 12.0f, 0.001f);

  // 速度环 PID
  FOC_PID_Init(&foc->speed_pid, FOC_PID_TYPE_PI, 0.5f, 0.1f, 0.0f, dt_speed,
               10.0f, 12.0f, 0.01f);

  // 位置环 PID
  FOC_PID_Init(&foc->position_pid, FOC_PID_TYPE_PI, 0.5f, 0.05f, 0.0f,
               dt_position, 100.0f, 500.0f, 0.01f);
}

float angle = 0.0f;

/* ----------------- 电流环 ----------------- */
void FOC_CurrentLoop_Update(FOC_Controller_t *foc,
                            const phase_current_t *i_abc) {
  if (!foc || !i_abc)
    return;

  float angle_deg;
  if (BSP_KTH7823_ReadAngle(foc->encoder, &angle_deg) != KTH7823_OK)
    return;

  angle = angle_deg;

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

  float pos_now;
  if (BSP_KTH7823_ReadAngle(foc->encoder, &pos_now) != KTH7823_OK)
    return;
  // 把角度转成弧度存储
  pos_now = pos_now * TWO_PI / 360.0f;
  // 机械角速度(rad/s)
  float delta = pos_now - foc->position_mech;
  if (delta > 180.0f)
    delta -= 360.0f;
  else if (delta < -180.0f)
    delta += 360.0f;

  foc->speed_mech = delta / foc->speed_pid.dt;
  foc->position_mech = pos_now;

  // 速度环生成 iq_ref
  foc->iq_ref =
      FOC_PID_Compute(&foc->speed_pid, foc->speed_mech, foc->speed_ref);
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
