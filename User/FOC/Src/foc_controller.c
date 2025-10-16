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
  foc->zero_offset_rad = MOTOR_DEFAULT_ZERO_ANGLE * M_PI / 180.0f;
  foc->pos_now_deg = 0.0f;
  foc->speed_now_rpm = 0.0f;
  foc->pos_out_deg = 0.0f;

  // 电流环 PID
  FOC_PID_Init(&foc->id_pid, FOC_PID_TYPE_PI, 0.0525f, 0.0005f, 0.0f,
               dt_current, 10.0f, 5.0f, 0.001f);
  FOC_PID_Init(&foc->iq_pid, FOC_PID_TYPE_PI, 0.125f, 0.075f, 0.0f, dt_current,
               10.0f, 30.0f, 0.001f);
  // 速度环 PLL
  PLL_Init(&foc->speed_pll, 27.5f, 12.5f, dt_speed);
  // 速度环 PID
  FOC_PID_Init(&foc->speed_pid, FOC_PID_TYPE_PI, 0.075f, 0.025f, 0.0f, dt_speed,
               10.0f, 20.0f, 0.001f);

  // 位置环 PID
  FOC_PID_Init(&foc->position_pid, FOC_PID_TYPE_PI, 1.25f, 0.005f, 0.0f,
               dt_position, 10.0f, 300.0f, 0.001f);
}

/* ----------------- 电流环 ----------------- */
void FOC_CurrentLoop_Update(FOC_Controller_t *foc) {
  if (!foc)
    return;

  // 读取编码器角度
  if (BSP_KTH7823_ReadAngle(foc->encoder, &foc->pos_now_deg) != KTH7823_OK)
    return;

  // 转成弧度 (0..2π)
  float mech_rad = foc->pos_now_deg * TWO_PI / 360.0f;

  // 加上零点校准偏移
  mech_rad -= foc->zero_offset_rad;
  if (mech_rad < 0.0f)
    mech_rad += TWO_PI;

  // 电角度 = 机械角度 * 极对数
  foc->theta_e = fmodf(mech_rad * MOTOR_POLE_PAIRS, TWO_PI);

  // 调用电流环控制算法
  FOC_UpdatePWM(foc);
}

/* ----------------- 速度环 ----------------- */
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
  foc->speed_now_rpm = speed_rad_s * 60.0f / (2.0f * M_PI);

  // --- PID 控制 ---
  // foc->iq_ref =
  //     FOC_PID_Compute(&foc->speed_pid, foc->speed_ref, foc->speed_now_rpm);

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

  foc->pos_out_deg = pos_now;

  /* 单圈最短路径误差 */
  float err = foc->position_ref - foc->pos_out_deg;
  if (err > 180.0f)
    err -= 360.0f;
  if (err < -180.0f)
    err += 360.0f;

  /* 位置 PID 用误差计算，输出 speed_ref */
  foc->speed_ref = FOC_PID_Compute(&foc->position_pid, err, 0.0f);
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
