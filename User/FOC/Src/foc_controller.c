#include "foc_controller.h"
#include "encoder.h"
#include "foc_core.h"
#include "motor_config.h"
#include <math.h>

#define TWO_PI 6.28318530718f
// constexpr float PI = 3.14159265359f;
static inline float norm_rad(float x) {
  while (x < 0.0f)
    x += TWO_PI;
  while (x >= TWO_PI)
    x -= TWO_PI;
  return x;
}
extern KTH7823_HandleTypeDef henc1;
/* ----------------- 初始化 ----------------- */
void FOC_Controller_Init(FOC_Controller_t *foc, float dt_current,
                         float dt_speed, float dt_position) {
  if (!foc)
    return;
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
  FOC_PID_Init(&foc->id_pid, FOC_PID_TYPE_PI, 0.0375f, 0.0075f, 0.0f,
               dt_current, 10.0f, 180.0f, 0.0005f);
  FOC_PID_Init(&foc->iq_pid, FOC_PID_TYPE_PI, 0.875f, 0.225f, 0.0005f,
               dt_current, 10.0f, 180.0f, 0.0005f);
  // 速度环 PLL
  PLL_Init(&foc->speed_pll, 27.5f, 12.5f, dt_speed);
  // 速度环 PID
  FOC_PID_Init(&foc->speed_pid, FOC_PID_TYPE_PI, 0.125f, 0.0875, 0.0f, dt_speed,
               25.0f, MOTOR_MAX_CURRENT, 0.001f);

  // 位置环 PID
  FOC_PID_Init(&foc->position_pid, FOC_PID_TYPE_PI, 1.25f, 0.005f, 0.0f,
               dt_position, 10.0f, 300.0f, 0.001f);
}

/* ----------------- 电流环 ----------------- */
void FOC_CurrentLoop_Update(FOC_Controller_t *foc) {
  if (!foc)
    return;

  // 读取编码器角度
  // 注意：如果您的电流环执行频率极高，此处的SPI读取可能会造成延时，应考虑将角度读取放在定时器更新中断中。
  if (Encoder_ReadAngle(&rotor_encoder, &foc->pos_now_deg) != ENCODER_OK)
    return;

  // 1. 转成机械弧度 (0..2π)
  float mech_rad = foc->pos_now_deg * TWO_PI / 360.0f;

  // 2. 机械角度转电角度
  float elec_rad = mech_rad * MOTOR_POLE_PAIRS;

  // 在电角度域应用零点校准偏移
  // foc->zero_offset_rad 是一个电角度值。
  elec_rad += foc->zero_offset_rad;

  // 4. 归一化电角度 (0..2π)
  foc->theta_e = norm_rad(elec_rad);
  // 5. 调用电流环控制算法
  // foc->theta_e 现在是经过零点修正的准确电角度
  FOC_UpdatePWM(foc);
}

/* ----------------- 速度环 ----------------- */
void FOC_PLLSpeedLoop_Update(FOC_Controller_t *foc) {
  if (!foc)
    return;

  // --- 角度转换为弧度 ---
  float pos_now_rad = foc->pos_now_deg * M_PI / 180.0f;

  // --- 使用 PLL 估算速度 ---
  float speed_rad_s = PLL_Update(&foc->speed_pll, pos_now_rad);

  // 转换为 rpm
  foc->speed_now_rpm = speed_rad_s * 60.0f / (2.0f * M_PI);

  // --- PID 控制 ---
  foc->iq_ref =
      FOC_PID_Compute(&foc->speed_pid, foc->speed_ref, foc->speed_now_rpm);

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
  if (Encoder_ReadAngle(&output_encoder, &foc->pos_out_deg) != ENCODER_OK)
    return;

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
