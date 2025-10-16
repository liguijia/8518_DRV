#ifndef __FOC_CONTROLLER_H__
#define __FOC_CONTROLLER_H__

#include "analog_signal.h"
#include "bsp_kth7823.h"
#include "bsp_pwm.h"
#include "foc_pid.h"
#include "pll_speed_estimator.h"
#include <stdbool.h>

/* ----------------- 结构体定义 ----------------- */

typedef struct {
  // === 电流测量 ===
  phase_current_t phase_current; ///< 三相电流 A, B, C

  // === 电流环 PID ===
  FOC_PID_ctrl_t id_pid;
  FOC_PID_ctrl_t iq_pid;

  // === 速度环 PID ===
  FOC_PID_ctrl_t speed_pid;

  // === 位置环 PID ===
  FOC_PID_ctrl_t position_pid;

  // === 速度估算器 ===
  PLL_Handle speed_pll;

  // === 参考值 ===
  float id_ref;
  float iq_ref;
  float speed_ref;
  float position_ref;

  // === 实时测量 ===
  //
  float theta_e;         ///< 电角度(rad)
  float zero_offset_rad; // 零点偏移（弧度）
  //
  float pos_now_deg; ///< 转子位置(°)
  float pos_out_deg; ///< 出轴位置(°)
  //
  float speed_now_rpm; ///< 转子速度(rpm)
  float speed_out_rpm; ///< 出轴速度(rpm)

  // === PWM 输出 ===
  FOC_PWM_t pwm;

  // === 编码器句柄 ===
  KTH7823_HandleTypeDef *encoder;
} FOC_Controller_t;

extern FOC_Controller_t foc;

/* ----------------- 外部接口函数 ----------------- */

// 初始化 FOC 控制器
void FOC_Controller_Init(FOC_Controller_t *foc, KTH7823_HandleTypeDef *encoder,
                         float dt_current, float dt_speed, float dt_position);

// 电流环更新
void FOC_CurrentLoop_Update(FOC_Controller_t *foc);

// 速度环更新
void FOC_PLLSpeedLoop_Update(FOC_Controller_t *foc);
// 位置环更新
void FOC_PositionLoop_Update(FOC_Controller_t *foc);

// 设置目标
void FOC_Controller_SetIdIq(FOC_Controller_t *foc, float id_ref, float iq_ref);
void FOC_Controller_SetSpeed(FOC_Controller_t *foc, float speed_ref);
void FOC_Controller_SetPosition(FOC_Controller_t *foc, float position_ref);

#endif /* __FOC_CONTROLLER_H__ */
