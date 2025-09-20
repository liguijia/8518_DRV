#ifndef __FOC_CONTROLLER_H__
#define __FOC_CONTROLLER_H__

#include "analog_signal.h"
#include "bsp_kth7823.h"
#include "bsp_pwm.h"
#include "foc_pid.h"
#include <stdbool.h>

/* ----------------- 结构体定义 ----------------- */

typedef struct {
  // 电流环 PID
  FOC_PID_ctrl_t id_pid;
  FOC_PID_ctrl_t iq_pid;

  // 速度环 PID
  FOC_PID_ctrl_t speed_pid;

  // 位置环 PID
  FOC_PID_ctrl_t position_pid;

  // 目标参考值
  float id_ref;
  float iq_ref;
  float speed_ref;
  float position_ref;

  // 当前测量值
  float theta_e;       // 电角度
  float speed_mech;    // 机械速度
  float position_mech; // 机械位置

  // PWM 输出
  FOC_PWM_t pwm;

  // 设备句柄
  KTH7823_HandleTypeDef *encoder;
} FOC_Controller_t;

/* 全局 FOC 控制器实例 */

extern FOC_Controller_t foc;

/* ----------------- 外部接口函数 ----------------- */

// 初始化 FOC 控制器
void FOC_Controller_Init(FOC_Controller_t *foc, KTH7823_HandleTypeDef *encoder,
                         float dt_current, float dt_speed, float dt_position);

// 电流环更新
void FOC_CurrentLoop_Update(FOC_Controller_t *foc,
                            const phase_current_t *i_abc);

// 速度环更新
void FOC_SpeedLoop_Update(FOC_Controller_t *foc);

// 位置环更新
void FOC_PositionLoop_Update(FOC_Controller_t *foc);

// 设置目标
void FOC_Controller_SetIdIq(FOC_Controller_t *foc, float id_ref, float iq_ref);
void FOC_Controller_SetSpeed(FOC_Controller_t *foc, float speed_ref);
void FOC_Controller_SetPosition(FOC_Controller_t *foc, float position_ref);

#endif /* __FOC_CONTROLLER_H__ */
