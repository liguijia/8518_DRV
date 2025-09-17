#include "bsp_pwm.h"
#include "foc_core.h"
#include "foc_pid.h"
#include <math.h>

#define SQRT3_BY_2 0.86602540378f
#define OPENLOOP_MAX_VOLT 0.8f    // 最大 q 轴电压（防止饱和）
#define OPENLOOP_MIN_VOLT 0.1f    // 最小 q 轴电压起始值
#define OPENLOOP_RAMP_STEP 0.002f // 每次更新电压增加量
#define OPENLOOP_SCALE 0.9f       // PWM 缩放系数，避免饱和

typedef struct {
  float theta_e;        // 电角度
  float q_voltage;      // 当前 q 轴电压
  float target_voltage; // 目标 q 轴电压
  float speed;          // 电角速度 rad/s
} FOC_OpenLoop_t;

// 初始化开环结构体
void FOC_OpenLoop_Init(FOC_OpenLoop_t *loop, float target_voltage,
                       float speed) {
  loop->theta_e = 0.0f;
  loop->q_voltage = OPENLOOP_MIN_VOLT;
  loop->target_voltage = target_voltage;
  if (loop->target_voltage > OPENLOOP_MAX_VOLT)
    loop->target_voltage = OPENLOOP_MAX_VOLT;
  loop->speed = speed;
}

// 开环强拖更新函数（100~200us 调用一次）
void FOC_OpenLoop_Update(FOC_OpenLoop_t *loop, FOC_PWM_t *pwm, float dt) {
  alpha_beta_t v_alpha_beta;
  dq_t v_dq;

  // 电压 ramp-up
  if (loop->q_voltage < loop->target_voltage)
    loop->q_voltage += OPENLOOP_RAMP_STEP;
  if (loop->q_voltage > loop->target_voltage)
    loop->q_voltage = loop->target_voltage;

  // 构造 d-q 电压（d=0, q=当前电压）
  v_dq.d = 0.0f;
  v_dq.q = loop->q_voltage;

  // 逆 Park 得到 αβ 电压
  InvPark(&v_dq, loop->theta_e, &v_alpha_beta);

  // SVPWM Offset 生成 PWM
  // 缩放防饱和
  v_alpha_beta.alpha *= OPENLOOP_SCALE;
  v_alpha_beta.beta *= OPENLOOP_SCALE;
  SVPWM_Offset_Float(&v_alpha_beta, pwm);

  // 输出 PWM
  BSP_PWM_Set_Duty(pwm);

  // 更新电角度
  loop->theta_e += loop->speed * dt;
  if (loop->theta_e > 2.0f * M_PI)
    loop->theta_e -= 2.0f * M_PI;
}
