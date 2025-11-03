#include "foc_selfcalib.h"
#include "bsp_pwm.h"
#include "foc_core.h"
#include "kth7823.h"
#include "motor_config.h"
#include <math.h>

/* 放到文件顶部（如果尚未定义） */
#ifndef M_PI_2
#define M_PI_2 (3.14159265358f / 2.0f)
#endif
#ifndef TWO_PI
#define TWO_PI (6.28318530718f)
#endif
extern KTH7823_HandleTypeDef henc1;
/* ----------------- 辅助：将角度归一化到 [0, TWO_PI) ----------------- */
static inline float norm_rad_2pi(float x) {
  while (x < 0.0f)
    x += TWO_PI;
  while (x >= TWO_PI)
    x -= TWO_PI;
  return x;
}

/* ----------------- 辅助：应用开环电压 (持续调用安全) ----------------- */
static void FOC_Align_Rotor_Apply(FOC_Controller_t *foc, float v_q,
                                  float theta_e) {
  dq_t v_dq;
  alpha_beta_t v_alpha_beta;

  v_dq.d = 0.0f;
  v_dq.q =
      v_q; /* 注意：这里用正号，具体方向请根据你系统的相序/极性调整测试时反号 */

  InvPark(&v_dq, theta_e, &v_alpha_beta);

  /* 将相电压按 Vbus 归一化到 [-0.5,0.5] 以适配 SVPWM_Offset_Float 的期望 */
  float Vbus = analogdata.input_voltage;
  if (Vbus <= 5.0f) {
    /* 如果 Vbus不可信，使用 MOTOR_MAX_VOLTAGE 做退化保护 */
    Vbus = MOTOR_MAX_VOLTAGE;
  }
  /* 理论上相电压最大可达 Vbus/2，对应归一化 ±0.5 */
  float scale = 1.0f / Vbus; /* v_phase / Vbus -> 在 [-0.5,0.5] 大小范围内 */
  v_alpha_beta.alpha *= scale;
  v_alpha_beta.beta *= scale;

  /* 限幅以防越界（防止 SVPWM 生成超限占空比） */
  const float LIM = 0.49f;
  if (v_alpha_beta.alpha > LIM)
    v_alpha_beta.alpha = LIM;
  if (v_alpha_beta.alpha < -LIM)
    v_alpha_beta.alpha = -LIM;
  if (v_alpha_beta.beta > LIM)
    v_alpha_beta.beta = LIM;
  if (v_alpha_beta.beta < -LIM)
    v_alpha_beta.beta = -LIM;

  SVPWM_Offset_Float(&v_alpha_beta, &foc->pwm);
  BSP_PWM_Set_Duty(&foc->pwm);
}

/* ----------------- 初始化 ----------------- */
void FOC_SelfCalib_Init(FOC_SelfCalib_Handle_t *calib,
                        const FOC_SelfCalib_Config_t *cfg) {
  if (calib == NULL || cfg == NULL)
    return;
  calib->cfg = *cfg;
  calib->state = SELF_CALIB_IDLE;
  calib->zero_offset = 0.0f;
}

/*-----------------执行零点自动校准
 * (阻塞实现，但期间持续输出)-----------------*/
FOC_SelfCalib_State_t FOC_SelfCalib_Execute(FOC_Controller_t *foc,
                                            FOC_SelfCalib_Handle_t *calib) {
  if (foc == NULL || calib == NULL)
    return SELF_CALIB_FAILED;
  if (calib->state == SELF_CALIB_RUNNING)
    return SELF_CALIB_FAILED;

  /* 基本参数检查 */
  float align_v = calib->cfg.align_voltage;
  float align_t = calib->cfg.align_time;
  if (align_t < 0.1f || align_t > 10.0f) {
    calib->state = SELF_CALIB_FAILED;
    return SELF_CALIB_FAILED;
  }

  calib->state = SELF_CALIB_RUNNING;

  /* 期望电角对齐值（电角），例如把转子锁在电角 pi/2 */
  const float theta_align = M_PI_2;

  /* 持续输出对齐电压，分片等待并持续输出，避免只输出一次后被别处覆盖 */
  uint32_t total_ms = (uint32_t)(align_t * 1000.0f);
  const uint32_t step_ms = 10u;
  uint32_t elapsed = 0u;

  /* 开始输出对齐电压 */
  while (elapsed < total_ms) {
    FOC_Align_Rotor_Apply(foc, align_v, theta_align);
    HAL_Delay(step_ms); /* 允许中断处理、ADC注入回调、并保持PWM */
    elapsed += step_ms;
  }

  /* 取若干次角度读数做平均以减小噪声（典型 16 次） */
  const int samples = 32;
  float sum_deg = 0.0f;
  int got = 0;
  for (int i = 0; i < samples; ++i) {
    float deg;
    if (KTH7823_ReadAngle(&henc1, &deg) == KTH7823_OK) {
      sum_deg += deg;
      ++got;
    }
    HAL_Delay(3);
  }

  /* 停止输出 PWM（或恢复到初始安全状态） */
  BSP_PWM_Stop();

  if (got == 0) {
    calib->state = SELF_CALIB_FAILED;
    return SELF_CALIB_FAILED;
  }

  float avg_deg = sum_deg / (float)got;
  /* 机械弧度 */
  float mech_rad = avg_deg * TWO_PI / 360.0f;

  /* 当前电角度（由机械角度与极对数决定） */
  float elec_rad_now = fmodf(mech_rad * MOTOR_POLE_PAIRS, TWO_PI);

  /* 正确的零点偏移计算（使得 theta_e = mech*pp + zero_offset = theta_align） */
  float zero_offset_rad = theta_align - elec_rad_now;
  zero_offset_rad = norm_rad_2pi(zero_offset_rad);

  /* 保存并返回 */
  foc->zero_offset_rad = zero_offset_rad;
  calib->zero_offset = zero_offset_rad;
  calib->state = SELF_CALIB_DONE;
  return SELF_CALIB_DONE;
}
