#include <math.h>

#include "analog_signal.h"
#include "bsp_pwm.h"
#include "foc_core.h"
#include "foc_pid.h"

#define SQRT3_BY_2 0.86602540378f

/* ------------------ Clarke ------------------ */
// 三电流 Clarke 变换
void Clarke(const phase_current_t *i_abc, alpha_beta_t *i_alpha_beta) {
  i_alpha_beta->alpha = (2.0f * i_abc->a - i_abc->b - i_abc->c) * (1.0f / 3.0f);
  i_alpha_beta->beta = (i_abc->b - i_abc->c) * (1.0f / 1.73205080757f); // 1/√3
}

/* ------------------ Inverse Clarke ------------------ */
void InvClarke(const alpha_beta_t *i_alpha_beta, phase_current_t *i_abc) {
  i_abc->a = i_alpha_beta->alpha;
  i_abc->b = -0.5f * i_alpha_beta->alpha + (SQRT3_BY_2 * i_alpha_beta->beta);
  i_abc->c = -0.5f * i_alpha_beta->alpha - (SQRT3_BY_2 * i_alpha_beta->beta);
}

/* ------------------ Park ------------------ */
void Park(const alpha_beta_t *i_alpha_beta, float theta_e, dq_t *i_dq) {
  float c = cosf(theta_e);
  float s = sinf(theta_e);

  i_dq->d = i_alpha_beta->alpha * c + i_alpha_beta->beta * s;
  i_dq->q = -i_alpha_beta->alpha * s + i_alpha_beta->beta * c;
}

void InvPark(const dq_t *i_dq, float theta_e, alpha_beta_t *i_alpha_beta) {
  float c = cosf(theta_e);
  float s = sinf(theta_e);

  i_alpha_beta->alpha = i_dq->d * c - i_dq->q * s;
  i_alpha_beta->beta = i_dq->d * s + i_dq->q * c;
}

/* ------------------ SVPWM Offset ------------------ */
static float foc_maxf(float a, float b) { return a > b ? a : b; }
static float foc_minf(float a, float b) { return a < b ? a : b; }

void SVPWM_Offset_Float(const alpha_beta_t *volt, FOC_PWM_t *pwm) {
  float va = volt->alpha;
  float vb = -0.5f * volt->alpha + SQRT3_BY_2 * volt->beta;
  float vc = -0.5f * volt->alpha - SQRT3_BY_2 * volt->beta;

  float vmax = foc_maxf(va, foc_maxf(vb, vc));
  float vmin = foc_minf(va, foc_minf(vb, vc));

  float vcom = 0.5f * (vmax + vmin);

  pwm->a = 0.5f + (va - vcom);
  pwm->b = 0.5f + (vb - vcom);
  pwm->c = 0.5f + (vc - vcom);
}
/* ------------------ FOC_Core------------------ */

FOC_PID_ctrl_t id_pid;
FOC_PID_ctrl_t iq_pid;

void FOC_Init(void) {
  // d 轴电流环
  FOC_PID_Init(&id_pid, FOC_PID_TYPE_PI,
               0.2f,    // Kp
               0.01f,   // Ki
               0.0f,    // Kd (PI 模式忽略)
               1e-4f,   // dt (100 µs 控制周期)
               10.0f,   // 积分限幅
               12.0f,   // 输出限幅
               0.001f); // d 滤波时间常数

  // q 轴电流环
  FOC_PID_Init(&iq_pid, FOC_PID_TYPE_PI,
               0.2f,  // Kp
               0.01f, // Ki
               0.0f,  // Kd (PI 模式忽略)
               1e-4f, // dt (100 µs 控制周期)
               10.0f, // 积分限幅
               12.0f, // 输出限幅
               0.001f // d 滤波时间常数
  );
}

void FOC_UpdatePWM(const phase_current_t *i_abc, float theta_e, FOC_PWM_t *pwm,
                   float id_ref, float iq_ref) {
  alpha_beta_t i_alpha_beta;
  dq_t i_dq;
  dq_t v_dq;
  alpha_beta_t v_alpha_beta;

  // 1. Clarke 变换
  Clarke(i_abc, &i_alpha_beta);

  // 2. Park 变换
  Park(&i_alpha_beta, theta_e, &i_dq);

  // 3. PI 控制器计算 dq 电压
  v_dq.d = FOC_PID_Compute(&id_pid, i_dq.d, id_ref); /* vd = 电流 d 轴控制 */
  v_dq.q = FOC_PID_Compute(&iq_pid, i_dq.q, iq_ref); /* vq = 电流 q 轴控制 */

  // 4. 逆 Park
  InvPark(&v_dq, theta_e, &v_alpha_beta);

  // 5. SVPWM Offset 生成 PWM
  SVPWM_Offset_Float(&v_alpha_beta, pwm);

  // 6. 应用 PWM
  BSP_PWM_Set_Duty(pwm);
}
