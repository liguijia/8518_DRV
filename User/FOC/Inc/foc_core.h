#ifndef FOC_CORE_H
#define FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_pwm.h"
#include "foc_pid.h"
#include <analog_signal.h>
// 三相坐标
typedef struct {
  float a;
  float b;
  float c;
} three_phase_t;

// αβ 坐标
typedef struct {
  float alpha;
  float beta;
} alpha_beta_t;

// dq 坐标
typedef struct {
  float d;
  float q;
} dq_t;

extern void InvPark(const dq_t *i_dq, float theta_e,
                    alpha_beta_t *i_alpha_beta);
extern void SVPWM_Offset_Float(const alpha_beta_t *volt, FOC_PWM_t *pwm);

extern void FOC_Init(void);
extern void FOC_UpdatePWM(const phase_current_t *i_abc, float theta_e,
                          FOC_PWM_t *pwm, FOC_PID_ctrl_t *id_pid, float id_ref,
                          FOC_PID_ctrl_t *iq_pid, float iq_ref);

#ifdef __cplusplus
}
#endif

#endif // FOC_CORE_H
