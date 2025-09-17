#ifndef FOC_CORE_H
#define FOC_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "analog_signal.h"
#include "bsp_pwm.h"
#include <stdint.h>

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

/* ------------------ Clarke ------------------ */
void Clarke(const phase_current_t *i_abc, alpha_beta_t *i_alpha_beta);
void InvClarke(const alpha_beta_t *i_alpha_beta, phase_current_t *i_abc);

/* ------------------ Park ------------------ */
void Park(const alpha_beta_t *i_alpha_beta, float theta_e, dq_t *i_dq);
void InvPark(const dq_t *i_dq, float theta_e, alpha_beta_t *i_alpha_beta);

/* ------------------ SVPWM Offset ------------------ */
void SVPWM_Offset_Float(const alpha_beta_t *volt, FOC_PWM_t *pwm);

#ifdef __cplusplus
}
#endif

#endif // FOC_CORE_H
