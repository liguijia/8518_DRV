#include "pll_speed_estimator.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float wrap_angle(float x) {
  while (x > M_PI)
    x -= 2.0f * M_PI;
  while (x < -M_PI)
    x += 2.0f * M_PI;
  return x;
}

void PLL_Init(PLL_Handle *pll, float kp, float ki, float Ts) {
  pll->theta = 0.0f;
  pll->omega = 0.0f;
  pll->integrator = 0.0f;
  pll->kp = kp;
  pll->ki = ki;
  pll->Ts = Ts;
}

float PLL_Update(PLL_Handle *pll, float theta_meas) {
  // 相位误差 (wrap 到 -pi ~ pi)
  float error = wrap_angle(theta_meas - pll->theta);

  // 积分器更新
  pll->integrator += pll->ki * pll->Ts * error;

  // 频率 (速度)
  pll->omega = pll->integrator + pll->kp * error;

  // 估算角度积分
  pll->theta = wrap_angle(pll->theta + pll->omega * pll->Ts);

  return pll->omega;
}