#ifndef PLL_SPEED_ESTIMATOR_H
#define PLL_SPEED_ESTIMATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float theta; // PLL 估算角度 (rad)
  float omega; // PLL 估算速度 (rad/s)

  float kp;         // 比例增益
  float ki;         // 积分增益
  float integrator; // 积分状态
  float Ts;         // 采样周期 (s)
} PLL_Handle;

/**
 * @brief 初始化 PLL
 * @param pll PLL handle
 * @param kp  比例增益
 * @param ki  积分增益
 * @param Ts  采样周期(s)
 */
void PLL_Init(PLL_Handle *pll, float kp, float ki, float Ts);

/**
 * @brief 更新 PLL，输入传感器角度，输出估算速度
 * @param pll PLL handle
 * @param theta_meas 输入角度(rad)，范围 [0, 2π)
 * @return 估算速度 (rad/s)
 */
float PLL_Update(PLL_Handle *pll, float theta_meas);

#ifdef __cplusplus
}
#endif
#endif // PLL_SPEED_ESTIMATOR_H
