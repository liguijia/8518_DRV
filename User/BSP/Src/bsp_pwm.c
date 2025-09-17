#include "bsp_pwm.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"

// 在头文件或函数前定义
#define CLAMP(x, low, high)                                                    \
  (((x) < (low)) ? (low) : (((x) > (high)) ? (high) : (x)))

// 静态内联函数：将 [0.0, 1.0] 的占空比转换为 CCR 值
static inline uint32_t duty_to_ccr(float duty) {
  return (uint32_t)(CLAMP(duty, 0.0f, 1.0f) * (float)TIM1_ARR + 0.5f);
}

void BSP_PWM_Set_Duty(float duty1, float duty2, float duty3) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)duty_to_ccr(duty1));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)duty_to_ccr(duty2));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)duty_to_ccr(duty3));
}

void BSP_PWM_Init() {
  // 启动 TIM1 的 3 个通道和它们的互补通道
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 通道 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 通道 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // 通道 3

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // 互补通道 1
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // 互补通道 2
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // 互补通道 3

  BSP_PWM_Set_Duty(0.5f, 0.25f, 0.75f); // 初始占空比为 0
}

void BSP_PWM_Stop() {
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // 通道 1
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); // 通道 2
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); // 通道 3

  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); // 互补通道 1
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); // 互补通道 2
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); // 互补通道 3

  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop(&htim1);
}
