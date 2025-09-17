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

FOC_PWM_t initial_pwm_duty = {0.3f, 0.45f, 0.65f};

void BSP_PWM_Set_Duty(const FOC_PWM_t *duty) {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint16_t)duty_to_ccr(duty->a));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)duty_to_ccr(duty->b));
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)duty_to_ccr(duty->c));
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

  BSP_PWM_Set_Duty(&initial_pwm_duty); // 初始占空比为 0
}

void BSP_PWM_Stop() {
  BSP_PWM_Set_Duty(&initial_pwm_duty);     // 初始占空比为 0
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // 通道 1
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); // 通道 2
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); // 通道 3

  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); // 互补通道 1
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); // 互补通道 2
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); // 互补通道 3

  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop(&htim1);
}
