#include "bsp_led.h"
#include "tim.h"

void BSP_BreathLED_Init(void) {
  // 初始化 LED 引脚
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // 通道 1
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}

void BSP_LED_Status(LED_MODE mode) {
  static LED_MODE current_mode = LED_OFF;
  if (mode != current_mode) {
    current_mode = mode;
    switch (mode) {
    case LED_OFF:
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
      break;
    case LED_ON:
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, TIM3_ARR);
      break;
    case LED_BREATH:
      // 呼吸灯效果由定时器中断处理
      break;
    case LED_FLASH_FAST:
      // 快速闪烁效果由定时器中断处理
      break;
    case LED_FLASH_SLOW:
      // 慢速闪烁效果由定时器中断处理
      break;
    default:
      break;
    }
  }
}
