#ifndef BSP_LED_H
#define BSP_LED_H

#include "stm32g4xx_hal.h"

// LED 模式枚举（扩展原有定义）
typedef enum {
  LED_OFF,        // 关闭
  LED_ON,         // 常亮
  LED_BREATH,     // 呼吸模式
  LED_FLASH_FAST, // 快闪（500ms 周期）
  LED_FLASH_SLOW  // 慢闪（1000ms 周期）
} LED_MODE;

// 外部声明 TIM3 句柄（由 CubeMX 生成）
extern TIM_HandleTypeDef htim3;

// 呼吸灯参数（可根据需求调整）
#define BREATH_PERIOD_MS 5000  // 呼吸周期（2秒）
#define BREATH_STEP 2          // 亮度步进值
#define FLASH_FAST_PERIOD 500  // 快闪周期（500ms）
#define FLASH_SLOW_PERIOD 2000 // 慢闪周期（1000ms）

// 初始化函数
void BSP_BreathLED_Init(void);

// 设置 LED 模式
void BSP_LED_Status(LED_MODE mode);

// 定时器中断中调用的更新函数（需在 TIM3 中断中调用）
void BSP_LED_Update(void);

#endif