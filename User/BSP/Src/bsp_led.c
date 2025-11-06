#include "bsp_led.h"
#include "bsp_delay.h"

// 静态变量（模块内部使用）
static LED_MODE current_mode = LED_OFF;
static uint32_t last_tick = 0;         // 记录上一次状态切换时间
static uint16_t breath_brightness = 0; // 呼吸灯亮度（0~ARR）
static int8_t breath_dir = 1; // 呼吸灯亮度变化方向（1：递增，-1：递减）

/**
 * @brief 初始化 LED 相关硬件（TIM3 CH1 PWM）
 */
void BSP_BreathLED_Init(void) {
  // 确保 TIM3 已配置为 PWM 模式（CubeMX 中需提前配置）
  // 启动 TIM3 计数器和 PWM 输出（CH1）
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // 初始关闭 LED
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  // 启动 TIM3 中断（用于呼吸灯和闪烁的定时更新）
  HAL_TIM_Base_Start_IT(&htim3);
}

/**
 * @brief 更新 LED 状态（需在 TIM3 中断中周期性调用）
 * @note 建议 TIM3 中断周期为 10ms（兼顾响应速度和性能）
 */
void BSP_LED_Update(void) {
  uint32_t current_tick = HAL_GetTick(); // 获取系统运行时间（ms）
  uint16_t arr = htim3.Instance->ARR; // 获取 TIM3 的 ARR 值（PWM 周期）

  switch (current_mode) {
  case LED_BREATH: {
    // 呼吸灯逻辑：按固定周期渐变亮度
    uint32_t breath_interval = BREATH_PERIOD_MS / (2 * (arr / BREATH_STEP));
    if (current_tick - last_tick >= breath_interval) {
      last_tick = current_tick;
      // 更新亮度（限制在 0~ARR 范围内）
      breath_brightness += breath_dir * BREATH_STEP;
      if (breath_brightness >= arr) {
        breath_brightness = arr;
        breath_dir = -1; // 达到最大亮度，开始递减
      } else if (breath_brightness <= 0) {
        breath_brightness = 0;
        breath_dir = 1; // 达到最小亮度，开始递增
      }
      // 设置 PWM 占空比（亮度）
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, breath_brightness);
    }
    break;
  }

  case LED_FLASH_FAST:
    // 快闪逻辑（500ms 周期：亮 250ms，灭 250ms）
    if (current_tick - last_tick >= FLASH_FAST_PERIOD / 2) {
      last_tick = current_tick;
      // 切换当前亮度（0 <-> ARR）
      uint16_t current_compare = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                            (current_compare == 0) ? arr : 0);
    }
    break;

  case LED_FLASH_SLOW:
    // 慢闪逻辑（1000ms 周期：亮 500ms，灭 500ms）
    if (current_tick - last_tick >= FLASH_SLOW_PERIOD / 2) {
      last_tick = current_tick;
      // 切换当前亮度（0 <-> ARR）
      uint16_t current_compare = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                            (current_compare == 0) ? arr : 0);
    }
    break;

  default:
    // 常亮/关闭模式无需更新（状态固定）
    break;
  }
}

/**
 * @brief 设置 LED 工作模式
 * @param mode: 目标模式（LED_OFF/ON/BREATH/FLASH_FAST/FLASH_SLOW）
 */
void BSP_LED_Status(LED_MODE mode) {
  if (mode == current_mode)
    return; // 模式未变化，无需处理

  current_mode = mode;
  uint16_t arr = htim3.Instance->ARR; // 获取 PWM 周期值

  switch (mode) {
  case LED_OFF:
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    break;

  case LED_ON:
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, arr);
    break;

  case LED_BREATH:
    // 初始化呼吸灯参数
    breath_brightness = 0;
    breath_dir = 1;
    last_tick = HAL_GetTick();
    break;

  case LED_FLASH_FAST:
  case LED_FLASH_SLOW:
    // 初始化闪烁状态（从灭开始）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    last_tick = HAL_GetTick();
    break;

  default:
    break;
  }
}