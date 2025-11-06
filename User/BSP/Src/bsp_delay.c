#include "bsp_delay.h"

// 全局定时器句柄（模块内部维护，不暴露）
static TIM_HandleTypeDef *g_delay_htim = NULL;

/**
 * @brief 启动定时器计数（内部函数）
 */
static void tim_start(void) {
  if (g_delay_htim == NULL)
    return;
  g_delay_htim->Instance->CNT = 0;            // 计数器清零
  g_delay_htim->Instance->CR1 |= TIM_CR1_CEN; // 使能计数
}

/**
 * @brief 停止定时器计数（内部函数）
 */
static void tim_stop(void) {
  if (g_delay_htim == NULL)
    return;
  g_delay_htim->Instance->CR1 &= ~TIM_CR1_CEN; // 禁用计数
}

/**
 * @brief 等待定时器计数达到目标值（内部函数）
 * @param target: 目标计数值（1计数=1us）
 */
static void tim_wait(uint16_t target) {
  if (g_delay_htim == NULL || target == 0)
    return;
  while (g_delay_htim->Instance->CNT < target) {
    // 空循环等待，不依赖中断
  }
}

int8_t BSP_Delay_Init(TIM_HandleTypeDef *htim) {
  if (htim == NULL)
    return -1;
  g_delay_htim = htim;
  tim_stop(); // 初始化为停止状态
  return 0;
}

void BSP_Delay_ns(uint16_t ns) {
  if (ns == 0)
    return;

  // 170MHz主频下，1个NOP指令周期≈5.88ns（1/170e6 ≈ 5.88e-9s）
  // 计算所需NOP指令数（向上取整，减少误差）
  const uint32_t ns_per_cycle = 6; // 近似6ns/周期
  uint32_t cycles = (ns + ns_per_cycle - 1) / ns_per_cycle;

  // 执行NOP指令（纯硬件延迟，不占用总线）
  for (uint32_t i = 0; i < cycles; i++) {
    __NOP();
  }
}

void BSP_Delay_us(uint16_t us) {
  if (g_delay_htim == NULL || us == 0)
    return;

  // 启动定时器并等待指定us数（1计数=1us）
  tim_start();
  tim_wait(us);
  tim_stop();
}

void BSP_Delay_ms(uint16_t ms) {
  if (ms == 0)
    return;

  // 基于us延迟实现ms级延迟（1ms=1000us）
  for (uint16_t i = 0; i < ms; i++) {
    BSP_Delay_us(1000); // 每次延迟1000us（1ms）
  }
}