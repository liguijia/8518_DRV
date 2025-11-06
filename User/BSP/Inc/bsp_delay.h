#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/**
 * @brief 初始化延迟模块（需在main中调用一次）
 * @param htim: CubeMX配置的TIM7句柄（已初始化完成）
 * @retval 0: 成功；-1: 失败（句柄为空）
 */
int8_t BSP_Delay_Init(TIM_HandleTypeDef *htim);

/**
 * @brief 纳秒级延迟（近似实现，最小精度≈6ns）
 * @param ns: 延迟时间（0~999 ns）
 * @note 基于170MHz主频的NOP指令（1指令周期≈5.88ns）
 */
void BSP_Delay_ns(uint16_t ns);

/**
 * @brief 微秒级延迟（高精度，1us精度）
 * @param us: 延迟时间（0~65535 us）
 * @note 基于TIM7定时器，支持中断中调用
 */
void BSP_Delay_us(uint16_t us);

/**
 * @brief 毫秒级延迟（基于us延迟实现，支持长延时）
 * @param ms: 延迟时间（0~65535 ms）
 * @note 可在中断中调用，无HAL_Delay依赖
 */
void BSP_Delay_ms(uint16_t ms);

#endif