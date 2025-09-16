// HAL & C
#include "adc.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include <stdint.h>
// USER
#include "bsp_adc.h"

//
#define ADC1_REGULAR_GROUP_DATA_LEN (2U)
#define ADC2_REGULAR_GROUP_DATA_LEN (2U)
#define ADC2_INJECTED_GROUP_DATA_LEN (3U)
//
uint16_t adc1_regulargroup_data[ADC1_REGULAR_GROUP_DATA_LEN] = {0};
uint16_t adc2_regulargroup_data[ADC2_REGULAR_GROUP_DATA_LEN] = {0};
uint16_t adc1_injectedgroup_data[ADC2_INJECTED_GROUP_DATA_LEN] = {0};
//
void BSP_ADC_Init(void) {
  // 启动 ADC 校准
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(10);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(10);

  // 启动 ADC regular
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_regulargroup_data,
                        ADC1_REGULAR_GROUP_DATA_LEN) != HAL_OK) {
  }
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_regulargroup_data,
                        ADC2_REGULAR_GROUP_DATA_LEN) != HAL_OK) {
  }

  // 启动 ADC injected
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
  }
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);

  // 启动 PWM 输出，触发采样
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, TIM1_ARR - 100);
}

uint32_t test1 = 0;
uint32_t test2 = 0;
uint32_t test3 = 0;

// DMA 转换完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // 处理 ADC1 的规则组数据
    test1++;
  }
  if (hadc->Instance == ADC2) {
    // 处理 ADC2 的规则组数据
    test2++;
  }
}

// 注入组转换完成回调
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // 处理 ADC1 的注入组数据
    test3++;
    adc1_injectedgroup_data[0] =
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    adc1_injectedgroup_data[1] =
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    adc1_injectedgroup_data[2] =
        HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
  }
}