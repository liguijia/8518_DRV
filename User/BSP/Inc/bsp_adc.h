#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "main.h"

//
#define ADC1_REG_LEN (2U)
#define ADC2_REG_LEN (2U)
#define ADC1_INJ_LEN (3U)
typedef struct {
  uint16_t adc1_reg[ADC1_REG_LEN]; // ADC1 规则组
  uint16_t adc2_reg[ADC2_REG_LEN]; // ADC2 规则组
  uint16_t adc1_inj[ADC1_INJ_LEN]; // ADC1 注入组
} BSP_ADC_Pack_t;

extern BSP_ADC_Pack_t adc_pack;

#define ADC1_REG(i) (adc_pack.adc1_reg[(i)])
#define ADC2_REG(i) (adc_pack.adc2_reg[(i)])

#define ADC1_INJ(i) (adc_pack.adc1_inj[(i)])

//
extern void BSP_ADC_Init(void);

#endif /* __BSP_ADC_H__ */
