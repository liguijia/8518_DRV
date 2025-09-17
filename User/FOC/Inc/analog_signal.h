#ifndef __ANALOG_SIGNAL_H__
#define __ANALOG_SIGNAL_H__

#include "main.h"
#include <stdint.h>

typedef struct {
  float mcu;
  float ntc1;
  float ntc2;
} temp_t;

typedef struct {
  float a;
  float b;
  float c;
} phase_current_t;

typedef struct {
  float input_voltage;           // 输入电压
  phase_current_t phase_current; // 三相电流
  temp_t temp;                   // 一行搞定所有温度
} analogdata_t;

extern analogdata_t analogdata;
extern void AnalogSignal_Process_Init(void);
extern void ADC1_ConvCpltCallback(void);
extern void ADC2_ConvCpltCallback(void);
extern void ADC1_InjectedConvCpltCallback(void);
#endif /* __ANALOG_SIGNAL_H__ */
