#include "analog_signal.h"
#include "bsp_adc.h"
#include "mean_filter.h"

#include "foc_controller.h"

#define FILTER_WINDOW_SIZE 8 // 定义滤波窗口大小

analogdata_t analogdata = {0};

// ADC校准参数结构体
typedef struct {
  float k;
  float b;
} adc_calibration_t;
typedef struct {
  uint32_t stm32id[3]; // 96 bit stm32 id

  adc_calibration_t vin;

  adc_calibration_t temp_mcu;
  adc_calibration_t temp_ntc1;
  adc_calibration_t temp_ntc2;

  adc_calibration_t ia;
  adc_calibration_t ib;
  adc_calibration_t ic;
} board_adc_calibration_t;
// ADC 校准参数
static board_adc_calibration_t adc_cali_array = {
    {0x00000000, 0x00000000, 0x00000222}, // STM32 唯一 ID
    // input voltage ：R1=49.9k  R2=3.3k
    {0.0170068027f, -0.0000000000f},
    // temperature：R1=10k  NTC=10k
    {0.322f, -279.0f},
    {-0.037833064f, 114.39492194f},
    {-0.037833064f, 114.39492194f},
    // 3phase current : R_sense=1mR  Gain=20
    {0.04029f, -82.5f},
    {0.04029f, -82.5f},
    {0.04029f, -82.5f},

};
static mean_filter_t vin = {0};
static mean_filter_t temp_mcu = {0};
static mean_filter_t temp_ntc1 = {0};
static mean_filter_t temp_ntc2 = {0};
static mean_filter_t ia = {0};
static mean_filter_t ib = {0};
static mean_filter_t ic = {0};

// 通用映射函数（单函数完成滤波+线性映射）
static inline float get_mapped_value(mean_filter_t *filter,
                                     const adc_calibration_t *calib) {
  // 1. 执行均值滤波 → 2. 应用线性校准公式
  return calib->k * mean_filter_calculate_average(filter) + calib->b;
}

void AnalogSignal_Process_Init(void) {
  // 初始化 ADC
  BSP_ADC_Init();

  // 初始化均值滤波器
  mean_filter_init(&vin, FILTER_WINDOW_SIZE);
  mean_filter_init(&temp_mcu, FILTER_WINDOW_SIZE * 16);
  mean_filter_init(&temp_ntc1, FILTER_WINDOW_SIZE);
  mean_filter_init(&temp_ntc2, FILTER_WINDOW_SIZE);
  mean_filter_init(&ia, FILTER_WINDOW_SIZE);
  mean_filter_init(&ib, FILTER_WINDOW_SIZE);
  mean_filter_init(&ic, FILTER_WINDOW_SIZE);
}
// 读取并处理  ADC1 规则组数据
void ADC1_ConvCpltCallback(void) {
  mean_filter_update(&temp_mcu, ADC1_REG(0));
  mean_filter_update(&vin, ADC1_REG(1));
  analogdata.temp.mcu = get_mapped_value(&temp_mcu, &adc_cali_array.temp_mcu);
  analogdata.input_voltage = get_mapped_value(&vin, &adc_cali_array.vin);
}
// 读取并处理  ADC2 规则组数据
void ADC2_ConvCpltCallback(void) {
  mean_filter_update(&temp_ntc1, ADC2_REG(0));
  mean_filter_update(&temp_ntc2, ADC2_REG(1));
  analogdata.temp.ntc1 =
      get_mapped_value(&temp_ntc1, &adc_cali_array.temp_ntc1);
  analogdata.temp.ntc2 =
      get_mapped_value(&temp_ntc2, &adc_cali_array.temp_ntc2);
}
// 读取并处理  ADC1 注入组数据
void ADC1_InjectedConvCpltCallback(void) {
  mean_filter_update(&ia, ADC1_INJ(0));
  mean_filter_update(&ib, ADC1_INJ(1));
  mean_filter_update(&ic, ADC1_INJ(2));
  analogdata.phase_current.a = get_mapped_value(&ia, &adc_cali_array.ia);
  analogdata.phase_current.b = get_mapped_value(&ib, &adc_cali_array.ib);
  analogdata.phase_current.c = get_mapped_value(&ic, &adc_cali_array.ic);

  /* 调用 FOC 电流环更新（24kHz） */
  FOC_CurrentLoop_Update(&foc, &analogdata.phase_current);
}