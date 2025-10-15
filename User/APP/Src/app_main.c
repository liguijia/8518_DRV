#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"

//
#include "analog_signal.h"
#include "app_main.h"
#include "bsp_can.h"
#include "bsp_kth7823.h"
#include "bsp_led.h"
#include "bsp_pwm.h"
#include "foc_controller.h"
#include "foc_openloop.h"
#include "motor_config.h"
#include "pll_speed_estimator.h"
#include <stdint.h>

// 测试用的 iq 参考值
float test_iq = 1.5f;
float test_speed = 100.0f;
float test_pos = 100.0f;
KTH7823_HandleTypeDef henc1;
FOC_Controller_t foc;
//
FOC_OpenLoop_t openloop;
FOC_PWM_t pwm;

// 初始化函数
void App_Main_Init(void) {
  // 初始化外设
  FOC_OpenLoop_Init(&openloop, 0.1f, 20.0f);
  AnalogSignal_Process_Init();

  BSP_FDCAN_Init();

  BSP_KTH7823_Init(&henc1, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0,
                   KTH7823_CW);
  FOC_Controller_Init(&foc, &henc1, FOC_DT_CURRENT, FOC_DT_SPEED,
                      FOC_DT_POSITION);
  HAL_TIM_Base_Start_IT(&htim6); // 启动 1ms 定时器中断
  HAL_TIM_Base_Start_IT(&htim3); // 启动 1ms 定时器中断
  BSP_PWM_Init();

  BSP_BreathLED_Init();
}

// 主循环
void App_Main_Loop(void) {
  // 设置一个固定的 iq 参考值用于测试
  // FOC_Controller_SetIdIq(&foc, 0.0f, test_iq);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    // FOC_Controller_SetPosition(&foc, test_pos);
    // FOC_PositionLoop_Update(&foc);
    FOC_OpenLoop_Update(&openloop, &pwm, FOC_DT_POSITION);
  }
  if (htim->Instance == TIM3) {
    // FOC_Controller_SetSpeed(&foc, test_speed); // 设置一个初始速度参考值
    FOC_PLLSpeedLoop_Update(&foc);
  }
}