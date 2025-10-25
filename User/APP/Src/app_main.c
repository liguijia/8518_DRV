#include "main.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include <stdint.h>

//
#include "analog_signal.h"
#include "app_main.h"
#include "bsp_can.h"
#include "bsp_flash.h"
#include "bsp_kth7823.h"
#include "bsp_led.h"
#include "bsp_pwm.h"
#include "foc_controller.h"
#include "foc_openloop.h"
#include "foc_selfcalib.h"
#include "motor_config.h"

// 测试用的 iq 参考值
float test_iq = 1.5f;
float test_id = 0.0f;
float test_speed = 100.0f;
float test_pos = 100.0f;
//
FOC_SelfCalib_Handle_t calib_handle;
uint8_t set_enc_zero = 0;
//
KTH7823_HandleTypeDef henc1;
FOC_Controller_t foc;
//
FOC_OpenLoop_t openloop;
FOC_PWM_t pwm;
//
uint32_t flash_counter = 0;
Flash_Status flash_status;
uint8_t flash_read = 0;
uint8_t flash_write = 0;
// 初始化函数
void App_Main_Init(void) {
  // 初始化外设
  BSP_FDCAN_Init();
  BSP_Flash_Init();
  BSP_KTH7823_Init(&henc1, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0,
                   KTH7823_CW);
  BSP_PWM_Init();

  // 初始化 FOC 控制器
  FOC_Controller_Init(&foc, &henc1, FOC_DT_CURRENT, FOC_DT_SPEED,
                      FOC_DT_POSITION);
  //
  FOC_SelfCalib_Config_t calib_cfg = {
      .align_voltage = 0.5f, // 2.5V 对齐电压
      .align_current = 0.0f, // 电压模式下忽略电流
      .align_time = 0.5f     // 0.5 秒对齐时间
  };
  FOC_SelfCalib_Init(&calib_handle, &calib_cfg);
  // FOC_SelfCalib_Execute(&foc, &calib_handle);

  //
  AnalogSignal_Process_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim3);

  BSP_BreathLED_Init();
  //
  //
  //
  // flash_status = BSP_Flash_Read(FLASH_USER_START_ADDR, &flash_counter,
  //                               sizeof(flash_counter));
}

// 主循环
void App_Main_Loop(void) {
  // 设置一个固定的 iq 参考值用于测试
  // FOC_Controller_SetIdIq(&foc, test_id, test_iq);

  if (flash_read) {
    flash_status = BSP_Flash_Read(FLASH_USER_START_ADDR, &flash_counter,
                                  sizeof(flash_counter));
    flash_read = 0;
  }
  if (flash_write) {
    flash_counter = flash_write;
    uint32_t page_num =
        (FLASH_USER_START_ADDR - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
    BSP_Flash_ErasePage(page_num);
    flash_status = BSP_Flash_Write(FLASH_USER_START_ADDR, &flash_counter,
                                   sizeof(flash_counter));
    flash_write = 0;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    // FOC_Controller_SetPosition(&foc, test_pos);
    // FOC_PositionLoop_Update(&foc);
  }
  if (htim->Instance == TIM3) {
    FOC_Controller_SetSpeed(&foc, test_speed); // 设置一个初始速度参考值
    FOC_PLLSpeedLoop_Update(&foc);
  }
}