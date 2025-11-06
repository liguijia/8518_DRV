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
#include "bsp_delay.h"
#include "bsp_flash.h"
#include "bsp_led.h"
#include "bsp_pwm.h"
#include "encoder.h"
#include "foc_controller.h"
#include "foc_openloop.h"
#include "foc_selfcalib.h"
#include "kth7823.h"
#include "motor_config.h"
#include "ramp_control.h"

// 6020test
#include "gm6020_ctrl.h"
GM6020_Handle_t gm6020_motor1, gm6020_motor5;
GM6020_Handle_t *motors[] = {&gm6020_motor1, &gm6020_motor5}; // 电机数组
static void gm6020_can_send(uint16_t id, uint8_t *data) {
  BSP_FDCAN_SendMsg(id, data); // 注册现有CAN发送函数
}
float gm6020_test = 100.0f;
// test

//
FOC_Controller_t foc;
//
FOC_OpenLoop_t openloop;
//

// 测试用的参考值
float test_iq = 0.0f;
float test_id = 0.0f;
float test_speed = 100.0f;
float test_pos = 100.0f;
// 自校准句柄和配置
FOC_SelfCalib_Handle_t calib_handle;
FOC_SelfCalib_Config_t calib_cfg = {
    .align_voltage = 1.5f, // 2.5V 对齐电压
    .align_current = 0.0f, // 电压模式下忽略电流
    .align_time = 1.0f     // 0.5 秒对齐时间
};
// 全局斜坡函数句柄
RampFunction_t speed_ramp;
float current_speed_ref = 0.0f; // 实际 FOC 速度环参考值

/*
  应用主初始化函数
*/
void App_Main_Init(void) {
  // 初始化外设
  BSP_Delay_Init(&htim17);
  BSP_FDCAN_Init();
  BSP_Flash_Init();
  // KTH7823_Init(&henc1, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0,
  // KTH7823_CW);

  Encoder_Register_Init(&rotor_encoder, ENCODER_TYPE_KTH7823, &hspi1,
                        SPI1_CS_GPIO_Port, SPI1_CS_Pin, 0xFFFF, KTH7823_CW);
  Encoder_Register_Init(&output_encoder, ENCODER_TYPE_MT6709, &hspi3,
                        SPI3_CS_GPIO_Port, SPI3_CS_Pin, 0xFFFF, MT6709_CW);
  AnalogSignal_Process_Init();
  //
  GM6020_Init(&gm6020_motor1, 1, gm6020_can_send); // ID1，注册发送回调

  // 配置PID参数（示例值，需根据实际调试）
  GM6020_PID_Params_t speed_pid = {.kp = 0.0075f,
                                   .ki = 0.00005f,
                                   .kd = 0.0f,
                                   .output_limit = 2.5f, // 最大输出电流2.5A
                                   .integral_limit = 0.5f};
  GM6020_PID_Params_t pos_pid = {.kp = 1.875f,
                                 .ki = 0.00005f,
                                 .kd = 0.0f,
                                 .output_limit = 600.0f, // 最大输出转速600rpm
                                 .integral_limit = 50.0f};
  GM6020_SetPIDParams(&gm6020_motor1, &speed_pid, &pos_pid);
  GM6020_SetMode(&gm6020_motor1, GM6020_MODE_SPEED);
  GM6020_ManagerInit(motors, 1);
  //
  BSP_PWM_Init();

  // 初始化 FOC 控制器
  FOC_Controller_Init(&foc, FOC_DT_CURRENT, FOC_DT_SPEED, FOC_DT_POSITION);
  //
  RampFunction_Init(&speed_ramp, 0.0f, 0.0f, 750.0f, RAMP_TYPE_LINEAR);

  // 自校准
  // FOC_SelfCalib_Init(&calib_handle, &calib_cfg);
  // FOC_SelfCalib_Execute(&foc, &calib_handle);

  foc.status = FOC_STATE_READY;
  //
  BSP_PWM_Start();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  //
  BSP_BreathLED_Init();
  BSP_LED_Status(LED_FLASH_SLOW);
}
uint16_t output_angle_deg;
/*
  应用主循环函数
*/
void App_Main_Loop(void) {
  // 设置一个固定的 iq 参考值用于测试
  // FOC_Controller_SetIdIq(&foc, test_id, test_iq);
  // 出轴编码器读取

  Encoder_ReadRaw(&output_encoder, &output_angle_deg);
}
/**
 * @brief  定时器中断回调函数
 * @retval TIM3: 用于LED状态指示
 * @retval TIM6: 用于位置环更新
 * @retval TIM7: 用于速度环更新
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    BSP_LED_Update();
  }
  if (htim->Instance == TIM6) {
    // FOC_Controller_SetPosition(&foc, test_pos);
    // FOC_PositionLoop_Update(&foc);
    GM6020_SetTarget(&gm6020_motor1, gm6020_test);
    GM6020_PIDCalculate();
    GM6020_SendAll();
  }
  if (htim->Instance == TIM7) {
    if (speed_ramp.target_value != test_speed) {
      speed_ramp.target_value = test_speed;
    }
    current_speed_ref = RampFunction_Update(&speed_ramp, FOC_DT_SPEED);
    FOC_Controller_SetSpeed(&foc, current_speed_ref); // 设置一个初始速度参考值
    FOC_PLLSpeedLoop_Update(&foc);
  }
}