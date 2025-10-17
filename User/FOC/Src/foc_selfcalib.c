#include "foc_selfcalib.h"
#include "bsp_kth7823.h"
#include "bsp_pwm.h"
#include "foc_core.h"
#include "motor_config.h"
#include <math.h>

// 假设这些常量和函数在其他头文件中可用
#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

/**
 * @brief 辅助函数：开环电压模式输出 PWM (用于校准)
 * * 此函数基于 voltage 和 theta_e 计算并输出 PWM 占空比，不依赖电流环。
 */
void FOC_Align_Rotor(FOC_Controller_t *foc, float voltage, float theta_e) {
  dq_t v_dq;
  alpha_beta_t v_alpha_beta;

  // 1. 构造 d-q 电压：纯 q 轴（Vd=0, Vq=Voltage）
  v_dq.d = 0.0f;
  v_dq.q = voltage;

  // 2. 逆Park变换
  InvPark(&v_dq, theta_e, &v_alpha_beta);

  // 3. 电压归一化 (使用 Vbus)
  float Vbus = analogdata.input_voltage;
  if (Vbus > 1.0f) {
    float scale = 1.0f / Vbus;
    v_alpha_beta.alpha *= scale;
    v_alpha_beta.beta *= scale;
  }

  // 4. SVPWM + 输出 PWM
  SVPWM_Offset_Float(&v_alpha_beta, &foc->pwm);
  BSP_PWM_Set_Duty(&foc->pwm);
}
/**
 * @brief 初始化自校准模块
 * @param calib 自校准句柄
 * @param cfg 配置参数
 */
void FOC_SelfCalib_Init(FOC_SelfCalib_Handle_t *calib,
                        const FOC_SelfCalib_Config_t *cfg) {

  if (!calib || !cfg) {
    return; // 安全检查
  }

  // 1. 复制配置参数
  // 可以使用 memcpy，但由于结构体较小，直接赋值更清晰：
  calib->cfg = *cfg;

  // 2. 重置校准状态
  calib->state = SELF_CALIB_IDLE;

  // 3. 重置结果
  calib->zero_offset = 0.0f;
}
/**
 * @brief 执行零点自动校准 (阻塞式实现)
 * @param foc FOC控制器句柄
 * @param calib 自校准句柄
 * @return 自校准状态
 */
FOC_SelfCalib_State_t FOC_SelfCalib_Execute(FOC_Controller_t *foc,
                                            FOC_SelfCalib_Handle_t *calib) {

  if (!foc || !calib) {
    return SELF_CALIB_FAILED;
  }

  // 状态检查
  if (calib->state == SELF_CALIB_RUNNING) {
    return SELF_CALIB_FAILED;
  }

  calib->state = SELF_CALIB_RUNNING;

  float target_voltage = calib->cfg.align_voltage;
  uint32_t delay_ms = (uint32_t)(calib->cfg.align_time * 1000.0f);

  // 对齐目标：施加纯 q 轴电压，期望转子对齐到电角度 π/2 (90度)
  const float theta_align = M_PI_2;

  // 安全检查：对齐时间或电压不合理则失败
  if (delay_ms < 500 || target_voltage < 0.0f) {
    calib->state = SELF_CALIB_FAILED;
    return SELF_CALIB_FAILED;
  }

  // ----------------------------------------------------
  // 1. 施加磁场锁定转子 (使用辅助函数)
  // ----------------------------------------------------
  FOC_Align_Rotor(foc, target_voltage, theta_align);

  // 2. 延时等待转子锁定
  HAL_Delay(delay_ms);

  // ----------------------------------------------------
  // 3. 读取角度并计算零点偏移
  // ----------------------------------------------------
  float pos_deg_lock;
  float zero_offset_rad = 0.0f;
  bool read_success = false;

  if (BSP_KTH7823_ReadAngle(foc->encoder, &pos_deg_lock) == KTH7823_OK) {
    read_success = true;

    // 1. 机械角度转弧度 (0 - 2π)
    float mech_rad_lock = pos_deg_lock * TWO_PI / 360.0f;

    // 2. 机械角度转电角度 (实际读数)
    float elec_rad_now = fmodf(mech_rad_lock * MOTOR_POLE_PAIRS, TWO_PI);

    // 3. 计算零点偏移： (实际读出的电角度) - (期望对齐的电角度 π/2)
    zero_offset_rad = elec_rad_now - theta_align;

    // 4. 归一化零点偏移 (0 到 2π)
    if (zero_offset_rad < 0.0f) {
      zero_offset_rad += TWO_PI;
    } else if (zero_offset_rad >= TWO_PI) {
      zero_offset_rad -= TWO_PI;
    }
  }

  // ----------------------------------------------------
  // 4. 清理并更新状态
  // ----------------------------------------------------
  // 立即停止 PWM 输出
  BSP_PWM_Halt();

  if (read_success) {
    // 更新 FOC 句柄和校准句柄
    foc->zero_offset_rad = zero_offset_rad;
    calib->zero_offset = zero_offset_rad;
    calib->state = SELF_CALIB_DONE;
    return SELF_CALIB_DONE;
  } else {
    calib->state = SELF_CALIB_FAILED;
    foc->zero_offset_rad = MOTOR_DEFAULT_ZERO_ANGLE * M_PI / 180.0f;
    return SELF_CALIB_FAILED;
  }
}