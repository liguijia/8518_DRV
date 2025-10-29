#include "ramp_control.h"
#include <math.h>

/**
 * @brief 初始化斜坡函数
 * @param ramp_func 斜坡函数句柄
 * @param start_value 初始值
 * @param target_value 目标值
 * @param ramp_rate 最大变化速率（单位：速度单位 / 秒）
 * @param ramp_type 斜坡类型
 */
void RampFunction_Init(RampFunction_t *ramp_func, float start_value,
                       float target_value, float ramp_rate,
                       RampType_t ramp_type) {
  ramp_func->current_value = start_value;
  ramp_func->target_value = target_value;
  ramp_func->ramp_rate = ramp_rate;
  // 已移除：ramp_func->last_update_time = current_time;
  ramp_func->ramp_type = ramp_type;
}

/**
 * @brief 更新斜坡函数的输出 (定频调用优化)
 * @param ramp_func 斜坡函数句柄
 * @param dt 时间步长（即调用周期，单位：秒）
 * @return 返回斜坡函数输出的值
 */
float RampFunction_Update(RampFunction_t *ramp_func, float dt) {
  // 线性斜坡的最大变化量: Rate * dt
  float max_change_linear = ramp_func->ramp_rate * dt;
  float actual_change; // 实际步进量

  // 根据不同的斜坡类型，计算变化量
  switch (ramp_func->ramp_type) {
  case RAMP_TYPE_LINEAR:
    actual_change = max_change_linear;
    break;

  case RAMP_TYPE_EXPONENTIAL:
    // 指数增长：使用 expf(dt) 作为乘数
    actual_change = max_change_linear * expf(dt);
    break;

  case RAMP_TYPE_LOGARITHMIC:
    // 对数增长：使用 logf(1.0f + dt) 作为乘数
    actual_change = max_change_linear * logf(10.0f + dt);
    break;

  default:
    // 如果类型未知，则默认使用线性变化
    actual_change = max_change_linear;
    break;
  }

  // -------------------------------------------------------------------
  // 统一处理更新逻辑：限制步进
  // -------------------------------------------------------------------

  if (ramp_func->target_value > ramp_func->current_value) {
    // 目标值大于当前值：正向步进
    ramp_func->current_value += actual_change;
    if (ramp_func->current_value > ramp_func->target_value) {
      ramp_func->current_value = ramp_func->target_value;
    }
  } else {
    // 目标值小于当前值：负向步进
    ramp_func->current_value -= actual_change;
    if (ramp_func->current_value < ramp_func->target_value) {
      ramp_func->current_value = ramp_func->target_value;
    }
  }

  return ramp_func->current_value;
}
