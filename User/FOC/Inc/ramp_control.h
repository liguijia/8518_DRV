#ifndef __RAMP_CONTROL_H__
#define __RAMP_CONTROL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 定义斜坡类型
typedef enum {
  RAMP_TYPE_LINEAR,      // 线性斜坡
  RAMP_TYPE_EXPONENTIAL, // 指数斜坡
  RAMP_TYPE_LOGARITHMIC, // 对数斜坡
} RampType_t;

// 斜坡控制结构体
typedef struct {
  float current_value; // 当前值
  float target_value;  // 目标值
  float ramp_rate;     // 最大变化速率（单位：速度单位 / 秒）
  // 已移除：float last_update_time; // 上次更新时间（单位：秒）
  RampType_t ramp_type; // 斜坡类型
} RampFunction_t;

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
                       RampType_t ramp_type);

/**
 * @brief 更新斜坡函数的输出
 * @param ramp_func 斜坡函数句柄
 * @param dt 时间步长（即调用周期，单位：秒）
 * @return 返回斜坡函数输出的值
 */
float RampFunction_Update(RampFunction_t *ramp_func, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __RAMP_CONTROL_H__ */
