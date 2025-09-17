#ifndef FOC_PID_H
#define FOC_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
  FOC_PID_TYPE_PI = 0,
  FOC_PID_TYPE_PID,
  FOC_PID_TYPE_INCREMENTAL
} FOC_PID_type_t;

typedef struct {
  /* gains */
  float kp;
  float ki; /* note: ki is in units of (output per second per error) */
  float kd;

  /* timing & filtering */
  float dt;           /* sample time in seconds (must > 0) */
  float d_filter_tau; /* derivative low-pass time constant (seconds). if <=0 ->
                         no filtering */

  /* state */
  float integral;       /* integrator state */
  float output;         /* last output (useful for incremental mode) */
  float prev_err;       /* e[k-1] */
  float prev_prev_err;  /* e[k-2] */
  float d_filter_state; /* filtered derivative state */

  /* limits */
  float integral_limit; /* max abs value of integral */
  float output_limit;   /* max abs output */

  /* mode */
  FOC_PID_type_t type;
} FOC_PID_ctrl_t;

/* 初始化
 *  - dt: 采样周期（秒）
 *  - d_filter_tau: 微分滤波时间常数（秒），建议比 dt 大几个量级（如
 * 0.001~0.01s），若 <=0 表示不滤波
 */
void FOC_PID_Init(FOC_PID_ctrl_t *pid, FOC_PID_type_t type, float kp, float ki,
                  float kd, float dt, float integral_limit, float output_limit,
                  float d_filter_tau);

/* 重置状态（积分、历史误差、输出） */
void FOC_PID_Reset(FOC_PID_ctrl_t *pid);

/* 标准计算接口（内部计算微分）：
 *  - setpoint: 目标值
 *  - measurement: 测量值
 * 返回：控制器输出（绝对值，已限幅）
 */
float FOC_PID_Compute(FOC_PID_ctrl_t *pid, float setpoint, float measurement);

/* 可选：外部提供微分量（例如你用观测器/滤波器计算出的 d_err/dt）：
 *  - d_err: (e[k] - e[k-1]) / dt 或者直接观测到的差分
 */
float FOC_PID_ComputeWithDerivative(FOC_PID_ctrl_t *pid, float setpoint,
                                    float measurement, float d_err);

#ifdef __cplusplus
}
#endif

#endif // FOC_PID_H
