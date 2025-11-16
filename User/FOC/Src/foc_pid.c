#include "foc_pid.h"
#include <math.h>

/* -------------------- 工具函数 -------------------- */
static inline float foc_pid_clamp_abs(float v, float lim) {
  if (lim <= 0.0f)
    return v;
  if (v > lim)
    return lim;
  if (v < -lim)
    return -lim;
  return v;
}

static inline float deriv_alpha(float dt, float tau) {
  if (tau <= 0.0f)
    return 1.0f;
  return dt / (tau + dt);
}

/* -------------------- 初始化 -------------------- */
void FOC_PID_Init(FOC_PID_ctrl_t *pid, FOC_PID_type_t type, float kp, float ki,
                  float kd, float dt, float integral_limit, float output_limit,
                  float d_filter_tau) {
  if (!pid)
    return;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = (dt > 0.0f) ? dt : 1e-4f;

  pid->d_filter_tau = d_filter_tau;

  pid->integral = 0.0f;
  pid->output = 0.0f;

  pid->prev_err = 0.0f;
  pid->prev_prev_err = 0.0f;

  pid->d_filter_state = 0.0f;

  pid->integral_limit = fabsf(integral_limit);
  pid->output_limit = fabsf(output_limit);

  pid->type = type;
}

/* -------------------- Reset -------------------- */
void FOC_PID_Reset(FOC_PID_ctrl_t *pid) {
  if (!pid)
    return;
  pid->integral = 0.0f;
  pid->output = 0.0f;
  pid->prev_err = 0.0f;
  pid->prev_prev_err = 0.0f;
  pid->d_filter_state = 0.0f;
}

/* -------------------- 核心计算 -------------------- */
static float foc_pid_compute_core(FOC_PID_ctrl_t *pid, float setpoint,
                                  float measurement, float *p_d_err) {
  float err = setpoint - measurement;
  float out = 0.0f;

  /* ----------- 计算原始 D ----------- */
  float raw_d = (p_d_err) ? (*p_d_err) : ((err - pid->prev_err) / pid->dt);

  /* ----------- D 一阶低通滤波 ----------- */
  float alpha = deriv_alpha(pid->dt, pid->d_filter_tau);
  pid->d_filter_state += alpha * (raw_d - pid->d_filter_state);
  float d_filtered = pid->d_filter_state;

  /* =============== PI 模式 =============== */
  if (pid->type == FOC_PID_TYPE_PI) {
    float P = pid->kp * err;
    float i_new = pid->integral + err * pid->dt;
    i_new = foc_pid_clamp_abs(i_new, pid->integral_limit);

    float out_cand = P + pid->ki * i_new;

    /* 条件积分 anti-windup */
    if (fabsf(out_cand) < pid->output_limit)
      pid->integral = i_new;

    out = P + pid->ki * pid->integral;
  }

  /* =============== 位置式 PID =============== */
  else if (pid->type == FOC_PID_TYPE_PID) {
    float P = pid->kp * err;
    float D = pid->kd * d_filtered;

    float i_new = pid->integral + err * pid->dt;
    i_new = foc_pid_clamp_abs(i_new, pid->integral_limit);

    float out_cand = P + pid->ki * i_new + D;

    if (fabsf(out_cand) < pid->output_limit)
      pid->integral = i_new;

    out = P + pid->ki * pid->integral + D;
  }

  /* =============== 增量式 PID =============== */
  else if (pid->type == FOC_PID_TYPE_INCREMENTAL) {
    /* 新增：对 D 项使用滤波后的差值，而不是二阶差分 */
    float delta_p = pid->kp * (err - pid->prev_err);
    float delta_i = pid->ki * err * pid->dt;
    float delta_d =
        pid->kd * d_filtered * pid->dt; // 改为连续时间一致的增量形式

    float delta = delta_p + delta_i + delta_d;

    pid->output += delta;
    pid->output = foc_pid_clamp_abs(pid->output, pid->output_limit);

    out = pid->output;
  }

  /* ------------------- 最终限幅 ------------------- */
  out = foc_pid_clamp_abs(out, pid->output_limit);

  /* ------------------- 更新历史误差 ------------------- */
  pid->prev_prev_err = pid->prev_err;
  pid->prev_err = err;

  if (pid->type != FOC_PID_TYPE_INCREMENTAL)
    pid->output = out;

  return out;
}

/* ------------------- 公共接口 ------------------- */
float FOC_PID_Compute(FOC_PID_ctrl_t *pid, float setpoint, float measurement) {
  return foc_pid_compute_core(pid, setpoint, measurement, NULL);
}

float FOC_PID_ComputeWithDerivative(FOC_PID_ctrl_t *pid, float setpoint,
                                    float measurement, float d_err) {
  return foc_pid_compute_core(pid, setpoint, measurement, &d_err);
}
