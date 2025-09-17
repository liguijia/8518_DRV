#include "foc_pid.h"
#include <math.h>

/* 内部限幅（对称） */
static inline float foc_pid_clamp_abs(float v, float lim) {
  if (lim <= 0.0f)
    return v;
  if (v > lim)
    return lim;
  if (v < -lim)
    return -lim;
  return v;
}

/* 计算 derivative lowpass alpha: alpha = dt / (tau + dt) */
static inline float deriv_alpha(float dt, float tau) {
  if (tau <= 0.0f)
    return 1.0f; /* no filtering */
  return dt / (tau + dt);
}

void FOC_PID_Init(FOC_PID_ctrl_t *pid, FOC_PID_type_t type, float kp, float ki,
                  float kd, float dt, float integral_limit, float output_limit,
                  float d_filter_tau) {
  if (!pid)
    return;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = (dt > 0.0f) ? dt : 1e-3f; /* 防止除以0 */
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

void FOC_PID_Reset(FOC_PID_ctrl_t *pid) {
  if (!pid)
    return;
  pid->integral = 0.0f;
  pid->output = 0.0f;
  pid->prev_err = 0.0f;
  pid->prev_prev_err = 0.0f;
  pid->d_filter_state = 0.0f;
}

/* 内部主计算（带可选外部 d_err） */
static float foc_pid_compute_core(FOC_PID_ctrl_t *pid, float setpoint,
                                  float measurement,
                                  float *p_d_err /* optional */) {
  float err = setpoint - measurement;
  float out = 0.0f;

  /* derivative: either provided (p_d_err non-null) or compute from err */
  float raw_d = 0.0f;
  if (p_d_err) {
    raw_d = *p_d_err;
  } else {
    raw_d = (err - pid->prev_err) / pid->dt;
  }

  /* derivative low-pass */
  float alpha = deriv_alpha(pid->dt, pid->d_filter_tau);
  pid->d_filter_state += alpha * (raw_d - pid->d_filter_state);
  float d_filtered = pid->d_filter_state;

  switch (pid->type) {
  case FOC_PID_TYPE_PI: {
    /* 条件积分（anti-windup）：先计算非积分项，再判断积分是否可提交 */
    float P = pid->kp * err;
    float integral_candidate = pid->integral + err * pid->dt;
    float I_part = pid->ki * integral_candidate;
    float out_candidate = P + I_part;
    /* 如果 out_candidate 在输出范围内就提交积分，否则不改变积分（简单条件积分）
     */
    if (out_candidate <= pid->output_limit &&
        out_candidate >= -pid->output_limit) {
      pid->integral =
          foc_pid_clamp_abs(integral_candidate, pid->integral_limit);
      out = P + pid->ki * pid->integral;
    } else {
      /* 不提交积分，输出仅为 P（或者仍可限幅） */
      out = P + pid->ki * pid->integral; /* 保持当前积分的贡献（或可选不加） */
      /* 也可以不加当前积分贡献： out = P; */
    }
  } break;

  case FOC_PID_TYPE_PID: {
    float P = pid->kp * err;
    float D = pid->kd * d_filtered;
    float integral_candidate = pid->integral + err * pid->dt;
    float out_candidate = P + pid->ki * integral_candidate + D;
    if (out_candidate <= pid->output_limit &&
        out_candidate >= -pid->output_limit) {
      pid->integral =
          foc_pid_clamp_abs(integral_candidate, pid->integral_limit);
      out = P + pid->ki * pid->integral + D;
    } else {
      /* 同上，保持当前积分或不加 */
      out = P + pid->ki * pid->integral + D;
    }
  } break;

  case FOC_PID_TYPE_INCREMENTAL: {
    /* 增量式 PID：计算 delta，累加到 pid->output */
    /* delta = Kp*(e - e[-1]) + Ki*e*dt + Kd*( (e - e[-1]) - (e[-1] - e[-2])
     * )/dt */
    float delta_p = pid->kp * (err - pid->prev_err);
    float delta_i = pid->ki * err * pid->dt; /* ki is per-second */
    float delta_d = 0.0f;
    if (pid->dt > 1e-12f) {
      delta_d = pid->kd *
                ((err - pid->prev_err) - (pid->prev_err - pid->prev_prev_err)) /
                pid->dt;
    }
    float delta = delta_p + delta_i + delta_d;
    pid->output += delta;
    pid->output = foc_pid_clamp_abs(pid->output, pid->output_limit);
    out = pid->output;
  } break;

  default:
    out = 0.0f;
    break;
  }

  /* 禁止持久的超限（最终限幅） */
  out = foc_pid_clamp_abs(out, pid->output_limit);

  /* 更新时间历史误差（所有模式都需要） */
  pid->prev_prev_err = pid->prev_err;
  pid->prev_err = err;

  /* 对于非增量 PID，更新 pid->output 以反映最新输出（方便外部查询） */
  if (pid->type != FOC_PID_TYPE_INCREMENTAL) {
    pid->output = out;
  }

  return out;
}

float FOC_PID_Compute(FOC_PID_ctrl_t *pid, float setpoint, float measurement) {
  return foc_pid_compute_core(pid, setpoint, measurement, NULL);
}

float FOC_PID_ComputeWithDerivative(FOC_PID_ctrl_t *pid, float setpoint,
                                    float measurement, float d_err) {
  return foc_pid_compute_core(pid, setpoint, measurement, &d_err);
}
