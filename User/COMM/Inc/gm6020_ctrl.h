#ifndef GM6020_CTRL_H
#define GM6020_CTRL_H

#include <stdbool.h>
#include <stdint.h>

// 控制模式枚举
typedef enum {
  GM6020_MODE_CURRENT, // 电流环（直接控制电流）
  GM6020_MODE_SPEED,   // 速度环（目标转速）
  GM6020_MODE_POSITION // 位置环（目标角度）
} GM6020_Mode_t;

// PID参数结构体
typedef struct {
  float kp;             // 比例系数
  float ki;             // 积分系数
  float kd;             // 微分系数
  float output_limit;   // 输出限幅
  float integral_limit; // 积分限幅
} GM6020_PID_Params_t;

// 电机反馈数据
typedef struct {
  uint16_t angle;   // 机械角度 (0~8191)
  int16_t speed;    // 转速 (rpm，带符号)
  int16_t current;  // 实际电流 (mA，带符号)
  uint8_t temp;     // 温度 (°C)
  uint8_t err_code; // 错误码 (0为正常)
} GM6020_Feedback_t;

// 电机控制句柄（包含PID相关参数）
typedef struct {
  uint8_t id;                    // 电机ID (1~8)
  GM6020_Mode_t mode;            // 控制模式
  GM6020_PID_Params_t speed_pid; // 速度环PID参数
  GM6020_PID_Params_t pos_pid;   // 位置环PID参数

  // 目标值（根据模式选择使用）
  float target_pos;     // 目标角度 (单位：°，范围0~360)
  float target_speed;   // 目标转速 (单位：rpm)
  float target_current; // 目标电流 (单位：A)

  // 内部状态量
  GM6020_Feedback_t feedback; // 反馈数据
  float pos_set;              // 位置环输出（作为速度环目标）
  float speed_set;            // 速度环输出（作为电流环目标）
  void (*can_send)(uint16_t id, uint8_t *data); // CAN发送回调

  // PID内部变量
  float pos_err;      // 位置误差
  float pos_err_last; // 上一次位置误差
  float pos_integral; // 位置积分

  float speed_err;      // 速度误差
  float speed_err_last; // 上一次速度误差
  float speed_integral; // 速度积分
} GM6020_Handle_t;

// 初始化电机管理器
void GM6020_ManagerInit(GM6020_Handle_t *motors[], uint8_t count);

// 初始化单个电机
void GM6020_Init(GM6020_Handle_t *motor, uint8_t id,
                 void (*can_send)(uint16_t, uint8_t *));

// 设置控制模式
void GM6020_SetMode(GM6020_Handle_t *motor, GM6020_Mode_t mode);

// 设置PID参数
void GM6020_SetPIDParams(GM6020_Handle_t *motor,
                         GM6020_PID_Params_t *speed_params,
                         GM6020_PID_Params_t *pos_params);

// 设置目标值（根据当前模式自动生效）
void GM6020_SetTarget(GM6020_Handle_t *motor, float target);

// 执行PID计算（需周期性调用，建议1ms~10ms）
void GM6020_PIDCalculate(void);

// 批量发送电流指令
void GM6020_SendAll(void);

// CAN接收回调
void GM6020_CAN_RxCallback(uint16_t std_id, uint8_t *data);

#endif