#ifndef __FOC_SELF_CALIB_H__
#define __FOC_SELF_CALIB_H__

#include "foc_controller.h" // 需要包含 FOC_Controller_t 定义

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机自校准配置结构
 */
typedef struct {
  float align_voltage; ///< 对齐电压 (V)
  float align_current; ///< 对齐电流 (A)
  float align_time;    ///< 对齐时间 (s)
} FOC_SelfCalib_Config_t;

/**
 * @brief 电机自校准状态
 */
typedef enum {
  SELF_CALIB_IDLE = 0,
  SELF_CALIB_RUNNING,
  SELF_CALIB_DONE,
  SELF_CALIB_FAILED
} FOC_SelfCalib_State_t;

/**
 * @brief 自校准对象句柄
 */
typedef struct {
  FOC_SelfCalib_Config_t cfg;
  FOC_SelfCalib_State_t state;
  float zero_offset; ///< 测得的电角度偏移 (rad)
} FOC_SelfCalib_Handle_t;

/**
 * @brief 初始化自校准模块
 * @param calib 自校准句柄
 * @param cfg 配置参数
 */
void FOC_SelfCalib_Init(FOC_SelfCalib_Handle_t *calib,
                        const FOC_SelfCalib_Config_t *cfg);

/**
 * @brief 执行零点自动校准
 * @param foc FOC控制器句柄
 * @param calib 自校准句柄
 * @return 自校准状态 (SELF_CALIB_DONE / SELF_CALIB_FAILED)
 */
FOC_SelfCalib_State_t FOC_SelfCalib_Execute(FOC_Controller_t *foc,
                                            FOC_SelfCalib_Handle_t *calib);

/**
 * @brief 获取校准状态
 */
FOC_SelfCalib_State_t FOC_SelfCalib_GetState(FOC_SelfCalib_Handle_t *calib);

/**
 * @brief 获取校准偏移量
 */
float FOC_SelfCalib_GetOffset(FOC_SelfCalib_Handle_t *calib);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SELF_CALIB_H__ */
