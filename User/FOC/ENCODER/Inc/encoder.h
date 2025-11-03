#ifndef ENCODER_H
#define ENCODER_H

#include "kth7823.h" // 包含 KTH7823 驱动的头文件
#include "main.h"
#include "mt6709.h" // 包含 MT6709 驱动的头文件
#include <stdint.h>

#define ENCODER_OK 0
#define ENCODER_ERROR -1
/* ========== 编码器类型枚举 ========== */
typedef enum {
  ENCODER_TYPE_NONE = 0,
  ENCODER_TYPE_KTH7823,
  ENCODER_TYPE_MT6709,
  // ... 其他编码器类型
} Encoder_Type_e;

/* ========== 通用编码器函数指针定义 ========== */
typedef uint8_t (*Encoder_Init_Func)(void *hdev, SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     uint16_t zero_offset,
                                     uint8_t direction_cw);
typedef uint8_t (*Encoder_ReadRaw_Func)(void *hdev, uint16_t *angle_raw);
typedef uint8_t (*Encoder_ReadAngle_Func)(void *hdev, float *angle_deg);
typedef uint8_t (*Encoder_SetZeroPosition_Func)(void *hdev,
                                                uint16_t zero_offset);
typedef uint8_t (*Encoder_SetRotationDirection_Func)(void *hdev,
                                                     uint8_t direction);

/* ========== 通用编码器句柄结构体 ========== */
typedef struct {
  Encoder_Type_e type;
  void *hdev_specific; // 指向特定编码器句柄 (例如 KTH7823_HandleTypeDef *)

  // 函数指针
  Encoder_Init_Func pf_init;
  Encoder_ReadRaw_Func pf_read_raw;
  Encoder_ReadAngle_Func pf_read_angle;
  Encoder_SetZeroPosition_Func pf_set_zero_position;
  Encoder_SetRotationDirection_Func pf_set_rotation_direction;

  uint16_t angle_raw; // 最近一次原始读数
  float angle_deg;    // 最近一次角度 (0..360)

} Encoder_HandleTypeDef;

/*  全局编码器句柄实例 */
extern Encoder_HandleTypeDef kth7823_encoder;
extern Encoder_HandleTypeDef mt6709_encoder;
/* ========== 外部接口函数 ========== */

/**
 * @brief 注册并初始化编码器
 * @param hencoder: 通用编码器句柄
 * @param type: 编码器类型
 * @param hspi: HAL SPI 句柄 (如果编码器使用 SPI)
 * @param cs_port: 片选端口 (如果编码器使用 SPI)
 * @param cs_pin: 片选引脚 (如果编码器使用 SPI)
 * @param zero_offset: 零点偏移 (如果编码器支持)
 * @param direction_cw: 旋转方向 (如果编码器支持)
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_Register_Init(Encoder_HandleTypeDef *hencoder,
                              Encoder_Type_e type, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin,
                              uint16_t zero_offset, uint8_t direction_cw);

/**
 * @brief 读取原始角度
 * @param hencoder: 通用编码器句柄
 * @param angle_raw: 输出原始角度
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_ReadRaw(Encoder_HandleTypeDef *hencoder, uint16_t *angle_raw);

/**
 * @brief 读取角度 (0..360度)
 * @param hencoder: 通用编码器句柄
 * @param angle_deg: 输出角度
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_ReadAngle(Encoder_HandleTypeDef *hencoder, float *angle_deg);

/**
 * @brief 设置编码器零点
 * @param hencoder: 通用编码器句柄
 * @param zero_offset: 零点偏移值
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_SetZeroPosition(Encoder_HandleTypeDef *hencoder,
                                uint16_t zero_offset);

/**
 * @brief 设置编码器旋转方向
 * @param hencoder: 通用编码器句柄
 * @param direction: 旋转方向 (KTH7823_CW 或 KTH7823_CCW)
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_SetRotationDirection(Encoder_HandleTypeDef *hencoder,
                                     uint8_t direction);

#endif /* ENCODER_H */