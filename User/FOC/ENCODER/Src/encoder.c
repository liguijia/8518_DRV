#include "encoder.h"
#include <stdlib.h> // for malloc/free

Encoder_HandleTypeDef rotor_encoder;
Encoder_HandleTypeDef output_encoder;

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
                              uint16_t zero_offset, uint8_t direction_cw) {
  if (hencoder == NULL) {
    return ENCODER_ERROR;
  }

  hencoder->type = type;
  hencoder->angle_raw = 0;
  hencoder->angle_deg = 0.0f;

  switch (type) {
  case ENCODER_TYPE_KTH7823: {
    KTH7823_HandleTypeDef *kth_hdev =
        (KTH7823_HandleTypeDef *)malloc(sizeof(KTH7823_HandleTypeDef));
    if (kth_hdev == NULL)
      return ENCODER_ERROR;
    hencoder->hdev_specific = kth_hdev;

    hencoder->pf_init = (Encoder_Init_Func)KTH7823_Init;
    hencoder->pf_read_raw = (Encoder_ReadRaw_Func)KTH7823_ReadRaw;
    hencoder->pf_read_angle = (Encoder_ReadAngle_Func)KTH7823_ReadAngle;
    hencoder->pf_set_zero_position =
        (Encoder_SetZeroPosition_Func)KTH7823_SetZeroPosition;
    hencoder->pf_set_rotation_direction =
        (Encoder_SetRotationDirection_Func)KTH7823_SetRotationDirection;

    if (hencoder->pf_init(hencoder->hdev_specific, hspi, cs_port, cs_pin,
                          zero_offset, direction_cw) != KTH7823_OK) {
      free(kth_hdev);
      return ENCODER_ERROR;
    }
  } break;

  case ENCODER_TYPE_MT6709: {
    MT6709_HandleTypeDef *mt_hdev =
        (MT6709_HandleTypeDef *)malloc(sizeof(MT6709_HandleTypeDef));
    if (mt_hdev == NULL)
      return ENCODER_ERROR;
    hencoder->hdev_specific = mt_hdev;

    hencoder->pf_init = (Encoder_Init_Func)MT6709_Init;
    hencoder->pf_read_raw = (Encoder_ReadRaw_Func)MT6709_ReadRaw;
    hencoder->pf_read_angle = (Encoder_ReadAngle_Func)MT6709_ReadAngle;
    // MT6709 暂时没有设置零点和方向的函数，置为 NULL
    hencoder->pf_set_zero_position = NULL;
    hencoder->pf_set_rotation_direction = NULL;

    if (hencoder->pf_init(hencoder->hdev_specific, hspi, cs_port, cs_pin, 0,
                          0) !=
        MT6709_OK) { // MT6709 init does not use zero_offset and direction_cw
      free(mt_hdev);
      return ENCODER_ERROR;
    }
  } break;

  case ENCODER_TYPE_NONE:
  default:
    return ENCODER_ERROR;
  }

  return ENCODER_OK;
}

/**
 * @brief 读取原始角度
 * @param hencoder: 通用编码器句柄
 * @param angle_raw: 输出原始角度
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_ReadRaw(Encoder_HandleTypeDef *hencoder, uint16_t *angle_raw) {
  if (hencoder == NULL || hencoder->pf_read_raw == NULL) {
    return ENCODER_ERROR;
  }
  uint8_t ret = hencoder->pf_read_raw(hencoder->hdev_specific, angle_raw);
  if (ret == KTH7823_OK || ret == MT6709_OK) { // 兼容 KTH7823_OK 和 MT6709_OK
    hencoder->angle_raw = *angle_raw;
    return ENCODER_OK;
  }
  return ENCODER_ERROR;
}

/**
 * @brief 读取角度 (0..360度)
 * @param hencoder: 通用编码器句柄
 * @param angle_deg: 输出角度
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_ReadAngle(Encoder_HandleTypeDef *hencoder, float *angle_deg) {
  if (hencoder == NULL || hencoder->pf_read_angle == NULL) {
    return ENCODER_ERROR;
  }
  uint8_t ret = hencoder->pf_read_angle(hencoder->hdev_specific, angle_deg);
  if (ret == KTH7823_OK || ret == MT6709_OK) { // 兼容 KTH7823_OK 和 MT6709_OK
    hencoder->angle_deg = *angle_deg;
    return ENCODER_OK;
  }
  return ENCODER_ERROR;
}

/**
 * @brief 设置编码器零点
 * @param hencoder: 通用编码器句柄
 * @param zero_offset: 零点偏移值
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_SetZeroPosition(Encoder_HandleTypeDef *hencoder,
                                uint16_t zero_offset) {
  if (hencoder == NULL || hencoder->pf_set_zero_position == NULL) {
    return ENCODER_ERROR;
  }
  uint8_t ret =
      hencoder->pf_set_zero_position(hencoder->hdev_specific, zero_offset);
  if (ret == KTH7823_OK || ret == MT6709_OK) {
    return ENCODER_OK;
  }
  return ENCODER_ERROR;
}

/**
 * @brief 设置编码器旋转方向
 * @param hencoder: 通用编码器句柄
 * @param direction: 旋转方向 (KTH7823_CW 或 KTH7823_CCW)
 * @retval ENCODER_OK / ENCODER_ERROR
 */
uint8_t Encoder_SetRotationDirection(Encoder_HandleTypeDef *hencoder,
                                     uint8_t direction) {
  if (hencoder == NULL || hencoder->pf_set_rotation_direction == NULL) {
    return ENCODER_ERROR;
  }
  uint8_t ret =
      hencoder->pf_set_rotation_direction(hencoder->hdev_specific, direction);
  if (ret == KTH7823_OK || ret == MT6709_OK) {
    return ENCODER_OK;
  }
  return ENCODER_ERROR;
}