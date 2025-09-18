#ifndef BSP_KTH7823_H
#define BSP_KTH7823_H

#include "main.h"
#include <stdint.h>

/* ========== KTH7823 寄存器地址 ========== */
#define KTH7823_REG_Z_LOW 0x00
#define KTH7823_REG_Z_HIGH 0x01
#define KTH7823_REG_GAINTRIM 0x02
#define KTH7823_REG_ABZ_CONFIG 0x04
#define KTH7823_REG_PPT_HIGH 0x05
#define KTH7823_REG_MAG_THRESH 0x06
#define KTH7823_REG_ABZ_LIMIT 0x08
#define KTH7823_REG_WRDIS 0x10

/* ========== 错误代码 ========== */
#define KTH7823_OK 0
#define KTH7823_ERROR -1
#define KTH7823_TIMEOUT -2
#define KTH7823_CRC_ERR -3

/* ========== 旋转方向 ========== */
#define KTH7823_CW 1  /* 顺时针 */
#define KTH7823_CCW 0 /* 逆时针 */

/* ========== 设备结构体 ========== */
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;

  uint16_t angle_raw; /* 最近一次原始读数 */
  float angle_deg;    /* 最近一次角度 (0..360) */
} KTH7823_HandleTypeDef;

/* ========== 外部接口函数 ========== */

/**
 * @brief 初始化 KTH7823 设备
 * @param hdev: 设备句柄 (指针)
 * @param hspi: HAL SPI 句柄
 * @param cs_port: 片选端口
 * @param cs_pin: 片选引脚
 * @param zero_offset: 若为 0xFFFF 则跳过写入零点，否则写入此 16-bit 零点值
 * @param direction_cw: 1 = 设置为顺时针，0 = 设置为逆时针
 * @retval KTH7823_OK / KTH7823_ERROR
 */
uint8_t BSP_KTH7823_Init(KTH7823_HandleTypeDef *hdev, SPI_HandleTypeDef *hspi,
                         GPIO_TypeDef *cs_port, uint16_t cs_pin,
                         uint16_t zero_offset, uint8_t direction_cw);

/**
 * @brief 读取原始角度寄存器（16-bit 原始）
 * @param hdev: 设备句柄
 * @param angle_raw: 输出原始值（若非 NULL）
 * @retval KTH7823_OK / KTH7823_ERROR
 */
uint8_t BSP_KTH7823_ReadRaw(KTH7823_HandleTypeDef *hdev, uint16_t *angle_raw);

/**
 * @brief 读取角度（度）
 * @param hdev: 设备句柄
 * @param angle_deg: 输出角度 (0..360)
 * @retval KTH7823_OK / KTH7823_ERROR
 */
uint8_t BSP_KTH7823_ReadAngle(KTH7823_HandleTypeDef *hdev, float *angle_deg);

/**
 * @brief 读取寄存器 (8-bit)
 */
uint8_t BSP_KTH7823_ReadRegister(KTH7823_HandleTypeDef *hdev, uint8_t reg_addr,
                                 uint8_t *reg_value);

/**
 * @brief 写入寄存器 (8-bit)
 */
uint8_t BSP_KTH7823_WriteRegister(KTH7823_HandleTypeDef *hdev, uint8_t reg_addr,
                                  uint8_t reg_value);

/**
 * @brief 设置零点（写入 Z_HIGH / Z_LOW）
 */
uint8_t BSP_KTH7823_SetZeroPosition(KTH7823_HandleTypeDef *hdev,
                                    uint16_t zero_offset);

/**
 * @brief 设置旋转方向（通过寄存器位写回）
 */
uint8_t BSP_KTH7823_SetRotationDirection(KTH7823_HandleTypeDef *hdev,
                                         uint8_t direction);

/**
 * @brief 锁定寄存器，写入 WRDIS 寄存器
 */
uint8_t BSP_KTH7823_LockRegisters(KTH7823_HandleTypeDef *hdev);

/**
 * @brief  计算零点寄存器值（便于设置）
 * @note   current_angle/desired_zero 输入均为 0..65535 (desired_zero 用 0..360)
 */
uint16_t BSP_KTH7823_CalculateZeroValue(uint16_t current_angle,
                                        uint16_t desired_zero_deg,
                                        uint8_t direction);

#endif /* BSP_KTH7823_H */
