#include "bsp_kth7823.h"
#include <string.h>

/* 超时时间 (ms) */
#ifndef KTH7823_SPI_TIMEOUT_MS
#define KTH7823_SPI_TIMEOUT_MS 20
#endif

/* ----------------- 私有工具函数 ----------------- */

/* CS 操作 */
static inline void BSP_KTH7823_CS_Enable(KTH7823_HandleTypeDef *hdev) {
  HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_RESET);
}
static inline void BSP_KTH7823_CS_Disable(KTH7823_HandleTypeDef *hdev) {
  HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_SET);
}

/* 通过 SPI 传输 16-bit（高字节先发），返回 16-bit 响应（高字节在 rx[0]） */
static uint16_t BSP_KTH7823_SPI_Transfer16(KTH7823_HandleTypeDef *hdev,
                                           uint16_t tx_word) {
  uint8_t tx[2], rx[2];
  tx[0] = (uint8_t)((tx_word >> 8) & 0xFF); /* 高字节先发 */
  tx[1] = (uint8_t)(tx_word & 0xFF);

  /* 使用字节传输 2 个字节，保证与 SPI 配置无关（MSB/LSB、8bit/16bit） */
  if (HAL_SPI_TransmitReceive(hdev->hspi, tx, rx, 2, KTH7823_SPI_TIMEOUT_MS) !=
      HAL_OK) {
    /* 如果失败返回 0xFFFF 作为错误标记（调用者需检测） */
    return 0xFFFFu;
  }

  return (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
}

/* ----------------- 寄存器读写实现 ----------------- */

/* 读寄存器 (8-bit)，返回高8位为数据（设备协议） */
uint8_t BSP_KTH7823_ReadRegister(KTH7823_HandleTypeDef *hdev, uint8_t reg_addr,
                                 uint8_t *reg_value) {
  if (hdev == NULL || reg_value == NULL)
    return KTH7823_ERROR;

  uint16_t cmd = (0x01u << 14) | ((uint16_t)(reg_addr & 0x3Fu) << 8);

  BSP_KTH7823_CS_Enable(hdev);

  /* 发送读命令（返回无用或状态），然后再发一次接收数据 */
  (void)BSP_KTH7823_SPI_Transfer16(hdev, cmd);
  uint16_t resp = BSP_KTH7823_SPI_Transfer16(hdev, 0x0000);

  BSP_KTH7823_CS_Disable(hdev);

  if (resp == 0xFFFFu)
    return KTH7823_ERROR;

  /* 设备的数据通常在高字节，根据手册取高字节 */
  *reg_value = (uint8_t)((resp >> 8) & 0xFFu);
  return KTH7823_OK;
}

/* 写寄存器 (8-bit) */
uint8_t BSP_KTH7823_WriteRegister(KTH7823_HandleTypeDef *hdev, uint8_t reg_addr,
                                  uint8_t reg_value) {
  if (hdev == NULL)
    return KTH7823_ERROR;

  uint16_t cmd = (0x02u << 14) | ((uint16_t)(reg_addr & 0x3Fu) << 8) |
                 ((uint16_t)reg_value & 0xFFu);

  BSP_KTH7823_CS_Enable(hdev);
  uint16_t resp = BSP_KTH7823_SPI_Transfer16(hdev, cmd);
  BSP_KTH7823_CS_Disable(hdev);

  if (resp == 0xFFFFu)
    return KTH7823_ERROR;

  /* 短暂延时做读回验证 */
  HAL_Delay(5);

  /* 读回校验（可选） */
  uint8_t read_back = 0;
  if (BSP_KTH7823_ReadRegister(hdev, reg_addr, &read_back) != KTH7823_OK)
    return KTH7823_ERROR;
  if (read_back != reg_value)
    return KTH7823_ERROR;

  return KTH7823_OK;
}

/* 锁定寄存器（写 WRDIS = 1） */
uint8_t BSP_KTH7823_LockRegisters(KTH7823_HandleTypeDef *hdev) {
  return BSP_KTH7823_WriteRegister(hdev, KTH7823_REG_WRDIS, 0x01u);
}

/* ----------------- 角度读取 ----------------- */

/* 读取原始 16-bit 角度寄存器（返回 16-bit 数据） */
uint8_t BSP_KTH7823_ReadRaw(KTH7823_HandleTypeDef *hdev, uint16_t *angle_raw) {
  if (hdev == NULL || angle_raw == NULL)
    return KTH7823_ERROR;

  BSP_KTH7823_CS_Enable(hdev);

  /* 读取角度命令：0x0000 (根据器件手册) */
  uint16_t resp = BSP_KTH7823_SPI_Transfer16(hdev, 0x0000);

  BSP_KTH7823_CS_Disable(hdev);

  if (resp == 0xFFFFu)
    return KTH7823_ERROR;

  *angle_raw = resp & 0xFFFFu;
  hdev->angle_raw = *angle_raw;
  return KTH7823_OK;
}

/* 读取角度并转换为度 (0..360) */
uint8_t BSP_KTH7823_ReadAngle(KTH7823_HandleTypeDef *hdev, float *angle_deg) {
  uint16_t raw;
  if (BSP_KTH7823_ReadRaw(hdev, &raw) != KTH7823_OK)
    return KTH7823_ERROR;

  /* 16-bit -> 0..360 mapping (65536 steps) */
  float deg = ((float)raw) * 360.0f / 65536.0f;
  hdev->angle_deg = deg;
  if (angle_deg)
    *angle_deg = deg;
  return KTH7823_OK;
}

/* ----------------- 高级功能 ----------------- */

/* 设置零点（写 Z_LOW / Z_HIGH） */
uint8_t BSP_KTH7823_SetZeroPosition(KTH7823_HandleTypeDef *hdev,
                                    uint16_t zero_offset) {
  if (hdev == NULL)
    return KTH7823_ERROR;

  uint8_t low = (uint8_t)(zero_offset & 0xFFu);
  uint8_t high = (uint8_t)((zero_offset >> 8) & 0xFFu);

  if (BSP_KTH7823_WriteRegister(hdev, KTH7823_REG_Z_LOW, low) != KTH7823_OK)
    return KTH7823_ERROR;
  if (BSP_KTH7823_WriteRegister(hdev, KTH7823_REG_Z_HIGH, high) != KTH7823_OK)
    return KTH7823_ERROR;

  /* 如果 Z 写入需要额外时间保证 NVM 编程，则上层可再延时 */
  HAL_Delay(20);
  return KTH7823_OK;
}

/* 设置旋转方向：读取寄存器 0x10，修改 bit0，写回 */
uint8_t BSP_KTH7823_SetRotationDirection(KTH7823_HandleTypeDef *hdev,
                                         uint8_t direction) {
  if (hdev == NULL)
    return KTH7823_ERROR;

  uint8_t rd = 0;
  if (BSP_KTH7823_ReadRegister(hdev, 0x10, &rd) != KTH7823_OK)
    return KTH7823_ERROR;

  if (direction == KTH7823_CW)
    rd |= 0x01u;
  else
    rd &= ~0x01u;

  if (BSP_KTH7823_WriteRegister(hdev, 0x10, rd) != KTH7823_OK)
    return KTH7823_ERROR;
  return KTH7823_OK;
}

/* 计算零点寄存器值（更直观的实现） */
uint16_t BSP_KTH7823_CalculateZeroValue(uint16_t current_angle,
                                        uint16_t desired_zero_deg,
                                        uint8_t direction) {
  /* 把 desired_zero_deg(0..360) 映射到 0..65535 */
  uint32_t desired_raw =
      (uint32_t)((uint32_t)desired_zero_deg * 65536UL / 360UL);
  int32_t delta = (int32_t)current_angle - (int32_t)desired_raw;

  if (direction == KTH7823_CW) {
    /* CW: we want Z such that adding Z produces desired => return -delta (two's
     * complement) */
    return (uint16_t)(-delta);
  } else {
    /* CCW: return delta */
    return (uint16_t)delta;
  }
}

/* ----------------- 初始化 ----------------- */

/**
 * 初始化设备：
 *  - 保存 SPI/CS 配置
 *  - 尝试读取角度验证是否响应
 *  - 可选写入零点、设置方向
 */
uint8_t BSP_KTH7823_Init(KTH7823_HandleTypeDef *hdev, SPI_HandleTypeDef *hspi,
                         GPIO_TypeDef *cs_port, uint16_t cs_pin,
                         uint16_t zero_offset, uint8_t direction_cw) {
  if (hdev == NULL || hspi == NULL)
    return KTH7823_ERROR;

  hdev->hspi = hspi;
  hdev->cs_port = cs_port;
  hdev->cs_pin = cs_pin;
  hdev->angle_raw = 0;
  hdev->angle_deg = 0.0f;

  /* 简单通信检测 */
  uint16_t raw;
  if (BSP_KTH7823_ReadRaw(hdev, &raw) != KTH7823_OK)
    return KTH7823_ERROR;

  /* 可选写零点 */
  if (zero_offset != 0xFFFFu) {
    if (BSP_KTH7823_SetZeroPosition(hdev, zero_offset) != KTH7823_OK)
      return KTH7823_ERROR;
  }

  /* 设置旋转方向 */
  if (BSP_KTH7823_SetRotationDirection(hdev, direction_cw) != KTH7823_OK) {
    /* 注意：如果方向寄存器读写失败，可考虑不把它当做致命错误 */
    return KTH7823_ERROR;
  }

  return KTH7823_OK;
}
