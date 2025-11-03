#include "mt6709.h"
#include <string.h>

/* ----------------- 私有工具函数 ----------------- */

/**
 * @brief 片选控制（低电平使能，高电平禁用，手册表17时序）
 */
static inline void MT6709_CS_Enable(MT6709_HandleTypeDef *hdev) {
  HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_RESET);
  HAL_Delay(1); // 满足 CSN 下降沿到 SCK 上升沿的最小延时（100ns，手册表17）
}

static inline void MT6709_CS_Disable(MT6709_HandleTypeDef *hdev) {
  HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief SPI 传输 16 位数据（手册7.4.2
 * SPI协议：MSB先发，SCK空闲低，上升沿采样）
 * @param tx_word: 发送的 16 位命令/数据
 * @param rx_word: 接收的 16 位响应（输出）
 * @retval MT6709_OK/MT6709_TIMEOUT
 */
static uint8_t MT6709_SPI_Transfer16(MT6709_HandleTypeDef *hdev,
                                     uint16_t tx_word, uint16_t *rx_word) {
  uint8_t tx[2] = {0}, rx[2] = {0};

  // 拆分 16 位数据为高8位+低8位（MSB先发，手册7.4.2）
  tx[0] = (uint8_t)((tx_word >> 8) & 0xFF);
  tx[1] = (uint8_t)(tx_word & 0xFF);

  // 调用 HAL SPI 收发函数（CubeMX 已初始化 SPI 参数）
  if (HAL_SPI_TransmitReceive(hdev->hspi, tx, rx, 2, MT6709_SPI_TIMEOUT_MS) !=
      HAL_OK) {
    return MT6709_TIMEOUT;
  }

  // 重组接收数据为 16 位
  *rx_word = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
  return MT6709_OK;
}

/**
 * @brief 生成 CRC8 校验码（手册7.4.2：多项式 X⁸+X⁴+X³+X²+1，初始值
 * 0xFF，余数取反）
 * @param data: 待校验数据缓冲区
 * @param len: 数据长度（字节）
 * @retval CRC8 校验值
 */
static uint8_t MT6709_CalcCRC8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF; // 初始值（手册7.4.2）
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      // 多项式 0x1D = X⁸+X⁴+X³+X²+1（手册7.4.2）
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x1D) : (crc << 1);
    }
  }
  return ~crc; // 余数取反（手册7.4.2）
}

/**
 * @brief 解锁寄存器（写 WRDIS=0，取消写保护，手册9章寄存器）
 */
static uint8_t MT6709_UnlockRegisters(MT6709_HandleTypeDef *hdev) {
  return MT6709_WriteRegister(hdev, MT6709_REG_WRDIS, 0x00);
}

/* ----------------- 寄存器读写实现 ----------------- */

uint8_t MT6709_ReadRegister(MT6709_HandleTypeDef *hdev, uint8_t reg_addr,
                            uint8_t *reg_value) {
  if (hdev == NULL || reg_value == NULL)
    return MT6709_ERROR;

  uint16_t cmd = 0, rx_word = 0;
  // SPI 读命令格式（手册表18）：
  // Bit15=1（读操作），Bit14-12=000（固定），Bit11-4=寄存器地址，Bit3-0=0（读1字无安全字）
  cmd = (0x01 << 15) | ((uint16_t)(reg_addr & 0xFF) << 4) | 0x00;

  MT6709_CS_Enable(hdev);
  uint8_t ret = MT6709_SPI_Transfer16(hdev, cmd, &rx_word);
  MT6709_CS_Disable(hdev);

  if (ret != MT6709_OK)
    return ret;
  // 寄存器值存储在接收数据的 Bit11-4（手册表18）
  *reg_value = (uint8_t)((rx_word >> 4) & 0xFF);
  return MT6709_OK;
}

uint8_t MT6709_WriteRegister(MT6709_HandleTypeDef *hdev, uint8_t reg_addr,
                             uint8_t reg_value) {
  if (hdev == NULL)
    return MT6709_ERROR;

  uint16_t cmd = 0, rx_word = 0;
  // SPI 写命令格式（手册表18）：
  // Bit15=0（写操作），Bit14-12=000（固定），Bit11-4=寄存器地址，Bit3-0=0（写1字无安全字）
  cmd = (0x00 << 15) | ((uint16_t)(reg_addr & 0xFF) << 4) |
        ((uint16_t)(reg_value & 0x0F) << 0);

  MT6709_CS_Enable(hdev);
  uint8_t ret = MT6709_SPI_Transfer16(hdev, cmd, &rx_word);
  MT6709_CS_Disable(hdev);

  if (ret != MT6709_OK)
    return ret;

  // 验证写入结果（延时 5ms 确保 EEPROM 写入完成，手册12章版本信息）
  HAL_Delay(5);
  uint8_t read_back = 0;
  if (MT6709_ReadRegister(hdev, reg_addr, &read_back) != MT6709_OK)
    return MT6709_ERROR;
  if (read_back != reg_value)
    return MT6709_ERROR;

  return MT6709_OK;
}

/* ----------------- 角度读取实现 ----------------- */

uint8_t MT6709_ReadRaw(MT6709_HandleTypeDef *hdev, uint16_t *angle_raw) {
  if (hdev == NULL || angle_raw == NULL)
    return MT6709_ERROR;

  uint8_t reg01 = 0, reg02 = 0;
  uint16_t raw = 0;

  // 读取角度高位寄存器（0x01：ANGLE[16:1]，手册表20）
  if (MT6709_ReadRegister(hdev, MT6709_REG_ANGLE_01, &reg01) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 读取角度低位寄存器（0x02：ANGLE[0]，手册表20）
  if (MT6709_ReadRegister(hdev, MT6709_REG_ANGLE_02, &reg02) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 重组 17 位角度值（ANGLE[16:0] = reg01[7:0] << 1 | reg02[7]，手册表20）
  raw = ((uint16_t)reg01 << 1) | ((uint16_t)(reg02 >> 7) & 0x01);
  hdev->angle_raw = raw;
  *angle_raw = raw;

  // 同步计算温度（手册7.4.3公式：T=27 + (TS_Code[10]·2⁸ - ΣTS_Code[0-9]·2⁻²)）
  int16_t ts_code = (reg02 & 0x7F) << 3; // TS_Code[6:0] 扩展为 10 位
  hdev->temp_c = 27 + (((ts_code >> 8) & 0x01) * 256 - (ts_code & 0xFF) / 4.0f);

  return MT6709_OK;
}

uint8_t MT6709_ReadAngle(MT6709_HandleTypeDef *hdev, float *angle_deg,
                         int16_t *temp_c) {
  uint16_t raw = 0;

  // 读取原始角度（17位，0~131071）
  if (MT6709_ReadRaw(hdev, &raw) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 转换角度为 0~360°（手册公式：θ=ANGLE/2¹⁷×360，2¹⁷=131072）
  float deg = ((float)raw / 131071.0f) * 360.0f;
  hdev->angle_deg = deg;

  // 输出角度和温度（若指针非 NULL）
  if (angle_deg != NULL)
    *angle_deg = deg;
  if (temp_c != NULL)
    *temp_c = hdev->temp_c;

  return MT6709_OK;
}

/* ----------------- 高级功能实现 ----------------- */

uint8_t MT6709_SetZeroPosition(MT6709_HandleTypeDef *hdev, float zero_deg) {
  if (hdev == NULL || zero_deg < 0.0f || zero_deg > 360.0f)
    return MT6709_ERROR;

  // 1. 解锁寄存器（取消写保护，手册9章）
  if (MT6709_UnlockRegisters(hdev) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 2. 计算 ZERO_POS 寄存器值（12位：0~4095对应0~360°，手册表13）
  uint16_t zero_pos = (uint16_t)((zero_deg / 360.0f) * 4095.0f);
  uint8_t zero_pos_high = (uint8_t)((zero_pos >> 4) & 0xFF); // ZERO_POS[11:4]
  uint8_t zero_pos_low = (uint8_t)(zero_pos & 0x0F);         // ZERO_POS[3:0]

  // 3. 写入 ZERO_POS 寄存器（分高低位，手册表13）
  if (MT6709_WriteRegister(hdev, MT6709_REG_ZERO_POS, zero_pos_high) !=
      MT6709_OK) {
    return MT6709_ERROR;
  }
  if (MT6709_WriteRegister(hdev, MT6709_REG_ZERO_POS + 1, zero_pos_low) !=
      MT6709_OK) {
    return MT6709_ERROR;
  }

  // 4. 锁定寄存器（恢复写保护，防止误修改）
  if (MT6709_LockRegisters(hdev) != MT6709_OK) {
    return MT6709_ERROR;
  }

  return MT6709_OK;
}

uint8_t MT6709_SetRotationDirection(MT6709_HandleTypeDef *hdev,
                                    uint8_t direction_cw) {
  if (hdev == NULL)
    return MT6709_ERROR;

  // 1. 解锁寄存器（手册9章）
  if (MT6709_UnlockRegisters(hdev) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 2. 读取当前旋转方向寄存器（手册表10）
  uint8_t rot_dir = 0;
  if (MT6709_ReadRegister(hdev, MT6709_REG_ROT_DIR, &rot_dir) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 3. 修改旋转方向（手册表10：ROT_DIR=0→CCW，1→CW）
  rot_dir = (direction_cw == MT6709_CW) ? 0x01 : 0x00;
  if (MT6709_WriteRegister(hdev, MT6709_REG_ROT_DIR, rot_dir) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 4. 锁定寄存器
  if (MT6709_LockRegisters(hdev) != MT6709_OK) {
    return MT6709_ERROR;
  }

  return MT6709_OK;
}

uint8_t MT6709_LockRegisters(MT6709_HandleTypeDef *hdev) {
  // 写 WRDIS=1，使能寄存器写保护（手册9章）
  return MT6709_WriteRegister(hdev, MT6709_REG_WRDIS, 0x01);
}

uint8_t MT6709_StartCalibration(MT6709_HandleTypeDef *hdev,
                                GPIO_TypeDef *cal_enb_port,
                                uint16_t cal_enb_pin) {
  if (hdev == NULL || cal_enb_port == NULL)
    return MT6709_ERROR;

  // 1. 拉低 CAL_ENB 引脚，启动自校准（手册8.1章节：低电平有效）
  HAL_GPIO_WritePin(cal_enb_port, cal_enb_pin, GPIO_PIN_RESET);
  HAL_Delay(200); // 等待校准状态稳定（手册8.1）

  // 2. 等待校准完成（最多等待30秒，每500ms检查一次校准状态，手册表23）
  uint8_t cal_state = 0;
  uint32_t timeout = HAL_GetTick() + 30000; // 30秒超时
  while (HAL_GetTick() < timeout) {
    if (MT6709_ReadRegister(hdev, MT6709_REG_CAL_STATE, &cal_state) !=
        MT6709_OK) {
      // 校准失败，先拉高 CAL_ENB 退出
      HAL_GPIO_WritePin(cal_enb_port, cal_enb_pin, GPIO_PIN_SET);
      return MT6709_ERROR;
    }

    if (cal_state == 0x02) { // 校准失败（手册表23）
      HAL_GPIO_WritePin(cal_enb_port, cal_enb_pin, GPIO_PIN_SET);
      return MT6709_CAL_ERR;
    } else if (cal_state == 0x03) { // 校准成功（手册表23）
      break;
    }

    HAL_Delay(500); // 500ms 轮询一次
  }

  // 3. 校准超时判断
  if (HAL_GetTick() >= timeout) {
    HAL_GPIO_WritePin(cal_enb_port, cal_enb_pin, GPIO_PIN_SET);
    return MT6709_TIMEOUT;
  }

  // 4. 校准成功，等待≥500ms后拉高 CAL_ENB 退出（手册8.1章节）
  HAL_Delay(500);
  HAL_GPIO_WritePin(cal_enb_port, cal_enb_pin, GPIO_PIN_SET);
  hdev->cal_state = cal_state;

  return MT6709_OK;
}

/* ----------------- 设备初始化实现 ----------------- */

uint8_t MT6709_Init(MT6709_HandleTypeDef *hdev, SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port, uint16_t cs_pin,
                    uint16_t zero_offset, uint8_t direction_cw) {
  if (hdev == NULL || hspi == NULL || cs_port == NULL)
    return MT6709_ERROR;

  // 1. 保存硬件配置（CubeMX 已初始化 SPI 和 GPIO）
  hdev->hspi = hspi;
  hdev->cs_port = cs_port;
  hdev->cs_pin = cs_pin;
  hdev->angle_raw = 0;
  hdev->angle_deg = 0.0f;
  hdev->temp_c = 0;
  hdev->cal_state = 0;

  // 2. 初始片选拉高（禁用状态）
  MT6709_CS_Disable(hdev);

  // 3. 验证 SPI 通信（读取原始角度判断设备是否响应，手册7.4.3）
  uint16_t raw = 0;
  if (MT6709_ReadRaw(hdev, &raw) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 4. 配置零点偏移（若 zero_offset≠0xFFFF）
  if (zero_offset != 0xFFFF) {
    // 12位零点值转换为角度（0~4095→0~360°，手册表13）
    float zero_deg = ((float)zero_offset / 4095.0f) * 360.0f;
    if (MT6709_SetZeroPosition(hdev, zero_deg) != MT6709_OK) {
      return MT6709_ERROR;
    }
  }

  // 5. 配置旋转方向
  if (MT6709_SetRotationDirection(hdev, direction_cw) != MT6709_OK) {
    return MT6709_ERROR;
  }

  // 6. 读取初始校准状态（手册表23）
  MT6709_ReadRegister(hdev, MT6709_REG_CAL_STATE, &hdev->cal_state);

  return MT6709_OK;
}