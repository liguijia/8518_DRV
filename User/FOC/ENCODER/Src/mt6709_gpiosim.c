#include "mt6709_gpiosim.h"
#include "bsp_delay.h"
static MT6709_GPIOConfig_t g_cfg;

/**
 * @brief 从 HAL 的 GPIO_Pin 位掩码 (GPIO_PIN_x) 获取引脚编号 (0-15)
 */
static uint32_t GetPinSource(uint16_t GPIO_Pin) {
  uint32_t pin_source = 0;
  while (GPIO_Pin > 1) {
    GPIO_Pin >>= 1;
    pin_source++;
  }
  return pin_source;
}

/**
 * @brief 快速切换 SDAT 为推挽输出模式
 */
static inline void sdat_output(void) {
  uint32_t pin_num = GetPinSource(g_cfg.sdat_pin);

  // 设置为推挽输出 (01)
  g_cfg.sdat_port->MODER &=
      ~(GPIO_MODER_MODE0 << (pin_num * 2U)); // Mask (e.g., 0b11)
  g_cfg.sdat_port->MODER |=
      (GPIO_MODE_OUTPUT_PP << (pin_num * 2U)); // Value (e.g., 0b01)

  // 确保是推挽 (0)
  g_cfg.sdat_port->OTYPER &= ~g_cfg.sdat_pin;
}

/**
 * @brief 快速切换 SDAT 为浮空输入模式
 */
static inline void sdat_input(void) {
  uint32_t pin_num = GetPinSource(g_cfg.sdat_pin);
  // 设置为输入 (00)
  g_cfg.sdat_port->MODER &= ~(GPIO_MODER_MODER0 << (pin_num * 2U));
  // 确保无上拉/下拉 (00)
  g_cfg.sdat_port->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin_num * 2U));
}

/**
 * @brief 简单的 SPI 延时
 * Datasheet (Table 17) 要求 TSCK (周期) 最小 125ns,
 * TSCKL/TSCKH (高/低电平时间) 最小 50ns。
 * 在高速 MCU (如 170MHz) 上, 几个 NOPs 是必要的。
 */
static inline void spi_delay(void) { BSP_Delay_ns(75); }

/**
 * @brief (已修正) 发送 16-bit 数据 (SPI Mode 1)
 * SCK 空闲低, 在上升沿改变数据, 在下降沿采样数据
 */
static void send_16bit(uint16_t data) {
  sdat_output(); // 设置为输出模式
  spi_delay();   // 确保模式切换生效

  for (int i = 0; i < 16; i++) {
    // 1. 在上升沿改变数据
    g_cfg.sck_port->BSRR = g_cfg.sck_pin; // SCK High
    if (data & 0x8000) {
      g_cfg.sdat_port->BSRR = g_cfg.sdat_pin;
    } else {
      g_cfg.sdat_port->BSRR = (uint32_t)g_cfg.sdat_pin << 16U;
    }
    data <<= 1;
    spi_delay(); // 保持 TSCKH (min 50ns)

    // 2. 在下降沿采样 (由从机完成)
    g_cfg.sck_port->BSRR = (uint32_t)g_cfg.sck_pin << 16U; // SCK Low
    spi_delay(); // 保持 TSCKL (min 50ns)
  }
}

/**
 * @brief (已修正) 接收 16-bit 数据 (SPI Mode 1)
 * SCK 空闲低, 在上升沿改变数据, 在下降沿采样数据
 */
static uint16_t recv_16bit(void) {
  uint16_t data = 0;
  sdat_input(); // 设置为输入模式
  spi_delay();  // 确保模式切换生效

  for (int i = 0; i < 16; i++) {
    // 1. 在上升沿改变数据 (由从机完成)
    g_cfg.sck_port->BSRR = g_cfg.sck_pin; // SCK High
    spi_delay();                          // 保持 TSCKH (min 50ns)
    // 2. 在下降沿采样数据 (由主机完成)
    g_cfg.sck_port->BSRR = (uint32_t)g_cfg.sck_pin << 16U; // SCK Low

    data <<= 1;
    if (g_cfg.sdat_port->IDR & g_cfg.sdat_pin) {
      data |= 1;
    }
    spi_delay(); // 保持 TSCKL (min 50ns)
  }
  return data;
}

void MT6709_GPIO_Init(const MT6709_GPIOConfig_t *config) {
  g_cfg = *config;

  // 确保 GPIO 已在 STM32CubeMX 中配置为：
  // CS, SCK: GPIO_Output, Push-Pull, High-Speed
  // SDAT(MOSI): GPIO_Output, Push-Pull, High-Speed
  //
  // 仅在模拟 SPI 时, 我们才动态更改 SDAT 的方向
  // 确保 SDAT 引脚的 OTYPER (Output Type) 已配置为 Push-Pull (推挽)

  // 拉高 CS (空闲)
  g_cfg.cs_port->BSRR = g_cfg.cs_pin;
  // SCK 默认低 (SPI Mode 1)
  g_cfg.sck_port->BSRR = (uint32_t)g_cfg.sck_pin << 16U;

  // 初始将 SDAT 设置为输出, 准备发送命令
  sdat_output();
}

/**
 * @brief (已修正) 读取原始 17-bit 角度数据
 * 执行两次独立的 SPI 事务
 */
uint32_t MT6709_ReadRawAngle(void) {
  uint16_t reg01, reg02;
  uint32_t angle_raw;

  // SPI 事务 1: 读取寄存器 0x01 (ANGLE[16:1])
  g_cfg.cs_port->BSRR = (uint32_t)g_cfg.cs_pin << 16U; // CS Low
  send_16bit(0x8010);                 // 发送读命令 (Reg 0x01, N=0)
  reg01 = recv_16bit();               // 接收数据
  g_cfg.cs_port->BSRR = g_cfg.cs_pin; // CS High

  spi_delay(); // 确保 CSN 保持高电平 (满足 TLATCH > 1us)

  // SPI 事务 2: 读取寄存器 0x02 (ANGLE[0])
  g_cfg.cs_port->BSRR = (uint32_t)g_cfg.cs_pin << 16U; // CS Low
  send_16bit(0x8020);                 // 发送读命令 (Reg 0x02, N=0)
  reg02 = recv_16bit();               // 接收数据
  g_cfg.cs_port->BSRR = g_cfg.cs_pin; // CS High

  // 组合 17-bit 角度数据 (根据手册 Table 20)
  // 17-bit 角度 = {reg01[15:0], reg02[15]}
  angle_raw = ((uint32_t)reg01 << 1) | ((reg02 >> 15) & 0x01);

  return angle_raw;
}

/**
 * @brief 将原始数据转换为 0-360 度的角度值
 */
float MT6709_ReadAngleDeg(void) {
  uint32_t raw = MT6709_ReadRawAngle();
  // 17-bit 分辨率 (2^17 = 131072)
  // 角度 = (raw / 2^17) * 360
  return (float)raw * 360.0f / 131072.0f;
}

/**
 * @brief 使用 Range=2 模式读取多个寄存器（0x01, 0x02, Safety Word）
 * @param reg_addr 起始寄存器地址（通常为 0x01）
 * @return 结构体包含角度、温度、安全字
 */
MT6709_MultiReadResult_t MT6709_ReadMulti(uint8_t reg_addr) {
  MT6709_MultiReadResult_t result = {0};

  uint16_t cmd = (0x8000 | (reg_addr << 4)) | 0x04; // Range=2

  g_cfg.cs_port->BSRR = (uint32_t)g_cfg.cs_pin << 16U; // CS Low
  send_16bit(cmd);

  uint16_t data1 = recv_16bit();           // Reg 0x01
  uint16_t data2 = recv_16bit();           // Reg 0x02
  uint16_t safety_word_raw = recv_16bit(); // Safety Word

  g_cfg.cs_port->BSRR = g_cfg.cs_pin; // CS High

  // --- 解析角度 ---
  uint32_t angle_raw = ((uint32_t)data1 << 1) | ((data2 >> 15) & 1U);
  result.angle_deg = (float)angle_raw * 360.0f / 131072.0f;

  // --- 解析温度 ---
  uint16_t ts_code = data2 & 0x07FF;     // 取 bit[10:0]
  uint8_t ts_high = (ts_code >> 10) & 1; // bit10
  uint16_t ts_low = ts_code & 0x3FF;     // bit9~0
  result.temperature_c = 27.0f + (ts_high * 256.0f - ts_low * 0.25f);

  // --- 解析 Safety Word ---
  result.safety_word.diag_stat = (safety_word_raw >> 12) & 0x0F;
  result.safety_word.customer_id = (safety_word_raw >> 8) & 0x0F;
  result.safety_word.crc = safety_word_raw & 0xFF;

  // --- 直接解析 DIAG_STAT 报警位 ---
  result.safety_word.diag_flags.signal_error =
      (result.safety_word.diag_stat & (1U << 0)) != 0; // S[12]
  result.safety_word.diag_flags.speed_or_cal_error =
      (result.safety_word.diag_stat & (1U << 1)) != 0; // S[13]
  result.safety_word.diag_flags.eeprom_error =
      (result.safety_word.diag_stat & (1U << 2)) != 0; // S[14]
  result.safety_word.diag_flags.under_voltage =
      (result.safety_word.diag_stat & (1U << 3)) != 0; // S[15]

  // --- 可选：对 CUSTOMER_ID 做合理性检查（例如非全0/全F）---
  // 这里不做强制处理，仅保留原始值供上层判断

  return result;
}