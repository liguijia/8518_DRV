#ifndef MT6709_H
#define MT6709_H

#include "main.h" // 依赖 CubeMX 生成的 HAL 头文件
#include <stdint.h>

/* ========== 错误代码定义 ========== */
#define MT6709_OK 0       // 操作成功
#define MT6709_ERROR -1   // 通用错误
#define MT6709_TIMEOUT -2 // SPI 超时错误
#define MT6709_CRC_ERR -3 // CRC 校验错误
#define MT6709_CAL_ERR -4 // 自校准失败

/* ========== 旋转方向定义（手册表10） ========== */
#define MT6709_CW 1  // 顺时针（A信号领先B信号）
#define MT6709_CCW 0 // 逆时针（B信号领先A信号）

/* ========== MT6709 关键寄存器地址（手册7.4.3章节） ========== */
#define MT6709_REG_ANGLE_01 0x01 // 角度高位寄存器（ANGLE[16:1]）
#define MT6709_REG_ANGLE_02                                                    \
  0x02 // 角度低位+温度寄存器（ANGLE[0] + TS_Code[10:0]）
#define MT6709_REG_ROT_DIR 0x03 // 旋转方向寄存器（ROT_DIR[0]）
#define MT6709_REG_ZERO_POS 0x04 // 零点位置寄存器（ZERO_POS[11:0]，12位）
#define MT6709_REG_CAL_STATE 0x05 // 校准状态寄存器（CAL_STATE[1:0]，手册表23）
#define MT6709_REG_DATA_FLAG 0x06 // 数据刷新标志寄存器（DATA_FLAG[0]）
#define MT6709_REG_WRDIS 0x07 // 写保护寄存器（WRDIS[0]，1=锁定，0=解锁）

/* ========== 超时时间配置（ms，可根据需求调整） ========== */
#ifndef MT6709_SPI_TIMEOUT_MS
#define MT6709_SPI_TIMEOUT_MS 20
#endif

/* ========== MT6709 设备句柄结构体 ========== */
typedef struct {
  SPI_HandleTypeDef *hspi; // SPI 句柄（CubeMX 已初始化的 SPI3）
  GPIO_TypeDef *cs_port;   // 片选端口（CubeMX 已配置，如 GPIOA）
  uint16_t cs_pin; // 片选引脚（CubeMX 已配置，如 GPIO_PIN_4）

  uint16_t angle_raw; // 17位原始角度值（0~131071，手册6.1章节）
  float angle_deg; // 转换后角度（0~360°，手册公式：θ=ANGLE/2¹⁷×360）
  int16_t temp_c; // 芯片温度（℃，手册7.4.3温度计算公式）
  uint8_t cal_state; // 校准状态（0=未校准，1=校准中，2=校准失败，3=校准成功）
} MT6709_HandleTypeDef;

/* ========== 外部接口函数声明 ========== */

/**
 * @brief 初始化 MT6709（绑定已配置的 SPI 和片选，验证通信）
 * @param hdev: MT6709 设备句柄
 * @param hspi: CubeMX 初始化完成的 SPI3 句柄
 * @param cs_port: CubeMX 配置的片选端口（如 GPIOA）
 * @param cs_pin: CubeMX 配置的片选引脚（如 GPIO_PIN_4）
 * @param zero_offset:
 * 零点偏移值（12位，0~4095对应0~360°；0xFFFF=不修改默认零点）
 * @param direction_cw: 旋转方向（MT6709_CW/MT6709_CCW）
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_Init(MT6709_HandleTypeDef *hdev, SPI_HandleTypeDef *hspi,
                    GPIO_TypeDef *cs_port, uint16_t cs_pin,
                    uint16_t zero_offset, uint8_t direction_cw);

/**
 * @brief 读取 17 位原始角度值
 * @param hdev: MT6709 设备句柄
 * @param angle_raw: 输出原始角度（0~131071）
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_ReadRaw(MT6709_HandleTypeDef *hdev, uint16_t *angle_raw);

/**
 * @brief 读取角度（0~360°）和芯片温度
 * @param hdev: MT6709 设备句柄
 * @param angle_deg: 输出角度（0~360°，可为 NULL）
 * @param temp_c: 输出温度（℃，可为 NULL）
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_ReadAngle(MT6709_HandleTypeDef *hdev, float *angle_deg,
                         int16_t *temp_c);

/**
 * @brief 读取 MT6709 寄存器（8位，手册7.4.2 SPI协议）
 * @param hdev: MT6709 设备句柄
 * @param reg_addr: 寄存器地址（0x00~0xFF）
 * @param reg_value: 输出寄存器值
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_ReadRegister(MT6709_HandleTypeDef *hdev, uint8_t reg_addr,
                            uint8_t *reg_value);

/**
 * @brief 写入 MT6709 寄存器（8位，需先解锁写保护）
 * @param hdev: MT6709 设备句柄
 * @param reg_addr: 寄存器地址（0x00~0xFF）
 * @param reg_value: 要写入的值
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_WriteRegister(MT6709_HandleTypeDef *hdev, uint8_t reg_addr,
                             uint8_t reg_value);

/**
 * @brief 设置 MT6709 零点位置（手册7.1章节，写入 ZERO_POS 寄存器）
 * @param hdev: MT6709 设备句柄
 * @param zero_deg: 目标零点角度（0~360°）
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_SetZeroPosition(MT6709_HandleTypeDef *hdev, float zero_deg);

/**
 * @brief 设置 MT6709 旋转方向（手册表10，ROT_DIR 寄存器）
 * @param hdev: MT6709 设备句柄
 * @param direction_cw: 旋转方向（MT6709_CW/MT6709_CCW）
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_SetRotationDirection(MT6709_HandleTypeDef *hdev,
                                    uint8_t direction_cw);

/**
 * @brief 锁定寄存器（写保护，防止误修改，手册7.4.2）
 * @param hdev: MT6709 设备句柄
 * @retval MT6709_OK/MT6709_ERROR/MT6709_TIMEOUT
 */
uint8_t MT6709_LockRegisters(MT6709_HandleTypeDef *hdev);

/**
 * @brief 执行 MT6709 自校准（手册8.1章节，简洁自校准，需磁环转动≥16周期）
 * @param hdev: MT6709 设备句柄
 * @param cal_enb_port: CubeMX 配置的 CAL_ENB 引脚端口（如 GPIOB）
 * @param cal_enb_pin: CubeMX 配置的 CAL_ENB 引脚（低电平有效）
 * @retval MT6709_OK/MT6709_CAL_ERR/MT6709_ERROR
 */
uint8_t MT6709_StartCalibration(MT6709_HandleTypeDef *hdev,
                                GPIO_TypeDef *cal_enb_port,
                                uint16_t cal_enb_pin);

#endif /* MT6709_H */