#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Flash配置参数（根据实际芯片修改）
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 2048U // STM32G431单页大小：2KB
#endif
#define FLASH_BASE_ADDR 0x08000000U // Flash起始地址
#define FLASH_USER_START_PAGE 60U // 用户数据起始页（需确保不覆盖程序）
#define FLASH_USER_END_PAGE 63U // 用户数据结束页（共4页=8KB）

// 计算用户区地址范围
#define FLASH_USER_START_ADDR                                                  \
  (FLASH_BASE_ADDR + FLASH_USER_START_PAGE * FLASH_PAGE_SIZE)
#define FLASH_USER_END_ADDR                                                    \
  (FLASH_BASE_ADDR + (FLASH_USER_END_PAGE + 1) * FLASH_PAGE_SIZE - 1U)

// 数据校验方式：简单CRC8
typedef uint8_t flash_crc_t;

// 操作状态枚举
typedef enum {
  FLASH_OK = 0x00,
  FLASH_ERR_ADDR = 0x01,  // 地址越界
  FLASH_ERR_LOCK = 0x02,  // 解锁失败
  FLASH_ERR_ERASE = 0x03, // 擦除失败
  FLASH_ERR_WRITE = 0x04, // 写入失败
  FLASH_ERR_CRC = 0x05    // 校验失败
} Flash_Status;

// 初始化Flash（主要配置等待周期）
void BSP_Flash_Init(void);

// 擦除指定页（必须是用户区内的页）
Flash_Status BSP_Flash_ErasePage(uint32_t page_num);

// 写入任意类型数据（自动处理对齐和校验）
Flash_Status BSP_Flash_Write(uint32_t addr, const void *data,
                             uint16_t data_size);

// 读取任意类型数据（带校验）
Flash_Status BSP_Flash_Read(uint32_t addr, void *data, uint16_t data_size);

// 辅助函数：计算CRC8校验值
flash_crc_t BSP_Flash_CalcCRC(const void *data, uint16_t len);

#endif