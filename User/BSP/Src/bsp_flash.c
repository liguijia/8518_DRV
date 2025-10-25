#include "bsp_flash.h"

// 内部函数声明
static uint32_t _get_page_num(uint32_t addr);
static bool _is_addr_valid(uint32_t addr, uint16_t size);

/**
 * @brief 初始化Flash（配置等待周期）
 * @note  假设系统时钟80MHz，Vcore=2.7V~3.6V，需2个等待周期
 */
void BSP_Flash_Init(void) { __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2); }

/**
 * @brief 擦除指定页
 * @param page_num：页号（需在FLASH_USER_START_PAGE ~
 * FLASH_USER_END_PAGE范围内）
 * @return 操作状态
 */
Flash_Status BSP_Flash_ErasePage(uint32_t page_num) {
  // 检查页号合法性
  if (page_num < FLASH_USER_START_PAGE || page_num > FLASH_USER_END_PAGE) {
    return FLASH_ERR_ADDR;
  }

  FLASH_EraseInitTypeDef erase_init = {0};
  uint32_t page_error = 0;
  Flash_Status status = FLASH_OK;

  // 解锁Flash
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return FLASH_ERR_LOCK;
  }

  // 清除所有错误标志
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  // 配置擦除参数
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.Banks = FLASH_BANK_1;
  erase_init.Page = page_num;
  erase_init.NbPages = 1;

  // 执行擦除
  if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
    status = FLASH_ERR_ERASE;
  }

  // 锁定Flash
  HAL_FLASH_Lock();
  return status;
}

/**
 * @brief 写入任意类型数据（带校验）
 * @param addr：写入地址（必须在用户区内且8字节对齐）
 * @param data：数据指针
 * @param data_size：数据大小（字节）
 * @return 操作状态
 */
Flash_Status BSP_Flash_Write(uint32_t addr, const void *data,
                             uint16_t data_size) {
  // 检查地址合法性、缓冲区有效性
  if (!_is_addr_valid(addr, data_size) || data == NULL || data_size == 0) {
    return FLASH_ERR_ADDR;
  }

  // 必须 8 字节对齐（STM32G4 Flash 编程要求）
  if ((addr & 0x07) != 0) {
    return FLASH_ERR_ADDR;
  }

  // 解锁 Flash
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return FLASH_ERR_LOCK;
  }

  // 清除所有错误标志
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  uint32_t write_addr = addr;
  const uint8_t *data_ptr = (const uint8_t *)data;
  Flash_Status status = FLASH_OK;

  // 按数据块写入（每块最多4字节数据 + 1字节CRC8，其余3字节保留）
  while (data_size > 0) {
    uint8_t block_size = (data_size >= 4) ? 4 : data_size;
    uint64_t double_word = 0; // 8字节 Double Word

    // ---- 构造 8 字节数据块 ----
    // 前 4 字节：原始数据
    memcpy(&double_word, data_ptr, block_size);

    // 第 5 字节（偏移4）：CRC8 校验值
    ((uint8_t *)&double_word)[4] = BSP_Flash_CalcCRC(data_ptr, block_size);
    // 后 3 字节保留为 0（已初始化为 0）

    // ---- 写入 Flash（8 字节原子写入）----
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr,
                          double_word) != HAL_OK) {
      status = FLASH_ERR_WRITE;
      break;
    }

    // ---- 更新指针 ----
    data_ptr += block_size;
    data_size -= block_size;
    write_addr += 8; // 移动到下一个 8 字节块
  }

  // 锁定 Flash
  HAL_FLASH_Lock();
  return status;
}

/**
 * @brief 读取任意类型数据（带校验）
 * @param addr：读取地址（必须在用户区内且8字节对齐）
 * @param data：接收数据的缓冲区
 * @param data_size：数据大小（字节）
 * @return 操作状态
 */
Flash_Status BSP_Flash_Read(uint32_t addr, void *data, uint16_t data_size) {
  // 检查地址合法性和缓冲区有效性
  if (!_is_addr_valid(addr, data_size) || data == NULL || data_size == 0) {
    return FLASH_ERR_ADDR;
  }

  // 必须 8 字节对齐（与写入一致）
  if ((addr & 0x07) != 0) {
    return FLASH_ERR_ADDR;
  }

  uint32_t read_addr = addr;
  uint8_t *data_ptr = (uint8_t *)data;

  while (data_size > 0) {
    uint8_t block_size = (data_size >= 4) ? 4 : data_size;

    // 读取存储的数据（4字节）和 CRC8（第5字节）
    uint32_t stored_data = *(volatile uint32_t *)(read_addr);
    uint8_t stored_crc = ((uint8_t *)read_addr)[4]; // 读取第5字节

    // 将 stored_data 拆解为字节数组（小端）
    uint8_t data_bytes[4];
    data_bytes[0] = (uint8_t)(stored_data >> 0);
    data_bytes[1] = (uint8_t)(stored_data >> 8);
    data_bytes[2] = (uint8_t)(stored_data >> 16);
    data_bytes[3] = (uint8_t)(stored_data >> 24);

    // 计算实际数据的 CRC8
    flash_crc_t calc_crc = BSP_Flash_CalcCRC(data_bytes, block_size);

    // 校验 CRC
    if (calc_crc != stored_crc) {
      return FLASH_ERR_CRC;
    }

    // 复制有效数据到输出缓冲区
    memcpy(data_ptr, data_bytes, block_size);

    // 移动指针
    data_ptr += block_size;
    data_size -= block_size;
    read_addr += 8; // 跳到下一个 8 字节块
  }

  return FLASH_OK;
}

/**
 * @brief 计算CRC8校验值（CRC8-ROHC，多项式0x31）
 * @param data：数据指针
 * @param len：数据长度（字节）
 * @return CRC8结果（8位）
 */
flash_crc_t BSP_Flash_CalcCRC(const void *data, uint16_t len) {
  const uint8_t *ptr = (const uint8_t *)data;
  uint8_t crc = 0xFF; // 初始值

  while (len--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// 内部函数：获取地址对应的页号
static uint32_t _get_page_num(uint32_t addr) {
  return (addr - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE;
}

// 内部函数：检查地址是否在用户区内且不越界
static bool _is_addr_valid(uint32_t addr, uint16_t size) {
  if (size == 0)
    return false;

  // 每 4 字节数据占用 8 字节 Flash 空间（4数据 + 1CRC + 3保留）
  uint32_t num_blocks = (size + 3) / 4; // 向上取整到 4 字节
  uint32_t required_size = num_blocks * 8;

  uint32_t end_addr = addr + required_size - 1;

  if (addr < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR) {
    return false;
  }

  uint32_t page_start = _get_page_num(addr);
  uint32_t page_end = _get_page_num(end_addr);

  return (page_start >= FLASH_USER_START_PAGE &&
          page_end <= FLASH_USER_END_PAGE);
}