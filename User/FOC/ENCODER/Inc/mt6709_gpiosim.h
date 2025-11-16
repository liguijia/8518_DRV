#ifndef MT6709_GPIOSIM_H
#define MT6709_GPIOSIM_H

#include "stdbool.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef struct {
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *sck_port;
  uint16_t sck_pin;
  GPIO_TypeDef *sdat_port;
  uint16_t sdat_pin;
} MT6709_GPIOConfig_t;
typedef struct {
  uint8_t diag_stat;   // DIAG_STAT: S[15:12]
  uint8_t customer_id; // CUSTOMER_ID: S[11:8]
  struct {
    bool signal_error;       // S[12]: Signal too small/large
    bool speed_or_cal_error; // S[13]: Speed too high / Self-cal fail
    bool eeprom_error;       // S[14]: EEPROM check failed
    bool under_voltage;      // S[15]: Under-voltage warning
  } diag_flags;

  uint8_t crc; // CRC: S[7:0]
} MT6709_SafetyWord_t;
typedef struct {
  float angle_deg;
  float temperature_c;
  MT6709_SafetyWord_t safety_word;
} MT6709_MultiReadResult_t;

void MT6709_GPIO_Init(const MT6709_GPIOConfig_t *config);
uint32_t MT6709_ReadRawAngle(void);
float MT6709_ReadAngleDeg(void);
MT6709_MultiReadResult_t MT6709_ReadMulti(uint8_t reg_addr);

#endif