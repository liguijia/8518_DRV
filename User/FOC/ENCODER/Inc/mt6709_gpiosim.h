#ifndef MT6709_GPIOSIM_H
#define MT6709_GPIOSIM_H

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

void MT6709_GPIO_Init(const MT6709_GPIOConfig_t *config);
uint32_t MT6709_ReadRawAngle(void);
float MT6709_ReadAngleDeg(void);

#endif