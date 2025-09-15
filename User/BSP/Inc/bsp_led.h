#ifndef BSP_LED_H
#define BSP_LED_H

#include "main.h"

typedef enum {
  LED_OFF = 0,
  LED_ON,
  LED_BREATH,
  LED_FLASH_FAST,
  LED_FLASH_SLOW
} LED_MODE;

extern void BSP_BreathLED_Init(void);
extern void BSP_LED_Status(LED_MODE mode);

#endif /* __BSP_LED_H__ */