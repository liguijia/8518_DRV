#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "main.h"
typedef struct {
  float a;
  float b;
  float c;
} FOC_PWM_t;

extern void BSP_PWM_Init(void);
extern void BSP_PWM_Set_Duty(const FOC_PWM_t *duty);

#endif /* __BSP_PWM_H__ */