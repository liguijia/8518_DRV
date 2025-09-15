#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

#define hcan_t FDCAN_HandleTypeDef

typedef struct {
  FDCAN_HandleTypeDef *hcan;
  FDCAN_TxHeaderTypeDef Header;
  uint8_t Data[8];
} FDCAN_TxFrame_TypeDef;

typedef struct {
  FDCAN_HandleTypeDef *hcan;
  FDCAN_RxHeaderTypeDef Header;
  uint8_t Data[8];
} FDCAN_RxFrame_TypeDef;

typedef enum { CAN_RES_OFF = 0, CAN_RES_ON } CAN_RES_STATUS;

extern FDCAN_TxFrame_TypeDef Can1TxFrame;

extern uint8_t bit8tofloat[4];

extern void BSP_FDCAN_Init(void);
extern void BSP_FDCAN_SendMsg(uint16_t id, uint8_t *tx_data);
extern void BSP_FDCAN_SetRes(CAN_RES_STATUS status);

#endif /* __BSP_CAN_H__ */
