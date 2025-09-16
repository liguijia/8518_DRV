// HAL & C
#include "fdcan.h"
#include "main.h"
#include "string.h"
// USER
#include "bsp_can.h"
void BSP_FDCAN_SetRes(CAN_RES_STATUS status) {
  if (status == CAN_RES_ON) {
    HAL_GPIO_WritePin(CAN_RES_GPIO_Port, CAN_RES_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(CAN_RES_GPIO_Port, CAN_RES_Pin, GPIO_PIN_RESET);
  }
}

/* ---------- FDCAN1 配置 ---------- */
FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;

FDCAN_TxFrame_TypeDef Can1TxFrame = {
    .hcan = &hfdcan1,
    .Header.IdType = FDCAN_STANDARD_ID,
    .Header.TxFrameType = FDCAN_DATA_FRAME,
    .Header.DataLength = 8,
    .Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .Header.BitRateSwitch = FDCAN_BRS_OFF,
    .Header.FDFormat = FDCAN_CLASSIC_CAN,
    .Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .Header.MessageMarker = 0,
};

static FDCAN_TxHeaderTypeDef Can1_Txheader = {
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_8,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
};

/* ---------- FDCAN1 初始化 ---------- */
void BSP_FDCAN_Init(void) {

  // 开启120R CAN终端电阻
  BSP_FDCAN_SetRes(CAN_RES_ON);

  // 配置can1
  FDCAN_FilterTypeDef FDCAN1_FilterConfig;

  FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x00000000;
  FDCAN1_FilterConfig.FilterID2 = 0x00000000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

/* ---------- FDCAN1 发送函数 ---------- */
void BSP_FDCAN_SendMsg(uint16_t id, uint8_t *tx_data) {
  Can1_Txheader.Identifier = id;
  HAL_FDCAN_AddMessageToTxFifoQ(Can1TxFrame.hcan, &Can1_Txheader, tx_data);
}
/* ---------- FDCAN1 接收中断及回调 ---------- */
static void FDCAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t Data[8]) {
  if ((*StdId) >= 0x201 && (*StdId) <= 0x208) {
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan == &hfdcan1) {
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header,
                           FDCAN1_RxFrame.Data);
    FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier,
                            FDCAN1_RxFrame.Data);
  }
}