/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  FDCanDrv.h
 * Description      :  CAN driver
 ******************************************************************************
 * @attention
 *
* COPYRIGHT:    Copyright (c) 2024  mingfei.tang@ti5robot.com

* CREATED BY:   mingfei.tang
* AUTHOR BY :   shirui.zhang
* DATE:         JUL 05th, 2024

 ******************************************************************************
 */
/* USER CODE END Header */
#include "FDCanDrv.h"
#include "fdcan.h"
#include "usart.h"

#define FACTOR 101 * 65536
#define BUFF_LENGTH 64
uint8_t cnt = 0;
uint8_t brs[] = {'-', 'B'};
uint8_t esi[] = {'-', 'E'};
uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[BUFF_LENGTH]; // receive buffer
uint8_t TxData[BUFF_LENGTH]; // transmit buffer

void CAN_ConfigFilter(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0;
    sFilterConfig.FilterID2 = 0;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Prepare Tx Header */
    TxHeader.Identifier = 1;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN_BRS_ON
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
}

// Can发送
uint16_t Send_FDCANTx(uint32_t DataLength, uint16_t *pTxData)
{
    TxHeader.DataLength = DataLength;
    // return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pTxData);
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pTxData) != HAL_OK)
    {
        printf("FDCAN1 Send Fail\n");
    }
}

// 发送位置模式
void Send_Position(float angle)
{
    uint8_t command_list[5] = {0x1E, 0x00, 0x00, 0x00, 0x00}; // 电机协议1E是位置值

    long int Position = (angle / 360) * FACTOR;

    command_list[1] = Position & 0xFF;
    command_list[2] = (Position >> 8) & 0xFF;
    command_list[3] = (Position >> 16) & 0xFF;
    command_list[4] = (Position >> 24) & 0xFF;

    Send_FDCANTx(FDCAN_DLC_BYTES_5, command_list);
}

void Send_Speed(float speed)
{
    uint8_t command_list[5] = {0x1D, 0x00, 0x00, 0x00, 0x00};
    long int Speed = speed * 101 * 100 / 360;

    command_list[1] = Speed & 0xFF;
    command_list[2] = (Speed >> 8) & 0xFF;
    command_list[3] = (Speed >> 16) & 0xFF;
    command_list[4] = (Speed >> 24) & 0xFF;

    Send_FDCANTx(FDCAN_DLC_BYTES_5, command_list);
}

// 轮询接收
// void CAN_Receive()
// {
//     if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
//     {

//         // 对于从机，接收到数据后，需要回复一个数据包
//         // Send_64byte_data();
//         // 对于主机，发一个短的数据包
//         printf("Length: %d\n", RxHeader.DataLength );
//         printf("FDFormat: %d\n", RxHeader.FDFormat);
//         printf("Data: ");
//         for(uint16_t i = 0; i < RxHeader.DataLength; i++)
//         {
//             printf("%d ", RxData[i]);
//         }

//         printf("\n");
//     }
// }

uint8_t can_dlc2len(uint32_t RxHeader_DataLength)
{
    return dlc2len[RxHeader_DataLength];
}

typedef struct
{
    uint8_t flag;
    uint8_t index;
    FDCAN_TxHeaderTypeDef TxHeader[16];
    uint8_t TxData[64 * 16];
} FDCAN_SendFailTypeDef;

FDCAN_SendFailTypeDef fdcan1_send_fail = {0};

void fdcan1_transmit(uint32_t can_id, uint32_t DataLength, uint8_t *tx_data)
{
    TxHeader.Identifier = can_id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    if (can_id < 0x800)
    { // exactly not right
        TxHeader.IdType = FDCAN_STANDARD_ID;
    }
    // TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = DataLength;
    // TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    // TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    // TxHeader.FDFormat = FDCAN_FD_CAN;
    // TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    // TxHeader.MessageMarker = 0; // Tx Event FIFO Use
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, tx_data) != HAL_OK)
    {
        printf("FDCAN1 Send Fail\n");
    }
}
void Send_64byte_data()
{
    // uint8_t command_list[12] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12};

    // Send_FDCANTx(FDCAN_DLC_BYTES_12, command_list);
    for (uint8_t i = 0; i < 64; i++)
    {
        TxData[i] = i;
    }
    fdcan1_transmit(0x10823456, FDCAN_DLC_BYTES_64, TxData);
    fdcan1_transmit(0x108, FDCAN_DLC_BYTES_64, TxData);

}
void CAN_Receive()
{
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        // printf("0x%8X, %02d, %c, %c:", RxHeader.Identifier,
        //        can_dlc2len(RxHeader.DataLength),
        //        brs[RxHeader.BitRateSwitch >> 20 & 0x1],
        //        esi[RxHeader.ErrorStateIndicator >> 31 & 0x1]);
        for (cnt = 0; cnt < can_dlc2len(RxHeader.DataLength); cnt++)
        {
            printf(" %02X", RxData[cnt]);
        }
        printf("\n\r");
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {

        // memset(&RxHeader, 0, sizeof(FDCAN_RxHeaderTypeDef));	//if use, lose frame

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
        if (hfdcan->Instance == FDCAN1)
        {
            printf("fdcan1, ");
        }
        else
        {
        }
        printf("0x%8X, %02d, %c, %c:", RxHeader.Identifier,
               can_dlc2len(RxHeader.DataLength),
               brs[RxHeader.BitRateSwitch >> 20 & 0x1],
               esi[RxHeader.ErrorStateIndicator >> 31 & 0x1]);
        for (cnt = 0; cnt < can_dlc2len(RxHeader.DataLength); cnt++)
        {
            printf(" %02X", RxData[cnt]);
        }
        printf("\n\r");
    }
}

/* End of this file */
