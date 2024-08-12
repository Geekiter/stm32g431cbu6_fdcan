/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name        :  FDCanDrv.h
 * Description      :  CAN driver 
 ******************************************************************************
 * @attention
 *
* COPYRIGHT:    Copyright (c) 2024  xxxx@ti5robot.com

* CREATED BY:   mingfei.tang
* AUTHOR BY :   shirui.zhang
* DATE:         JUL 05th, 2024
*
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#ifndef __FDCanDrv_H
#define __FDCanDrv_H

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "main.h"

#ifdef _cplusplus
extern "C" {
#endif 

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "main.h"
#include "fdcan.h"
#include "stdio.h"
#ifdef _cplusplus
extern "C" {
#endif 

void CAN_ConfigFilter(void);
uint16_t Send_FDCANTx(uint32_t DataLength, uint16_t *pTxData);
void Send_Position( float angle );
void Send_Speed( float speed );
void CAN_Receive(void);
void Send_64byte_data(void);
uint8_t can_dlc2len(uint32_t RxHeader_DataLength);
#ifdef _cplusplus
}
#endif   

#endif    /* __FDCanDrv_H */
