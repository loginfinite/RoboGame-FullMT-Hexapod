/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "stm32f1xx_it.h"
#include "Servo.h"
#include "HexaPod.h"
#include <stdio.h>
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef BLUETOOTH_UART;
extern UART_HandleTypeDef SERVO_UART;
/* USER CODE BEGIN EV */
char BLUETOOH_RX_BUF[400];
char SERVO_RX_BUF[400];
char CMD_ARG_BUF[100];
HexaPod hexRobo;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&BLUETOOTH_UART);
  /* USER CODE BEGIN USART2_IRQn 1 */
  if (__HAL_UART_GET_FLAG(&BLUETOOTH_UART, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&BLUETOOTH_UART, UART_IT_IDLE)){// 确认产生了串口空闲中断{
      __HAL_UART_CLEAR_IDLEFLAG(&BLUETOOTH_UART); // 清除空闲中断标志
      HAL_UART_AbortReceive_IT(&BLUETOOTH_UART); // 终止中断式接
      int bluetoothRevFlag =0;
      /***数据处理的部***/

      bluetoothRevFlag = BLUETOOH_DATA_PROCESSER(BLUETOOH_RX_BUF);

      if (bluetoothRevFlag) {
          char *revMesg = "Command receive\n";
          HAL_UART_Transmit_IT(&BLUETOOTH_UART, revMesg, 17);
      }
      else {
          char *revMesg = "Unknown command\n";
          HAL_UART_Transmit_IT(&BLUETOOTH_UART,revMesg,17);
      }

      HAL_UART_Receive_IT(&BLUETOOTH_UART,BLUETOOH_RX_BUF, 100); // 开启一次新的中断式接收
}
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&SERVO_UART);
  /* USER CODE BEGIN USART3_IRQn 1 */
    if (__HAL_UART_GET_FLAG(&SERVO_UART, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&SERVO_UART, UART_IT_IDLE)){// 确认产生了串口空闲中断{
        __HAL_UART_CLEAR_IDLEFLAG(&SERVO_UART); // 清除空闲中断标志
        HAL_UART_AbortReceive_IT(&SERVO_UART);
        SERVO_DATA_PROCESSER(SERVO_RX_BUF);
        HAL_UART_Receive_IT(&SERVO_UART,SERVO_RX_BUF, 100); // 启一次新的中断式接收
    }
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
int BLUETOOH_DATA_PROCESSER(char* dataBuf){
    int dataLen,servoID,dataIter;
    uint16_t angleUpper8B,angleLower8B;
    if(dataBuf[0] == FRAME_HEADER && dataBuf[1] == FRAME_HEADER){
        dataLen = dataBuf[2];
        switch (dataBuf[3]) {
            case getAngle:
                hexRobo.Command = getAngle;
                hexRobo.isMsgReceive = 1;
                break;
            case setAngle:
                hexRobo.Command = setAngle;
                char speed = dataBuf[4];
                CMD_ARG_BUF[0] = speed;
                for(dataIter = 5;dataIter < dataLen;dataIter+=3){
                    servoID = dataBuf[dataIter];
                    angleLower8B = dataBuf[dataIter+1];
                    angleUpper8B = dataBuf[dataIter+2];
                    hexRobo.RawAngleData[servoID-1] = ( (angleLower8B) | (angleUpper8B)<<8 );
                }
                hexRobo.isMsgReceive = 1;
                break;
            case unlockLeg:
                hexRobo.Command = unlockLeg;
                CMD_ARG_BUF[0] = dataBuf[4];
                hexRobo.isMsgReceive = 1;
                break;
            default:
                hexRobo.Command = idle;
                return -1;
        }
        return 1;
    }
    return -1;
}


int SERVO_DATA_PROCESSER(char* dataBuf){
    int dataLen,servoID,dataIter;
    uint16_t angleUpper8B,angleLower8B;
    hexRobo.mutexRxFromServo = 1;
    if(dataBuf[0] == FRAME_HEADER && dataBuf[1] == FRAME_HEADER){
        dataLen = dataBuf[2] + 2;//帧头+数据=总长度
        switch (dataBuf[3]) {
            case CMD_GET_SERVO_ANGLE:
                for(dataIter = 5;dataIter < dataLen;dataIter+=3){
                 servoID = dataBuf[dataIter];
                 angleLower8B = dataBuf[dataIter+1];
                 angleUpper8B = dataBuf[dataIter+2];
                 hexRobo.RawAngleData[servoID-1] = ( (angleLower8B) | (angleUpper8B)<<8 );
                }
                break;
            default:break;
        }
    }
    hexRobo.isMsgReceive = 1;
    hexRobo.mutexRxFromServo = 0;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &SERVO_UART){

    }else if(huart == &BLUETOOTH_UART){

    }else{

    }
}
/* USER CODE END 1 */
