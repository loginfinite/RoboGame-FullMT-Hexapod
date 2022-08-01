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
extern UART_HandleTypeDef BLUETOOH_UART;
extern UART_HandleTypeDef SERVO_UART;
/* USER CODE BEGIN EV */
char BLUETOOH_RX_BUF[400];
char SERVO_RX_BUF[400];
int isUartRxCompleted;
int Command_Flag=0;
int CMD_BUF[4];
extern HexaPod hexRobo;
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
  HAL_UART_IRQHandler(&BLUETOOH_UART);
  /* USER CODE BEGIN USART2_IRQn 1 */
  if (__HAL_UART_GET_FLAG(&BLUETOOH_UART, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&BLUETOOH_UART, UART_IT_IDLE)){// 确认产生了串口空闲中断{
      __HAL_UART_CLEAR_IDLEFLAG(&BLUETOOH_UART); // 清除空闲中断标志
      HAL_UART_AbortReceive_IT(&BLUETOOH_UART); // 终止中断式接
      int bluetoohrev =0;

      /***数据处理的部***/
      bluetoohrev = BLUETOOH_DATA_PROCESSER(BLUETOOH_RX_BUF);
      char * revMesg = "command receive\n";
      HAL_UART_Transmit_IT(&BLUETOOH_UART,revMesg,17);

      HAL_UART_Receive_IT(&BLUETOOH_UART,BLUETOOH_RX_BUF, 100); // �??启一次新的中断式接收
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
        HAL_UART_AbortReceive_IT(&SERVO_UART); // 终止中断式接�??
        SERVO_DATA_PROCESSER(SERVO_RX_BUF);
        HAL_UART_Receive_IT(&SERVO_UART,SERVO_RX_BUF, 100); // �??启一次新的中断式接收
    }

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
int BLUETOOH_DATA_PROCESSER(char* dataBuf){
//    if(dataBuf[0]!=FRAME_HEAD || dataBuf[1] != FRAME_HEAD){
//        return -1;
//    }
    int Num = dataBuf[1];
    int CMDType = dataBuf[0];
    char c;
    switch (CMDType) {
        case BLUETOOH_CMD_GET_ANGLE:
            return 1;
        case '2':
            Command_Flag = 2;
            return 2;
            break;
        case '3':
            Command_Flag = 3;
            return 3;
            ;break;
        case '4':
            Command_Flag = 4;
            return 4;
            break;
        case '5':
            Command_Flag = 5;
            return 5;
            break;
        case 'u':
            Command_Flag = 6;
            sscanf(&dataBuf[0],"%c %d",&c,&CMD_BUF[0]);
            return 6;
            break;
        case 's':
            Command_Flag = 7;
            sscanf(&dataBuf[0],"%c %d %d %d %d",&c,&CMD_BUF[0],&CMD_BUF[1],&CMD_BUF[2],&CMD_BUF[3]);
            return 7;
            break;
        case 'm':
            Command_Flag = 8;
            sscanf(&dataBuf[0],"%c %d",&c,&CMD_BUF[0]);
            return 8;
            break;
        case 'e':
            Command_Flag = 9;
            sscanf(&dataBuf[0],"%c %d",&c,&CMD_BUF[0]);
            return 9;
            break;
        case 'a':
            Command_Flag = 10;
            return 10;
            break;
        default:
            Command_Flag = 1;
            return -2;
    }
}

int SERVO_DATA_PROCESSER(char* dataBuf){
    isUartRxCompleted = 0;
    int servoNum = 0;
    int i=0;
    int id;
    int angle;
    if(dataBuf[0]==FRAME_HEADER&&dataBuf[1]==FRAME_HEADER){
        switch (dataBuf[3]){
            case CMD_GET_SERVO_ANGLE:
                servoNum = dataBuf[4];
                for(i=0;i<servoNum;++i){
                    id = dataBuf[5+i*3];
                    angle = (
                            ((uint16_t)dataBuf[6+i*3]) | (((uint16_t)dataBuf[7+i*3])<<8)
                            );
                    AngleBuf[id-1] = angle;
                }
                ;break;
            default:
                isUartRxCompleted = 1;
                return -1;
        }
        isUartRxCompleted = 1;
        return 1;
    }else{
        isUartRxCompleted = 1;
        return -1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &SERVO_UART){

    }else if(huart == &BLUETOOH_UART){

    }else{

    }
}
/* USER CODE END 1 */
