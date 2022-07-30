/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "Servo.h"
#include "HexaPod.h"
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern int Command_Flag;
extern int CMD_BUF[4];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef BLUETOOH_UART;
UART_HandleTypeDef SERVO_UART;

/* USER CODE BEGIN PV */
extern char BLUETOOH_RX_BUF[400];
extern char SERVO_RX_BUF[400];
HexaPod hexRobo;
extern uint16_t AngleBuf[18];
uint16_t DefaultAngle[18] = {479,435,240,466,496,324,488,574,722,501,429,177,434,475,374,496,456,616};
double DefaultTheta[18] = {0,15,30,0,15,30,0,15,30,0,15,30,0,15,30,0,15,30};
double LeftUpTheta[18] = {0,-10,125,0,-10,125,0,-10,125,0,15,30,0,15,30,0,15,30};
double RightUpTheta[18] = {0,15,30,0,15,30,0,15,30,0,-10,125,0,-10,125,0,-10,125};
double LeftUpTwistTheta[18] = {10,-10,125,10,-10,125,10,-10,125,0,15,30,0,15,30,0,15,30};
double LeftDownTwistTheta[18] = {10,15,30,10,15,30,10,15,30,0,15,30,0,15,30,0,15,30};
double RightUpTwistTheta[18] =  {0,15,30,0,15,30,0,15,30,10,-10,125,10,-10,125,10,-10,125};
double RightDownTwistTheta[18] = {0,15,30,0,15,30,0,15,30,10,15,30,10,15,30,10,15,30};

double Twist1[18] = {7,8,100,7,8,100,7,8,100,0,15,30,0,15,30,0,15,30};
double Twist2[18] = {7,15,30,7,15,30,7,15,30,0,15,30,0,15,30,0,15,30};
double Twist3[18] = {7,15,30,7,15,30,7,15,30,7,8,100,7,8,100,7,8,100};
double Twist4[18] = {0,15,30,0,15,30,0,15,30,7,8,100,7,8,100,7,8,100};
double Twist5[18] = {0,15,30,0,15,30,0,15,30,7,15,30,7,15,30,7,15,30};
double Twist6[18] = {7,8,100,7,8,100,7,8,100,7,15,30,7,15,30,7,15,30};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void sendAngleData(void );
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void sendAngleData(void ){

    int i=0;
    char str[]="Leg:XX||JointA:XXX,JointB:XXX,JointC:XXX\n";
    char sendingBuf[60];
    for(i=0;i<60;++i){
        sendingBuf[i]='\0';
    }
    for(i=0;i<18;i+=3){

        double BUF_NUM_1,BUF_NUM_2,BUF_NUM_3;
        BUF_NUM_1 = ConVertAngleBuf[i];
        BUF_NUM_2 = ConVertAngleBuf[i+1];
        BUF_NUM_3 = ConVertAngleBuf[i+2];
        sprintf(sendingBuf,"Leg:XX--JointA:%.2f--JointB:%.2f--JointC:%.2f\n\0",BUF_NUM_1,BUF_NUM_2,BUF_NUM_3);
        switch (i) {
            case 0:
                sendingBuf[4] ='L';
                sendingBuf[5] ='F';
                break;
            case 3:
                sendingBuf[4] ='R';
                sendingBuf[5] ='M';
                break;
            case 6:
                sendingBuf[4] ='L';
                sendingBuf[5] ='H';
                break;
            case 9:
                sendingBuf[4] ='R';
                sendingBuf[5] ='F';
                break;
            case 12:
                sendingBuf[4] ='L';
                sendingBuf[5] ='M';
                break;
            case 15:
                sendingBuf[4] ='R';
                sendingBuf[5] ='H';
                break;
            default:;break;
        }
        HAL_UART_Transmit(&BLUETOOH_UART,sendingBuf,60,HAL_MAX_DELAY);
        HAL_Delay(50);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  char* data0 = "stm init...\n\r";
    HAL_UART_Transmit(&BLUETOOH_UART,data0,13,HAL_MAX_DELAY);
    __HAL_UART_CLEAR_IDLEFLAG(&SERVO_UART);//娓呴櫎绌洪棽涓柇鏍囧織
    __HAL_UART_ENABLE_IT(&SERVO_UART, UART_IT_IDLE | UART_IT_RXNE);//�??鍚┖闂蹭腑鏂拰鎺ユ敹涓�?
    HAL_UART_Receive_IT(&SERVO_UART, (uint8_t *)SERVO_RX_BUF, 100);//�??鍚竴娆′腑鏂紡鎺ユ敹
    __HAL_UART_CLEAR_IDLEFLAG(&BLUETOOH_UART);//娓呴櫎绌洪棽涓柇鏍囧織
    __HAL_UART_ENABLE_IT(&BLUETOOH_UART, UART_IT_IDLE | UART_IT_RXNE);//�??鍚┖闂蹭腑鏂拰鎺ユ敹涓�?
    HAL_UART_Receive_IT(&BLUETOOH_UART, (uint8_t *)BLUETOOH_RX_BUF, 100);//�??鍚竴娆′腑鏂紡鎺ユ敹

    Uart_Init(&SERVO_UART);
    HexaPod_Init(&hexRobo);
    //setLegsDefault();
    //setLegsDefault();
  /* USER CODE END 2 */
    /* USER CODE BEGIN WHILE */


    /* Infinite loop */
  while (1)switch (Command_Flag) {

          {
          case 2:
              setAllLegs(AngleBuf,DefaultTheta);
              Command_Flag = 1;
              break;
          case 3:
              lockAllLegs();
              Command_Flag = 1;
              break;
          case 4:
              unlockAllLegs();
              Command_Flag=1;
              break;
          case 5:
              getAllLegsAngle();
              sendAngleData();
              Command_Flag=1;
              break;
          case 6:
              switch (CMD_BUF[0]) {
                  case LeftFront:
                      unlockLeg(&hexRobo, LeftFront);
                      break;
                  case LeftMiddle:
                      unlockLeg(&hexRobo, LeftMiddle);
                      break;
                  case LeftHind:
                      unlockLeg(&hexRobo, LeftHind);
                      break;
                  case RightFront:
                      unlockLeg(&hexRobo, RightFront);
                      break;
                  case RightMiddle:
                      unlockLeg(&hexRobo, RightMiddle);
                      break;
                  case RightHide:
                      unlockLeg(&hexRobo, RightHide);
                  default:break;
              }
              Command_Flag = 1;
              break;
          case 7:
              setLegAngle(CMD_BUF[0],CMD_BUF[1],CMD_BUF[2],CMD_BUF[3]);
              Command_Flag = 1;
              break;
          case 8:
              setAllMidLeg(AngleBuf,CMD_BUF[0]);
              Command_Flag = 1;
              break;
          case 9:
              setAllEndLeg(AngleBuf,CMD_BUF[0]);
              Command_Flag = 1;
              break;
          case 10:
              setAllLegs(AngleBuf,Twist1);
              HAL_Delay(1000);
              setAllLegs(AngleBuf,Twist2);
              HAL_Delay(1000);
              setAllLegs(AngleBuf,Twist3);
              HAL_Delay(1000);
              setAllLegs(AngleBuf,Twist4);
              HAL_Delay(1000);
              setAllLegs(AngleBuf,Twist5);
              HAL_Delay(1000);
              setAllLegs(AngleBuf,Twist6);
              HAL_Delay(1000);
              Command_Flag = 1;
              break;
          default:;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  BLUETOOH_UART.Instance = USART2;
  BLUETOOH_UART.Init.BaudRate = 9600;
  BLUETOOH_UART.Init.WordLength = UART_WORDLENGTH_8B;
  BLUETOOH_UART.Init.StopBits = UART_STOPBITS_1;
  BLUETOOH_UART.Init.Parity = UART_PARITY_NONE;
  BLUETOOH_UART.Init.Mode = UART_MODE_TX_RX;
  BLUETOOH_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  BLUETOOH_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&BLUETOOH_UART) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  SERVO_UART.Instance = USART3;
  SERVO_UART.Init.BaudRate = 9600;
  SERVO_UART.Init.WordLength = UART_WORDLENGTH_8B;
  SERVO_UART.Init.StopBits = UART_STOPBITS_1;
  SERVO_UART.Init.Parity = UART_PARITY_NONE;
  SERVO_UART.Init.Mode = UART_MODE_TX_RX;
  SERVO_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  SERVO_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&SERVO_UART) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
