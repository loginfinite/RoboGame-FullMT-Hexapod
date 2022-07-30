//
// Created by 35802 on 2022/7/17.
//
#ifndef UARTTEST_SERVO_H
#define UARTTEST_SERVO_H
#include "stm32f1xx_hal.h"
#define TOTAL_SERVOS_NUM 30

#define FRAME_HEADER 0x55             //帧头
#define CMD_SERVO_MOVE 0x03           //舵机移动指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //获取电池电压指令
#define CMD_SERVO_UNLOAD 0x14 //断掉舵机电力
#define CMD_GET_SERVO_ANGLE 0x15 //获取舵机角度

extern int isUartRxCompleted;
extern uint8_t LobotRxBuf[32];
extern uint16_t batteryVolt;
extern UART_HandleTypeDef *ServoHuart;
extern uint16_t AngleBuf[TOTAL_SERVOS_NUM];
extern double ConvertAngleBuf[TOTAL_SERVOS_NUM];

void unloadServos(uint8_t Num,uint16_t Time,...);
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void getBatteryVoltage(void);
void Uart_Init(UART_HandleTypeDef *huart);
void getServoAngle(uint8_t Num,uint16_t Time,...);
void convertAngle(double * AngleBuf, uint16_t * ConvertAngleBuf);
void deConvertAngle(double * convertAngleBuf, uint16_t * angleBuf);
#endif //UARTTEST_SERVO_H
