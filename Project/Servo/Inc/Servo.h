//
// Created by 35802 on 2022/7/17.
//

#ifndef UARTTEST_SERVO_H
#define UARTTEST_SERVO_H
#include "stm32f1xx_hal.h"


#define FRAME_HEADER 0x55             //帧头
#define CMD_SERVO_MOVE 0x03           //舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06     //运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07    //停止动作组指令
#define CMD_ACTION_GROUP_SPEED 0x0B   //设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //获取电池电压指令
#define CMD_SERVO_UNLOAD 0x14 //断掉舵机电力
#define CMD_GET_SERVO_ANGLE 0x15
extern int isUartRxCompleted;
extern uint8_t LobotRxBuf[16];
extern uint16_t batteryVolt;
extern void receiveHandle(void);
extern UART_HandleTypeDef *ServoHuart;
extern uint16_t AngleBuf[18];
extern double ConVertAngleBuf[18];

typedef struct _lobot_servo_ {  //舵机ID,舵机目标位置
    uint8_t ID;
    uint16_t Position;
} LobotServo;

void unloadServos(uint8_t Num,uint16_t Time,...);
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void getBatteryVoltage(void);
void Uart_Init(UART_HandleTypeDef *huart);
void getServoAngle(uint8_t Num,uint16_t Time,...);
void convertAngle(uint8_t Num,double * AngleBuf, uint16_t * ConvertAngleBuf);
void deConvertAngle(uint8_t Num,uint16_t * AngleBuf, double * ConvertAngleBuf);
#endif //UARTTEST_SERVO_H
