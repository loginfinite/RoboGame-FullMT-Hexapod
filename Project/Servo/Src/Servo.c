//
// Created by 35802 on 2022/7/17.
//
#include <stdarg.h>
#include "Servo.h"
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位

UART_HandleTypeDef * ServoHuart;
uint8_t LobotTxBuf[128];  //发送缓存
uint8_t LobotRxBuf[16];
uint16_t batteryVolt;



void Uart_Init(UART_HandleTypeDef *huart){
    ServoHuart = huart;
}
/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Time:转动时间
                    舵机ID取值:0<=舵机ID<=31,Time取值: Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    if (servoID > 31 || !(Time > 0)) {  //舵机ID不能打于31,可根据对应控制板修改
        return;
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;    //填充帧头
    LobotTxBuf[2] = 8;
    LobotTxBuf[3] = CMD_SERVO_MOVE;           //数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
    LobotTxBuf[4] = 1;                        //要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);       //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);      //取得时间的高八位
    LobotTxBuf[7] = servoID;                  //舵机ID
    LobotTxBuf[8] = GET_LOW_BYTE(Position);   //取得目标位置的低八位
    LobotTxBuf[9] = GET_HIGH_BYTE(Position);  //取得目标位置的高八位
    HAL_UART_Transmit(ServoHuart,LobotTxBuf,10, HAL_MAX_DELAY);
}

/*********************************************************************************
 * Function:  moveServosByArray
 * Description： 控制多个舵机转动
 * Parameters:   servos[]:舵机结体数组，Num:舵机个数,Time:转动时间
                    0 < Num <= 32,Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t index = 7;
    uint8_t i = 0;

    if (Num < 1 || Num > 32 || !(Time > 0)) {
        return;                                          //舵机数不能为零和大与32，时间不能为零
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                       //数据长度 = 要控制舵机数*3+5
    LobotTxBuf[3] = CMD_SERVO_MOVE;                    //填充舵机移动指令
    LobotTxBuf[4] = Num;                               //要控制的舵机个数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);                //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);               //取得时间的高八位

    for (i = 0; i < Num; i++) {                        //循环填充舵机ID和对应目标位置
        LobotTxBuf[index++] = servos[i].ID;              //填充舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//填充目标位置高八位
    }
    HAL_UART_Transmit(ServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}

/*********************************************************************************
 * Function:  moveServos
 * Description： 控制多个舵机转动
 * Parameters:   Num:舵机个数,Time:转动时间,...:舵机ID,转动角，舵机ID,转动角度 如此类推
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t index = 7;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;  //

    va_start(arg_ptr, Time); //取得可变参数首地址
    if (Num < 1 || Num > 32) {
        return;               //舵机数不能为零和大与32，时间不能小于0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num * 3 + 5;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_MOVE;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数
    LobotTxBuf[5] = GET_LOW_BYTE(Time);         //取得时间的低八位
    LobotTxBuf[6] = GET_HIGH_BYTE(Time);        //取得时间的高八位

    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
        temp = va_arg(arg_ptr, int);  //可变参数中取得对应目标位置
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); //填充目标位置低八位
        LobotTxBuf[index++] = GET_HIGH_BYTE(temp);//填充目标位置高八位
    }

    va_end(arg_ptr);  //置空arg_ptr
    //发送
    HAL_UART_Transmit(ServoHuart,LobotTxBuf,LobotTxBuf[2] + 2, HAL_MAX_DELAY);
}


void unloadServos(uint8_t Num,uint16_t Time,...){
    uint8_t index = 5;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;
    va_start(arg_ptr, Time); //取得可变参数首地址
    if (Num < 1 || Num > 32) {
        return;               //舵机数不能为零和大与32，时间不能小于0
    }
    LobotTxBuf[0] = LobotTxBuf[1] = FRAME_HEADER;      //填充帧头
    LobotTxBuf[2] = Num + 3;                //数据长度 = 要控制舵机数 * 3 + 5
    LobotTxBuf[3] = CMD_SERVO_UNLOAD;             //舵机移动指令
    LobotTxBuf[4] = Num;                        //要控制舵机数
    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
    }
    va_end(arg_ptr);  //置空arg_ptr
    HAL_UART_Transmit(ServoHuart,LobotTxBuf, LobotTxBuf[2] + 2,HAL_MAX_DELAY);    //发送
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description： 发送获取电池电压命令
 * Return:       无返回
 * Others:
 **********************************************************************************/
void getBatteryVoltage(void)
{
//	uint16_t Voltage = 0;
    LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = 2;             //数据长度，数据帧除帧头部分数据字节数，此命令固定为2
    LobotTxBuf[3] = CMD_GET_BATTERY_VOLTAGE;  //填充获取电池电压命令
    HAL_UART_Transmit_IT(ServoHuart,LobotTxBuf,4);
}

void getServoAngle(uint8_t Num,uint16_t Time,...){
    uint8_t index = 5;
    uint8_t i = 0;
    uint16_t temp;
    va_list arg_ptr;  //
    LobotTxBuf[0] = FRAME_HEADER;  //填充帧头
    LobotTxBuf[1] = FRAME_HEADER;
    LobotTxBuf[2] = Num+3;
    LobotTxBuf[3] = CMD_GET_SERVO_ANGLE;
    LobotTxBuf[4] = Num;
    va_start(arg_ptr, Time); //取得可变参数首地址
    for (i = 0; i < Num; i++) {//从可变参数中取得并循环填充舵机ID和对应目标位置
        temp = va_arg(arg_ptr, int);//可参数中取得舵机ID
        LobotTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
    }
    va_end(arg_ptr);  //置空arg_ptr
    HAL_UART_Transmit_IT(ServoHuart,LobotTxBuf, LobotTxBuf[2] + 2);    //发送
}

void convertAngle(uint8_t Num,double * convertAngleBuf, uint16_t * angleBuf){
    int i=0;
    double base = 500;
    for(;i<Num;i+=3){
        if(i==12){
         convertAngleBuf[i] = ((((double)(angleBuf[i]-440.0))/base)*90.0);
        }
        else
         convertAngleBuf[i] = ((((double)(angleBuf[i]-500.0))/base)*90.0);
    }
    for(i=1;i<Num+1;i+=3){
        if(i==7){
            double base_for_id_8 = 383.0;
            convertAngleBuf[i] = ((1-((((double )angleBuf[i])-240.0)/base_for_id_8))*90.0);
        }else{
            double base_for_mid = 370.0;
            convertAngleBuf[i] = ((1-((((double )angleBuf[i])-200.0)/base_for_mid))*90.0);
        }
    }
    for(i=2;i<Num+2;i+=3){
        switch(i){
            case 2:convertAngleBuf[i] = ((((((double )angleBuf[i])-120.0)/360))*90.0);break;
            case 5:convertAngleBuf[i] = ((((((double )angleBuf[i])-192.0)/370))*90.0);break;
            case 8:convertAngleBuf[i] = ((((((double )angleBuf[i])-540.0)/360))*90.0);break;
            case 11:convertAngleBuf[i] = ((((((double )angleBuf[i])-70.0)/360))*90.0);break;
            case 14:convertAngleBuf[i] = ((((((double )angleBuf[i])-181.0)/369))*90.0);break;
            case 17:convertAngleBuf[i] = ((((((double )angleBuf[i])-680.0)/360))*90.0);break;
            default:;
        }
    }
}

void deConvertAngle(uint8_t Num,uint16_t * angleBuf, double * convertAngleBuf){
    int i=0;
    double base = 500;
    double nineT = 90;
    for(;i<Num;i+=3){
        if(i==12){
            angleBuf[i] = (uint16_t)((convertAngleBuf[i]/nineT)*base+440.0);
        }
        else
            angleBuf[i] = (uint16_t)((convertAngleBuf[i]/nineT)*base+500.0);
    }
    for(i=1;i<Num+1;i+=3){
        if(i==7){
            double base_for_id_8 = 383.0;
            angleBuf[i] = (uint16_t)(((1.0-(convertAngleBuf[i]/90.0))*base_for_id_8)+240.0);
            //convertAngleBuf[i] = (int)((1-((((double )angleBuf[i])-240.0)/base_for_id_8))*90);
        }else{
            double base_for_mid = 370.0;
            angleBuf[i] = (uint16_t)(((1.0-(convertAngleBuf[i]/nineT))*base_for_mid)+200.0);
            //convertAngleBuf[i] = (int)((1-((((double )angleBuf[i])-200.0)/base_for_mid))*90);
        }
    }
    for(i=2;i<Num+2;i+=3){
        switch(i){
            case 2://convertAngleBuf[i] = (int)((((((double )angleBuf[i])-120.0)/360))*90);
            angleBuf[i] = (uint16_t)((convertAngleBuf[i]/90.0)*360.0+120.0);
            break;
            case 5://convertAngleBuf[i] = (int)((((((double )angleBuf[i])-140.0)/360))*90);
            angleBuf[i] = (uint16_t)((convertAngleBuf[i]/90.0)*370.0+192.0);
            break;
            case 8://convertAngleBuf[i] = (int)((((((double )angleBuf[i])-540.0)/360))*90);
            angleBuf[i] = (uint16_t)((convertAngleBuf[i]/90.0)*360.0+540.0);
            break;
            case 11://convertAngleBuf[i] = (int)((((((double )angleBuf[i])-70.0)/360))*90);
                angleBuf[i] = (uint16_t)((convertAngleBuf[i]/90.0)*360.0+70.0);
            break;
            case 14://convertAngleBuf[i] = (int)((((((double )angleBuf[i])-181.0)/369))*90);
                angleBuf[i] = (uint16_t)((convertAngleBuf[i]/90.0)*369.0+181.0);
            break;
            case 17://convertAngleBuf[i] = (int)((((((double )angleBuf[i])-500.0)/360))*90);
                angleBuf[i] = (uint16_t)((convertAngleBuf[i]/90.0)*360.0+680.0);
            break;
            default:;
        }
    }
}