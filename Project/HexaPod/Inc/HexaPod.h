//
// Created by 35802 on 2022/7/18.
//

#ifndef UARTTEST_HEXAPOD_H
#define UARTTEST_HEXAPOD_H
#include "Servo.h"
#include "InverseKinematics.h"
#define STRUCTURE_RADIUS 100
#define DEFAULT_HEIGHT 100
#define DEFAULT_LEG_LENGTH 100
#define TOTAL_LEGS 12
typedef enum JointIndex{
    Coxa=0,
    Femur,
    Tibia,
}JointIndex;

typedef enum LegIndex{
    LeftFront=0,
    LeftMiddle,
    LeftHind,
    RightFront,
    RightMiddle,
    RightHind,
    LeftFrontUpper,
    LeftMiddleUpper,
    LeftHindUpper,
    RightFrontUpper,
    RightMiddleUpper,
    RightHideUpper,
    All
}LegIndex;

typedef enum CoordinateIndex{
    X=0,
    Y,
    Z
}CoordinateIndex;

typedef enum LegStatus{
    Lock = 0,
    Unlock,
}LegStatus;

typedef struct HexaPod{
    uint16_t RawAngleData[TOTAL_SERVOS_NUM];
    double convertedAngleData[TOTAL_SERVOS_NUM];
    double coordinateData[TOTAL_LEGS][3];
    LegStatus legStatus[TOTAL_LEGS];
    int mutexRxFromServo;
    void (*move)(const double ** tipCoordinate, uint16_t Time);
    void (*unlockLegs)(LegIndex leg);
    void (*rollStraight)(uint16_t Time, uint16_t scope);
    void (*rollTurnLeft)(uint16_t Time, uint16_t scope);
    void (*rollTurnRight)(uint16_t Time, uint16_t scope);
}HexaPod;


static inline void moveLegsByArray(const double *theta, uint16_t Time);
static inline void unlockLegs(LegIndex leg);
static void coordinateFrameConvert(double ** centerCoordinateFrame,const double ** tipCoordinateFrame);
static inline void moveByCoordinate(const double ** Coordinate, uint16_t Time);
void HexaPod_Init(HexaPod * Robo);
#endif //UARTTEST_HEXAPOD_H
