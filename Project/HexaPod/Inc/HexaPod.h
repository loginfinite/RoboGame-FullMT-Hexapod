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
#define CRAWL_LEGS 6
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

typedef enum runMode{
    debug = 0,
    run,
}runMode;

typedef enum CommandType{
    idle
}CommandType;

typedef struct HexaPod{
    uint16_t RawAngleData[TOTAL_SERVOS_NUM];
    double convertedAngleData[TOTAL_SERVOS_NUM];
    double coordinateData[TOTAL_LEGS][3];
    LegStatus legStatus[TOTAL_LEGS];
    int mutexRxFromServo;
    CommandType Command;
    runMode mode;

    void (*move)(const double ** tipCoordinate, uint16_t Time);
    void (*moveSingleLeg)(LegIndex leg,const double *theta, uint16_t Time);
    void (*unlockLegs)(LegIndex leg);
    void (*getAngle)();

    void (*crawlStraight)(uint16_t Time, uint16_t scope);
    void (*crawlTurnLeft)(uint16_t Time, uint16_t scope);
    void (*crawlTurnRight)(uint16_t Time, uint16_t scope);

    void (*rollStraight)(uint16_t Time, uint16_t scope);
    void (*rollTurnLeft)(uint16_t Time, uint16_t scope);
    void (*rollTurnRight)(uint16_t Time, uint16_t scope);
}HexaPod;

static inline void moveSingleLeg(LegIndex leg,const double *theta, uint16_t Time);
static inline void unlockLegs(LegIndex leg);
static inline void getLegsAngle();

static void coordinateFrameConvert(double ** centerCoordinateFrame,const double ** tipCoordinateFrame);

static inline void moveLegsByAngle(const double *theta, uint16_t Time);
static inline void moveLegsByCoordinate(const double ** Coordinate, uint16_t Time);

static void crawStraight(uint16_t Time, uint16_t scope);
static void rollStraight(uint16_t Time, uint16_t scope);

void HexaPod_Init(HexaPod * Robo, runMode mode);
#endif //UARTTEST_HEXAPOD_H
