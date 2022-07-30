//
// Created by 35802 on 2022/7/18.
//

#ifndef UARTTEST_HEXAPOD_H
#define UARTTEST_HEXAPOD_H
#include "Servo.h"
#include "InverseKinematics.h"
#define ROUND_R 100
typedef enum LegIndex{
    LeftFront=0,
    LeftMiddle=1,
    LeftHind=2,
    RightFront=3,
    RightMiddle=4,
    RightHide=5
}LegIndex;
typedef enum LegState{
    Lock,
    Unlock
}LegState;

typedef struct Leg{
    LegState state;
    int legID;
}Leg;

typedef struct HexaPod{
    Leg legs[6];
}HexaPod;

int getLegID(LegIndex index);
void HexaPod_Init(HexaPod*hexapod);
void getLegsAngle(LegIndex index);
void getAllLegsAngle();
void lockLeg(HexaPod* hexapod, LegIndex index);
void unlockLeg(HexaPod* hexapod, LegIndex index);
void unlockAllLegs();
void lockAllLegs();
void setLegsDefault();
void setLegAngle(LegIndex index,double thetaA,double thetaB,double thetaC);
void setAllLegs(uint16_t *angle,double *theta);
void setAllMidLeg(uint16_t *angle,double theta);
void setAllEndLeg(uint16_t *angle,double theta);
#endif //UARTTEST_HEXAPOD_H
