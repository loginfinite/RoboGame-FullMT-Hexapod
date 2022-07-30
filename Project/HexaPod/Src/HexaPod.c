//
// Created by 35802 on 2022/7/18.
//
#include "HexaPod.h"
int getLegID(LegIndex index){
    switch (index) {
        case LeftFront:
            return 1;
        case LeftMiddle:
            return 10;
        case LeftHind:
            return 4;
        case RightFront:
            return 13;
        case RightMiddle:
            return 7;
        case RightHide:
            return 16;
        default:
            return -1;
    }
}

void HexaPod_Init(HexaPod* hexapod){
    LegIndex legIndex=LeftFront;
    for(;legIndex<=RightHide;++legIndex){
        hexapod->legs[legIndex].state = Lock;
        hexapod->legs[legIndex].legID = getLegID(legIndex);//该机械足的腿跟舵机ID，ID+1,ID+2为该机械足中端，末端舵机ID号
    }
    lockAllLegs();
}

void getLegsAngle(LegIndex index){
    int id = getLegID(index);
    getServoAngle(3,0,index,index+1,index+2);
}

void getAllLegsAngle(){
    getServoAngle(18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
    while(!isUartRxCompleted);
    isUartRxCompleted = 0;
    convertAngle(ConvertAngleBuf, AngleBuf);
}

void unlockLeg(HexaPod* hexapod, LegIndex index){
    int ID = hexapod->legs[index].legID;
    hexapod->legs[index].state = Unlock;
    unloadServos(3,0, ID,ID+1,ID+2);
}

void lockLeg(HexaPod* hexapod, LegIndex index){
    getLegsAngle(index);
    while(!isUartRxCompleted);
    int ID = hexapod->legs[index].legID;
    moveServos(3,0,1000,ID,AngleBuf[ID],ID+1,AngleBuf[ID+1],ID+2,AngleBuf[ID+2]);
}

void lockAllLegs(){
    getAllLegsAngle();
    moveServos(18,0,
               1,AngleBuf[0],
               2,AngleBuf[1],
               3,AngleBuf[2],
               4,AngleBuf[3],
               5,AngleBuf[4],
               6,AngleBuf[5],
               7,AngleBuf[6],
               8,AngleBuf[7],
               9,AngleBuf[8],
               10,AngleBuf[9],
               11,AngleBuf[10],
               12,AngleBuf[11],
               13,AngleBuf[12],
               14,AngleBuf[13],
               15,AngleBuf[14],
               16,AngleBuf[15],
               17,AngleBuf[16],
               18,AngleBuf[17]);
}

void unlockAllLegs(){
    unloadServos(18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
}

void setLegsDefault(){
    moveServos(18,1000,
    1,479,
    2,435,
    3,240,
    4,466,
    5,496,
    6,324,
    7,488,
    8,574,
    9,722,
    10,501,
    11,429,
    12,177,
    13,434,
    14,475,
    15,374,
    16,496,
    17,456,
    18,616
    );
}

void setLegAngle(LegIndex index,double thetaA,double thetaB,double thetaC){
    int ID_0 = getLegID(index);
    if (ID_0 == -1)
        return;
    uint16_t angle[18];
    double theta[18];
    theta[ID_0-1]=thetaA;
    theta[ID_0]=thetaB;
    theta[ID_0+1]=thetaC;
    deConvertAngle(angle,theta);
    moveServos(3,1000,
               ID_0,angle[ID_0-1],
               ID_0+1,angle[ID_0],
               ID_0+2,angle[ID_0+1]);
}

void setAllLegs(uint16_t *angle,double *theta){
    deConvertAngle(angle,theta);
    moveServos(18,1000,
               1,angle[0],
               2,angle[1],
               3,angle[2],
               4,angle[3],
               5,angle[4],
               6,angle[5],
               7,angle[6],
               8,angle[7],
               9,angle[8],
               10,angle[9],
               11,angle[10],
               12,angle[11],
               13,angle[12],
               14,angle[13],
               15,angle[14],
               16,angle[15],
               17,angle[16],
               18,angle[17]);
}

void setAllMidLeg(uint16_t *angle,double theta){
    double thetaA[18] = {theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta};
    deConvertAngle(angle,thetaA);
    moveServos(6,1000,
               2,angle[1],
               5,angle[4],
               8,angle[7],
               11,angle[10],
               14,angle[13],
               17,angle[16]);
}

void setAllEndLeg(uint16_t *angle,double theta){
    double thetaA[18] = {theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta,theta};
    deConvertAngle(angle,thetaA);
    moveServos(6,1000,
               3,angle[2],
               6,angle[5],
               9,angle[8],
               12,angle[11],
               15,angle[14],
               18,angle[17]);
}