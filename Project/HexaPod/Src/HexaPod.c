//
// Created by 35802 on 2022/7/18.
//
#include "HexaPod.h"
#include <math.h>
static inline void moveLegsByArray(const double *theta, uint16_t Time){
    uint16_t  rawAngle[TOTAL_SERVOS_NUM];
    deConvertAngle(theta, rawAngle);
    moveServosByArray(TOTAL_SERVOS_NUM, Time,rawAngle);
}

static inline void unlockLegs(LegIndex leg) {
    int IDarray[TOTAL_SERVOS_NUM],i;
    for (i = 0; i < TOTAL_SERVOS_NUM; ++i) {
        IDarray[i] = i + 1;
    }
    if (leg == All) {
        unloadServosByArray(TOTAL_SERVOS_NUM, 0, IDarray);
    }else{
        double LegCoxaServoID  = 3*leg+Coxa;
        double LegFemurServoID = 3*leg+Femur;
        double LegTibiaServoID = 3*leg+Tibia;
        unloadServos(3,0,LegCoxaServoID,LegFemurServoID,LegTibiaServoID);
    }
}

static void coordinateFrameConvert(double ** centerCoordinate,const double ** tipCoordinate){
    centerCoordinate[LeftFront][X] = tipCoordinate[LeftFront][X]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * sin(60);
    centerCoordinate[LeftFront][Y] = tipCoordinate[LeftFront][Y]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * cos(60);
    centerCoordinate[LeftFront][Z] = DEFAULT_HEIGHT - tipCoordinate[LeftFront][Z];
    centerCoordinate[RightFront][X] = tipCoordinate[LeftFront][X]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * sin(60);
    centerCoordinate[RightFront][Y] = tipCoordinate[LeftFront][Y]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * cos(60);
    centerCoordinate[RightFront][Z] = DEFAULT_HEIGHT - tipCoordinate[LeftFront][Z];
    centerCoordinate[RightMiddle][X] = tipCoordinate[LeftMiddle][X];
    centerCoordinate[RightMiddle][Y] = tipCoordinate[LeftMiddle][Y]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS);
    centerCoordinate[RightMiddle][Z] = DEFAULT_HEIGHT - tipCoordinate[LeftMiddle][Z];
    centerCoordinate[LeftMiddle][X] = tipCoordinate[LeftMiddle][X];
    centerCoordinate[LeftMiddle][Y] = tipCoordinate[LeftMiddle][Y]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS);
    centerCoordinate[LeftMiddle][Z] = DEFAULT_HEIGHT - tipCoordinate[LeftMiddle][Z];
    centerCoordinate[LeftHind][X] = -tipCoordinate[LeftHind][X]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * sin(60);
    centerCoordinate[LeftHind][Y] = tipCoordinate[LeftHind][Y]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * cos(60);
    centerCoordinate[LeftHind][Z] = DEFAULT_HEIGHT - tipCoordinate[LeftHind][Z];
    centerCoordinate[RightHind][X] = -tipCoordinate[LeftHind][X]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * sin(60);
    centerCoordinate[RightHind][Y] = tipCoordinate[LeftHind][Y]+(DEFAULT_LEG_LENGTH+STRUCTURE_RADIUS) * cos(60);
    centerCoordinate[RightHind][Z] = DEFAULT_HEIGHT - tipCoordinate[LeftFront][Z];
}

static inline void moveByCoordinate(const double ** moveCoordinate, uint16_t Time){
    double ** Coordinate[TOTAL_LEGS];
    double Theta[TOTAL_LEGS * 3];
    coordinateFrameConvert((double **)Coordinate, moveCoordinate);
    coordinateToThetaArray(&Theta[LeftFront],*Coordinate[LeftFront]);
    coordinateToThetaArray(&Theta[LeftHind],*Coordinate[LeftHind]);
    coordinateToThetaArray(&Theta[LeftMiddle],*Coordinate[LeftMiddle]);
    coordinateToThetaArray(&Theta[RightFront],*Coordinate[RightFront]);
    coordinateToThetaArray(&Theta[RightHind],*Coordinate[RightHind]);
    coordinateToThetaArray(&Theta[RightMiddle],*Coordinate[RightMiddle]);
    moveLegsByArray(Theta,Time);
}

void HexaPod_Init(HexaPod * Robo){
    Robo->move = moveByCoordinate;
    Robo->unlockLegs = unlockLegs;
}