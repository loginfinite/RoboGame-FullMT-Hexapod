//
// Created by 35802 on 2022/7/18.
//
#include "HexaPod.h"
#include <math.h>
#include "InverseKinematics.h"
void HexaPod_Init(HexaPod * Robo, runMode mode){
    Robo->mode = mode;
    Robo->move = moveLegsByCoordinate;
    Robo->unlockLegs = unlockLegs;
    Robo->getAngle = getLegsAngle;
    Robo->moveSingleLeg = moveSingleLeg;
}

static inline void moveSingleLeg(LegIndex leg,const double *theta, uint16_t Time){
    double LegCoxaServoID  = 3*leg+Coxa;
    double LegFemurServoID = 3*leg+Femur;
    double LegTibiaServoID = 3*leg+Tibia;
    if(leg<=LeftFrontUpper){
        moveServos(3,Time,LegCoxaServoID,theta[0],
                            LegFemurServoID,theta[1],
                            LegTibiaServoID,theta[2]);
    }else{
        moveServos(2,Time,LegCoxaServoID,theta[0],
                                LegFemurServoID,theta[1]);
    }
}

static inline void moveLegsByAngle(const double *theta, uint16_t Time){
    uint16_t  rawAngle[CRAWL_LEGS * 3];
    deConvertAngle(theta, rawAngle);
    moveServosByArray(CRAWL_LEGS * 3, Time,rawAngle);
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

static inline void moveLegsByCoordinate(const double ** moveCoordinate, uint16_t Time){
    double ** Coordinate[CRAWL_LEGS];
    double Theta[CRAWL_LEGS * 3];
    coordinateFrameConvert((double **)Coordinate, moveCoordinate);
    coordinateToThetaArray(&Theta[LeftFront],*Coordinate[LeftFront]);
    coordinateToThetaArray(&Theta[LeftHind],*Coordinate[LeftHind]);
    coordinateToThetaArray(&Theta[LeftMiddle],*Coordinate[LeftMiddle]);
    coordinateToThetaArray(&Theta[RightFront],*Coordinate[RightFront]);
    coordinateToThetaArray(&Theta[RightHind],*Coordinate[RightHind]);
    coordinateToThetaArray(&Theta[RightMiddle],*Coordinate[RightMiddle]);
    moveLegsByAngle(Theta,Time);
}

static inline void getLegsAngle(){
    getServoAngle(18,0,
                  1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,
                  19,20,21,22,23,24,25,26,27,28,29,30);
}

static void crawStraight(uint16_t Time, uint16_t scope){

}



