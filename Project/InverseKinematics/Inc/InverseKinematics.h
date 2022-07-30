//
// Created by 35802 on 2022/7/19.
//

#ifndef UARTTEST_INVERSEKINEMATICS_H
#define UARTTEST_INVERSEKINEMATICS_H
#define LIMB_A_LENGTH 46.5
#define LIMB_B_LENGTH 77
#define LIMB_C_LENGTH 190
typedef struct coordinate{
    double X;
    double Y;
    double Z;
}coordinate;
typedef struct theta{
    double thetaA;
    double thetaB;
    double thetaC;
}theta;
void coordinateToTheta(theta *thetaForJoint,double X,double Y,double Z);
void thetaToCoordinate(coordinate *coordinateForTip,double thetaA,double thetaB,double thetaC);
#endif //UARTTEST_INVERSEKINEMATICS_H
