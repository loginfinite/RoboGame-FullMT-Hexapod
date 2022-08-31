//
// Created by 35802 on 2022/7/19.
//
#include "InverseKinematics.h"
#include "math.h"
void coordinateToThetaArray(double *thetaForJoint,const double *Tip){
    double AL,alpha1,alpha2,beta,y_dot;
    double X,Y,Z;
    X = Tip[0];
    Y = Tip[1];
    Z = Tip[2];
    y_dot = Y/(cos(atan(X/Y)));
    AL = sqrt(((pow(y_dot-LIMB_A_LENGTH,2)+ pow(Z,2))));
    alpha1 = acos(sqrt((pow(LIMB_B_LENGTH,2)+pow(AL,2)-pow(LIMB_C_LENGTH,2))/(2*LIMB_B_LENGTH*AL)));
    alpha2 = acos(sqrt((pow(LIMB_C_LENGTH,2)+pow(AL,2)-pow(LIMB_B_LENGTH,2))/(2*LIMB_C_LENGTH*AL)));
    beta = atan(Z/(y_dot-LIMB_A_LENGTH));
    thetaForJoint[0] = atan(X/Y);
    thetaForJoint[1] = beta - alpha1;
    thetaForJoint[2] = alpha1 + alpha2;
}
void thetaToCoordinateArray(double *Tip,const double *thetaForJoint){
    double y_dot,thetaA,thetaB,thetaC;
    thetaA = thetaForJoint[0];
    thetaB = thetaForJoint[1];
    thetaC = thetaForJoint[2];
    y_dot = LIMB_A_LENGTH + LIMB_B_LENGTH*cos(thetaB) + LIMB_C_LENGTH*cos(thetaB + thetaC);
    Tip[0] = LIMB_B_LENGTH * sin(thetaB) + LIMB_C_LENGTH * sin(thetaB + thetaC);
    Tip[1] = y_dot * sin(thetaA);
    Tip[2] = y_dot * cos(thetaA);
}
