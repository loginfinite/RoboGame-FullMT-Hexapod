//
// Created by 35802 on 2022/7/19.
//
#include "InverseKinematics.h"
#include "math.h"
void coordinateToTheta(theta *thetaForJoint,double X,double Y,double Z){
    double AL,alpha1,alpha2,beta,y_dot;
    y_dot = Y/(cos(atan(X/Y)));
    AL = sqrt(((pow(y_dot-LIMB_A_LENGTH,2)+ pow(Z,2))));
    alpha1 = acos(sqrt(
            (pow(LIMB_B_LENGTH,2)+pow(AL,2)-pow(LIMB_C_LENGTH,2))/(2*LIMB_B_LENGTH*AL)
            ));
    alpha2 = acos(sqrt(
            (pow(LIMB_C_LENGTH,2)+pow(AL,2)-pow(LIMB_B_LENGTH,2))/(2*LIMB_C_LENGTH*AL)
            ));
    beta = atan(Z/(y_dot-LIMB_A_LENGTH));
    thetaForJoint->thetaA = atan(X/Y);
    thetaForJoint->thetaB = beta - alpha1;
    thetaForJoint->thetaC = alpha1 + alpha2;
}
void thetaToCoordinate(coordinate *Tip,double thetaA,double thetaB,double thetaC){
    double y_dot;
    y_dot = LIMB_A_LENGTH + LIMB_B_LENGTH*cos(thetaB) + LIMB_C_LENGTH*cos(thetaB + thetaC);
    Tip->Z = LIMB_B_LENGTH * sin(thetaB) + LIMB_C_LENGTH * sin(thetaB + thetaC);
    Tip->X = y_dot * sin(thetaA);
    Tip->Y = y_dot * cos(thetaA);
}
