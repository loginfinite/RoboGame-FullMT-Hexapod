//
// Created by 35802 on 2022/7/19.
//

#ifndef UARTTEST_INVERSEKINEMATICS_H
#define UARTTEST_INVERSEKINEMATICS_H
#define LIMB_A_LENGTH 46.5
#define LIMB_B_LENGTH 77
#define LIMB_C_LENGTH 190
inline void coordinateToThetaArray(double *thetaForJoint,const double * coordinateForTip);
inline void thetaToCoordinateArray(double *coordinateForTip,const double * thetaForJoint);
#endif //UARTTEST_INVERSEKINEMATICS_H
