#ifndef _PID_H
#define _PID_H

#include "def.h"

// PID Control variable
extern PID_PARA pidRoll;
extern PID_PARA pidPitch;
extern PID_PARA pidYaw;

int computePID(PID_PARA &pidPara, double target, double current, float gyro);


#endif /* PID_H_ */
