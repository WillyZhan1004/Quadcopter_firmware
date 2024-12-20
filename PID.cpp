#include "PID.h"

//                 kp,  ki,  kd,   stateIntegral
PID_PARA pidRoll= {1.0, 0.0, 0.0, 0.0};
PID_PARA pidPitch= {1.2, 0.0, 0.0, 0.0};
PID_PARA pidYaw= {0.0, 0.0, 0.7 , 0.0};

int computePID(PID_PARA &pidPara, double target, double current, float gyro) {
  double error = current - target;
  double proportion = pidPara.kp * error;
  pidPara.stateIntegral += error;
  double integral = pidPara.ki * pidPara.stateIntegral;
  double derivative = pidPara.kd * gyro;

  return proportion + integral + derivative;
}
