#ifndef _IMU_H
#define _IMU_H

#include "def.h"
#include <Kalman.h>
#include "LSM6DS3.h"
#include "Wire.h"

// Create the Kalman instances
extern Kalman kalmanX;
extern Kalman kalmanY;

extern float gyroX_Bias;
extern float gyroY_Bias;
extern float gyroZ_Bias;

extern AttitudeAngle kalAngle;

AttitudeAngle updateAttitude(LSM6DS3 &imu, uint32_t &timer);
void printAcclGyro(LSM6DS3 &imu);
void calGyroBias(LSM6DS3 &imu);

#endif /* _IMU_H */