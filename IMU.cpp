#include "IMU.h"

// Create the Kalman instances
Kalman kalmanX;
Kalman kalmanY;

float gyroX_Bias = 0.77;
float gyroY_Bias = -2.14;
float gyroZ_Bias = -0.39;

AttitudeAngle kalAngle;

AttitudeAngle updateAttitude(LSM6DS3 &imu, uint32_t &timer) {

  AttitudeAngle newAttitude;

  float accX = imu.readFloatAccelX();
  float accY = imu.readFloatAccelY();
  float accZ = imu.readFloatAccelZ();
  float gyroX = imu.readFloatGyroX() - gyroX_Bias;
  float gyroY = imu.readFloatGyroY() - gyroY_Bias;
  float gyroZ = imu.readFloatGyroZ() - gyroZ_Bias;

  double gyroXrate = gyroX / 131.0;  // Convert to deg/s
  double gyroYrate = gyroY / 131.0;  // Convert to deg/s

  double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
  timer = micros();

  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH
  newAttitude.roll = atan2(accY, accZ) * RAD_TO_DEG;
  newAttitude.pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((newAttitude.roll < -90 && kalAngle.roll > 90) || (newAttitude.roll > 90 && kalAngle.roll < -90)) {
    kalmanX.setAngle(newAttitude.roll);
    kalAngle.roll = newAttitude.roll;
  } else
    kalAngle.roll = kalmanX.getAngle(newAttitude.roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngle.roll) > 90) gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading

  kalAngle.pitch = kalmanY.getAngle(newAttitude.pitch, gyroYrate, dt);

#else
  newAttitude.roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  newAttitude.pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((newAttitude.ptich < -90 && kalAngle.pitch > 90) || (newAttitude.ptich > 90 && kalAngle.pitch < -90)) {
    kalmanY.setAngle(newAttitude.ptich);
    kalAngle.pitch = newAttitude.ptich;
  } else
    kalAngle.pitch = kalmanY.getAngle(newAttitude.ptich, gyroYrate, dt);  // Calculate the angle using a Kalman filter

  if (abs(kalAngle.pitch) > 90) gyroXrate = -gyroXrate;  // Invert rate, so it fits the restriced accelerometer reading

  kalAngle.roll = kalmanX.getAngle(newAttitude.roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

#if 0
  Serial.print(newAttitude.roll);
  Serial.print("\t");
  Serial.print(kalAngle.roll);
  Serial.print("\t\t");
  Serial.print(newAttitude.pitch);
  Serial.print("\t");
  Serial.print(kalAngle.pitch);
  Serial.println();
#endif

  return newAttitude;
}

void printAcclGyro(LSM6DS3 &imu) {
  float accX = imu.readFloatAccelX();
  float accY = imu.readFloatAccelY();
  float accZ = imu.readFloatAccelZ();
  float gyroX = imu.readFloatGyroX() - gyroX_Bias;
  float gyroY = imu.readFloatGyroY() - gyroY_Bias;
  float gyroZ = imu.readFloatGyroZ() - gyroZ_Bias;
  float acclCombine = sqrt(sq(accX) + sq(accY) + sq(accZ));

  Serial.print("Accl: ");
  Serial.print(acclCombine);
  Serial.print("\t");
  Serial.print(accX);
  Serial.print("\t");
  Serial.print(accY);
  Serial.print("\t");
  Serial.print(accZ);
  Serial.print("\t");
  Serial.print("Gyro: ");
  Serial.print(gyroX);
  Serial.print("\t");
  Serial.print(gyroY);
  Serial.print("\t");
  Serial.print(gyroZ);
  Serial.println();
}

void calGyroBias(LSM6DS3 &imu) {
  float acclNew, acclOld, acclDiff;
  int stabeFlag = 0;
  while (1) {
    acclOld = acclNew;
    acclNew = sqrt(sq(imu.readFloatAccelX()) + sq(imu.readFloatAccelY()) + sq(imu.readFloatAccelZ()));
    acclDiff = acclNew - acclOld;
    if ((acclDiff / acclNew) < 0.01) stabeFlag++;
    else stabeFlag = 0;

    if (stabeFlag == 100) break;
    delay(10);
  }

  gyroX_Bias = 0.0;
  gyroY_Bias = 0.0;
  gyroZ_Bias = 0.0;

  for (int i = 0; i < SENSOR_CAL_TIMES; i++) {
    gyroX_Bias += imu.readFloatGyroX();
    gyroY_Bias += imu.readFloatGyroY();
    gyroZ_Bias += imu.readFloatGyroZ();
  }
  gyroX_Bias /= SENSOR_CAL_TIMES;
  gyroY_Bias /= SENSOR_CAL_TIMES;
  gyroZ_Bias /= SENSOR_CAL_TIMES;
#if 0
  Serial.print("gyroX_Bias: ");
  Serial.print(gyroX_Bias);
  Serial.print("\t");
  Serial.print("gyroY_Bias: ");
  Serial.print(gyroY_Bias);
  Serial.print("\t");
  Serial.print("gyroZ_Bias: ");
  Serial.print(gyroZ_Bias);
  Serial.println();
#endif
}
