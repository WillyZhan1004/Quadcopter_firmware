#ifndef DEF_H
#define DEF_H

/*
Moter Pin Definiation
   (MOTOR0)   (MOTOR3)
       \        /
        \      /
           ()
        /      \
       /        \
   (MOTOR1)   (MOTOR2)
*/
#define NUM_MOTORS 4
#define MOTOR0_PIN 0
#define MOTOR1_PIN 1
#define MOTOR2_PIN 2
#define MOTOR3_PIN 3

#define THROTTLE_MAX 220
#define THROTTLE_MIN 0
#define THROTTLE_UNIT 5
#define ADJUST_ANGLE_UNIT 5

#define RESTRICT_PITCH  // Comment out to restrict roll to ±90deg instead

#define Serialbuad 9600
#define SENSOR_CAL_TIMES 1000

struct IMUData {
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

struct AttitudeAngle {
  double roll;
  double pitch;
  double yaw;
};

struct PID_PARA {
  double kp;
  double ki;
  double kd;
  double stateIntegral;
};


#endif