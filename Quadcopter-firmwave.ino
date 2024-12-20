#include <ArduinoBLE.h>
#include "def.h"
#include "set_pin.h"
#include "IMU.h"
#include "BLE.h"
#include "PID.h"
#include "LSM6DS3.h"
#include "Wire.h"

int MOTOR_Output[NUM_MOTORS] = { 0 };

int Throttle = 0;

AttitudeAngle currentAngle = { 0.0, 0.0, 0.0 };
AttitudeAngle targetAngle = { 0.0, 0.0, 0.0 };

uint32_t timer;

LSM6DS3 myIMU(I2C_MODE, 0x6A);  // LSM6DS3 address: 0x6A

void updateMotorOutputs(int Output[4]);
void reduceMotorOutputs();

void setup() {
  Serial.begin(Serialbuad);
  setupPins();
  initializeBLE();

  digitalWrite(LED_RED, LOW);

  if (myIMU.begin() == 0) Serial.println("LSM6DS3 initialize OK!");
  else Serial.println("Could not find a valid LSM6DS3 sensor, check wiring!");
  delay(1000);  // Wait for sensor to stabilize

  // force Calbriation Gyro Sensor
  calGyroBias(myIMU);

  timer = micros();

  currentAngle = updateAttitude(myIMU, timer);

  kalmanX.setAngle(currentAngle.roll);
  kalmanY.setAngle(currentAngle.pitch);

  digitalWrite(LED_RED, HIGH);
}

void loop() {
  int rollOutput, pitchOutput, yawOutput;

  BLEDevice central = BLE.central();  // listen for Bluetooth® Low Energy peripherals to connect:

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    Throttle = 0;  // reset

    // while the central is still connected to peripheral:
    while (central.connected()) {
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_GREEN, LOW);

      if (switchCharacteristic.written()) {
        char command = (char)switchCharacteristic.value();
        switch (command) {
          case 'L': targetAngle.roll -= ADJUST_ANGLE_UNIT; break;
          case 'R': targetAngle.roll = ADJUST_ANGLE_UNIT; break;
          case 'F': targetAngle.pitch -= ADJUST_ANGLE_UNIT; break;
          case 'B': targetAngle.pitch = ADJUST_ANGLE_UNIT; break;
          case 'U': Throttle += THROTTLE_UNIT; break;
          case 'D': Throttle -= THROTTLE_UNIT; break;
          case 'G': calGyroBias(myIMU); break;
          default: break;
        }
      }
      if (Throttle > THROTTLE_MAX) Throttle = THROTTLE_MAX;
      if (Throttle < THROTTLE_MIN) Throttle = THROTTLE_MIN;

      currentAngle = updateAttitude(myIMU, timer);
      rollOutput = computePID(pidRoll, targetAngle.roll, kalAngle.roll, myIMU.readFloatGyroX());
      pitchOutput = computePID(pidPitch, targetAngle.pitch, kalAngle.pitch, myIMU.readFloatGyroY());
      yawOutput = pidYaw.kd * myIMU.readFloatGyroZ();

      if (Throttle != THROTTLE_MIN) {
        MOTOR_Output[3] = Throttle - pitchOutput - rollOutput - yawOutput;
        MOTOR_Output[1] = Throttle + pitchOutput + rollOutput - yawOutput;
        MOTOR_Output[0] = Throttle - pitchOutput + rollOutput + yawOutput;
        MOTOR_Output[2] = Throttle + pitchOutput - rollOutput + yawOutput;
      } else memset(MOTOR_Output, 0, sizeof(MOTOR_Output));

      for (int i = 0; i < 4; i++) {
        MOTOR_Output[i] = constrain(MOTOR_Output[i], THROTTLE_MIN, THROTTLE_MAX);
#if 1
        Serial.print(MOTOR_Output[i]);
        if (i == 3) Serial.print("\n");
        else Serial.print("\t");
#endif
      }

      updateMotorOutputs(MOTOR_Output);
    }
    digitalWrite(LED_GREEN, HIGH);
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());

  } else {
    reduceMotorOutputs();
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);

#if 0
    printAcclGyro(myIMU);
#endif
  }
}

void updateMotorOutputs(int Output[4]) {
  analogWrite(MOTOR0_PIN, Output[0]);
  analogWrite(MOTOR2_PIN, Output[2]);
  analogWrite(MOTOR1_PIN, Output[1]);
  analogWrite(MOTOR3_PIN, Output[3]);
}

void reduceMotorOutputs() {
  int totalMotorOutput = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    totalMotorOutput += MOTOR_Output[i];
  }

  if (totalMotorOutput > 0) {
    for (int j = 0; j < THROTTLE_MAX / THROTTLE_UNIT; j++) {
      digitalWrite(LED_BLUE, LOW);
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (MOTOR_Output[i] < 0) {
          MOTOR_Output[i] = 0;
        } else {
          MOTOR_Output[i] -= THROTTLE_UNIT;
        }
      }
      updateMotorOutputs(MOTOR_Output);
      delay(100);
      digitalWrite(LED_BLUE, HIGH);
      delay(100);
    }
  }
}
