#include <Evo.h>
#include <EV3Motor.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "MPU9250.h"

// Create sensor objects
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU9250 mpu;
EV3Motor rearMotor(M1, false);
EV3Motor frontMotor(M2, false);
EVO evo;
VL53L0X_RangingMeasurementData_t measure;

// Function to print calibration parameters (unchanged)
void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}

// Left turn function (unchanged)
void leftTurn() {
  rearMotor.resetAngle();
  rearMotor.move(150);
  delay(1300);
  rearMotor.move(0);
  frontMotor.resetAngle();
  frontMotor.move(150);
  delay(1500);
  frontMotor.move(0);
  rearMotor.move(-150);
}

// Right turn function (unchanged)
void rightTurn() {
  rearMotor.resetAngle();
  rearMotor.run(-150);
  delay(1300);
  rearMotor.run(0);
  frontMotor.resetAngle();
  frontMotor.run(150);
  delay(1500);
  frontMotor.run(0);
  rearMotor.run(150);
}

// Function to read from left sensor
int leftSensorReading() {
  int leftSensor = 0;
  evo.selectI2CChannel(1);
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    leftSensor = measure.RangeMilliMeter;
    if (leftSensor > 2500) {
      leftSensor = 2500;
    } 
    Serial.print("Left Sensor Distance (mm): ");
    Serial.println(leftSensor);
  }
  return leftSensor;
}

// Function to read from right sensor
int rightSensorReading() {
  int rightSensor = 0;
  evo.selectI2CChannel(2);
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    rightSensor = measure.RangeMilliMeter;
    if (rightSensor > 2500) {
      rightSensor = 2500;
    }
    Serial.print("Right Sensor Distance (mm): ");
    Serial.println(rightSensor);
  }
  return rightSensor;
}

// Setup function
void setup() {
  Serial.begin(115200);
  evo.begin();
  while (!Serial) {
    delay(1);
  }

  // Initialize sensors
  evo.selectI2CChannel(1);
  if (!lox.begin()) {
    Serial.println(F("Left sensor failed to boot VL53L0X"));
    while (1);
  }
  
  evo.selectI2CChannel(2);
  if (!lox.begin()) {
    Serial.println(F("Right sensor failed to boot VL53L0X"));
    while (1);
  }

  // Initialize display and motors
  evo.beginDisplay();
  Serial.print("Battery Voltage: ");
  Serial.println(evo.getBattery());
  Serial.println("Do not let batteries go below 6.0V");
  evo.writeToDisplay("Battery", 0, true);
  evo.writeToDisplay(evo.getBattery(), 1);
  delay(500);
  rearMotor.begin();
  delay(1000);
  frontMotor.begin();
  delay(1000);

  // Play tone
  evo.playTone(NOTE_G4, 300);
}

// Main loop function
void loop() {
  int leftDistance = leftSensorReading();
  int rightDistance = rightSensorReading();
  delay(1000);
  if (leftDistance > 250){
    Serial.println("truning left");
    leftTurn();
    leftDistance = 2500;
 }  else if (rightDistance > 250){
    Serial.println("truning right");
    evo.selectI2CChannel(5);
    rightTurn();

 delay(100);  // Adjust delay as needed
  }else {
    
  }
} 

