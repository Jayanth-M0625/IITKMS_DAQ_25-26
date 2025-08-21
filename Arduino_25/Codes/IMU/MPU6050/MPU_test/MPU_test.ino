
#include <Wire.h>
#include <MPU6050.h>  // Good MPU9250 libraries often reuse MPU6050 DMP parts

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  
  Serial.println("MPU6050 connection: ");
  Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");

  Serial.println("Starting Calibration...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  Serial.println("Calibration done.");
  
  Serial.println("Accelerometer Offsets:");
  Serial.print(mpu.getXAccelOffset()); Serial.print(", ");
  Serial.print(mpu.getYAccelOffset()); Serial.print(", ");
  Serial.println(mpu.getZAccelOffset());

  Serial.println("Gyroscope Offsets:");
  Serial.print(mpu.getXGyroOffset()); Serial.print(", ");
  Serial.print(mpu.getYGyroOffset()); Serial.print(", ");
  Serial.println(mpu.getZGyroOffset());
}

void loop() {
  // Empty loop for calibration only
}
