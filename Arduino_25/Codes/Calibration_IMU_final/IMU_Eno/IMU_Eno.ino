#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, 0x28);

void setup () {

Serial.begin (115200);
myIMU.begin();
delay(1000);
// int8_t temp= myIMU.getTemp();
myIMU.setExtCrystalUse(true);
myIMU.setMode(OPERATION_MODE_NDOF);


}

void loop() {
  uint8_t system, gyro, accel, mg =0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyrom = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = myIMU.getTemp();

  // Print data to Serial Monitor
  Serial.print("Accelerometer (m/s²): X=");
  Serial.print(acc.x()); Serial.print(" Y=");
  Serial.print(acc.y()); Serial.print(" Z=");
  Serial.println(acc.z());

  // Serial.print("Gyroscope (°/s): X=");
  // Serial.print(gyro.x()); Serial.print(" Y=");
  // Serial.print(gyro.y()); Serial.print(" Z=");
  // Serial.println(gyro.z());

  // Serial.print("Magnetometer (µT): X=");
  // Serial.print(mag.x()); Serial.print(" Y=");
  // Serial.print(mag.y()); Serial.print(" Z=");
  // Serial.println(mag.z());
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.println(system);


  Serial.print("Temperature (°C): ");
  Serial.println(temp);

  Serial.println("---------------------------");
  delay(1000);  // Wait for 1 second before next reading
}
