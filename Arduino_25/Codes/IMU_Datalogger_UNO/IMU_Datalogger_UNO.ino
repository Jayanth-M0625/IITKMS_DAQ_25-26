#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

// BNO055 object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// SD card chip select pin
const int chipSelect = 4;

// Files for calibration and data logging
const char* calibrationFile = "calib.txt";
const char* dataFile = "imu_data.txt";

// Variables for timing
const unsigned long periodIMU = 10000; // IMU period in microseconds
unsigned long previousTimeIMU = 0;
unsigned long currentTime = 0;
unsigned long initialTime = 0;

//------------------------------------------------------------------------------------------------//
void setup(void) {
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor to open

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("No BNO055 detected. Check wiring or I2C address!");
    while (1);
  }
  Serial.println("BNO055 detected!");

  // Restore calibration offsets from SD card
  restoreCalibration();

  // Set up BNO055 operation mode
  bno.setExtCrystalUse(true);

  // Create or clear the data file
  if (SD.exists(dataFile)) {
    SD.remove(dataFile);
  }
  File file = SD.open(dataFile, FILE_WRITE);
  if (file) {
    file.println("Timestamp, Roll, Pitch, Yaw, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z");
    file.close();
  } else {
    Serial.println("Error creating data file!");
  }

  // Start timing
  initialTime = micros();
  previousTimeIMU = initialTime;
}

//------------------------------------------------------------------------------------------------//
void loop(void) {
  currentTime = micros();

  if (currentTime - previousTimeIMU >= periodIMU) {
    // Get IMU data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // Log data to SD card
    logData(euler, accelerometer, gyroscope);

    // Save calibration offsets periodically (e.g., every 10 seconds)
    static unsigned long lastSaveTime = 0;
    if (millis() - lastSaveTime > 10000) {
      saveCalibration();
      lastSaveTime = millis();
    }

    previousTimeIMU += periodIMU; // Update previousTime
  }
}

//------------------------------------------------------------------------------------------------//
void logData(imu::Vector<3> &euler, imu::Vector<3> &accel, imu::Vector<3> &gyro) {
  File file = SD.open(dataFile, FILE_WRITE);
  if (file) {
    file.print(currentTime - initialTime);
    file.print(", ");
    file.print(euler.x()); file.print(", ");
    file.print(euler.y()); file.print(", ");
    file.print(euler.z()); file.print(", ");
    file.print(accel.x()); file.print(", ");
    file.print(accel.y()); file.print(", ");
    file.print(accel.z()); file.print(", ");
    file.print(gyro.x()); file.print(", ");
    file.print(gyro.y()); file.print(", ");
    file.println(gyro.z());
    file.close();
  } else {
    Serial.println("Error opening data file!");
  }
}

//------------------------------------------------------------------------------------------------//
void saveCalibration() {
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);

  File file = SD.open(calibrationFile, FILE_WRITE);
  if (file) {
    file.write((uint8_t*)&offsets, sizeof(offsets));
    file.close();
    Serial.println("Calibration offsets saved.");
  } else {
    Serial.println("Error saving calibration offsets!");
  }
}

//------------------------------------------------------------------------------------------------//
void restoreCalibration() {
  if (SD.exists(calibrationFile)) {
    File file = SD.open(calibrationFile, FILE_READ);
    if (file) {
      adafruit_bno055_offsets_t offsets;
      file.read((uint8_t*)&offsets, sizeof(offsets));
      file.close();

      bno.setSensorOffsets(offsets);
      Serial.println("Calibration offsets restored.");
    } else {
      Serial.println("Error reading calibration file!");
    }
  } else {
    Serial.println("No calibration file found. Please calibrate the sensor.");
    calibrateSensor();
  }
}

//------------------------------------------------------------------------------------------------//
void calibrateSensor() {
  Serial.println("Please calibrate the sensor. Move it around until all calibration values reach 3.");

  while (!bno.isFullyCalibrated()) {
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    delay(500);
  }

  Serial.println("Sensor fully calibrated!");
  saveCalibration();
}
