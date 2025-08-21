#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h> // Include the SD library

// Pin configuration for SD card module
const int chipSelect = 43;

// variables:

const int periodIMU = 10000;                            // IMU period in micros
unsigned long previousTimeIMU = 0;                      // end time of the last sensor period in micros
unsigned long currentTime = 0;                          // actual time in micros
unsigned long initialTime = 0;                          // time when main loop begin for the first time in micros
float vel_X = 0.0;
float vel_Y = 0.0;
float vel_Z = 0.0;

#define BNO055_SAMPLERATE_DELAY_MS (periodIMU/1000)     // IMU sample rate initialisation in ms

Adafruit_BNO055 bno = Adafruit_BNO055();

//------------------------------------------------------------------------------------------------//
void setup(void)
{
  Serial.begin(9600);
  Serial.println("BNO055 Data Test");
  Serial.println("");

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");

  // Initialise the sensor
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");  // There was a problem detecting the BNO055 ... check your connections
    while (1);
  }
  delay(1000);
  
  // Display current temperature
  int8_t temp = bno.getTemp();   
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  // Display column headers
  Serial.println( F(
    "\t\t\t"  
    "Position "
    "\t\t\t\t"
    "Angular velocity"
    "\t\t\t\t"
    "Acceleration"
    "\t\t\t\t"
    "Velocity"
    "\t\t\t\t"
    "Calibration"));
    
  // Display column sub-headers
  Serial.println( F(
    "Time "
    "\t\t"
    "Roll \t Pitch \t Yaw"
    "\t\t"
    "Roll rate \t Pitch rate \t Yaw rate"
    " \t\t"
    "Accel X \t Accel Y \tAccel Z"
    "\t\t"
    "Vel_X \t Vel_Y \tVel_Z"
    "\t\t"
    "Sys \t Gyro \t Accel \t Mag") );

  Serial.flush();             // wait for header line to go out
  
  initialTime = micros();         // start timing from NOW
  previousTimeIMU = initialTime;
  
}

//------------------------------------------------------------------------------------------------//
void loop(void)
{
  currentTime = micros();

  if (currentTime - previousTimeIMU >= periodIMU) {
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    float dt = float(currentTime - previousTimeIMU) / 1e6;

    // Integrate accelerometer data to obtain velocity
    vel_X += accelerometer.x() * dt;
    vel_Y += accelerometer.y() * dt;
    vel_Z += (accelerometer.z()-9.8) * dt;

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);   // calibration statut

    Serial.print(currentTime - initialTime);        // current time at the begining of each loop
    Serial.print("\t\t");

    Serial.print(euler.z());
    Serial.print("\t");
    Serial.print(euler.y());
    Serial.print("\t ");
    Serial.print(euler.x());
    Serial.print("\t\t");

    Serial.print(gyroscope.x());
    Serial.print("\t");
    Serial.print(gyroscope.y());
    Serial.print("\t");
    Serial.print(gyroscope.z());
    Serial.print(" \t\t");

    Serial.print(accelerometer.x());
    Serial.print("\t");
    Serial.print(accelerometer.y());
    Serial.print("\t");
    Serial.print(accelerometer.z());
    Serial.print("\t\t");

    Serial.print(vel_X);
    Serial.print("\t ");
    Serial.print(vel_Y);
    Serial.print("\t ");
    Serial.print(vel_Z);
    Serial.print("\t\t");

    Serial.print(system, DEC);
    Serial.print("\t ");
    Serial.print(gyro, DEC);
    Serial.print("\t");
    Serial.print(accel, DEC);
    Serial.print("\t ");
    Serial.println(mag, DEC);

    // Store data in SD card
    File dataFile = SD.open("30Apr_1.csv", FILE_WRITE);
    if (dataFile) {
      
      dataFile.print(currentTime - initialTime);
      dataFile.print(",");
      dataFile.print(euler.z());
      dataFile.print(",");
      dataFile.print(euler.y());
      dataFile.print(",");
      dataFile.print(euler.x());
      dataFile.print(",");
      dataFile.print(gyroscope.x());
      dataFile.print(",");
      dataFile.print(gyroscope.y());
      dataFile.print(",");
      dataFile.print(gyroscope.z());
      dataFile.print(",");
      dataFile.print(accelerometer.x());
      dataFile.print(",");
      dataFile.print(accelerometer.y());
      dataFile.print(",");
      dataFile.print(accelerometer.z());
      dataFile.print(",");
      dataFile.print(vel_X);
      dataFile.print(",");
      dataFile.print(vel_Y);
      dataFile.print(",");
      dataFile.print(vel_Z);
      dataFile.print(",");
      dataFile.print(system, DEC);
      dataFile.print(",");
      dataFile.print(gyro, DEC);
      dataFile.print(",");
      dataFile.print(accel, DEC);
      dataFile.print(",");
      dataFile.println(mag, DEC);
      dataFile.close();
    } 
    else {
      Serial.println("Error opening data file.");
    }

    previousTimeIMU += periodIMU;             // uptade previousTime
  }
}
