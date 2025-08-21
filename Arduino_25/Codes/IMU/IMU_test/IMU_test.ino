#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BNO_RST 5  // Reset pin connected to digital pin 5
//int attempts =0;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
int8_t temp = 0;
//
/*
    Arduino setup function (automatically called at startup)
*/
//
void setup(void)
{
  Serial.begin(115200);
  pinMode(BNO_RST, OUTPUT);
  
  // Toggle reset
  digitalWrite(BNO_RST, LOW);
  delay(10);  // Hold reset low for 10ms
  digitalWrite(BNO_RST, HIGH);
  delay(50);  // Give sensor time to boot
  Serial.println("Toggled BNO \n");
  

  while (!Serial) delay(10);  // wait for serial port to open!
  Wire.begin();
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  // I2C Scanner
  Serial.println("Scanning for I2C devices...");
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scan complete\n");

  Serial.println("Orientation Sensor Raw Data Test\n");
  
//  while(attempts){
//  // Try initializing BNO with default address (you can swap 0x28/0x29 based on scan)
//    if(!bno.begin())
//    {
//      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    }
//    attempts--;
//    delay(1000);
//  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

//
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
//
void loop(void)
{

  if(!bno.begin())
    {
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
      Serial.print("Attempt:");
      Serial.print(temp, DEC);
      Serial.print("\n");
    }
    temp++;
    delay(1000);
//  // Possible vector values can be:
//  // - VECTOR_ACCELEROMETER - m/s^2
//  // - VECTOR_MAGNETOMETER  - uT
//  // - VECTOR_GYROSCOPE     - rad/s
//  // - VECTOR_EULER         - degrees
//  // - VECTOR_LINEARACCEL   - m/s^2
//  // - VECTOR_GRAVITY       - m/s^2
//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//
//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");
//
//  /*
//  // Quaternion data
//  imu::Quaternion quat = bno.getQuat();
//  Serial.print("qW: ");
//  Serial.print(quat.w(), 4);
//  Serial.print(" qX: ");
//  Serial.print(quat.x(), 4);
//  Serial.print(" qY: ");
//  Serial.print(quat.y(), 4);
//  Serial.print(" qZ: ");
//  Serial.print(quat.z(), 4);
//  Serial.print("\t\t");
//  */
//
//  /* Display calibration status for each sensor. */
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
//
//  delay(BNO055_SAMPLERATE_DELAY_MS);
}
