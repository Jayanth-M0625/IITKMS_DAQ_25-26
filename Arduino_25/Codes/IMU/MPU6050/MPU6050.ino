#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

// RX, TX pins for HC-05 Bluetooth module
SoftwareSerial bluetooth(51, 50);

// Variables to store decoded values
int joystickX = 0;
int joystickY = 0;
int forwardBackwardOffset = 0;
int turnOffset = 0;
String receivedString = ""; // For storing incoming Bluetooth data

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = 183.5;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

double Kp = 25;
double Kd = 1.7;
double Ki = 270;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.75;
double motorSpeedFactorRight = 0.48;

// MOTOR CONTROLLER
#define MOTOR1_ENABLE_PIN A0
#define MOTOR2_ENABLE_PIN A1
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 4
#define ENA 5
#define ENB 6

int currentSpeed = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Initializing..."));

    bluetooth.begin(9600);
    Serial.println("Bluetooth receiver ready");

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24;
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255); 
        Serial.println(F("DMP ready!"));
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR2_ENABLE_PIN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    digitalWrite(MOTOR1_ENABLE_PIN, HIGH);
    digitalWrite(MOTOR2_ENABLE_PIN, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    Serial.println(F("Setup complete."));
}

void processBluetoothData() {
    int commaIndex = receivedString.indexOf(',');
    if (commaIndex > 0) {
        String xString = receivedString.substring(0, commaIndex);
        String yString = receivedString.substring(commaIndex + 1);

        joystickX = xString.toInt();
        joystickY = yString.toInt();

        movingAngleOffset = (float)joystickY * 1.9/ 255;
        turnOffset = (float)joystickX * 30 / 255;
        setpoint = originalSetpoint + movingAngleOffset;

//        Serial.print("Received: X=");
//        Serial.print(joystickX);
//        Serial.print(", Y=");
//        Serial.println(joystickY);
    }
}

void move(int balanceSpeed, int minAbsSpeed) {
    int leftSpeed = balanceSpeed + turnOffset;
    int rightSpeed = balanceSpeed - turnOffset;

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);
    digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);

    analogWrite(ENA, abs(leftSpeed) * motorSpeedFactorLeft);
    analogWrite(ENB, abs(rightSpeed) * motorSpeedFactorRight);
}

void loop() {
    while (bluetooth.available() > 0) {
        char c = bluetooth.read();
        if (c == '.') {
            processBluetoothData();
            receivedString = "";
        } else {
            receivedString += c;
        }
    }

    if (!dmpReady) return;

    if (mpuInterrupt || fifoCount >= packetSize) {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        } else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            input = ypr[1] * 180/M_PI + 180;

//            pid.Compute();
            move(output, MIN_ABS_SPEED);
        }
    }
    Serial.print("Yaw: ");
    Serial.print(ypr[0] * 180/M_PI);  // convert from radians to degrees
    Serial.print("  Pitch: ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("  Roll: ");
    Serial.println(ypr[2] * 180/M_PI);

}
