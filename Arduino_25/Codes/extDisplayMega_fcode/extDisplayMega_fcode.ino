
/*
 * Arduino Mega 2560 Dashboard - Semicircular Speed Gauge & SOC Bar
 * Hardware: 3.5" TFT LCD Shield (480x320) + IR RPM Sensor + SOC Input
 * 
 * Pin Configuration:
 * - IR RPM Sensor: A9 (Analog pin 9)
 * - SOC Input: A7 (Analog pin 7) 
 * - LED Indicator: Pin 13 (Built-in LED)
 * - TFT Shield connects via digital pins (standard MCUFRIEND shield)
 * 
 * Display Layout:
 * - Row 1 (2/3 of screen): Semicircular speedometer gauge (0-120 KMH)
 * - Row 2 (1/3 of screen): SOC horizontal bar display
 */

#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h>

#define DEMO_MODE  // Comment this line to use real sensors

// Pin definitions
const int RPM_PIN = A9;        // IR sensor for RPM (Analog pin 9)
const int SOC_PIN = A7;        // SOC input (Analog pin 7)
const int LED_PIN = 13;        // Built-in LED indicator

// TFT Display setup
MCUFRIEND_kbv tft;
uint16_t ID;

// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define ORANGE  0xFD20
#define DARKGREY 0x7BEF

// Display dimensions
const int16_t SCREEN_W = 480;
const int16_t SCREEN_H = 320;

// Layout dimensions (2:1 ratio - Row1: 213px, Row2: 107px)
const int16_t ROW1_H = (SCREEN_H * 2) / 3;  // 213 pixels for speed gauge
const int16_t ROW2_H = SCREEN_H - ROW1_H;   // 107 pixels for SOC

// Speedometer gauge parameters
const int16_t GAUGE_CENTER_X = SCREEN_W / 2;    // 240
const int16_t GAUGE_CENTER_Y = ROW1_H - 30;     // 183 (bottom of row 1)
const int16_t GAUGE_RADIUS = 140;               // Gauge radius
const int16_t GAUGE_INNER_RADIUS = 120;         // Inner radius for needle
const float DEG2RAD = 0.0174533;               // Degrees to radians conversion

// Speed parameters
const int MAX_SPEED = 120;                      // Maximum speed in KMH
const int SPEED_RANGE_START = 210;              // Start angle (degrees) - bottom left
const int SPEED_RANGE_END = 330;                // End angle (degrees) - bottom right

// Wheel specifications for RPM to KMH conversion
const float WHEEL_DIAMETER_CM = 50.8;  // Wheel diameter in cm (20 inch =50.8cm)
const float WHEEL_CIRCUMFERENCE_M = (WHEEL_DIAMETER_CM * 3.14159) / 100.0;  // Circumference in meters

// RPM measurement variables
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
unsigned long lastCalculationTime = 0;
float currentSpeed = 0.0;  // Speed in KMH

// SOC variables
int socPercent = 0;

// Display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 250;  // Update every 250ms

// Demo mode variables
#ifdef DEMO_MODE
unsigned long lastDemoUpdate = 0;
float demoSpeedTarget = 0;
int demoSocTarget = 50;
#endif

void setup() {
  Serial.begin(115200);

  // Initialize pins
  pinMode(RPM_PIN, INPUT);
  pinMode(SOC_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initialize TFT display
  ID = tft.readID();
  Serial.print("TFT ID = 0x");
  Serial.println(ID, HEX);

  tft.begin(ID);
  tft.setRotation(1);  // Landscape mode (480x320)
  tft.fillScreen(BLACK);

  drawStaticElements();

  Serial.println("Arduino Mega 2560 Dashboard with Semicircular Gauge Initialized");
  Serial.print("Wheel circumference: ");
  Serial.print(WHEEL_CIRCUMFERENCE_M);
  Serial.println(" meters");
}

void loop() {
  unsigned long currentTime = millis();

#ifdef DEMO_MODE
  // Demo mode - generate fake data
  if (currentTime - lastDemoUpdate >= 100) {
    lastDemoUpdate = currentTime;

    // Generate smooth speed changes (0-120 KMH)
    float speedChange = (sin(currentTime / 3000.0) + 1) * 60;  // 0-120 range
    currentSpeed = speedChange;

    // Generate SOC changes
    socPercent = 30 + (sin(currentTime / 8000.0) * 35) + 35;
    socPercent = constrain(socPercent, 0, 100);

    // Simulate LED activity
    if (currentSpeed > 5) {
      digitalWrite(LED_PIN, (currentTime / 300) % 2);  // Blink when moving
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }
#else
  // Real sensor mode
  readSensors();
#endif

  // Update display
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = currentTime;
    updateDisplay();
  }
}

#ifndef DEMO_MODE
void readSensors() {
  static unsigned long lastRpmRead = 0;
  static int lastRpmValue = 0;
  unsigned long currentTime = millis();

  // Read RPM sensor (IR sensor)
  int rpmValue = analogRead(RPM_PIN);

  // Detect pulse (when sensor value changes significantly)
  if (abs(rpmValue - lastRpmValue) > 100 && (currentTime - lastRpmRead) > 50) {
    pulseCount++;
    lastRpmRead = currentTime;
    digitalWrite(LED_PIN, HIGH);  // LED on when pulse detected
  }
  lastRpmValue = rpmValue;

  // Calculate speed every second
  if (currentTime - lastCalculationTime >= 1000) {
    // Calculate RPM (assuming 1 pulse per revolution)
    float rpm = pulseCount * 60.0;  // pulses per minute

    // Convert RPM to KMH
    // Formula: Speed (km/h) = RPM × wheel circumference (m) × 60 (min/h) / 1000 (m/km)
    currentSpeed = (rpm * WHEEL_CIRCUMFERENCE_M * 60.0) / 1000.0;

    // Reset for next calculation
    pulseCount = 0;
    lastCalculationTime = currentTime;

    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(", Speed: ");
    Serial.print(currentSpeed);
    Serial.println(" km/h");
  }

  // Read SOC sensor
  int socRaw = analogRead(SOC_PIN);
  socPercent = map(socRaw, 0, 1023, 0, 100);  // Convert to percentage
  socPercent = constrain(socPercent, 0, 100);

  // Turn off LED after brief period if no pulses
  static unsigned long ledOffTime = 0;
  if (digitalRead(LED_PIN) == HIGH) {
    if (ledOffTime == 0) ledOffTime = currentTime;
    if (currentTime - ledOffTime > 100) {
      digitalWrite(LED_PIN, LOW);
      ledOffTime = 0;
    }
  }
}
#endif

void drawStaticElements() {
  // Draw row divider
  tft.drawLine(0, ROW1_H, SCREEN_W, ROW1_H, WHITE);

  // Draw speedometer gauge background
  drawSpeedometerBackground();

  // Row 2 - SOC section labels
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(20, ROW1_H + 10);
  tft.print("BATTERY SOC");

  tft.setTextSize(2);
  tft.setCursor(SCREEN_W - 50, ROW1_H + 20);
  tft.print("%");
}

void drawSpeedometerBackground() {
  // Draw gauge outer circle
  tft.drawCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_RADIUS, WHITE);
  tft.drawCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_RADIUS - 1, WHITE);

  // Draw gauge arc (semicircle from 210° to 330°)
  drawGaugeArc(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_RADIUS - 5, 
               SPEED_RANGE_START, SPEED_RANGE_END, WHITE);

  // Draw speed markings and numbers
  drawSpeedMarkings();

  // Draw center point
  tft.fillCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, 8, WHITE);
  tft.fillCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, 6, BLACK);

  // Draw "KM/H" label
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(2);
  tft.setCursor(GAUGE_CENTER_X - 25, GAUGE_CENTER_Y + 30);
  tft.print("KM/H");
}

void drawGaugeArc(int16_t x, int16_t y, int16_t radius, int startAngle, int endAngle, uint16_t color) {
  // Draw arc using line segments
  for(int angle = startAngle; angle <= endAngle; angle += 2) {
    float rad = angle * DEG2RAD;
    int16_t x1 = x + cos(rad) * radius;
    int16_t y1 = y + sin(rad) * radius;
    tft.drawPixel(x1, y1, color);
  }
}

void drawSpeedMarkings() {
  // Draw major tick marks and numbers every 20 KMH
  for(int speed = 0; speed <= MAX_SPEED; speed += 20) {
    int angle = map(speed, 0, MAX_SPEED, SPEED_RANGE_START, SPEED_RANGE_END);
    float rad = angle * DEG2RAD;

    // Outer tick mark
    int16_t x1 = GAUGE_CENTER_X + cos(rad) * (GAUGE_RADIUS - 15);
    int16_t y1 = GAUGE_CENTER_Y + sin(rad) * (GAUGE_RADIUS - 15);
    int16_t x2 = GAUGE_CENTER_X + cos(rad) * (GAUGE_RADIUS - 25);
    int16_t y2 = GAUGE_CENTER_Y + sin(rad) * (GAUGE_RADIUS - 25);

    tft.drawLine(x1, y1, x2, y2, WHITE);

    // Speed numbers
    int16_t textX = GAUGE_CENTER_X + cos(rad) * (GAUGE_RADIUS - 40);
    int16_t textY = GAUGE_CENTER_Y + sin(rad) * (GAUGE_RADIUS - 40);

    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(1);
    tft.setCursor(textX - 6, textY - 4);
    tft.print(speed);
  }

  // Draw minor tick marks every 10 KMH
  for(int speed = 10; speed < MAX_SPEED; speed += 20) {
    int angle = map(speed, 0, MAX_SPEED, SPEED_RANGE_START, SPEED_RANGE_END);
    float rad = angle * DEG2RAD;

    int16_t x1 = GAUGE_CENTER_X + cos(rad) * (GAUGE_RADIUS - 15);
    int16_t y1 = GAUGE_CENTER_Y + sin(rad) * (GAUGE_RADIUS - 15);
    int16_t x2 = GAUGE_CENTER_X + cos(rad) * (GAUGE_RADIUS - 20);
    int16_t y2 = GAUGE_CENTER_Y + sin(rad) * (GAUGE_RADIUS - 20);

    tft.drawLine(x1, y1, x2, y2, DARKGREY);
  }
}

void updateDisplay() {
  // Update speedometer needle
  updateSpeedometerNeedle();

  // Update digital speed display
  updateDigitalSpeed();

  // Update SOC display (Row 2)
  updateSOCDisplay();
}

void updateSpeedometerNeedle() {
  static float lastNeedleAngle = SPEED_RANGE_START;

  // Calculate needle angle based on current speed
  float needleAngle = map(constrain(currentSpeed, 0, MAX_SPEED), 0, MAX_SPEED, 
                         SPEED_RANGE_START, SPEED_RANGE_END);

  // Erase old needle (draw in black)
  drawNeedle(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_INNER_RADIUS, lastNeedleAngle, BLACK);

  // Draw new needle
  uint16_t needleColor = GREEN;
  if (currentSpeed > 60) needleColor = YELLOW;
  if (currentSpeed > 90) needleColor = ORANGE;
  if (currentSpeed > 110) needleColor = RED;

  drawNeedle(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_INNER_RADIUS, needleAngle, needleColor);

  // Redraw center point
  tft.fillCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, 8, WHITE);
  tft.fillCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, 6, BLACK);

  lastNeedleAngle = needleAngle;
}

void drawNeedle(int16_t x, int16_t y, int16_t length, float angle, uint16_t color) {
  float rad = angle * DEG2RAD;
  int16_t x1 = x + cos(rad) * length;
  int16_t y1 = y + sin(rad) * length;

  // Draw needle as a thick line
  tft.drawLine(x, y, x1, y1, color);
  tft.drawLine(x-1, y, x1-1, y1, color);
  tft.drawLine(x+1, y, x1+1, y1, color);
  tft.drawLine(x, y-1, x1, y1-1, color);
  tft.drawLine(x, y+1, x1, y1+1, color);
}

void updateDigitalSpeed() {
  // Clear digital speed area
  tft.fillRect(GAUGE_CENTER_X - 40, GAUGE_CENTER_Y - 20, 80, 25, BLACK);

  // Display current speed digitally in center of gauge
  tft.setTextColor(CYAN, BLACK);
  tft.setTextSize(3);

  String speedStr = String(currentSpeed, 1);  // 1 decimal place
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(speedStr, 0, 0, &x1, &y1, &w, &h);

  int centerX = GAUGE_CENTER_X - w/2;
  int centerY = GAUGE_CENTER_Y - 10;

  tft.setCursor(centerX, centerY);
  tft.print(speedStr);
}

void updateSOCDisplay() {
  const int BAR_X = 50;
  const int BAR_Y = ROW1_H + 50;
  const int BAR_W = SCREEN_W - 150;
  const int BAR_H = 30;

  // Clear SOC value area
  tft.fillRect(SCREEN_W - 100, ROW1_H + 45, 80, 40, BLACK);

  // Display SOC percentage
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(4);
  tft.setCursor(SCREEN_W - 90, ROW1_H + 50);
  tft.print(socPercent);

  // Clear bar area
  tft.fillRect(BAR_X, BAR_Y, BAR_W, BAR_H, BLACK);

  // Draw bar outline
  tft.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, WHITE);

  // Fill bar based on SOC percentage
  int fillWidth = map(socPercent, 0, 100, 0, BAR_W - 2);

  // Color based on SOC level
  uint16_t barColor = RED;
  if (socPercent > 20) barColor = ORANGE;
  if (socPercent > 40) barColor = YELLOW;
  if (socPercent > 60) barColor = GREEN;

  tft.fillRect(BAR_X + 1, BAR_Y + 1, fillWidth, BAR_H - 2, barColor);

  // Add SOC level indicators
  drawSOCIndicators(BAR_X, BAR_Y, BAR_W, BAR_H);
}

void drawSOCIndicators(int x, int y, int w, int h) {
  // Draw level indicators at 25%, 50%, 75%
  int mark25 = x + (w * 25) / 100;
  int mark50 = x + (w * 50) / 100;
  int mark75 = x + (w * 75) / 100;

  tft.drawLine(mark25, y, mark25, y + h, WHITE);
  tft.drawLine(mark50, y, mark50, y + h, WHITE);
  tft.drawLine(mark75, y, mark75, y + h, WHITE);

  // Add small labels
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(mark25 - 5, y + h + 5);
  tft.print("25");
  tft.setCursor(mark50 - 5, y + h + 5);
  tft.print("50");
  tft.setCursor(mark75 - 5, y + h + 5);
  tft.print("75");
}
