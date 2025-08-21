#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>      // Core graphics library
#include <MCUFRIEND_kbv.h>     // Hardware-specific library

static const int RXPin = 18, TXPin = 19;
static const uint32_t GPSBaud = 9600;

const int hall_pin = 52;
unsigned long hall_thresh = 500;
unsigned long timeout_thresh = 20000000; //(in microseconds);

#define BLACK   0x0000
#define WHITE   0xFFFF
#define YELLOW  0xFFE0
#define RED     0xF800

SoftwareSerial ss(RXPin, TXPin);
MCUFRIEND_kbv tft;

int x_max = 480;  // TFT display width
int y_max = 320;  // TFT display height
int x_center = x_max / 2;
int y_center = y_max / 2;

float speed_kmh = 0;  // Speed in km/h
float last_speed_kmh = -1;
float rpm = 0;

float total_distance = 0.0;  // Total distance in km
float last_latitude = 0.0, last_longitude = 0.0;

float SOC = 100.0;  // State of Charge (%) - can range from 0 to 100

void setup() {
    Serial.begin(115200);  // Initialize serial monitor
    Serial1.begin(9600);   // GPS module
    Serial2.begin(9600);   // Hall effect sensor

    pinMode(hall_pin, INPUT);

    uint16_t ID = tft.readID();
    if (ID == 0xD3D3) ID = 0x9481;  // Handling specific displays
    tft.begin(ID);

    tft.setRotation(3);  // Landscape orientation
    tft.fillScreen(BLACK);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);

    drawSOCBar(SOC);  // Initial SOC display
    updateDistanceDisplay();
    updateSpeedDisplay(0);
    updateRPM();
}

void loop() {
    static String sentence = "";

    while (Serial1.available() > 0) {
        char c = Serial1.read();

        if (c == '\n') {
            sentence.trim();
            if (sentence.startsWith("$GNRMC")) {
                parseGNRMC(sentence);
            }
            sentence = "";
        } else {
            sentence += c;
        }
    }

    RPM();
    updateRPM();

    // Simulate SOC decrease for demonstration (remove in real use)
    SOC -= 0.1;
    if (SOC < 0) SOC = 100;  // Reset to 100 when it reaches 0
    drawSOCBar(SOC);  // Update the SOC bar based on the new value
}

void drawSOCBar(float soc) {
    int barWidth = map(soc, 0, 100, 0, x_max);  // Map SOC % to screen width
    uint16_t color = tft.color565(
        map(soc, 0, 100, 255, 255),  // Red stays constant
        map(soc, 0, 100, 0, 255),    // Green decreases as SOC drops
        0);                          // Blue = 0 for yellow-red gradient

    tft.fillRect(0, 0, x_max, 30, BLACK);  // Clear previous bar
    tft.fillRect(0, 0, barWidth, 30, color);  // Draw new SOC bar
    tft.setCursor(10, 5);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("SOC: ");
    tft.print(soc, 1);  // Display SOC value with 1 decimal place
    tft.print("%");
}

void parseGNRMC(String sentence) {
    int commaIndex1 = sentence.indexOf(',');
    int commaIndex2;
    int fieldIndex = 1;

    String latStr, lonStr;
    char latHem, lonHem;

    while (commaIndex1 != -1) {
        commaIndex2 = sentence.indexOf(',', commaIndex1 + 1);
        String field = sentence.substring(commaIndex1 + 1, commaIndex2);

        if (fieldIndex == 3) latStr = field;
        else if (fieldIndex == 4) latHem = field.charAt(0);
        else if (fieldIndex == 5) lonStr = field;
        else if (fieldIndex == 6) lonHem = field.charAt(0);
        else if (fieldIndex == 7) {
            float speed_knots = field.toFloat();
            speed_kmh = speed_knots * 1.852;

            if (speed_kmh != last_speed_kmh) {
                updateSpeedDisplay(speed_kmh);
                last_speed_kmh = speed_kmh;
            }
        }

        commaIndex1 = commaIndex2;
        fieldIndex++;
    }

    if (latStr.length() > 0 && lonStr.length() > 0) {
        float latitude = convertToDegrees(latStr.toFloat(), latHem);
        float longitude = convertToDegrees(lonStr.toFloat(), lonHem);

        if (last_latitude != 0.0 && last_longitude != 0.0) {
            total_distance += calculateDistance(last_latitude, last_longitude, latitude, longitude);
            updateDistanceDisplay();
        }

        last_latitude = latitude;
        last_longitude = longitude;
    }
}

void updateSpeedDisplay(float speed) {
    tft.fillRect(x_center - 150, y_center - 40, 350, 100, BLACK);  // Clear previous speed
    tft.setCursor(x_center - 150, y_center - 40);
    tft.setTextSize(10);
    tft.print(speed, 2);
    tft.setTextSize(3);
    tft.print(" km/h");
}

void updateDistanceDisplay() {
    tft.fillRect(x_max - 200, y_max - 40, 200, 50, BLACK);  // Clear previous distance
    tft.setCursor(x_max - 200, y_max - 40);
    tft.setTextSize(6);
    tft.print(total_distance, 2);
    tft.setTextSize(3);
    tft.print(" km");
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float rad_lat1 = radians(lat1);
    float rad_lon1 = radians(lon1);
    float rad_lat2 = radians(lat2);
    float rad_lon2 = radians(lon2);

    float dlon = rad_lon2 - rad_lon1;
    float dlat = rad_lat2 - rad_lat1;

    float a = pow(sin(dlat / 2), 2) + cos(rad_lat1) * cos(rad_lat2) * pow(sin(dlon / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371.0 * c;  // Earth's radius in km
}

void RPM() {
    unsigned long hall_sensor_val = pulseIn(hall_pin, LOW, timeout_thresh);
    if (hall_sensor_val == 0) rpm = 0;
    else rpm = 6000000.0 / hall_sensor_val;
}

void updateRPM() {
    tft.fillRect(10, y_max - 60, 200, 70, BLACK);  // Clear previous RPM
    tft.setCursor(10, y_max - 40);
    tft.setTextSize(6);
    tft.print(rpm);
    tft.setTextSize(3);
    tft.print(" RPM");
}

float convertToDegrees(float value, char hemisphere) {
    int degrees = int(value / 100);
    float minutes = value - (degrees * 100);
    float result = degrees + minutes / 60.0;
    return (hemisphere == 'S' || hemisphere == 'W') ? -result : result;
}
