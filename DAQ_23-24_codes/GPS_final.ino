#include <SoftwareSerial.h>

static const int RXPin = 18, TXPin = 19;
static const uint32_t GPSBaud = 9600;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
String receivedLine = "";

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  Serial.println("GPS GNRMC Data");
}

void loop() {
  while (ss.available() > 0) {
    char c = ss.read();
    if (c == '\n') { // End of line
      if (receivedLine.startsWith("$GNRMC")) {
        Serial.println(receivedLine);
        parseGNRMC(receivedLine); // Print the GNRMC line
      }
      receivedLine = ""; // Clear the line buffer
    } else {
      receivedLine += c; // Add character to line buffer
    }
  }
}
void parseGNRMC(String sentence) {
  // Split the sentence into fields
  int commaIndex1 = sentence.indexOf(',');
  int commaIndex2;
  int fieldIndex = 1;

  while (commaIndex1 != -1 && fieldIndex <= 7) {
    commaIndex2 = sentence.indexOf(',', commaIndex1 + 1);
    String field = sentence.substring(commaIndex1 + 1, commaIndex2);
    
    if (fieldIndex == 7) {
      float speed_knots = field.toFloat();
      float speed_kmh = speed_knots * 1.852;
      float speed_ms = speed_knots * 0.51444;

      Serial.print("Speed: ");
      Serial.print(speed_knots);
      Serial.print(" knots, ");
      Serial.print(speed_kmh);
      Serial.print(" km/h, ");
      Serial.print(speed_ms);
      Serial.println(" m/s");
    }
    commaIndex1 = commaIndex2;
    fieldIndex++;}}
