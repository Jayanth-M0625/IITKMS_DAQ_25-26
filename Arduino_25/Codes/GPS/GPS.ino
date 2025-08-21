/*   
modified on Sep 27, 2020
Modified by MohammedDamirchi from https://github.com/mikalhart/TinyGPSPlus
Home 
*/ 

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

/*
   This sample demonstrates TinyGPS++'s capacity for extracting custom
   fields from any NMEA sentence.  TinyGPS++ has built-in facilities for
   extracting latitude, longitude, altitude, etc., from the $GPGLL and
   $GPRMC sentences.  But with the TinyGPSCustom type, you can extract
   other NMEA fields, even from non-standard NMEA sentences.

   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 4(RX) and 3(TX).
*/
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/*
   By declaring TinyGPSCustom objects like this, we announce that we
   are interested in the 15th, 16th, and 17th fields in the $GPGSA
   sentence, respectively the PDOP (F("positional dilution of precision")),
   HDOP (F("horizontal...")), and VDOP (F("vertical...")).

   (Counting starts with the field immediately following the sentence name,
   i.e. $GPGSA.  For more information on NMEA sentences, consult your
   GPS module's documentation and/or http://aprs.gids.nl/nmea/.)

   If your GPS module doesn't support the $GPGSA sentence, then you
   won't get any output from this program.
*/

TinyGPSCustom pdop(gps, "GNGLL", 1); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GNGLL", 3); // $GPGSA sentence, 16th element

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Serial.println(F("UsingCustomFields.ino"));
  Serial.println(F("Demonstrating how to extract any NMEA field using TinyGPSCustom"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void loop()
{
  Serial.print(F(" LAT=")); Serial.print(atof(pdop.value())/100,7);
  Serial.print(F("\tLON=")); Serial.println(atof(hdop.value())/100,7);
  delay(100);

  while (ss.available() > 0)
    gps.encode(ss.read());



  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

// Split a string into parts
int splitString(String data, char delimiter, String* parts, int maxParts) {
  int count = 0;
  int index = 0;
  while (data.indexOf(delimiter) != -1 && count < maxParts) {
    index = data.indexOf(delimiter);
    parts[count++] = data.substring(0, index);
    data = data.substring(index + 1);
  }
  parts[count++] = data;
  return count;
}

// Parse GPGGA Sentence
void parseGPGGA(String* parts, int numParts) {
  if (numParts < 15) return; // Basic validation
  
  String timeUTC = parts[1];
  String latitude = parts[2];
  String latDir = parts[3];
  String longitude = parts[4];
  String lonDir = parts[5];
  String fixQuality = parts[6];
  String satellites = parts[7];
  String altitude = parts[9];

  Serial.println("--- GPGGA Data ---");
  Serial.print("Time (UTC): "); Serial.println(timeUTC);
  Serial.print("Latitude: "); Serial.println(latitude + latDir);
  Serial.print("Longitude: "); Serial.println(longitude + lonDir);
  Serial.print("Fix Quality: "); Serial.println(fixQuality);
  Serial.print("Satellites: "); Serial.println(satellites);
  Serial.print("Altitude: "); Serial.println(altitude + " meters");
}

// Parse GPRMC Sentence
void parseGPRMC(String* parts, int numParts) {
  if (numParts < 12) return;
  
  String timeUTC = parts[1];
  String status = parts[2];
  String latitude = parts[3];
  String latDir = parts[4];
  String longitude = parts[5];
  String lonDir = parts[6];
  String speedKnots = parts[7];
  String date = parts[9];

  Serial.println("--- GPRMC Data ---");
  Serial.print("Status: "); Serial.println(status);
  Serial.print("Speed (knots): "); Serial.println(speedKnots);
  Serial.print("Date: "); Serial.println(date);
}

// Optional: Checksum Validation
bool validateChecksum(String sentence) {
  int asteriskIndex = sentence.indexOf('*');
  if (asteriskIndex == -1) return false;
  
  String checksum = sentence.substring(asteriskIndex + 1);
  sentence = sentence.substring(0, asteriskIndex);
  
  byte calculated = 0;
  for (char c : sentence) {
    if (c == '$') continue;
    calculated ^= c;
  }
  
  return (checksum.equalsIgnoreCase(String(calculated, HEX)));
}
