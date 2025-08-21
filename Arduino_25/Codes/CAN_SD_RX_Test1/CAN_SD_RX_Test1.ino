#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>

#define CAN_CS_PIN 10  // Chip Select pin for MCP2515
#define SD_CS_PIN 4   // Chip Select pin for SD card
//#define SHUTDOWN_BUTTON_PIN 2 // Button for safe shutdown

MCP_CAN CAN(CAN_CS_PIN); // Create CAN object
//bool shutdownRequested = false; // Flag for shutdown

void setup() {
    Serial.begin(9600);

    // Initialize CAN
    uint8_t attempt = 3;
    while(attempt){
      if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
          Serial.println("CAN initialized successfully!");
      } else {
          Serial.println("CAN initialization failed!");
          delay(500); // Halt if initialization fails
      }
      attempt-=1;
    }
    CAN.setMode(MCP_NORMAL); // Set CAN to normal mode

    // Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1); // Halt if SD card initialization fails
    }
    Serial.println("SD card initialized successfully!");

//    // Initialize shutdown button
//    pinMode(SHUTDOWN_BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
    // Check if shutdown button is pressed
//    if (digitalRead(SHUTDOWN_BUTTON_PIN) == LOW) {
//        shutdownRequested = true;
//        Serial.println("Shutdown requested. Stopping CAN communication and closing files...");
//    }

    // Check if a CAN message is available
    if (/*!shutdownRequested &&*/ CAN.checkReceive() == CAN_MSGAVAIL) { // uncomment for shutdown button
        uint32_t rx_id;
        uint8_t rx_ext, rx_len;
        uint8_t rx_data[8];

        // Read CAN message
        if (CAN.readMsgBuf(&rx_id, &rx_ext, &rx_len, rx_data) == CAN_OK) {
            // Print received message to Serial Monitor
            Serial.print("Received CAN ID: 0x");
            Serial.print(rx_id, HEX);
            Serial.print(", Data: ");
            for (int i = 0; i < rx_len; i++) {
                Serial.print(rx_data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // Preprocess data (example: convert data to a string)
            char data_str[50];
            sprintf(data_str, "ID: 0x%X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                    rx_id, rx_data[0], rx_data[1], rx_data[2], rx_data[3],
                    rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

            // Store data on SD card
            File file = SD.open("can_log.txt", FILE_WRITE);
            if (file) {
                file.println(data_str);
                file.close();
                Serial.println("Data written to SD card!");
            } else {
                Serial.println("Error opening file on SD card!");
            }
        }
    }

//    // If shutdown is requested, stop the loop
//    if (shutdownRequested) {
//        while (1); // Halt the program
//    }
}
