#include <mcp_can.h>
#include <SPI.h>

#define CAN_CS_PIN 10 // Chip Select pin for MCP2515

MCP_CAN CAN(CAN_CS_PIN); // Create CAN object

void setup() {
    Serial.begin(9600);

    // Initialize CAN
    if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN initialized successfully!");
    } else {
        Serial.println("CAN initialization failed!");
        while (1); // Halt if initialization fails
    }
    CAN.setMode(MCP_NORMAL); // Set CAN to normal mode
}

void loop() {
    // Define CAN message
    uint32_t tx_id = 0x100; // CAN ID
    uint8_t tx_len = 8;     // Data length (0–8 bytes)
    uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // Data

    // Send CAN message
    if (CAN.sendMsgBuf(tx_id, 0, tx_len, tx_data) == CAN_OK) {
        Serial.println("Message sent successfully!");
    } else {
        Serial.println("Message sending failed!");
    }

    delay(1000); // Send a message every second
}
