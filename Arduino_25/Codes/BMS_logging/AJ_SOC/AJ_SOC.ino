#include <mcp_can.h>
#include <SPI.h>

const int CAN_CS_PIN = 10;
MCP_CAN CAN(CAN_CS_PIN);   
const unsigned long REQUEST_ID = 0x7E0;  // BMS Request ID
const unsigned long RESPONSE_ID = 0x7E8; // BMS Response ID

void setup() {
    Serial.begin(115200);
    while (!Serial);

    if (CAN.begin(MCP_ANY, 500000, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN BUS Initialized Successfully");
    } else {
        Serial.println("CAN BUS Initialization Failed");
        while (1);
    }

    CAN.setMode(MCP_NORMAL);  
    Serial.println("CAN BUS in Normal Mode");
}

void loop() {
    unsigned char socRequest[8] = {0x02, 0x22, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00}; // Request PID 0xF00F
    if (CAN.sendMsgBuf(REQUEST_ID, 0, 8, socRequest) == CAN_OK) {
        Serial.println("SOC Request Sent");
    } else {
        Serial.println("Error Sending SOC Request");
    }

    delay(100); // Wait for the response

    // Check for a response
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
        unsigned char len = 0;
        unsigned char buf[8];
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();

        if (canId == RESPONSE_ID) { // Ensure it’s the correct response ID
            if (buf[2] == 0xF0 && buf[3] == 0x0F) { // Match PID for SOC
                int rawSOC = buf[4];  // SOC is in the 5th byte
                float soc = rawSOC * 0.5;  // Apply scaling
                Serial.print("State of Charge: ");
                Serial.print(soc);
                Serial.println(" %");
            } else {
                Serial.println("Unexpected PID in response");
            }
        }
    }

    delay(1000); 
}
