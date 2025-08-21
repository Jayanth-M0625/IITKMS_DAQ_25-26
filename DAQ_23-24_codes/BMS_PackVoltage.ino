#include <mcp_can.h>
#include <SPI.h>

const int CAN_CS_PIN = 10;
MCP_CAN CAN(CAN_CS_PIN); 

const unsigned long REQUEST_ID = 0x7E0;
const unsigned long RESPONSE_ID = 0x7E8;

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
    unsigned char voltageRequest[8] = {0x02, 0x22, 0xF0, 0x0D, 0x00, 0x00, 0x00, 0x00};
    if (CAN.sendMsgBuf(REQUEST_ID, 0, 8, voltageRequest) == CAN_OK) {
        Serial.println("Pack Voltage Request Sent");
    } else {
        Serial.println("Error Sending Pack Voltage Request");
    }

    delay(100);

    if (CAN.checkReceive() == CAN_MSGAVAIL) {
        unsigned char len = 0;
        unsigned char buf[8];
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();

        if (canId == RESPONSE_ID) { 
            if (buf[2] == 0xF0 && buf[3] == 0x0D) { 
                int rawVoltage = (buf[4] << 8) | buf[5];
                float voltage = rawVoltage * 0.1; 
                Serial.print("Pack Voltage: ");
                Serial.print(voltage);
                Serial.println(" V");
            } else {
                Serial.println("Unexpected PID in response");
            }
        }
    }
    delay(10000);
}