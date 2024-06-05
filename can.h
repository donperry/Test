#pragma once
#include <FlexCAN_T4.h>
//#include "frostbyteSettings.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Flags to track request status
uint8_t sent_rpm_rq = 0;
uint8_t sent_map_rq = 0;
uint8_t sent_iat_rq = 0;
uint8_t sent_maf_rq = 0;
elapsedMillis canReadTimer = 0;
void CANUpdateRead();
void sendRequest(uint8_t pid);
void handleRPMResponse(const CAN_message_t& msg);
void handleMAPResponse(const CAN_message_t& msg);
void handleIATResponse(const CAN_message_t& msg);
void handleMAFResponse(const CAN_message_t& msg);

struct CANbusDATA {
    short rpmCAN;
    short mapCAN;
    short iatCAN;
    float mafCAN; // MAF value in g/s
} canData;

void CANUpdateRead() {
    // Send requests if not already sent
    if (!sent_rpm_rq) {
        sendRequest(settings.RPMAddress); // PID for RPM
        sent_rpm_rq = 1;
    }

    if (!sent_map_rq) {
        sendRequest(settings.MAPAddress); // PID for MAP
        sent_map_rq = 1;
    }

    if (!sent_iat_rq) {
        sendRequest(settings.IATAddress); // PID for IAT
        sent_iat_rq = 1;
    }

    if (!sent_maf_rq) {
        sendRequest(0x10); // PID for MAF
        sent_maf_rq = 1;
    }

    // Read incoming messages and handle responses
    CAN_message_t inMsg;
    if (can1.read(inMsg)) {
        if (inMsg.id == 0x7E8) { // Response ID for ECU
            if (inMsg.buf[2] == settings.RPMAddress) { // RPM
                handleRPMResponse(inMsg);
                sent_rpm_rq = 0;
            }
            else if (inMsg.buf[2] == settings.MAPAddress) { // MAP
                handleMAPResponse(inMsg);
                sent_map_rq = 0;
            }
            else if (inMsg.buf[2] == settings.IATAddress) { // IAT
                handleIATResponse(inMsg);
                sent_iat_rq = 0;
            }
            else if (inMsg.buf[2] == 0x10) { // MAF
                handleMAFResponse(inMsg);
                sent_maf_rq = 0;
            }
        }
    }
    /*
    Serial.println("IAT:\t" + String(canData.iatCAN));
    Serial.println("MAP:\t" + String(canData.mapCAN));
    Serial.println("RPM:\t" + String(canData.rpmCAN));
    Serial.println("MAF:\t" + String(canData.mafCAN));

    Serial.println();
    */

    // delay(10); // Add a small delay to not overload the CAN bus
}

void sendRequest(uint8_t pid) {
    CAN_message_t msg;
    msg.id = 0x7DF; // Standard ID for OBD-II request
    msg.len = 8;
    msg.buf[0] = 0x02; // Number of additional bytes
    msg.buf[1] = 0x01; // Show current data
    msg.buf[2] = pid;  // PID
    for (int i = 3; i < 8; i++) {
        msg.buf[i] = 0x00;
    }
    can1.write(msg);
}

void handleRPMResponse(const CAN_message_t& msg) {
    uint16_t rpm = ((uint16_t)msg.buf[3] << 8) | msg.buf[4];
    rpm = rpm / 4; // The formula to calculate RPM from the response
    canData.rpmCAN = rpm; // Store the RPM value in the struct
}

void handleMAPResponse(const CAN_message_t& msg) {
    int mapValue = msg.buf[3]; // MAP value is a single byte
    canData.mapCAN = mapValue; // Store the MAP value in the struct
}

void handleIATResponse(const CAN_message_t& msg) {
    int iatValue = msg.buf[3]; // IAT value is a single byte
    canData.iatCAN = iatValue; // Store the IAT value in the struct
}

void handleMAFResponse(const CAN_message_t& msg) {
    uint16_t mafRaw = ((uint16_t)msg.buf[3] << 8) | msg.buf[4];
    float maf = mafRaw / 100.0; // The formula to calculate MAF in g/s from the response
    canData.mafCAN = maf; // Store the MAF value in the struct
}
