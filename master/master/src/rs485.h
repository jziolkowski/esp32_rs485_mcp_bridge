#ifndef RS485_H
#define RS485_H

#include "common.h" // Include common definitions (Arduino.h, IPAddress.h, shared enums/structs)

// --- RS-485 Hardware Pin Configuration ---
#define RS485_RX_PIN            5
#define RS485_TX_PIN            17
#define RS485_DE_PIN            32
#define RS485_NRE_PIN           33
#define RS485_BAUD_RATE         57600


// --- RS-485 Protocol Defines (Master requests & Slave responses) ---
#define MASTER_POLL_START_BYTE          0xBB
#define MASTER_ID                       0x00 // ESP32's own ID on the bus
#define REQ_STATUS_POLL                 0x01 // Request for button status data
#define REQ_DISCOVERY_POLL              0x02 // Request for device presence/ACK

#define RESP_STATUS_DATA                0xAA // Start byte for status data response
#define RESP_DISCOVERY_ACK              0xCC // Start byte for discovery ACK response

#define MASTER_REQUEST_PACKET_LENGTH    5 // StartByte + MasterID + RequestType + TargetID + Checksum
#define SLAVE_STATUS_RESPONSE_LENGTH    5 // StartByte + DeviceID + RawMCPHigh + RawMCPLow + Checksum
#define SLAVE_ACK_RESPONSE_LENGTH       4 // StartByte + DeviceID + AckType + Checksum


// --- Global Variables for Polling State (Defined in rs485.cpp, declared extern here) ---
// These variables manage the RS-485 polling state machine.
extern PollingState currentPollingState; 
extern bool discoveredDevices[16];
extern unsigned long lastSeenTimestamp[16];
extern int currentPollingTargetID;
extern unsigned long lastPollRequestSentTime;
extern unsigned long lastDiscoveryRescanTime;

// RS-485 Receive Buffer (Managed by RS-485 Polling functions)
extern byte rs485_recv_buffer[SLAVE_STATUS_RESPONSE_LENGTH]; // Defined with max length
extern byte rs485_recv_index;
extern bool rs485_packet_started;

// Flags for "immediate hop" optimization
extern bool responseReceivedForCurrentSlot; 
extern int lastPolledIDForTimeoutCheck; 


// --- Global Variables for Button State Management (Defined in rs485.cpp) ---
// These manage the logical state of buttons.
extern ButtonStatusESP32 allDeviceButtonStates[16][12];
extern uint16_t lastRawMCPInputs[16];


// --- Function Prototypes for RS-485 Polling Module ---
// These functions are implemented in rs485.cpp
void rs485_polling_setup();     // Initializes RS-485 UART and polling state
void rs485_polling_loop();      // Handles the main polling state machine loop logic

void rs485_sendPollRequest(byte requestType, byte targetDeviceID); // Sends a poll request packet
void rs485_processIncomingData();                               // Processes incoming RS-485 bytes
void rs485_processSlaveStatusResponse(byte deviceID, uint16_t rawMCPInputs); // Handles parsed status response
void rs485_processSlaveDiscoveryAck(byte deviceID);               // Handles parsed discovery ACK
void rs485_runButtonStateMachine(byte deviceID, byte buttonIndex, bool currentPhysicalPressed); // Helper for button state transitions


#endif // RS485_H