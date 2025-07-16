#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>        // For core Arduino types and functions
#include <IPAddress.h>      // For IPAddress type

// --- Broad Library Includes (as per your working setup) ---
// These are included here to resolve compilation issues in other modules,
// even if individual modules don't directly use all contents.
#include <WiFi.h>           // For WiFi.h components, often used by ETH.h
#include <ETH.h>            // For ETH.h specific types and functions
#include <PubSubClient.h>   // For PubSubClient class definition
#include <ArduinoJson.h>    // For ArduinoJson library (e.g., StaticJsonDocument)
#include <HardwareSerial.h> // For HardwareSerial class definition


// --- Button Timing and Debounce ---
const unsigned long DEBOUNCE_DELAY              = 50;
const unsigned long LONG_PRESS_THRESHOLD        = 200; 


// --- Button Logical States ---
enum ButtonLogicalStateMain {
    MAIN_OFF,
    MAIN_PRESSED,
    MAIN_HELD
};

const char* btnStateStrings[] = { // Mapping for JSON
    "OFF",
    "PRESSED",
    "HELD"
};


// --- Data Structure for Each Button's State Machine ---
struct ButtonStatusESP32 {
    ButtonLogicalStateMain state;
    unsigned long pressStartTime;
    unsigned long lastPhysicalChange;
    bool longPressEventSent;
};


// --- Polling Management Enum ---
enum PollingState { DISCOVERY_MODE, NORMAL_MODE };


// --- Timing for Polling/Discovery ---
const unsigned long NOMINAL_SINGLE_POLL_TRANSACTION_MS  = 10;
const unsigned long JITTER_BUFFER_MS                    = 200; 
const unsigned long MINIMUM_CALCULATED_TIMEOUT_MS       = 250; 
const unsigned long POLL_SLOT_DURATION_MS               = 20; 

const unsigned long FULL_DISCOVERY_RESCAN_INTERVAL_MS   = 60 * 1000;


// --- Global Flags and Objects (Defined in main.cpp, declared extern here) ---
// These are the main application-level objects and flags that represent global state
// or provide shared client services.
extern bool ethGotIP;             // Flag indicating Ethernet IP address has been obtained
extern WiFiClient ethClient;      // WiFiClient object used by PubSubClient for Ethernet connection
extern PubSubClient mqttClient;   // MQTT client object
extern HardwareSerial SerialRS485; // HardwareSerial object for RS-485 communication


// --- Global Variables for Polling State (Defined in rs485.cpp, declared extern here) ---
// These variables manage the RS-485 polling state machine and are defined within rs485.cpp
extern PollingState currentPollingState; 
extern bool discoveredDevices[16];
extern unsigned long lastSeenTimestamp[16];
extern int currentPollingTargetID;
extern unsigned long lastPollRequestSentTime;
extern unsigned long lastDiscoveryRescanTime;
extern bool rs485_packet_started;
extern byte rs485_recv_buffer[]; // Declared as array, size defined where it's defined (rs485.cpp)
extern byte rs485_recv_index;
extern bool responseReceivedForCurrentSlot; 
extern int lastPolledIDForTimeoutCheck; 


// --- Global Variables for Button State Management (Defined in rs485.cpp) ---
// These manage the logical state of buttons and are defined within rs485.cpp
extern ButtonStatusESP32 allDeviceButtonStates[16][12];
extern uint16_t lastRawMCPInputs[16];


#endif // COMMON_H