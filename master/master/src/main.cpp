#include "common.h" // Include common definitions, which includes Arduino.h and all core library headers
#include "ethernet.h" // Include header for Ethernet module
#include "mqtt.h"     // Include header for MQTT module
#include "rs485.h"    // Include header for RS-485 polling module


// --- Global Flags and Objects (Defined here, extern declared in common.h) ---
bool ethGotIP = false;
WiFiClient ethClient;
PubSubClient mqttClient(ethClient);
HardwareSerial SerialRS485(2);


// --- Function Prototypes for Module Setup/Loop ---
// These are functions called by main.cpp from other modules.
void ethernet_setup();
void mqtt_setup();
void mqtt_loop();
void rs485_polling_setup();
void rs485_polling_loop();


void setup() {
  Serial.begin(115200);
  delay(1000);
  while (!Serial && millis() < 5000) {
    // Wait for serial monitor or 5 seconds
  }

  Serial.println("\n--- ESP32-ETH01 System Start (Refactored) ---");

  // --- Set Device Hostname ---
  WiFi.setHostname(MQTT_CLIENT_ID);
  Serial.printf("Setting hostname to: %s\n", MQTT_CLIENT_ID);

  // --- Disable Wi-Fi to dedicate resources to Ethernet ---
  WiFi.mode(WIFI_OFF);
  btStop();

  // --- Setup Ethernet (now includes event registration internally) ---
  ethernet_setup(); // Call setup function from Ethernet module

  // --- Setup MQTT Client ---
  mqtt_setup(); // Call setup function from MQTT Handler module

  // --- Setup RS-485 Polling ---
  rs485_polling_setup(); // Call setup function from RS-485 Polling module

  Serial.println("--- Setup Complete. Waiting for IP address and MQTT ---");
}


void loop() {
  // --- MQTT Client Loop ---
  mqtt_loop(); 

  // --- RS-485 Master Polling Logic ---
  rs485_polling_loop(); 
}