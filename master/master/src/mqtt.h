#ifndef MQTT_H
#define MQTT_H

#include "common.h" // Includes Arduino.h, IPAddress.h, common enums/structs

// --- MQTT Configuration Constants (Specific to this module) ---
const char* MQTT_HOST                   = "mqtt.iot";
const int   MQTT_PORT                   = 1883;
const char* MQTT_CLIENT_ID              = "esp32_buttons"; // Hostname will also be set to this
const char* MQTT_LWT_TOPIC              = "tele/esp32_buttons/LWT";
const char* MQTT_LWT_MESSAGE_OFFLINE    = "Offline";
const char* MQTT_CMD_RESTART_TOPIC      = "cmnd/esp32_buttons/RESTART";
const char* MQTT_STATUS_TOPIC           = "stat/esp32_buttons/STATUS"; // For general status, discovery, and errors
const char* MQTT_DATA_TOPIC             = "sensors/buttons/events"; // For button data

// --- MQTT Periodic Status Variables (moved from global scope in main) ---
const unsigned long STATUS_PUBLISH_INTERVAL_MS = 60 * 1000; // 60 seconds


// --- Function Prototypes for MQTT Module ---
// These functions are implemented in mqtt.cpp
void mqtt_setup(); // Initializes MQTT client settings
void mqtt_loop(); // Handles mqttClient.loop() and periodic publishes
void mqtt_callback(char* topic, byte* payload, unsigned int length); // Callback for PubSubClient
void mqtt_reconnect(); // Manages MQTT connection/reconnection
void mqtt_disconnect(); // Disconnects MQTT client
void mqtt_publishLWT(const char* message, bool retained); // Publishes LWT message
void mqtt_publishIPStatus(); // Publishes device's IP status
void mqtt_publishDeviceStatus(byte deviceID, const char* status); // Publishes device connection status
void mqtt_publishButtonState(byte deviceID); // Publishes button states


#endif // MQTT_H