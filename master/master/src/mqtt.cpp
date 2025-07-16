#include "mqtt.h"   // Include the header for this module (contains prototypes and MQTT constants)
#include "common.h" // Include common definitions (contains externs for global objects like mqttClient, ethGotIP, SerialRS485)


// --- MQTT Module Implementation ---

// Initializes MQTT client settings
void mqtt_setup() {
  Serial.println("Configuring MQTT client...");
  mqttClient.setServer(MQTT_HOST, MQTT_PORT); // MQTT_HOST, MQTT_PORT are from mqtt.h
  mqttClient.setCallback(mqtt_callback);      // mqtt_callback is defined in this file
}

// Handles mqttClient.loop() and periodic publishes
void mqtt_loop() {
  // Only try to connect/reconnect MQTT if Ethernet has an IP
  if (ethGotIP) { // ethGotIP is extern from common.h
    if (!mqttClient.connected()) { // mqttClient is extern from common.h
      mqtt_reconnect(); // Call reconnect function
    }
    mqttClient.loop(); // PubSubClient's internal loop, must be called frequently

    // --- MQTT Periodic Status Publish ---
    static unsigned long lastStatusPublishTime = 0;
    if (mqttClient.connected() && (millis() - lastStatusPublishTime >= STATUS_PUBLISH_INTERVAL_MS)) { // STATUS_PUBLISH_INTERVAL_MS from mqtt.h
      mqtt_publishIPStatus(); // Call publish function
      lastStatusPublishTime = millis();
    }
  } else {
    // If Ethernet doesn't have an IP, print basic status
    static unsigned long lastNetworkStatusPrintTime = 0;
    if (millis() - lastNetworkStatusPrintTime >= 5000) {
        Serial.println("ESP32: Waiting for Ethernet IP...");
        lastNetworkStatusPrintTime = millis();
    }
  }
}

// Callback for PubSubClient: handles incoming MQTT messages
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT RX: Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messagePayload = "";
  for (unsigned int i = 0; i < length; i++) { // Using unsigned int for loop counter to match length
    messagePayload += (char)payload[i];
  }
  Serial.println(messagePayload);

  // Handle RESTART command
  if (String(topic) == MQTT_CMD_RESTART_TOPIC) { // MQTT_CMD_RESTART_TOPIC from mqtt.h
    if (messagePayload == "1") {
      Serial.println("MQTT RX: Received RESTART command. Restarting ESP32...");
      mqtt_publishLWT("Restarting", true); // Send "Restarting" LWT using helper
      delay(100);
      ESP.restart(); // Perform software restart
    }
  }
}

// Manages MQTT connection/reconnection
void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) { // mqttClient is extern from common.h
    Serial.print("MQTT: Attempting connection to "); Serial.print(MQTT_HOST); Serial.print("..."); // MQTT_HOST from mqtt.h
    // Attempt to connect with Last Will and Testament (LWT)
    if (mqttClient.connect(MQTT_CLIENT_ID, NULL, NULL, MQTT_LWT_TOPIC, 0, true, MQTT_LWT_MESSAGE_OFFLINE)) { // MQTT_CLIENT_ID, MQTT_LWT_TOPIC, MQTT_LWT_MESSAGE_OFFLINE from mqtt.h
      Serial.println("connected!");
      mqtt_publishLWT("Online", true); // Send "Online" LWT using helper
      // Subscribe to command topic
      mqttClient.subscribe(MQTT_CMD_RESTART_TOPIC); // MQTT_CMD_RESTART_TOPIC from mqtt.h
      Serial.printf("MQTT: Subscribed to %s\n", MQTT_CMD_RESTART_TOPIC);
      mqtt_publishIPStatus(); // Publish IP status immediately after (re)connection
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Disconnects MQTT client
void mqtt_disconnect() {
  if (mqttClient.connected()) { // mqttClient is extern from common.h
      mqttClient.disconnect();
      Serial.println("MQTT client disconnected.");
  }
}

// Publishes LWT message
void mqtt_publishLWT(const char* message, bool retained) {
    if (mqttClient.connected()) { // mqttClient is extern from common.h
        mqttClient.publish(MQTT_LWT_TOPIC, message, retained); // MQTT_LWT_TOPIC from mqtt.h
        Serial.printf("MQTT TX: LWT '%s' published to %s\n", message, MQTT_LWT_TOPIC);
    } else {
        Serial.println("MQTT TX: Not connected, cannot publish LWT.");
    }
}

// Publishes device's IP status as JSON
void mqtt_publishIPStatus() {
  if (!mqttClient.connected()) { // mqttClient is extern from common.h
    Serial.println("MQTT: Not connected, cannot publish IP status.");
    return;
  }

  if (!ethGotIP) { // ethGotIP is extern from common.h
      Serial.println("MQTT: Ethernet not up, cannot publish IP status.");
      return;
  }

  StaticJsonDocument<256> doc; // ArduinoJson v7 syntax

  doc["ip_address"] = ETH.localIP().toString(); // ETH is a global object from common.h -> main.cpp

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.printf("MQTT: Publishing IP status to %s: %s\n", MQTT_STATUS_TOPIC, jsonString.c_str()); // MQTT_STATUS_TOPIC from mqtt.h
  mqttClient.publish(MQTT_STATUS_TOPIC, jsonString.c_str()); // mqttClient is extern from common.h
}

// Publishes device connection status as JSON
void mqtt_publishDeviceStatus(byte deviceID, const char* status) {
  if (!mqttClient.connected()) { // mqttClient is extern from common.h
    Serial.println("MQTT: Not connected, cannot publish device status.");
    return;
  }
  StaticJsonDocument<256> doc;
  doc["device_id"] = deviceID;
  doc["status"] = status;
  String jsonString;
  serializeJson(doc, jsonString);
  mqttClient.publish(MQTT_STATUS_TOPIC, jsonString.c_str()); // MQTT_STATUS_TOPIC from mqtt.h
  Serial.printf("Published Device Status: ID %d, Status: %s\n", deviceID, status);
}

// Publishes button states of a device as JSON
void mqtt_publishButtonState(byte deviceID) {
  if (!mqttClient.connected()) { // mqttClient is extern from common.h
    Serial.println("MQTT: Not connected, cannot publish button state.");
    return;
  }

  if (!ethGotIP) { // ethGotIP is extern from common.h
      Serial.println("MQTT: Ethernet not up, cannot publish button state.");
      return;
  }

  StaticJsonDocument<256> doc; // Capacity for {"device_id":N, "btn1":"STATE", ...}

  doc["device_id"] = deviceID;

  for (int i = 0; i < 12; i++) { // Iterate only through the 12 relevant buttons
    String btnKey = "btn" + String(i + 1); // btn1, btn2, ..., btn12
    // allDeviceButtonStates and btnStateStrings are extern from common.h (defined in rs485.cpp)
    doc[btnKey] = btnStateStrings[allDeviceButtonStates[deviceID][i].state]; 
  }

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.printf("MQTT: Publishing button state to %s: %s\n", MQTT_DATA_TOPIC, jsonString.c_str()); // MQTT_DATA_TOPIC from mqtt.h
  mqttClient.publish(MQTT_DATA_TOPIC, jsonString.c_str()); // mqttClient is extern from common.h
}