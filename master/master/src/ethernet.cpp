#include "ethernet.h" // Include the header for this module
#include "common.h"   // Include common definitions
#include "mqtt.h"     // NEW: Include MQTT module header to call its functions


// --- Ethernet Module Implementation ---

void ethernet_setup() {
  // --- Register Ethernet event handlers (Moved here to consolidate Ethernet setup) ---
  // Handlers are defined below in this file.
  Serial.println("Registering Ethernet event handlers...");
  WiFi.onEvent(ETH_GotIP_Handler, ARDUINO_EVENT_ETH_GOT_IP);
  WiFi.onEvent(ETH_Stop_Handler, ARDUINO_EVENT_ETH_STOP);

  // Configure and Start Ethernet
  Serial.println("Starting Ethernet connection with specific ETH.begin() signature...");

  // ETH.begin(phy_addr, power, mdc, mdio, type, clock_mode, use_mac_from_efuse)
  ETH.begin(1, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE, true);
  Serial.println("ETH.begin() called with user-specified signature.");
}

// --- Ethernet Event Handlers Implementation ---

void ETH_GotIP_Handler(arduino_event_id_t event, arduino_event_info_t info) {
  Serial.printf("Ethernet link UP, DHCP successful, IP: %s\n", IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
  Serial.printf("MAC: %s\n", ETH.macAddress().c_str());
  Serial.printf("Hostname: %s\n", WiFi.getHostname());
  ethGotIP = true; // Update global flag (defined in common.h)

  // Call MQTT module functions to handle MQTT reconnection and status updates
  Serial.println("ETH: IP obtained. Attempting immediate MQTT reconnect.");
  mqtt_reconnect(); // Call reconnect from MQTT module (defined in mqtt.cpp)
  mqtt_publishLWT("Online", true); // Send "Online" LWT (defined in mqtt.cpp)
  mqtt_publishIPStatus(); // Publish initial IP status (defined in mqtt.cpp)
}

void ETH_Stop_Handler(arduino_event_id_t event, arduino_event_info_t info) {
  Serial.println("Ethernet link DOWN or IP lost!");
  ethGotIP = false; // Update global flag

  // Call MQTT module functions to handle MQTT disconnection and LWT offline message
  mqtt_disconnect(); // NEW: Call disconnect from MQTT module (defined in mqtt.cpp)
  mqtt_publishLWT(MQTT_LWT_MESSAGE_OFFLINE, true); // NEW: Send "Offline" LWT (defined in mqtt.cpp)
}