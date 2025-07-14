#include <Arduino.h>
#include <WiFi.h>   // Required for WiFi.mode() and WiFi.onEvent(), even if not using Wi-Fi
#include <ETH.h>    // Required for Ethernet library functions
#include <PubSubClient.h> // Required for MQTT client functionality
#include <ArduinoJson.h>  // Required for JSON payload creation
#include <HardwareSerial.h> // Required for ESP32's second UART

// --- Ethernet Configuration for ESP32-ETH01 (LAN8720A) ---
#define ETH_MDC_PIN    23
#define ETH_MDIO_PIN   18
#define ETH_TYPE       ETH_PHY_LAN8720
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN  16 

// --- Dedicated PHY Reset Pin (Optional, requires board wiring) ---
// If your ESP32-ETH01 has a specific GPIO connected to the LAN8720A's _nRESET pin,
// define it here. Common pins found on some boards are GPIO5.
// If not used/available on your board, leave this line commented out.
// #define ETH_PHY_RESET_PIN 5 


// --- RS-485 Configuration for ST3485DE (using Hardware Serial2) ---
// !!! IMPORTANT: YOU MUST FIND AND VERIFY THESE PINS ON YOUR BOARD'S SCHEMATIC !!!
// !!! These are common free UART2 pins, but may differ on your specific ESP32-ETH01 variant.
#define RS485_RX_PIN     5  // ESP32 GPIO for RS-485 Receive (RO from ST3485DE) - UART2 RX
#define RS485_TX_PIN     17 // ESP32 GPIO for RS-485 Transmit (DI to ST3485DE) - UART2 TX
#define RS485_DE_PIN     32 // ESP32 GPIO for ST3485DE DE pin (Driver Enable - Active HIGH)
#define RS485_NRE_PIN    33 // ESP32 GPIO for ST3485DE ~RE pin (Receiver Enable - Active LOW)
#define RS485_BAUD_RATE 57600 // Must match slave ATtiny


// Use Serial2 for RS-485 communication, remapped to custom pins
HardwareSerial SerialRS485(2); 


// --- Global Flag for Ethernet IP Status ---
bool ethGotIP = false;

// --- MQTT Configuration ---
const char* MQTT_HOST = "mqtt.iot";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "esp32_buttons";
const char* MQTT_LWT_TOPIC = "tele/esp32_buttons/LWT";
const char* MQTT_LWT_MESSAGE_OFFLINE = "Offline";
const char* MQTT_CMD_RESTART_TOPIC = "cmnd/esp32_buttons/RESTART";
const char* MQTT_STATUS_TOPIC = "stat/esp32_buttons/STATUS"; // Used for general status and RTT

WiFiClient ethClient;
PubSubClient mqttClient(ethClient);

// --- MQTT Periodic Status Variables ---
unsigned long lastStatusPublishTime = 0;
const unsigned long STATUS_PUBLISH_INTERVAL_MS = 60 * 1000;

// --- RS-485 Ping-Pong Test Variables ---
unsigned long lastPingTime = 0;
const unsigned long PING_INTERVAL_MS = 5 * 1000;
const unsigned long PONG_TIMEOUT_MS = 5000; // <--- UPDATED TO 5 SECONDS


// --- Function Prototypes ---
void ETH_GotIP(arduino_event_id_t event, arduino_event_info_t info);
void ETH_Stop(arduino_event_id_t event, arduino_event_info_t info);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void publishIPStatus(); // Publishes JSON with IP
void performRs485PingPong(); // Publishes Ping-Pong result with RTT


void setup() {
  Serial.begin(115200);
  delay(1000);
  while (!Serial && millis() < 5000) {
    // Wait for serial monitor or 5 seconds
  }

  Serial.println("\n--- ESP32-ETH01 System Start ---");

  // --- Set Device Hostname ---
  WiFi.setHostname(MQTT_CLIENT_ID);
  Serial.printf("Setting hostname to: %s\n", MQTT_CLIENT_ID);

  // --- Disable Wi-Fi to dedicate resources to Ethernet ---
  WiFi.mode(WIFI_OFF);
  btStop();


  // --- Configure and Start Ethernet ---
  Serial.println("Starting Ethernet connection with specific ETH.begin() signature...");

  ETH.begin(1, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE, true);
  Serial.println("ETH.begin() called with user-specified signature.");


  // --- Dedicated PHY Hardware Reset (if available and defined) ---
#ifdef ETH_PHY_RESET_PIN
  Serial.println("Performing dedicated PHY hardware reset...");
  pinMode(ETH_PHY_RESET_PIN, OUTPUT);
  digitalWrite(ETH_PHY_RESET_PIN, LOW);
  delay(50);
  digitalWrite(ETH_PHY_RESET_PIN, HIGH);
  delay(100);
  Serial.println("Dedicated PHY hardware reset complete.");
#endif

  // Register Ethernet event handlers
  WiFi.onEvent(ETH_GotIP, ARDUINO_EVENT_ETH_GOT_IP);
  WiFi.onEvent(ETH_Stop, ARDUINO_EVENT_ETH_STOP);

  // --- Setup RS-485 UART ---
  SerialRS485.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_NRE_PIN, OUTPUT);
  // Set ST3485DE to Receive mode by default: DE LOW, ~RE LOW
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_NRE_PIN, LOW);
  Serial.print("RS-485 initialized on RX:"); Serial.print(RS485_RX_PIN);
  Serial.print("/TX:"); Serial.print(RS485_TX_PIN);
  Serial.print("/DE:"); Serial.print(RS485_DE_PIN);
  Serial.print("/~RE:"); Serial.print(RS485_NRE_PIN);
  Serial.print(" at "); Serial.print(RS485_BAUD_RATE); Serial.println(" bps.");


  // --- Setup MQTT Client ---
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  Serial.println("--- Setup Complete. Waiting for IP address and MQTT ---");
}


void loop() {
  // --- MQTT Client Loop ---
  if (ethGotIP) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();

    if (mqttClient.connected() && (millis() - lastStatusPublishTime >= STATUS_PUBLISH_INTERVAL_MS)) {
      publishIPStatus();
      lastStatusPublishTime = millis();
    }
  } else {
    static unsigned long lastNetworkStatusPrintTime = 0;
    if (millis() - lastNetworkStatusPrintTime >= 5000) {
        Serial.println("ESP32: Waiting for Ethernet IP...");
        lastNetworkStatusPrintTime = millis();
    }
  }

  // --- RS-485 Ping-Pong Test Polling ---
  if (millis() - lastPingTime >= PING_INTERVAL_MS) {
    if (ethGotIP && mqttClient.connected()) { // Only ping if network and MQTT are up
        performRs485PingPong();
    } else {
        Serial.println("RS-485 Ping: Skipping, network/MQTT not ready.");
    }
    lastPingTime = millis();
  }
}


// --- Ethernet Event Handlers ---

void ETH_GotIP(arduino_event_id_t event, arduino_event_info_t info) {
  Serial.printf("Ethernet link UP, DHCP successful, IP: %s\n", IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
  Serial.printf("MAC: %s\n", ETH.macAddress().c_str());
  Serial.printf("Hostname: %s\n", WiFi.getHostname());
  ethGotIP = true;
  Serial.println("ETH: IP obtained. Attempting immediate MQTT reconnect.");
  reconnectMQTT();
  mqttClient.publish(MQTT_LWT_TOPIC, "Online", true);
  publishIPStatus();
}

void ETH_Stop(arduino_event_id_t event, arduino_event_info_t info) {
  Serial.println("Ethernet link DOWN or IP lost!");
  ethGotIP = false;
  if (mqttClient.connected()) {
      mqttClient.disconnect();
      Serial.println("MQTT client disconnected due to ETH DOWN.");
  }
  mqttClient.publish(MQTT_LWT_TOPIC, MQTT_LWT_MESSAGE_OFFLINE, true);
}


// --- MQTT Callbacks and Functions ---

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT RX: Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messagePayload = "";
  for (int i = 0; i < length; i++) {
    messagePayload += (char)payload[i];
  }
  Serial.println(messagePayload);

  if (String(topic) == MQTT_CMD_RESTART_TOPIC) {
    if (messagePayload == "1") {
      Serial.println("MQTT RX: Received RESTART command. Restarting ESP32...");
      mqttClient.publish(MQTT_LWT_TOPIC, "Restarting", true);
      delay(100);
      ESP.restart();
    }
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("MQTT: Attempting connection to "); Serial.print(MQTT_HOST); Serial.print("...");
    if (mqttClient.connect(MQTT_CLIENT_ID, NULL, NULL, MQTT_LWT_TOPIC, 0, true, MQTT_LWT_MESSAGE_OFFLINE)) {
      Serial.println("connected!");
      mqttClient.publish(MQTT_LWT_TOPIC, "Online", true);
      mqttClient.subscribe(MQTT_CMD_RESTART_TOPIC);
      Serial.printf("MQTT: Subscribed to %s\n", MQTT_CMD_RESTART_TOPIC);
      publishIPStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void publishIPStatus() {
  if (!mqttClient.connected()) {
    Serial.println("MQTT: Not connected, cannot publish IP status.");
    return;
  }

  if (!ethGotIP) {
      Serial.println("MQTT: Ethernet not up, cannot publish IP status.");
      return;
  }

  StaticJsonDocument<256> doc;

  doc["ip_address"] = ETH.localIP().toString();

  String jsonString;
  serializeJson(doc, jsonString);

  Serial.printf("MQTT: Publishing IP status to %s: %s\n", MQTT_STATUS_TOPIC, jsonString.c_str());
  mqttClient.publish(MQTT_STATUS_TOPIC, jsonString.c_str());
}


// --- RS-485 Ping-Pong Test Function ---
void performRs485PingPong() {
  Serial.println("RS-485 Ping: Sending 0xAA...");
  unsigned long rttStartTime = millis(); // Record start time for RTT measurement

  // Clear RX buffer before sending to avoid old data
  while (SerialRS485.available()) {
    SerialRS485.read();
  }

  // Set ST3485DE to Transmit Mode: DE HIGH, ~RE HIGH
  digitalWrite(RS485_NRE_PIN, HIGH); // Disable Receiver (~RE LOW to HIGH)
  digitalWrite(RS485_DE_PIN, HIGH);  // Enable Driver (DE LOW to HIGH)
  delay(1); // Short delay for transceiver to switch modes

  SerialRS485.write(0xAA); // Send the ping
  SerialRS485.flush(); // Wait for transmit buffer to empty

  // Set ST3485DE back to Receive Mode: DE LOW, ~RE LOW
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_NRE_PIN, LOW);
  Serial.println("RS-485 Ping: Sent 0xAA. Waiting for 0xBB...");

  unsigned long loopStartTime = millis(); // For timeout logic
  byte receivedByte = 0;
  bool pongReceived = false; // True if 0xBB was received
  bool responseArrived = false; // True if ANY byte arrived within timeout
  unsigned long rtt = 0; // Initialize RTT

  while (millis() - loopStartTime < PONG_TIMEOUT_MS) {
    if (SerialRS485.available()) {
      receivedByte = SerialRS485.read();
      responseArrived = true; // A byte arrived
      rtt = millis() - rttStartTime; // Calculate RTT as soon as a byte arrives

      if (receivedByte == 0xBB) {
        pongReceived = true; // Received expected pong
        break; // Exit loop
      } else {
        // Received something, but it was not 0xBB. This is a "FAILED" case.
        pongReceived = false;
        break; // Exit loop on first unexpected byte
      }
    }
  }
  
  StaticJsonDocument<128> doc; // Increased capacity to hold "response", "time", and potentially "error_byte"
  String jsonString;

  if (responseArrived && pongReceived) { // pongReceived is true only if 0xBB was received
    Serial.printf("RS-485 Pong: Received 0xBB. Ping-pong SUCCESS! RTT: %lu ms\n", rtt);
    doc["response"] = "OK";
    doc["time"] = rtt;
  } else if (responseArrived && !pongReceived) { // Something was received, but not 0xBB
    Serial.printf("RS-485 Pong: Received unexpected byte 0x%02X. Ping-pong FAILED! RTT: %lu ms\n", receivedByte, rtt);
    doc["response"] = "FAILED";
    doc["time"] = rtt;
    doc["error_byte"] = receivedByte; // Log the unexpected byte for debugging
  } else { // No byte received within timeout
    rtt = PONG_TIMEOUT_MS; // RTT is effectively the timeout duration if nothing arrived
    Serial.printf("RS-485 Pong: Timeout. Ping-pong FAILED! RTT: %lu ms\n", rtt);
    doc["response"] = "TIMEOUT";
    doc["time"] = rtt;
  }

  serializeJson(doc, jsonString);
  mqttClient.publish(MQTT_STATUS_TOPIC, jsonString.c_str());
}