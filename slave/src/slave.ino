#include <Arduino.h>
#include <Wire.h>         // Required for I2C communication with MCP23017
#include <SoftwareSerial.h> // Required for SoftwareSerial communication

// Note: No external MCP23017 library is used in this version for MCP control.
// All MCP operations are done via direct I2C (Wire library) register writes/reads.

/****************************************************************************************
 * GPIO PIN MAPPING (ADJUST THESE!)
 *
 * This code implements the full slave protocol for the ATtiny unit.
 *
 * --------------------------------------------------------------------------------------
 * Example for a standard Arduino Uno/Nano/Duemilanove:
 * - RS485_RX_PIN: D10, RS485_TX_PIN: D11
 * - RS485_DE_PIN: D5, RS485_NRE_PIN: D6 (Moved to avoid conflict with D2/D3 for interrupt)
 * - MCP_INT_PIN: D2 (Standard external interrupt pin 0, INT0)
 * - LED_PIN: D13
 *
 * I2C (Wire) uses default A4 (SDA), A5 (SCL).
 * HardwareSerial (Serial) uses default D0 (RX), D1 (TX) for debug.
 ****************************************************************************************/

// --- ADJUST THESE DEFINITIONS ---
// RS-485 SoftwareSerial pins
#define RS485_RX_PIN    10 // Connect to ST3485DE RO
#define RS485_TX_PIN    11 // Connect to ST3485DE DI
// RS-485 Transceiver control pins (for ST3485DE)
#define RS485_DE_PIN    5  // Connect to ST3485DE DE pin (Driver Enable - Active HIGH)
#define RS485_NRE_PIN   6  // Connect to ST3485DE ~RE pin (Receiver Enable - Active LOW)
// MCP23017 Interrupt pin
#define MCP_INT_PIN     2  // FIXED: Using Digital Pin 2 for external interrupt (INT0)
// Heartbeat LED pin (will now indicate RS-485 RX activity)
#define LED_PIN         13 // Built-in LED

// --- RS-485 Baud Rate (Must match ESP32 Master) ---
#define RS485_BAUD_RATE 57600

// Initialize SoftwareSerial for RS-485 communication
SoftwareSerial myRs485Serial(RS485_RX_PIN, RS485_TX_PIN);

// MCP23017 I2C Address (A0, A1, A2 all low - default)
#define MCP23017_ADDRESS 0x20

// --- Master (ESP32) -> Slave (ATtiny) Request Types ---
#define MASTER_POLL_START_BYTE   0xBB
#define MASTER_ID                0x00 // ESP32's ID on the bus (for verification) - Not directly used by slave logic
#define REQ_STATUS_POLL          0x01 // Request for button status data
#define REQ_DISCOVERY_POLL       0x02 // Request for device presence/ACK

// --- Slave (ATtiny) -> Master (ESP32) Response Types ---
#define RESP_STATUS_DATA         0xAA // Start byte for status data response
#define RESP_DISCOVERY_ACK       0xCC // Start byte for discovery ACK response

// --- Packet Lengths (Crucial for parsing on both ends) ---
#define MASTER_REQUEST_PACKET_LENGTH 5 // StartByte + MasterID + RequestType + TargetID + Checksum
#define SLAVE_STATUS_RESPONSE_LENGTH 5 // StartByte + DeviceID + RawMCPHigh + RawMCPLow + Checksum
#define SLAVE_ACK_RESPONSE_LENGTH    4 // StartByte + DeviceID + AckType + Checksum

// --- Global Variables ---
volatile bool mcpInterruptFlag = false; // Set by ISR; useful for future sleep/wake or debugging.
uint16_t previousRawMCPInputs = 0xFFFF; // Stores the last *raw* 16-bit state of MCP pins.
                                        // Initialized to all HIGH (released/DIPs off).
byte myDeviceID;                        // This unit's unique ID, read from DIP switches on MCP23017

// --- RS-485 Receive Buffer for incoming Master Polls ---
byte rs485_recv_buffer[MASTER_REQUEST_PACKET_LENGTH];
byte rs485_recv_index = 0;
bool rs485_packet_started = false;

// --- LED Variables (no longer for 1s heartbeat, now for RX activity) ---
// No longer need lastLedToggleTime and LED_BLINK_INTERVAL_MS


// --- Function Prototypes ---
void setupMCP23017();      // Configures the MCP23017 via direct Wire calls
uint16_t readMCP23017GPIO(); // Reads all 16 pins from MCP23017 via direct Wire calls
void mcpInterruptISR();    // ISR for the MCP23017's INT pin
void readAndClearMcpInterrupts(); // Debug function using direct Wire calls
void sendStatusResponse(); // Sends the current button status data to the Master
void sendDiscoveryAck();   // Sends a simple acknowledgment during discovery


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n--- ATtiny Slave: Full Protocol Start (RS485 RX LED) ---");

  // --- Setup LED Pin for RS-485 RX activity ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with LED off
  Serial.print("LED configured on pin "); Serial.println(LED_PIN);

  // Initialize I2C bus for communication with MCP23017
  Wire.begin();
  Serial.println("I2C Wire library initialized.");

  // --- Configure RS-485 control pins for ST3485DE ---
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_NRE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW); // Set ST3485DE to Receive mode (DE LOW)
  digitalWrite(RS485_NRE_PIN, LOW); // (~RE LOW)
  Serial.println("RS-485 DE/~RE pins configured to Receive mode.");

  // Initialize SoftwareSerial for RS-485 communication
  myRs485Serial.begin(RS485_BAUD_RATE);
  Serial.print("SoftwareSerial for RS-485 initialized on RX:");
  Serial.print(RS485_RX_PIN); Serial.print("/TX:"); Serial.print(RS485_TX_PIN);
  Serial.print(" at "); Serial.print(RS485_BAUD_RATE); Serial.println(" bps.");

  // Configure the MCP23017 I/O expander
  setupMCP23017(); // Configures MCP23017 including interrupts
  Serial.println("MCP23017 setup commands sent.");
  readAndClearMcpInterrupts(); // Clear any pending MCP interrupts from setup
  previousRawMCPInputs = readMCP23017GPIO(); // Initial read for ID
  byte dipSwitchRaw = (byte)(previousRawMCPInputs >> 12);
  myDeviceID = ~dipSwitchRaw & 0x0F; // Store ID for future use
  Serial.print("Device ID: "); Serial.println(myDeviceID);

  pinMode(MCP_INT_PIN, INPUT_PULLUP);
  attachInterrupt(INT0, mcpInterruptISR, FALLING); // Interrupt is attached
  Serial.print("MCP23017 Interrupt attached to INT0 (Digital Pin 2).\n");


  Serial.println("--- Setup Complete. Entering loop ---");
  Serial.println("Waiting for Master polls...");
}


void loop() {
  // --- RS-485 Data Reception (listening for Master Poll Requests) ---
  while (myRs485Serial.available()) {
    // Toggle LED to indicate RS-485 byte reception
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 

    byte incomingByte = myRs485Serial.read();
    // Serial.print("RS485 RX Byte: 0x"); Serial.println(incomingByte, HEX); // VERBOSE RX LOGGING

    // Packet reception state machine
    if (rs485_packet_started) {
      rs485_recv_buffer[++rs485_recv_index] = incomingByte; // Store the byte (user's preferred style)

      // Serial.print("  RX Buffer [len "); Serial.print(rs485_recv_index + 1); Serial.print("]: "); // DEBUG: Print current buffer contents
      // for (int i=0; i<rs485_recv_index + 1; i++) { 
      //   Serial.print("0x"); Serial.print(rs485_recv_buffer[i], HEX); Serial.print(" ");
      // }
      // Serial.println();

      if (rs485_recv_index >= MASTER_REQUEST_PACKET_LENGTH - 1) { // Check if full packet (index is 0-based)
        // Serial.println("  Full packet length reached.");
        if (rs485_recv_buffer[0] == MASTER_POLL_START_BYTE) {
          byte masterID = rs485_recv_buffer[1];
          byte requestType = rs485_recv_buffer[2];
          byte targetID = rs485_recv_buffer[3];
          byte receivedChecksum = rs485_recv_buffer[4];

          byte calculatedChecksum = MASTER_POLL_START_BYTE ^ masterID ^ requestType ^ targetID;

          // Serial.print("  Packet Parsed: MasterID=0x"); Serial.print(masterID, HEX);
          // Serial.print(", ReqType=0x"); Serial.print(requestType, HEX);
          // Serial.print(", TargetID=0x"); Serial.print(targetID, HEX);
          // Serial.print(", RxChksum=0x"); Serial.println(receivedChecksum, HEX);

          if (calculatedChecksum == receivedChecksum) {
            // Serial.println("  Checksum OK.");
            if (targetID == myDeviceID) {
              Serial.println("  Poll is for THIS device! Responding...");
              if (requestType == REQ_STATUS_POLL) {
                Serial.println("  Requesting Status...");
                sendStatusResponse(); // Call function which has its own print
              } else if (requestType == REQ_DISCOVERY_POLL) {
                Serial.println("  Requesting Discovery ACK...");
                sendDiscoveryAck(); // Call function which has its own print
              } else {
                Serial.print("  Unknown Request Type: 0x"); Serial.println(requestType, HEX);
              }
            } else {
              
              // Serial.print("  Poll is for ID 0x"); Serial.print(targetID, HEX); Serial.println(", not us. Ignoring.");
            }
          } else {
            Serial.print("  Checksum MISMATCH! Expected 0x"); Serial.print(calculatedChecksum, HEX);
            Serial.print(", Got 0x"); Serial.println(receivedChecksum, HEX);
            Serial.println("  Discarding packet.");
          }
        } else {
          Serial.print("  Sync Error: Expected 0x"); Serial.print(MASTER_POLL_START_BYTE, HEX);
          Serial.print(", Got 0x"); Serial.println(rs485_recv_buffer[0], HEX);
        }
        // Reset receiver state for the next packet regardless of outcome.
        rs485_recv_index = 0;
        rs485_packet_started = false;
      }
    } else if (incomingByte == MASTER_POLL_START_BYTE) { // Found master poll start byte
      // Serial.println("RS485: Master Poll Start Byte detected (0xBB).");
      rs485_packet_started = true;
      rs485_recv_index = 0; // Store start byte at index 0, index remains 0 initially
      rs485_recv_buffer[0] = incomingByte; // Store start byte at index 0
    } else {
      Serial.print("RS485: Discarding unexpected byte (not start byte): 0x"); Serial.println(incomingByte, HEX);
    }
  }

  // MCP23017 interrupt flag is not processed in loop for this basic RS-485 test.
  // The ISR will still fire and set `mcpInterruptFlag`, but it's not checked here.
  // `readMCP23017GPIO()` will be called when RS-485 response is sent, which clears MCP's INT flags.
}


// --- MCP23017 Specific Functions (using ONLY Wire.h) ---

// Configures the MCP23017 I/O Expander via direct Wire.h calls.
void setupMCP23017() {
  // Set all 16 pins as inputs
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x00); Wire.write(0xFF); Wire.endTransmission(); // IODIRA: all inputs (0xFF)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x01); Wire.write(0xFF); Wire.endTransmission(); // IODIRB: all inputs (0xFF)
  Serial.println("MCP23017: IODIRA/B set to all inputs.");

  // Enable internal pull-up resistors for all 16 input pins
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0C); Wire.write(0xFF); Wire.endTransmission(); // GPPUA: enable pull-ups (0xFF)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0D); Wire.write(0xFF); Wire.endTransmission(); // GPPUB: enable pull-ups (0xFF)
  Serial.println("MCP23017: GPPUA/B pull-ups enabled.");

  // Configure IOCON register for mirrored interrupts and active-low output
  // CORRECTED VALUE: 0b01000000 (0x40) to set MIRROR (Bit 6)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0A); Wire.write(0b01000000); Wire.endTransmission(); // IOCON: MIRROR enabled (0x40)
  Serial.println("MCP23017: IOCON configured (MIRROR enabled, active-low).");

  // --- CRITICAL FIX: Correct GPINTEN addresses! ---
  // Enable interrupt-on-change for all 16 input pins
  // GPINTENA is at 0x04, GPINTENB is at 0x05
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x04); Wire.write(0xFF); Wire.endTransmission(); // GPINTENA: interrupt on change (0xFF)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x05); Wire.write(0xFF); Wire.endTransmission(); // GPINTENB: interrupt on change (0xFF)
  Serial.println("MCP23017: GPINTENA/B interrupts enabled.");

  // Explicitly set INTCON (Interrupt Control) registers to 0x00
  // INTCONA is at 0x08, INTCONB is at 0x09
  // Setting to 0x00 means "compare against previous value" (interrupt on any change)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x08); Wire.write(0x00); Wire.endTransmission(); // INTCONA: Compare to previous (0x00)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x09); Wire.write(0x00); Wire.endTransmission(); // INTCONB: Compare to previous (0x00)
  Serial.println("MCP23017: INTCONA/B set to interrupt on any change.");
}

// Reads the current state of all 16 GPIO pins from the MCP23017 via direct Wire.h calls.
// Reading these registers also clears any pending interrupt flags on the MCP23017.
uint16_t readMCP23017GPIO() {
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x12); Wire.endTransmission(); // Start reading from GPIOA
  Wire.requestFrom(MCP23017_ADDRESS, 2); // Request 2 bytes (GPIOA and GPIOB)

  uint8_t gpioA = Wire.read();
  uint8_t gpioB = Wire.read();

  return (gpioB << 8) | gpioA; // Combine into a 16-bit value (GPB as high byte, GPA as low byte)
}

// Reads MCP23017 interrupt registers using direct Wire calls and prints them for debugging.
// This serves as the explicit interrupt clearing mechanism in this code.
void readAndClearMcpInterrupts() {
    // Read INTF (Interrupt Flag) registers
    Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0E); Wire.endTransmission(false);
    Wire.requestFrom(MCP23017_ADDRESS, 2);
    uint8_t intfa = Wire.read(); uint8_t intfb = Wire.read();
    Serial.print("  INTF (Flagged pins): GPB="); Serial.print(intfb, BIN); Serial.print(", GPA="); Serial.println(intfa, BIN);

    // Read INTCAP (Interrupt Captured) registers. This action clears the interrupt on the MCP23017 INT pin.
    Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x10); Wire.endTransmission(false);
    Wire.requestFrom(MCP23017_ADDRESS, 2);
    uint8_t intcapa = Wire.read(); uint8_t intcapb = Wire.read();
    Serial.print("  INTCAP (State at interrupt): GPB="); Serial.print(intcapb, BIN); Serial.print(", GPA="); Serial.println(intcapa, BIN);
    Serial.println("--- End MCP Interrupt Clear & Debug ---");
}

// Interrupt Service Routine (ISR) for the MCP23017's INT pin.
void mcpInterruptISR() {
  mcpInterruptFlag = true;
}

// Sends the current 16-bit raw MCP input state when polled by the Master for status.
void sendStatusResponse() {
  Serial.println("RS485 TX: Sending Status Response...");

  // Set ST3485DE to Transmit Mode: DE HIGH, ~RE HIGH
  digitalWrite(RS485_NRE_PIN, HIGH); // Disable Receiver (~RE HIGH)
  digitalWrite(RS485_DE_PIN, HIGH);  // Enable Driver (DE HIGH)
  delay(1); // Short delay for transceiver to switch modes

  // Get the freshest raw MCP input state right before sending
  uint16_t currentRawState = readMCP23017GPIO();
  Serial.print("  Current Raw State (to send): 0b");
  Serial.println(currentRawState, BIN);

  byte highByte = (byte)(currentRawState >> 8); // Extract bits 15-8
  byte lowByte = (byte)(currentRawState & 0xFF); // Extract bits 7-0

  // Calculate the checksum for the slave status response packet
  byte checksum = RESP_STATUS_DATA ^ myDeviceID ^ highByte ^ lowByte;

  myRs485Serial.write(RESP_STATUS_DATA); // Start byte (0xAA)
  myRs485Serial.write(myDeviceID);       // This unit's Device ID
  myRs485Serial.write(highByte);         // Raw MCP input high byte
  myRs485Serial.write(lowByte);          // Raw MCP input low byte
  myRs485Serial.write(checksum);         // Checksum
  myRs485Serial.flush(); // Wait for the entire transmit buffer to empty (ensures packet is sent)
  Serial.print("RS485 TX: Sent packet: 0x"); Serial.print(RESP_STATUS_DATA, HEX);
  Serial.print(" 0x"); Serial.print(myDeviceID, HEX);
  Serial.print(" 0x"); Serial.print(highByte, HEX);
  Serial.print(" 0x"); Serial.print(lowByte, HEX);
  Serial.print(" 0x"); Serial.println(checksum, HEX);

  // After transmission, switch back to Receive mode: DE LOW, ~RE LOW
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_NRE_PIN, LOW);
  Serial.println("RS485 TX: Switched back to Receive mode.");
}

// Sends a simple acknowledgment (ACK) when polled by the Master for discovery.
void sendDiscoveryAck() {
    Serial.println("RS485 TX: Sending Discovery ACK...");

    digitalWrite(RS485_NRE_PIN, HIGH); // Disable Receiver (~RE HIGH)
    digitalWrite(RS485_DE_PIN, HIGH); // Enable Driver (DE HIGH)
    delay(1); // Short delay for stability

    byte ackType = 0x01; // Simple ACK type (e.g., "I'm here and ready")
    byte checksum = RESP_DISCOVERY_ACK ^ myDeviceID ^ ackType; // Calculate checksum for ACK packet

    myRs485Serial.write(RESP_DISCOVERY_ACK); // Start byte (0xCC)
    myRs485Serial.write(myDeviceID);         // This unit's Device ID
    myRs485Serial.write(ackType);            // ACK Type
    myRs485Serial.write(checksum);           // Checksum
    myRs485Serial.flush();
    Serial.print("RS485 TX: Sent packet: 0x"); Serial.print(RESP_DISCOVERY_ACK, HEX);
    Serial.print(" 0x"); Serial.print(myDeviceID, HEX);
    Serial.print(" 0x"); Serial.print(ackType, HEX);
    Serial.print(" 0x"); Serial.println(checksum, HEX);

    Serial.println("  Discovery ACK Sent.");

    digitalWrite(RS485_DE_PIN, LOW);
    digitalWrite(RS485_NRE_PIN, LOW);
    Serial.println("Switched back to Receive mode.");
}