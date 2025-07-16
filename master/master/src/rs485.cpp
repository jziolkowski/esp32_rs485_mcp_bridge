#include "rs485.h"  // Include the header for this module (contains prototypes and RS-485 constants)
#include "common.h" // Include common definitions (contains externs for global objects, shared enums/structs)
#include "mqtt.h"   // Include MQTT module header to call its publishing functions


// --- Global Variables (Defined here, extern declared in rs485.h) ---
// These variables manage the RS-485 polling state machine and button states.
PollingState currentPollingState = DISCOVERY_MODE; 
bool discoveredDevices[16];
unsigned long lastSeenTimestamp[16];
int currentPollingTargetID = 0;
unsigned long lastPollRequestSentTime = 0;
unsigned long lastDiscoveryRescanTime = 0;

byte rs485_recv_buffer[SLAVE_STATUS_RESPONSE_LENGTH]; // Defined with max length
byte rs485_recv_index = 0;
bool rs485_packet_started = false;

bool responseReceivedForCurrentSlot = false; 
int lastPolledIDForTimeoutCheck = -1; 

ButtonStatusESP32 allDeviceButtonStates[16][12];
uint16_t lastRawMCPInputs[16];


// --- RS-485 Polling Module Implementation ---

void rs485_polling_setup() {
  SerialRS485.begin(RS485_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_NRE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_NRE_PIN, LOW);
  Serial.print("RS-485 initialized on RX:"); Serial.print(RS485_RX_PIN);
  Serial.print("/TX:"); Serial.print(RS485_TX_PIN);
  Serial.print("/DE:"); Serial.print(RS485_DE_PIN);
  Serial.print("/~RE:"); Serial.print(RS485_NRE_PIN);
  Serial.print(" at "); Serial.print(RS485_BAUD_RATE); Serial.println(" bps.");

  // Initialize polling state variables (important for cold boot)
  for (int i = 0; i < 16; i++) {
    discoveredDevices[i] = false;
    lastSeenTimestamp[i] = 0;
    lastRawMCPInputs[i] = 0xFFFF; // All inputs released (HIGH)
    for (int j = 0; j < 12; j++) {
      allDeviceButtonStates[i][j].state = MAIN_OFF;
      allDeviceButtonStates[i][j].pressStartTime = 0;
      allDeviceButtonStates[i][j].lastPhysicalChange = 0;
      allDeviceButtonStates[i][j].longPressEventSent = false;
    }
  }
  currentPollingState = DISCOVERY_MODE;
  currentPollingTargetID = 0;
  lastPollRequestSentTime = millis(); // Initialize lastPollRequestSentTime for polling timing
  lastDiscoveryRescanTime = millis();
}


void rs485_polling_loop() {
  // Process any incoming RS-485 data immediately
  rs485_processIncomingData();

  // Calculate numActiveDevicesInCycle for timeout calculation
  unsigned long numActiveDevicesInCycle = 0;
  for (int i = 0; i < 16; i++) {
      if (discoveredDevices[i]) {
          numActiveDevicesInCycle++;
      }
  }
  // During discovery mode or if no devices are discovered yet, assume max devices (16) for cycle time
  if (currentPollingState == DISCOVERY_MODE || numActiveDevicesInCycle == 0) {
      numActiveDevicesInCycle = 16; 
  }

  // Calculate current effective timeout (always based on worst-case cycle for disconnection detection)
  // Timeout = (Number of active devices * Slot Duration) + Jitter Buffer
  unsigned long calculatedDynamicTimeout = numActiveDevicesInCycle * POLL_SLOT_DURATION_MS + JITTER_BUFFER_MS;
  // Ensure timeout is not too short even with very few devices
  if (calculatedDynamicTimeout < MINIMUM_CALCULATED_TIMEOUT_MS) {
      calculatedDynamicTimeout = MINIMUM_CALCULATED_TIMEOUT_MS;
  }
  Serial.printf("DEBUG: Current calculated timeout: %lu ms (based on %lu active devices)\n", calculatedDynamicTimeout, numActiveDevicesInCycle);


  // --- Device Timeout Check (for the device that was polled in the *previous* slot) ---
  // This check applies if we are in NORMAL_MODE and a device was actually polled before.
  if (currentPollingState == NORMAL_MODE && lastPolledIDForTimeoutCheck != -1 && discoveredDevices[lastPolledIDForTimeoutCheck]) {
      if (millis() - lastSeenTimestamp[lastPolledIDForTimeoutCheck] > calculatedDynamicTimeout) {
          Serial.printf("Device ID %d timed out (no response) after %lu ms! (Calculated timeout was: %lu ms)\n", lastPolledIDForTimeoutCheck, millis() - lastSeenTimestamp[lastPolledIDForTimeoutCheck], calculatedDynamicTimeout);
          discoveredDevices[lastPolledIDForTimeoutCheck] = false; // Mark as disconnected
          mqtt_publishDeviceStatus(lastPolledIDForTimeoutCheck, "disconnected"); // Call MQTT module function
        // Reset lastPolledIDForTimeoutCheck to -1 after handling the timeout/disconnection
        // This prevents immediate re-check of the same device before it can be re-discovered
        lastPolledIDForTimeoutCheck = -1;
      }
  }


  // --- Polling State Machine (with "immediate hop" optimization) ---
  // A poll is sent if:
  // 1. The full POLL_SLOT_DURATION_MS has passed since the last poll was sent. (Max wait for a response in a slot)
  // OR
  // 2. The response for the *current* poll has already been received (responseReceivedForCurrentSlot is true),
  //    AND a minimal time for the transaction has passed (NOMINAL_SINGLE_POLL_TRANSACTION_MS)
  if ( (millis() - lastPollRequestSentTime >= POLL_SLOT_DURATION_MS) || 
       (responseReceivedForCurrentSlot && (millis() - lastPollRequestSentTime >= NOMINAL_SINGLE_POLL_TRANSACTION_MS)) 
     )
  {
    responseReceivedForCurrentSlot = false; // Reset for the next poll
    
    // --- Determine next device to poll ---
    if (currentPollingState == DISCOVERY_MODE) {
      if (currentPollingTargetID < 16) {
        rs485_sendPollRequest(REQ_DISCOVERY_POLL, currentPollingTargetID);
        lastPolledIDForTimeoutCheck = currentPollingTargetID; // Store ID for timeout check
        currentPollingTargetID++;
        lastPollRequestSentTime = millis(); // Update time when this poll was sent
      } else { // Finished one full discovery scan (0-15)
        Serial.println("Finished discovery scan. Entering NORMAL mode.");
        currentPollingState = NORMAL_MODE; // Transition to normal operating mode
        currentPollingTargetID = 0; // Reset for next normal polling cycle (start from ID 0)
        lastDiscoveryRescanTime = millis(); // Reset full rescan timer
        lastPolledIDForTimeoutCheck = -1; // Reset as we are starting a new cycle
      }
    } else { // NORMAL_MODE
      // Periodically re-enter DISCOVERY_MODE for a full rescan
      if (millis() - lastDiscoveryRescanTime >= FULL_DISCOVERY_RESCAN_INTERVAL_MS) {
          Serial.println("Starting full discovery rescan...");
          currentPollingState = DISCOVERY_MODE;
          currentPollingTargetID = 0;
          lastPolledIDForTimeoutCheck = -1; // Reset as we are starting discovery
          lastPollRequestSentTime = millis(); // Reset for discovery polling
          // lastDiscoveryRescanTime will be reset when discovery scan is complete or at end of NORMAL cycle
          return; // Skip normal polling for this loop iteration
      }

      bool foundNext = false;
      // Loop to find the next active device to poll, starting from currentPollingTargetID
      for (int i = 0; i < 16; i++) {
          int id = (currentPollingTargetID + i) % 16; // Start from current position and wrap around
          if (discoveredDevices[id]) { // If this device ID is marked as discovered
              currentPollingTargetID = id; // Set this as our new target for this poll
              foundNext = true;
              break;
          }
      }

      if (foundNext) {
          rs485_sendPollRequest(REQ_STATUS_POLL, currentPollingTargetID);
          lastPolledIDForTimeoutCheck = currentPollingTargetID; // Store for timeout check
          currentPollingTargetID = (currentPollingTargetID + 1) % 16; // Move to the next for the next iteration (wrap if needed)
          lastPollRequestSentTime = millis(); // Update time when this poll was sent
      } else {
          // No active devices found in the NORMAL_MODE list.
          // This implies all previously discovered devices are now disconnected or none were ever found.
          Serial.println("No active devices found. Forcing next discovery scan.");
          currentPollingTargetID = 0;
          currentPollingState = DISCOVERY_MODE; // Force rediscovery
          lastPolledIDForTimeoutCheck = -1; // Reset as we are forcing discovery
          lastPollRequestSentTime = millis(); // Reset for discovery polling
          lastDiscoveryRescanTime = millis(); // Reset rescan timer immediately
      }
    }
  }
}


// --- RS-485 Polling Functions Implementation ---

// This function sends a poll request to a target slave device.
void rs485_sendPollRequest(byte requestType, byte targetDeviceID) {
  // Clear RX buffer before sending to avoid old data
  while (SerialRS485.available()) {
    SerialRS485.read();
  }

  byte checksum = MASTER_POLL_START_BYTE ^ MASTER_ID ^ requestType ^ targetDeviceID;

  // Set ST3485DE to Transmit Mode: DE HIGH, ~RE HIGH
  digitalWrite(RS485_NRE_PIN, HIGH); // Disable Receiver (~RE LOW to HIGH)
  digitalWrite(RS485_DE_PIN, HIGH);  // Enable Driver (DE LOW to HIGH)
  delay(1); // Short delay for transceiver to switch modes

  SerialRS485.write(MASTER_POLL_START_BYTE);
  SerialRS485.write(MASTER_ID);
  SerialRS485.write(requestType);
  SerialRS485.write(targetDeviceID);
  SerialRS485.write(checksum);
  SerialRS485.flush(); // Wait for transmit buffer to empty

  // Set ST3485DE back to Receive Mode: DE LOW, ~RE LOW
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_NRE_PIN, LOW);
  
  // lastPollRequestSentTime is updated in loop() after this call
}

// This function processes any incoming RS-485 data from slaves.
void rs485_processIncomingData() {
    // Keep processing bytes as long as they are available in the receive buffer
    while (SerialRS485.available()) {
        byte incomingByte = SerialRS485.read();

        // If not started, check if current byte is a start byte
        if (!rs485_packet_started) {
            if (incomingByte == RESP_STATUS_DATA || incomingByte == RESP_DISCOVERY_ACK) {
                rs485_packet_started = true;
                rs485_recv_buffer[0] = incomingByte;
                rs485_recv_index = 1; // Start storing from index 1
            }
            continue; // Go to next byte
        }

        // If started, store the byte
        rs485_recv_buffer[rs485_recv_index++] = incomingByte;

        // Check for full packet received based on the detected start byte
        if (rs485_recv_buffer[0] == RESP_STATUS_DATA) {
            if (rs485_recv_index >= SLAVE_STATUS_RESPONSE_LENGTH) {
                // Full Status Packet Received
                byte receivedID = rs485_recv_buffer[1];
                uint16_t rawMCPHigh = rs485_recv_buffer[2];
                uint16_t rawMCPLow = rs485_recv_buffer[3];
                uint16_t rawMCPInputs = (rawMCPHigh << 8) | rawMCPLow;
                byte receivedChecksum = rs485_recv_buffer[4];

                byte calculatedChecksum = RESP_STATUS_DATA ^ receivedID ^ rawMCPHigh ^ rawMCPLow;

                if (calculatedChecksum == receivedChecksum) {
                    rs485_processSlaveStatusResponse(receivedID, rawMCPInputs);
                    responseReceivedForCurrentSlot = true; // Set flag for "immediate hop"
                } else {
                    Serial.println("RS-485 STATUS Checksum Mismatch!");
                }
                // Reset receiver state for next packet
                rs485_recv_index = 0;
                rs485_packet_started = false;
            }
        } else if (rs485_recv_buffer[0] == RESP_DISCOVERY_ACK) {
            if (rs485_recv_index >= SLAVE_ACK_RESPONSE_LENGTH) {
                // Full Discovery ACK Packet Received
                byte receivedID = rs485_recv_buffer[1];
                byte ackType = rs485_recv_buffer[2];
                byte receivedChecksum = rs485_recv_buffer[3];

                byte calculatedChecksum = RESP_DISCOVERY_ACK ^ receivedID ^ ackType;
                if (calculatedChecksum == receivedChecksum) {
                    rs485_processSlaveDiscoveryAck(receivedID);
                } else {
                    Serial.println("RS-485 ACK Checksum Mismatch!");
                }
                // Reset receiver state for next packet
                rs485_recv_index = 0;
                rs485_packet_started = false;
            }
        }
    }
}

// Handles processing of a received status data packet
void rs485_processSlaveStatusResponse(byte deviceID, uint16_t currentRawMCPInputs) {
  // Validate deviceID
  if (deviceID >= 16) {
    Serial.printf("Invalid DeviceID (%d) received in status response.\n", deviceID);
    return;
  }

  // If this device was previously marked as disconnected, publish "connected"
  if (!discoveredDevices[deviceID]) {
      discoveredDevices[deviceID] = true;
      mqtt_publishDeviceStatus(deviceID, "connected"); // Call MQTT module function
  }
  lastSeenTimestamp[deviceID] = millis(); // Update last seen time for this device

  // Extract relevant 12 button bits from the 16-bit raw input
  uint16_t currentButtonInputs = (currentRawMCPInputs & 0x00FF) | ((currentRawMCPInputs >> 4) & 0x0F00);
  uint16_t previousButtonInputs = (lastRawMCPInputs[deviceID] & 0x00FF) | ((lastRawMCPInputs[deviceID] >> 4) & 0x0F00);

  // Run button state machine for each of the 12 buttons
  bool deviceStateChanged = false; // Flag to decide if we should publish this device's states

  for (int i = 0; i < 12; i++) {
    bool currentPhysicalPressed = !bitRead(currentButtonInputs, i); // Buttons are active-low
    bool previousPhysicalPressed = !bitRead(previousButtonInputs, i);

    // --- Step 1: Detect raw physical changes and update debounce timer ---
    if (currentPhysicalPressed != previousPhysicalPressed) {
      allDeviceButtonStates[deviceID][i].lastPhysicalChange = millis();
    }

    // --- Step 2: Apply debounce logic ---
    if (millis() - allDeviceButtonStates[deviceID][i].lastPhysicalChange < DEBOUNCE_DELAY) {
      continue; // Still in debounce period, ignore this button for now
    }

    // --- Step 3: Button State Machine Transitions ---
    switch (allDeviceButtonStates[deviceID][i].state) {
      case MAIN_OFF:
        if (currentPhysicalPressed) {
          allDeviceButtonStates[deviceID][i].state = MAIN_PRESSED;
          allDeviceButtonStates[deviceID][i].pressStartTime = millis();
          allDeviceButtonStates[deviceID][i].longPressEventSent = false;
          deviceStateChanged = true; // State changed, flag for publish
        }
        break;

      case MAIN_PRESSED:
        if (!currentPhysicalPressed) { // Button released before LONG_PRESS_THRESHOLD
          allDeviceButtonStates[deviceID][i].state = MAIN_OFF;
          deviceStateChanged = true; // State changed, flag for publish
        } else if (millis() - allDeviceButtonStates[deviceID][i].pressStartTime >= LONG_PRESS_THRESHOLD) {
          // Button held long enough for LONG_PRESS_TRIGGERED
          allDeviceButtonStates[deviceID][i].state = MAIN_HELD;
          allDeviceButtonStates[deviceID][i].longPressEventSent = true;
          deviceStateChanged = true; // State changed, flag for publish
        }
        break;

      case MAIN_HELD:
        if (!currentPhysicalPressed) { // Button released after LONG_PRESS_THRESHOLD
          allDeviceButtonStates[deviceID][i].state = MAIN_OFF;
          deviceStateChanged = true; // State changed, flag for publish
        }
        break;
    }
  }

  // Update last raw inputs for this device for next comparison
  lastRawMCPInputs[deviceID] = currentRawMCPInputs;

  // If any button's logical state changed for this device, publish the updated state
  if (deviceStateChanged) {
    mqtt_publishButtonState(deviceID); // Call MQTT module function
  }
}

// Handles processing of a received discovery ACK packet
void rs485_processSlaveDiscoveryAck(byte deviceID) {
    // Validate deviceID
    if (deviceID >= 16) {
        Serial.printf("Invalid DeviceID (%d) received in ACK response.\n", deviceID);
        return;
    }
    // Serial.printf("Received DISCOVERY ACK from ID %d\n", deviceID); // Uncomment for verbose

    // If this device was previously marked as disconnected, publish "connected"
    if (!discoveredDevices[deviceID]) {
        discoveredDevices[deviceID] = true;
        mqtt_publishDeviceStatus(deviceID, "connected"); // Call MQTT module function
    }
    lastSeenTimestamp[deviceID] = millis(); // Update last seen time for this device
}

// This helper function is specific to rs485.cpp as it acts on internal button states.
// It's not called directly from outside the rs485 module logic.
void rs485_runButtonStateMachine(byte deviceID, byte buttonIndex, bool currentPhysicalPressed) {
    // This function body was integrated directly into processSlaveStatusResponse.
    // It's left as a placeholder for completeness, but it's not called explicitly.
    // Its logic is in rs485_processSlaveStatusResponse function body.
}