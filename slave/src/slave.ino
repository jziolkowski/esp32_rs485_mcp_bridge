#include <Arduino.h>
#include <Wire.h>         // Required for I2C communication with MCP23017
#include <SoftwareSerial.h> // Required for SoftwareSerial communication

// Note: No external MCP23017 library is used in this version for MCP control.
// All MCP operations are done via direct I2C (Wire library) register writes/reads.

/****************************************************************************************
 * GPIO PIN MAPPING (ADJUST THESE!)
 *
 * This code focuses on establishing basic RS-485 communication.
 *
 * --------------------------------------------------------------------------------------
 * Example for a standard Arduino Uno/Nano/Duemilanove:
 * - RS485_RX_PIN: D10, RS485_TX_PIN: D11
 * - RS485_DE_PIN: D5, RS485_NRE_PIN: D6 (Moved to avoid conflict with D2/D3 for interrupt)
 * - MCP_INT_PIN: D2 (Standard external interrupt pin 0, directly referred to as INT0)
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
// MCP23017 Interrupt pin (will be initialized but not actively used in loop for this test)
#define MCP_INT_PIN     2
// Heartbeat LED pin
#define LED_PIN         13 // Built-in LED

// --- RS-485 Baud Rate ---
#define RS485_BAUD_RATE 57600

// Initialize SoftwareSerial for RS-485 communication
SoftwareSerial myRs485Serial(RS485_RX_PIN, RS485_TX_PIN);

// MCP23017 I2C Address (A0, A1, A2 all low - default)
#define MCP23017_ADDRESS 0x20

// --- Global Variables (MCP-related vars remain but won't be used in loop for this test) ---
volatile bool mcpInterruptFlag = false; // Still declared, but its check in loop removed
uint16_t previousRawMCPInputs = 0xFFFF;
byte myDeviceID; // Still read but not used in loop logic here

// --- LED Blinking Variables ---
unsigned long lastLedToggleTime = 0;
const unsigned long LED_BLINK_INTERVAL_MS = 1000;
bool ledState = LOW;

// --- Function Prototypes ---
void setupMCP23017();      // Configures the MCP23017 via direct Wire calls
uint16_t readMCP23017GPIO(); // Reads all 16 pins from MCP23017 via direct Wire calls
void mcpInterruptISR();    // ISR for the MCP23017's INT pin
void readAndClearMcpInterrupts(); // Debug function using direct Wire calls


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n--- ATtiny Slave: Basic RS-485 Echo Test ---");

  // --- Setup LED Pin for Heartbeat ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  // Initialize I2C bus (MCP23017 is still initialized, but its data not processed in loop)
  Wire.begin();
  Serial.println("I2C Wire library initialized (using default SDA/SCL pins).");

  // --- Configure RS-485 control pins for ST3485DE ---
  pinMode(RS485_DE_PIN, OUTPUT);
  pinMode(RS485_NRE_PIN, OUTPUT);
  // Set ST3485DE to Receive mode (default): DE LOW, ~RE LOW
  digitalWrite(RS485_DE_PIN, LOW);
  digitalWrite(RS485_NRE_PIN, LOW);
  Serial.println("RS-485 DE/~RE pins configured to Receive mode.");

  // Initialize SoftwareSerial for RS-485 communication
  myRs485Serial.begin(RS485_BAUD_RATE);
  Serial.print("SoftwareSerial for RS-485 initialized on RX:");
  Serial.print(RS485_RX_PIN);
  Serial.print("/TX:");
  Serial.print(RS485_TX_PIN);
  Serial.print(" at ");
  Serial.print(RS485_BAUD_RATE);
  Serial.println(" bps.");

  // --- MCP23017 Setup (Configuration and initial read, but loop won't process interrupts) ---
  setupMCP23017();
  Serial.println("MCP23017 setup commands sent (interrupts configured but not actively processed in loop).");
  readAndClearMcpInterrupts(); // Clear any pending MCP interrupts from setup
  previousRawMCPInputs = readMCP23017GPIO(); // Initial read
  byte dipSwitchRaw = (byte)(previousRawMCPInputs >> 12);
  myDeviceID = ~dipSwitchRaw & 0x0F; // Store ID for future use

  pinMode(MCP_INT_PIN, INPUT_PULLUP);
  attachInterrupt(INT0, mcpInterruptISR, FALLING); // Interrupt is attached but its flag won't be processed here
  Serial.print("MCP23017 Interrupt attached to INT0 (Digital Pin 2), but not used in this test.\n");


  Serial.println("--- Setup Complete. Entering loop ---");
  Serial.println("Waiting for RS-485 byte 0xAA. Will respond with 0xBB.");
}


void loop() {
  // LED Blinking (heartbeat)
  if (millis() - lastLedToggleTime >= LED_BLINK_INTERVAL_MS) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastLedToggleTime = millis();
  }

  // --- Basic RS-485 Echo Logic ---
  if (myRs485Serial.available()) {
    byte receivedByte = myRs485Serial.read();
    Serial.print("RS485 RX: Received 0x");
    Serial.println(receivedByte, HEX);

    if (receivedByte == 0xAA) {
      Serial.println("RS485 TX: Received 0xAA. Responding with 0xBB...");
      
      // Set ST3485DE to Transmit Mode: DE HIGH, ~RE HIGH
      digitalWrite(RS485_NRE_PIN, HIGH); // Disable Receiver (~RE LOW to HIGH)
      digitalWrite(RS485_DE_PIN, HIGH);  // Enable Driver (DE LOW to HIGH)
      delay(1); // Small delay for transceiver to switch modes

      myRs485Serial.write(0xBB); // Send response
      myRs485Serial.flush(); // Wait for transmit buffer to empty

      // Set ST3485DE back to Receive Mode: DE LOW, ~RE LOW
      digitalWrite(RS485_DE_PIN, LOW);
      digitalWrite(RS485_NRE_PIN, LOW);
      Serial.println("RS485 TX: Sent 0xBB. Switched back to Receive mode.");
    } else {
      Serial.println("RS485 RX: Received unexpected byte.");
    }
  }

  // MCP23017 interrupt flag is not processed in loop for this basic RS-485 test.
}


// --- MCP23017 Specific Functions (using ONLY Wire.h) ---

// Configures the MCP23017 I/O Expander via direct Wire.h calls.
void setupMCP23017() {
  // Set all 16 pins as inputs
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x00); Wire.write(0xFF); Wire.endTransmission(); // IODIRA: all inputs (0xFF)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x01); Wire.write(0xFF); Wire.endTransmission(); // IODIRB: all inputs (0xFF)

  // Enable internal pull-up resistors for all 16 input pins
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0C); Wire.write(0xFF); Wire.endTransmission(); // GPPUA: enable pull-ups (0xFF)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0D); Wire.write(0xFF); Wire.endTransmission(); // GPPUB: enable pull-ups (0xFF)

  // Configure IOCON register for mirrored interrupts and active-low output
  // CORRECTED VALUE: 0b01000000 (0x40) to set MIRROR (Bit 6)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x0A); Wire.write(0b01000000); Wire.endTransmission(); // IOCON: MIRROR enabled (0x40)

  // --- CRITICAL FIX: Correct GPINTEN addresses! ---
  // Enable interrupt-on-change for all 16 input pins
  // GPINTENA is at 0x04, GPINTENB is at 0x05
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x04); Wire.write(0xFF); Wire.endTransmission(); // GPINTENA: interrupt on change (0xFF)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x05); Wire.write(0xFF); Wire.endTransmission(); // GPINTENB: interrupt on change (0xFF)

  // Explicitly set INTCON (Interrupt Control) registers to 0x00
  // INTCONA is at 0x08, INTCONB is at 0x09
  // Setting to 0x00 means "compare against previous value" (interrupt on any change)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x08); Wire.write(0x00); Wire.endTransmission(); // INTCONA: Compare to previous (0x00)
  Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x09); Wire.write(0x00); Wire.endTransmission(); // INTCONB: Compare to previous (0x00)
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
    // Serial.print("  INTF (Flagged pins): GPB="); Serial.print(intfb, BIN); Serial.print(", GPA="); Serial.println(intfa, BIN); // Commented for brevity

    // Read INTCAP (Interrupt Captured) registers. This action clears the interrupt on the MCP23017 INT pin.
    Wire.beginTransmission(MCP23017_ADDRESS); Wire.write(0x10); Wire.endTransmission(false);
    Wire.requestFrom(MCP23017_ADDRESS, 2);
    uint8_t intcapa = Wire.read(); uint8_t intcapb = Wire.read();
    // Serial.print("  INTCAP (State at interrupt): GPB="); Serial.print(intcapb, BIN); Serial.print(", GPA="); Serial.println(intcapa, BIN); // Commented for brevity
    // Serial.println("--- End MCP Interrupt Clear & Debug ---"); // Commented for brevity
}

// Interrupt Service Routine (ISR) for the MCP23017's INT pin.
void mcpInterruptISR() {
  mcpInterruptFlag = true;
}