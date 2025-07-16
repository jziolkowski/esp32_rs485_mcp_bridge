#ifndef ETHERNET_H
#define ETHERNET_H

#include "common.h" // Include common definitions, which includes Arduino.h etc.

// --- Ethernet Configuration for ESP32-ETH01 (LAN8720A) ---
// These defines describe the GPIO pin mapping for the LAN8720A PHY on your board.
#define ETH_MDC_PIN             23
#define ETH_MDIO_PIN            18
#define ETH_TYPE                ETH_PHY_LAN8720
#define ETH_CLK_MODE            ETH_CLOCK_GPIO0_IN // External 50MHz clock input from GPIO0
#define ETH_POWER_PIN           16                 // GPIO pin to power cycle/enable the LAN8720A PHY (HIGH to enable)


// --- Function Prototypes for Ethernet Module ---
// These functions are implemented in ethernet.cpp
void ethernet_setup(); // Initializes the Ethernet interface
void ETH_GotIP_Handler(arduino_event_id_t event, arduino_event_info_t info); // Callback for IP obtained event
void ETH_Stop_Handler(arduino_event_id_t event, arduino_event_info_t info);   // Callback for Ethernet stop/IP lost event


#endif // ETHERNET_H