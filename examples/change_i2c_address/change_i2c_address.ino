/**
 * MCP4728 I2C Address Write Example
 * 
 * This example demonstrates how to change the I2C address of an MCP4728 DAC.
 * The address change is permanent and stored in EEPROM.
 * 
 * IMPORTANT: The LDAC pin MUST be connected to a controllable GPIO pin for
 * this to work. If LDAC is permanently pulled low on your breakout board,
 * you will need to:
 *   1. Cut the trace to the pull-down resistor, or
 *   2. Desolder the pull-down resistor, and
 *   3. Connect LDAC to a GPIO pin
 * 
 * Hardware Connections:
 * - SDA: Connect to your board's SDA pin
 * - SCL: Connect to your board's SCL pin  
 * - LDAC: Connect to a GPIO pin (e.g., pin 2 on Arduino, GPIO 2 on Pico)
 * - VDD: 3.3V or 5V (depending on your board)
 * - VSS: GND
 * 
 * This example will:
 * 1. Read the current I2C address
 * 2. Change it from 0x60 to 0x61
 * 3. Verify the change was successful
 */

#include <MCP4728.h>

// Pin definitions - adjust for your board
#define LDAC_PIN 2  // Change this to match your wiring

// Address configuration
#define CURRENT_ADDR 0x60  // Factory default
#define NEW_ADDR 0x61      // New address to assign

MCP4728 dac;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=================================");
  Serial.println("MCP4728 I2C Address Change Example");
  Serial.println("=================================\n");
  
  // Step 1: Read current address
  Serial.println("Step 1: Reading current I2C address...");
  uint8_t detected_addr = dac.readI2CAddress(SDA, SCL, LDAC_PIN);
  
  if (detected_addr != 0xFF) {
    Serial.print("Current address: 0x");
    Serial.println(detected_addr, HEX);
  } else {
    Serial.println("ERROR: Failed to read address!");
    Serial.println("Check that LDAC is connected and not permanently pulled low.");
    while (1) {
      delay(1000);
    }
  }
  
  // Step 2: Initialize at current address
  Serial.println("\nStep 2: Initializing DAC...");
  if (!dac.begin(detected_addr)) {
    Serial.println("ERROR: Failed to initialize MCP4728!");
    Serial.println("Check I2C connections (SDA, SCL) and power.");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("DAC initialized successfully");
  
  // Step 3: Test DAC by setting a value
  Serial.println("\nStep 3: Testing DAC functionality...");
  if (dac.setChannelValue(MCP4728_CHANNEL_A, 2048)) {
    Serial.println("DAC test successful (set channel A to mid-scale)");
  } else {
    Serial.println("WARNING: DAC test failed");
  }
  
  // Step 4: Change address
  Serial.println("\nStep 4: Changing I2C address...");
  Serial.print("Changing from 0x");
  Serial.print(CURRENT_ADDR, HEX);
  Serial.print(" to 0x");
  Serial.println(NEW_ADDR, HEX);
  
  bool success = dac.writeI2CAddress(CURRENT_ADDR, NEW_ADDR, SDA, SCL, LDAC_PIN);
  
  if (success) {
    Serial.println("✓ Address change command successful!");
  } else {
    Serial.println("✗ Address change FAILED!");
    Serial.println("\nPossible causes:");
    Serial.println("- LDAC is not connected or is permanently pulled low");
    Serial.println("- Current address is incorrect");
    Serial.println("- I2C communication error");
    while (1) {
      delay(1000);
    }
  }
  
  // Step 5: Verify new address
  Serial.println("\nStep 5: Verifying address change...");
  delay(200); // Give EEPROM time to settle
  
  uint8_t verify_addr = dac.readI2CAddress(SDA, SCL, LDAC_PIN);
  
  if (verify_addr == NEW_ADDR) {
    Serial.println("✓ SUCCESS! Address changed successfully!");
    Serial.print("New address confirmed: 0x");
    Serial.println(verify_addr, HEX);
  } else {
    Serial.println("✗ ERROR: Address verification failed!");
    Serial.print("Expected 0x");
    Serial.print(NEW_ADDR, HEX);
    Serial.print(", got 0x");
    Serial.println(verify_addr, HEX);
  }
  
  // Step 6: Re-initialize at new address
  Serial.println("\nStep 6: Re-initializing at new address...");
  if (dac.begin(NEW_ADDR)) {
    Serial.println("✓ DAC re-initialized at new address");
    
    // Test it works
    if (dac.setChannelValue(MCP4728_CHANNEL_A, 4095)) {
      Serial.println("✓ DAC functioning correctly at new address");
    }
  } else {
    Serial.println("✗ Failed to initialize at new address");
  }
  
  Serial.println("\n=================================");
  Serial.println("Address change complete!");
  Serial.println("=================================");
  Serial.println("\nIMPORTANT: Update your code to use the new address (0x");
  Serial.print(NEW_ADDR, HEX);
  Serial.println(") in future sketches.");
  Serial.println("\nThe new address is stored in EEPROM and will persist");
  Serial.println("across power cycles and resets.");
}

void loop() {
  // Nothing to do - address change is complete
  delay(1000);
}