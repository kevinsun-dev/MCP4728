/**
 * MCP4728 General Call Commands Example
 * 
 * This example demonstrates General Call commands that affect ALL MCP4728
 * devices on the I2C bus. These commands DO NOT require LDAC control.
 * 
 * Hardware:
 * - One or more MCP4728 devices on the I2C bus
 * 
 * General Call Commands:
 * 1. Reset - Loads EEPROM values to input registers (all devices)
 * 2. Wake-up - Wakes all devices from power-down mode
 * 3. Update - Triggers simultaneous DAC output update (all devices)
 * 
 * NOTE: These commands affect ALL MCP4728 devices on the bus, regardless
 * of their I2C address.
 */

#include <MCP4728.h>

MCP4728 dac;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n========================================");
  Serial.println("MCP4728 General Call Commands Example");
  Serial.println("========================================\n");
  
  // Initialize at default address
  if (!dac.begin(0x60)) {
    Serial.println("ERROR: Failed to find MCP4728!");
    Serial.println("Check I2C connections and power.");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ MCP4728 initialized successfully\n");
  
  Serial.println("WARNING: General Call commands affect ALL MCP4728");
  Serial.println("devices on the I2C bus, regardless of address!\n");
  delay(2000);
  
  // Example 1: General Call Reset
  Serial.println("Example 1: General Call Reset");
  Serial.println("-----------------------------");
  Serial.println("This loads EEPROM values to all input registers");
  Serial.println("Useful for recovering known-good state from EEPROM\n");
  
  // First, set some values
  Serial.println("Setting current values:");
  dac.setChannelValue(MCP4728_CHANNEL_A, 1000);
  dac.setChannelValue(MCP4728_CHANNEL_B, 2000);
  Serial.print("  Before reset - Channel A: ");
  Serial.println(dac.getChannelValue(MCP4728_CHANNEL_A));
  
  delay(1000);
  
  Serial.println("\nIssuing General Call Reset...");
  if (dac.generalCallReset()) {
    Serial.println("✓ Reset successful");
    Serial.print("  After reset - Channel A: ");
    Serial.println(dac.getChannelValue(MCP4728_CHANNEL_A));
    Serial.println("  (Values restored from EEPROM)");
  } else {
    Serial.println("✗ Reset failed");
  }
  
  delay(3000);
  
  // Example 2: General Call Wake-up
  Serial.println("\nExample 2: General Call Wake-up");
  Serial.println("-------------------------------");
  Serial.println("This wakes all devices from power-down mode\n");
  
  // Put a channel to sleep
  Serial.println("Putting Channel B into power-down mode...");
  dac.setPowerDown(MCP4728_CHANNEL_B, MCP4728_PD_MODE_GND_1K);
  Serial.println("  Channel B is powered down (1kΩ to GND)");
  
  delay(2000);
  
  Serial.println("\nIssuing General Call Wake-up...");
  if (dac.generalCallWakeup()) {
    Serial.println("✓ Wake-up successful");
    Serial.println("  All channels restored to normal power mode");
  } else {
    Serial.println("✗ Wake-up failed");
  }
  
  delay(3000);
  
  // Example 3: General Call Software Update
  Serial.println("\nExample 3: General Call Software Update");
  Serial.println("---------------------------------------");
  Serial.println("This triggers DAC output update when UDAC=1\n");
  
  Serial.println("NOTE: This is useful when you've written values");
  Serial.println("with UDAC=1 (update deferred) and want to update");
  Serial.println("all DAC outputs simultaneously.\n");
  
  // Write values with UDAC=1 (deferred update)
  Serial.println("Writing values with deferred update (UDAC=1):");
  dac.setChannelValue(MCP4728_CHANNEL_A, 500, 
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X,
                      MCP4728_PD_MODE_NORMAL, true);  // UDAC=1
  dac.setChannelValue(MCP4728_CHANNEL_B, 1500,
                      MCP4728_VREF_VDD, MCP4728_GAIN_1X,
                      MCP4728_PD_MODE_NORMAL, true);  // UDAC=1
  Serial.println("  Values written but outputs not yet updated");
  
  delay(2000);
  
  Serial.println("\nIssuing General Call Software Update...");
  if (dac.generalCallUpdate()) {
    Serial.println("✓ Update successful");
    Serial.println("  All DAC outputs updated simultaneously");
  } else {
    Serial.println("✗ Update failed");
  }
  
  Serial.println("\n========================================");
  Serial.println("Examples complete!");
  Serial.println("========================================");
  Serial.println("\nNote: General Call commands are broadcast");
  Serial.println("commands that work without LDAC control.");
}

void loop() {
  // Nothing to do - examples run once in setup()
  delay(1000);
}