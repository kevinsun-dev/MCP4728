/**
 * MCP4728 Basic Operations Example
 * 
 * This example demonstrates basic DAC operations and parameter control
 * without requiring LDAC pin connection.
 * 
 * Hardware:
 * - MCP4728 connected via I2C (SDA, SCL)
 * - Optional: Multimeter to measure outputs on VOUTA, VOUTB, VOUTC, VOUTD
 * 
 * This example demonstrates:
 * 1. Basic DAC value writing
 * 2. Voltage reference control (VDD vs Internal 2.048V)
 * 3. Gain control (1X vs 2X)
 * 4. Reading current settings
 * 5. EEPROM operations
 */

#include <MCP4728.h>

MCP4728 dac;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=================================");
  Serial.println("MCP4728 Basic Operations Example");
  Serial.println("=================================\n");
  
  // Initialize at default address
  if (!dac.begin(0x60)) {
    Serial.println("ERROR: Failed to find MCP4728!");
    Serial.println("Check I2C connections and power.");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ MCP4728 initialized successfully\n");
  
  // Example 1: Set simple DAC values
  Serial.println("Example 1: Setting DAC values");
  Serial.println("------------------------------");
  dac.setChannelValue(MCP4728_CHANNEL_A, 0);      // 0V
  dac.setChannelValue(MCP4728_CHANNEL_B, 1024);   // 1/4 scale
  dac.setChannelValue(MCP4728_CHANNEL_C, 2048);   // 1/2 scale (mid-point)
  dac.setChannelValue(MCP4728_CHANNEL_D, 4095);   // Full scale
  Serial.println("Set Ch A=0, B=1024, C=2048, D=4095");
  delay(1000);
  
  // Example 2: Read back values
  Serial.println("\nExample 2: Reading DAC values");
  Serial.println("------------------------------");
  Serial.print("Channel A: ");
  Serial.println(dac.getChannelValue(MCP4728_CHANNEL_A));
  Serial.print("Channel B: ");
  Serial.println(dac.getChannelValue(MCP4728_CHANNEL_B));
  Serial.print("Channel C: ");
  Serial.println(dac.getChannelValue(MCP4728_CHANNEL_C));
  Serial.print("Channel D: ");
  Serial.println(dac.getChannelValue(MCP4728_CHANNEL_D));
  delay(2000);
  
  // Example 3: Voltage reference control
  Serial.println("\nExample 3: Voltage Reference Control");
  Serial.println("-------------------------------------");
  Serial.println("Setting all channels to internal 2.048V reference...");
  dac.setAllVref(MCP4728_VREF_INTERNAL, MCP4728_VREF_INTERNAL,
                 MCP4728_VREF_INTERNAL, MCP4728_VREF_INTERNAL);
  
  // With internal reference and no gain, max output = 2.048V
  dac.setChannelValue(MCP4728_CHANNEL_A, 4095);  // Should output ~2.048V
  Serial.println("Channel A set to 4095 (should be ~2.048V with internal Vref)");
  delay(2000);
  
  // Example 4: Gain control (2X gain)
  Serial.println("\nExample 4: Gain Control");
  Serial.println("-----------------------");
  Serial.println("Setting 2X gain on all channels...");
  dac.setAllGain(MCP4728_GAIN_2X, MCP4728_GAIN_2X,
                 MCP4728_GAIN_2X, MCP4728_GAIN_2X);
  
  // With internal reference and 2X gain, max output = 4.096V
  dac.setChannelValue(MCP4728_CHANNEL_A, 4095);  // Should output ~4.096V
  Serial.println("Channel A set to 4095 (should be ~4.096V with 2X gain)");
  delay(2000);
  
  // Example 5: Fast write (all channels at once)
  Serial.println("\nExample 5: Fast Write (All Channels)");
  Serial.println("-------------------------------------");
  dac.fastWrite(1000, 2000, 3000, 4000);
  Serial.println("Set all channels: A=1000, B=2000, C=3000, D=4000");
  delay(2000);
  
  // Example 6: Power down mode
  Serial.println("\nExample 6: Power Down Mode");
  Serial.println("--------------------------");
  Serial.println("Powering down Channel B (1kΩ to GND)...");
  dac.setPowerDown(MCP4728_CHANNEL_B, MCP4728_PD_MODE_GND_1K);
  Serial.println("Channel B is now in power-down mode");
  Serial.println("(Output should be near 0V through 1kΩ resistor)");
  delay(2000);
  
  Serial.println("\nWaking up Channel B...");
  dac.setPowerDown(MCP4728_CHANNEL_B, MCP4728_PD_MODE_NORMAL);
  Serial.println("Channel B restored to normal operation");
  delay(2000);
  
  // Example 7: Save to EEPROM
  Serial.println("\nExample 7: Save to EEPROM");
  Serial.println("-------------------------");
  Serial.println("Current settings will be saved to EEPROM...");
  Serial.println("These will be the power-on defaults.");
  
  if (dac.saveToEEPROM()) {
    Serial.println("✓ Settings saved to EEPROM successfully");
  } else {
    Serial.println("✗ Failed to save to EEPROM");
  }
  delay(2000);
  
  // Example 8: Read EEPROM values
  Serial.println("\nExample 8: Read EEPROM Values");
  Serial.println("-----------------------------");
  Serial.print("EEPROM Channel A: ");
  Serial.println(dac.getEEPROMValue(MCP4728_CHANNEL_A));
  Serial.print("EEPROM Channel B: ");
  Serial.println(dac.getEEPROMValue(MCP4728_CHANNEL_B));
  Serial.print("EEPROM Vref A: ");
  Serial.println(dac.getVrefEEPROM(MCP4728_CHANNEL_A) == MCP4728_VREF_INTERNAL ? 
                 "Internal" : "VDD");
  Serial.print("EEPROM Gain A: ");
  Serial.println(dac.getGainEEPROM(MCP4728_CHANNEL_A) == MCP4728_GAIN_2X ? 
                 "2X" : "1X");
  
  Serial.println("\n=================================");
  Serial.println("Examples complete!");
  Serial.println("=================================");
}

void loop() {
  // Continuously sweep Channel A from 0 to 4095
  static uint16_t value = 0;
  static int8_t direction = 1;
  
  dac.setChannelValue(MCP4728_CHANNEL_A, value);
  
  value += direction * 10;
  if (value >= 4095) direction = -1;
  if (value <= 0) direction = 1;
  
  delay(1);  // Smooth sweep
}