# Arduino-compatible library for MCP4728

## Fork Information

This is a fork of the Adafruit MCP4728 library maintained by Kevin Sun. 

**Upstream Repository:** https://github.com/adafruit/Adafruit_MCP4728

### Differences from Upstream

This fork adds the following functionality not present in the original Adafruit library:

- **I2C Address Write/Read**: Change the MCP4728's I2C address programmatically and read the current address
  - Addresses can be changed from 0x60-0x67 (factory default is 0x60)
  - Changes are stored in EEPROM and persist across power cycles
  - Requires LDAC pin connection and control for proper operation

- **General Call Commands**:
  - **General Call Reset**: Reset all MCP4728 devices on the bus, load EEPROM to registers
  - **General Call Wake-up**: Wake all devices from power-down mode
  - **General Call Software Update**: Update all DAC outputs simultaneously

- **Individual Parameter Control**
  - Set Vref, Gain, and Power-Down modes independently without changing DAC values
  - Control individual channels or all channels at once
  - Read current settings from both input registers and EEPROM

- **Enhanced Register Reading**
  - Read Vref, Gain, and Power-Down settings from input registers
  - Read EEPROM settings for all parameters
  - Automatic register caching for improved performance

- **Naming**: Library names referencing "Adafruit" have been updated to reflect forked nature and avoid improper use of company trademarks
  - `Adafruit_MCP4728` -> `MCP4728`

---

This is a MCP4728 4-Channel 12-Bit I2C DAC Library for Arduino

Originally designed to work with the Adafruit MCP4728 Breakout Board
[<img src="https://raw.githubusercontent.com/adafruit/Adafruit_MCP4728/refs/heads/master/assets/board.jpg" width="500px">](https://www.adafruit.com/products/4470)


This chip uses I2C to communicate, 2 pins are required to interface, with an additional pin for I2C address updates

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

# Dependencies
* [Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO)

# Usage

## Basic Operations

### Initializing the DAC

```cpp
#include 

MCP4728 dac;

void setup() {
  // Initialize at default address (0x60)
  if (!dac.begin()) {
    Serial.println("Failed to find MCP4728");
    return;
  }
  
  // Or initialize at a specific address
  // dac.begin(0x61);
}
```

### Writing DAC Values

```cpp
// Write to a single channel (0-4095 for 12-bit resolution)
dac.setChannelValue(MCP4728_CHANNEL_A, 2048);  // Mid-scale

// Write to all channels at once (fastest method)
dac.fastWrite(0, 1024, 2048, 4095);  // A, B, C, D

// Write with specific parameters
dac.setChannelValue(MCP4728_CHANNEL_A, 2048,
                    MCP4728_VREF_INTERNAL,    // Use 2.048V internal reference
                    MCP4728_GAIN_2X,          // 2X gain for 4.096V output
                    MCP4728_PD_MODE_NORMAL,   // Normal operation
                    false);                   // Update immediately (UDAC=0)
```

### Reading DAC Values

```cpp
// Read current channel value
uint16_t value = dac.getChannelValue(MCP4728_CHANNEL_A);

// Read value stored in EEPROM (power-on default)
uint16_t eeprom_value = dac.getEEPROMValue(MCP4728_CHANNEL_A);
```

### Saving to EEPROM

```cpp
// Save current settings to EEPROM (becomes power-on default)
dac.saveToEEPROM();
// Settings persist across power cycles
```

## Changing the I2C Address

The MCP4728 supports 8 different I2C addresses (0x60-0x67). The factory default is 0x60. You can change this address programmatically:

```cpp
#include <MCP4728.h>

MCP4728 dac;

void setup() {
  Serial.begin(115200);
  
  // Initialize at current address (0x60 is factory default)
  if (!dac.begin(0x60)) {
    Serial.println("Failed to find MCP4728");
    return;
  }
  
  // Change address from 0x60 to 0x61
  // Parameters: current_address, new_address, sda_pin, scl_pin, ldac_pin
  bool success = dac.writeI2CAddress(0x60, 0x61, SDA, SCL, LDAC_PIN);
  
  if (success) {
    Serial.println("Address changed successfully!");
    
    // Re-initialize at new address
    dac.begin(0x61);
  } else {
    Serial.println("Address change failed!");
  }
}
```

**Important Notes:**
- The **LDAC pin must be connected** and controllable for address write/read to work
- If LDAC is permanently pulled low on your board, you cannot change the address
- The address change is permanent (stored in EEPROM)
- On some boards, you may need to find the LDAC trace for your MCP4728 and connect it to a GPIO pin
    - While not reccomended, you can hold a jumper wire in contact with the LDAC pin and run the address update script
- Address changes take ~100ms to complete due to EEPROM write time

## Reading the Current I2C Address

You can read the current I2C address without knowing it in advance:

```cpp
#include <MCP4728.h>

MCP4728 dac;

void setup() {
  Serial.begin(115200);
  
  // Read current address (requires SDA, SCL, and LDAC pins)
  uint8_t current_addr = dac.readI2CAddress(SDA, SCL, LDAC_PIN);
  
  if (current_addr != 0xFF) {
    Serial.print("Current I2C address: 0x");
    Serial.println(current_addr, HEX);
    
    // Initialize at detected address
    dac.begin(current_addr);
  } else {
    Serial.println("Failed to read address!");
  }
}
```

## Using General Call Commands

General Call commands affect **all MCP4728 devices** on the I2C bus:

```cpp
// Reset all devices (loads EEPROM values to input registers)
dac.generalCallReset();

// Wake all devices from power-down
dac.generalCallWakeup();

// Update all DAC outputs simultaneously
dac.generalCallUpdate();
```

## Controlling Vref, Gain, and Power-Down

You can control these parameters independently of DAC values:

```cpp
// Set voltage reference (per-channel or all at once)
dac.setVref(MCP4728_CHANNEL_A, MCP4728_VREF_INTERNAL);  // 2.048V internal
dac.setAllVref(MCP4728_VREF_VDD, MCP4728_VREF_VDD,     // Use VDD as reference
               MCP4728_VREF_VDD, MCP4728_VREF_VDD);

// Set gain (1X or 2X, only applies with internal Vref)
dac.setGain(MCP4728_CHANNEL_A, MCP4728_GAIN_2X);        // 2X gain = 4.096V
dac.setAllGain(MCP4728_GAIN_1X, MCP4728_GAIN_1X,
               MCP4728_GAIN_1X, MCP4728_GAIN_1X);

// Power down channels to save power
dac.setPowerDown(MCP4728_CHANNEL_B, MCP4728_PD_MODE_GND_1K);   // 1kΩ to GND
dac.setAllPowerDown(MCP4728_PD_MODE_NORMAL,     // Channel A normal
                    MCP4728_PD_MODE_GND_1K,     // Channel B: 1kΩ to GND
                    MCP4728_PD_MODE_GND_100K,   // Channel C: 100kΩ to GND  
                    MCP4728_PD_MODE_GND_500K);  // Channel D: 500kΩ to GND
```

## Reading Current Settings

Read current configuration from input registers or EEPROM:

```cpp
// Read from input registers (current active settings)
MCP4728_vref_t vref = dac.getVref(MCP4728_CHANNEL_A);
MCP4728_gain_t gain = dac.getGain(MCP4728_CHANNEL_A);
MCP4728_pd_mode_t pd = dac.getPowerDown(MCP4728_CHANNEL_A);

// Read from EEPROM (power-on default settings)
MCP4728_vref_t vref_eeprom = dac.getVrefEEPROM(MCP4728_CHANNEL_A);
MCP4728_gain_t gain_eeprom = dac.getGainEEPROM(MCP4728_CHANNEL_A);
MCP4728_pd_mode_t pd_eeprom = dac.getPowerDownEEPROM(MCP4728_CHANNEL_A);

// Values are automatically cached during begin() and after certain operations
uint16_t current_value = dac.getChannelValue(MCP4728_CHANNEL_A);
uint16_t eeprom_value = dac.getEEPROMValue(MCP4728_CHANNEL_A);
```

Written by Bryan Siepert for Adafruit Industries.
Modified and maintained by Kevin Sun.
BSD license, check license.txt for more information
All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "MCP4728" and install the library.