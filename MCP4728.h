/*!
 *  @file MCP4728.h
 *
 * 	I2C Driver for the MCP4728 4-Channel 12-Bit I2C DAC library
 *
 * 	This is a fork of a library for the Adafruit MCP4728 breakout:
 * 	https://www.adafruit.com/products/4470
 *
 *  The following text comes from the original developers at Adafruit,
 *  Kevin does not work there, but thinks their products are dope!
 * 
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _MCP4728_H
#define _MCP4728_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>

#define MCP4728_I2CADDR_DEFAULT 0x60 ///< MCP4728 default i2c address

#define MCP4728_MULTI_IR_CMD                                                   \
  0x40 ///< Command to write to the input register only
#define MCP4728_MULTI_EEPROM_CMD                                               \
  0x50 ///< Command to write to the input register and EEPROM
#define MCP4728_FAST_WRITE_CMD                                                 \
  0xC0 ///< Command to write all channels at once with

// General Call Commands (Section 5.4)
#define MCP4728_GENERAL_CALL_RESET 0x06   ///< General Call Reset command
#define MCP4728_GENERAL_CALL_WAKEUP 0x09  ///< General Call Wake-up command  
#define MCP4728_GENERAL_CALL_UPDATE 0x08  ///< General Call Software Update command
#define MCP4728_GENERAL_CALL_ADDR 0x00    ///< General Call address

// Individual Register Write Commands (Section 5.6.5, 5.6.6, 5.6.7)
#define MCP4728_VREF_WRITE_CMD 0x80       ///< Write Vref to input registers
#define MCP4728_GAIN_WRITE_CMD 0xC0       ///< Write Gain to input registers
#define MCP4728_PD_WRITE_CMD 0xA0         ///< Write Power-Down to input registers

/**
 * @brief Power status values
 *
 * Allowed values for `setPowerMode`.
 */

typedef enum pd_mode {
  MCP4728_PD_MODE_NORMAL, ///< Normal; the channel outputs the given value as
                          ///< normal.
  MCP4728_PD_MODE_GND_1K, ///< VOUT is loaded with 1 kΩ resistor to ground. Most
                          ///< of the channel circuits are powered off.
  MCP4728_PD_MODE_GND_100K, ///< VOUT is loaded with 100 kΩ resistor to ground.
                            ///< Most of the channel circuits are powered off.
  MCP4728_PD_MODE_GND_500K, ///< VOUT is loaded with 500 kΩ resistor to ground.
                            ///< Most of the channel circuits are powered off.
} MCP4728_pd_mode_t;

/**
 * @brief Example enum values
 *
 * Allowed values for `setGain`.
 */
typedef enum gain {
  MCP4728_GAIN_1X,
  MCP4728_GAIN_2X,
} MCP4728_gain_t;

/**
 * @brief Ex
 *
 * Allowed values for `setVref`.
 */
typedef enum vref {
  MCP4728_VREF_VDD,
  MCP4728_VREF_INTERNAL,
} MCP4728_vref_t;

/**
 * @brief Example enum values
 *
 * Allowed values for `setChannelGain`.
 */
typedef enum channel {
  MCP4728_CHANNEL_A,
  MCP4728_CHANNEL_B,
  MCP4728_CHANNEL_C,
  MCP4728_CHANNEL_D,
} MCP4728_channel_t;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MCP4728 I2C Digital Potentiometer
 */
class MCP4728 {
public:
  MCP4728();
  bool begin(uint8_t i2c_address = MCP4728_I2CADDR_DEFAULT,
             TwoWire *wire = &Wire);

  bool setChannelValue(MCP4728_channel_t channel, uint16_t new_value,
                       MCP4728_vref_t new_vref = MCP4728_VREF_VDD,
                       MCP4728_gain_t new_gain = MCP4728_GAIN_1X,
                       MCP4728_pd_mode_t new_pd_mode = MCP4728_PD_MODE_NORMAL,
                       bool udac = false);

  bool fastWrite(uint16_t channel_a_value, uint16_t channel_b_value,
                 uint16_t channel_c_value, uint16_t channel_d_value);
  bool saveToEEPROM(void);

  uint16_t getChannelValue(MCP4728_channel_t channel);
  uint16_t getEEPROMValue(MCP4728_channel_t channel);
  
  // General Call Commands
  bool generalCallReset(void);
  bool generalCallWakeup(void);
  bool generalCallUpdate(void);
  
  // Individual parameter control
  bool setVref(MCP4728_channel_t channel, MCP4728_vref_t vref);
  bool setAllVref(MCP4728_vref_t vref_a, MCP4728_vref_t vref_b,
                  MCP4728_vref_t vref_c, MCP4728_vref_t vref_d);
  bool setGain(MCP4728_channel_t channel, MCP4728_gain_t gain);
  bool setAllGain(MCP4728_gain_t gain_a, MCP4728_gain_t gain_b,
                  MCP4728_gain_t gain_c, MCP4728_gain_t gain_d);
  bool setPowerDown(MCP4728_channel_t channel, MCP4728_pd_mode_t pd_mode);
  bool setAllPowerDown(MCP4728_pd_mode_t pd_a, MCP4728_pd_mode_t pd_b,
                       MCP4728_pd_mode_t pd_c, MCP4728_pd_mode_t pd_d);
  
  // Read current settings from input registers
  MCP4728_vref_t getVref(MCP4728_channel_t channel);
  MCP4728_gain_t getGain(MCP4728_channel_t channel);
  MCP4728_pd_mode_t getPowerDown(MCP4728_channel_t channel);
  
  // Read EEPROM settings
  MCP4728_vref_t getVrefEEPROM(MCP4728_channel_t channel);
  MCP4728_gain_t getGainEEPROM(MCP4728_channel_t channel);
  MCP4728_pd_mode_t getPowerDownEEPROM(MCP4728_channel_t channel);

  bool writeI2CAddress(uint8_t current_address, uint8_t new_address, 
                       uint8_t sda_pin, uint8_t scl_pin, uint8_t ldac_pin);
  uint8_t readI2CAddress(uint8_t sda_pin, uint8_t scl_pin, uint8_t ldac_pin);

private:
  bool _init(void);
  bool _readRegisters(void);  // Read all 24 bytes to update cached values
  
  // Cached register values
  uint16_t _values[4];        // Current DAC values
  MCP4728_vref_t _vref[4];    // Current Vref settings
  MCP4728_gain_t _gain[4];    // Current Gain settings
  MCP4728_pd_mode_t _pd[4];   // Current Power-Down settings
  
  uint16_t _values_eeprom[4];        // EEPROM DAC values
  MCP4728_vref_t _vref_eeprom[4];    // EEPROM Vref settings
  MCP4728_gain_t _gain_eeprom[4];    // EEPROM Gain settings
  MCP4728_pd_mode_t _pd_eeprom[4];   // EEPROM Power-Down settings
  
  // Bit-bang I2C helpers for address write (requires precise LDAC timing)
  void _i2c_bb_init(uint8_t sda_pin, uint8_t scl_pin);
  void _i2c_bb_start(uint8_t sda_pin, uint8_t scl_pin);
  void _i2c_bb_stop(uint8_t sda_pin, uint8_t scl_pin);
  bool _i2c_bb_write_byte(uint8_t data, uint8_t sda_pin, uint8_t scl_pin);
  bool _i2c_bb_write_byte_ldac(uint8_t data, uint8_t sda_pin, uint8_t scl_pin, 
                                uint8_t ldac_pin);
  uint8_t _i2c_bb_read_byte(bool ack, uint8_t sda_pin, uint8_t scl_pin);

  Adafruit_I2CDevice *i2c_dev;
};

#endif