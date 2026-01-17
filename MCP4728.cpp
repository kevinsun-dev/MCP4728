/*!
 *  @file MCP4728.cpp
 *
 *  @mainpage MCP4728 4-Channel 12-Bit I2C DAC library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the MCP4728 4-Channel 12-Bit I2C DAC library
 *
 * 	This is a fork of a library for the Adafruit MCP4728 breakout:
 * 	https://www.adafruit.com/product/4470
 *
 *  The following text comes from the original developers at Adafruit,
 *  Kevin does not work there, but thinks their products are dope!
 * 
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *
 *  @section author Author
 *
 *  Kevin Sun
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "MCP4728.h"
#include "Arduino.h"
#include <Wire.h>

/*!
 *    @brief  Instantiates a new MCP4728 class
 */
MCP4728::MCP4728(void) {
  // Initialize cached values to defaults
  for (int i = 0; i < 4; i++) {
    _values[i] = 0;
    _vref[i] = MCP4728_VREF_INTERNAL;
    _gain[i] = MCP4728_GAIN_1X;
    _pd[i] = MCP4728_PD_MODE_NORMAL;
    
    _values_eeprom[i] = 0;
    _vref_eeprom[i] = MCP4728_VREF_INTERNAL;
    _gain_eeprom[i] = MCP4728_GAIN_1X;
    _pd_eeprom[i] = MCP4728_PD_MODE_NORMAL;
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
boolean MCP4728::begin(uint8_t i2c_address, TwoWire *wire) {
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }
  
  // Read current register values
  _readRegisters();

  return true;
}

/**
 * @brief Sets the input register for a given channel to the specified settings
 *
 * @param channel The channel to update
 * @param new_value The new value to assign
 * @param new_vref Optional vref setting - Defaults to `MCP4728_VREF_VDD`
 * @param new_gain Optional gain setting - Defaults to `MCP4728_GAIN_1X`
 * @param new_pd_mode Optional power down mode setting - Defaults to
 * `MCP4728_PD_MOOE_NORMAL`
 * @param udac Optional UDAC setting - Defaults to `false`, latching
 * immediately. Set to `true` to latch when the LDAC pin is pulled low
 *
 * @return true if the write was successful
 * @return false if there was an error with I2C communication between the MCU
 * and the DAC
 */
bool MCP4728::setChannelValue(
    MCP4728_channel_t channel, uint16_t new_value, MCP4728_vref_t new_vref,
    MCP4728_gain_t new_gain, MCP4728_pd_mode_t new_pd_mode, bool udac) {

  uint8_t output_buffer[3];

  // build the setter header/ "address"
  // 0 1 0 0 0 DAC1 DAC0 UDAC[A]
  uint8_t sequential_write_cmd = MCP4728_MULTI_IR_CMD;
  sequential_write_cmd |= (channel << 1);
  sequential_write_cmd |= udac;

  output_buffer[0] = sequential_write_cmd;
  // VREF PD1 PD0 Gx D11 D10 D9 D8 [A] D7 D6 D5 D4 D3 D2 D1 D0 [A]
  new_value |= (new_vref << 15);
  new_value |= (new_pd_mode << 13);
  new_value |= (new_gain << 12);

  output_buffer[1] = new_value >> 8;
  output_buffer[2] = new_value & 0xFF;

  if (!i2c_dev->write(output_buffer, 3)) {
    return false;
  }
  return true;
}
/**
 * @brief Set the values of all four channels simultaneously with minimal delay
 * or configuration
 *
 * @param channel_a_value The value to assign to channel A
 * @param channel_b_value The value to assign to channel B
 * @param channel_c_value The value to assign to channel C
 * @param channel_d_value The value to assign to channel D
 * @return true if the write was successful
 * @return false if there was an error with I2C communication between the MCU
 * and the DAC
 */
bool MCP4728::fastWrite(uint16_t channel_a_value,
                                 uint16_t channel_b_value,
                                 uint16_t channel_c_value,
                                 uint16_t channel_d_value) {

  uint8_t output_buffer[8];

  output_buffer[0] = channel_a_value >> 8;
  output_buffer[1] = channel_a_value & 0xFF;

  output_buffer[2] = channel_b_value >> 8;
  output_buffer[3] = channel_b_value & 0xFF;

  output_buffer[4] = channel_c_value >> 8;
  output_buffer[5] = channel_c_value & 0xFF;

  output_buffer[6] = channel_d_value >> 8;
  output_buffer[7] = channel_d_value & 0xFF;

  if (!i2c_dev->write(output_buffer, 8)) {
    return false;
  }
  return true;
}

/**
 * @brief Saves the DAC's input register settings to the internal EEPROM,
 * makeing them the default values when the ADC is powered on
 *
 * @return true if the write was successful
 * @return false if there was an error with I2C communication between the MCU
 * and the DAC */

bool MCP4728::saveToEEPROM(void) {
  uint8_t input_buffer[24];
  uint8_t output_buffer[9];

  i2c_dev->read(input_buffer, 24);

  // build header byte 0 1 0 1 0 DAC1 DAC0 UDAC [A]
  uint8_t eeprom_write_cmd = MCP4728_MULTI_EEPROM_CMD; // 0 1 0 1 0 xxx
  eeprom_write_cmd |=
      (MCP4728_CHANNEL_A << 1); // DAC1 DAC0, start at channel A obvs
  eeprom_write_cmd |= 0;        // UDAC ; yes, latch please
  // First byte is the write command+options
  output_buffer[0] = eeprom_write_cmd;

  // copy the incoming input register bytes to the outgoing buffer
  // Channel A
  output_buffer[1] = input_buffer[1];
  output_buffer[2] = input_buffer[2];
  // Channel B
  output_buffer[3] = input_buffer[7];
  output_buffer[4] = input_buffer[8];
  // Channel C
  output_buffer[5] = input_buffer[13];
  output_buffer[6] = input_buffer[14];
  // Channel D
  output_buffer[7] = input_buffer[19];
  output_buffer[8] = input_buffer[20];

  if (!i2c_dev->write(output_buffer, 9)) {
    return false;
  }
  delay(15);
  return true;
}

/**
 * @brief Read the current value of one DAC value.
 *
 * @param channel the channel to read
 * @return current value of the specified channel
 */

uint16_t MCP4728::getChannelValue(MCP4728_channel_t channel) {
  uint16_t value;
  uint8_t input_buffer[24];
  /* 24 bytes are (3 bytes outreg, 3 bytes EEPROM) x 4 channels */
  uint8_t reg_base = 6 * ((uint8_t)channel);

  i2c_dev->read(input_buffer, 24);
  value =
      input_buffer[reg_base + 2] + ((0x0F & input_buffer[reg_base + 1]) << 8);

  return value;
}

/**
 * @brief Read the current value of one EEPROM value.
 *
 * @param channel the channel to read
 * @return current value of the specified channel
 */

uint16_t MCP4728::getEEPROMValue(MCP4728_channel_t channel) {
  uint16_t value;
  uint8_t input_buffer[24];
  /* 24 bytes are (3 bytes outreg, 3 bytes EEPROM) x 4 channels */
  uint8_t reg_base = 6 * ((uint8_t)channel);

  i2c_dev->read(input_buffer, 24);
  value =
      input_buffer[reg_base + 5] + ((0x0F & input_buffer[reg_base + 4]) << 8);

  return value;
}

/**
 * @brief Read all 24 bytes from the device to update cached register values
 * 
 * @return true if read was successful
 * @return false if there was an I2C error
 */
bool MCP4728::_readRegisters(void) {
  uint8_t buffer[24];
  
  if (!i2c_dev->read(buffer, 24)) {
    return false;
  }
  
  // Parse the 24 bytes: 6 bytes per channel (3 input reg, 3 EEPROM)
  for (int ch = 0; ch < 4; ch++) {
    int offset = ch * 6;
    
    // Input register (bytes 0-2 of each channel's 6 bytes)
    uint8_t byte1 = buffer[offset + 1];
    uint8_t byte2 = buffer[offset + 2];
    
    _vref[ch] = (MCP4728_vref_t)((byte1 >> 7) & 0x01);
    _pd[ch] = (MCP4728_pd_mode_t)((byte1 >> 5) & 0x03);
    _gain[ch] = (MCP4728_gain_t)((byte1 >> 4) & 0x01);
    _values[ch] = ((byte1 & 0x0F) << 8) | byte2;
    
    // EEPROM (bytes 3-5 of each channel's 6 bytes)
    byte1 = buffer[offset + 4];
    byte2 = buffer[offset + 5];
    
    _vref_eeprom[ch] = (MCP4728_vref_t)((byte1 >> 7) & 0x01);
    _pd_eeprom[ch] = (MCP4728_pd_mode_t)((byte1 >> 5) & 0x03);
    _gain_eeprom[ch] = (MCP4728_gain_t)((byte1 >> 4) & 0x01);
    _values_eeprom[ch] = ((byte1 & 0x0F) << 8) | byte2;
  }
  
  return true;
}

/**
 * @brief General Call Reset - resets all devices on the I2C bus
 * Loads EEPROM values to input registers for all MCP4728 devices
 * 
 * @return true if command was sent successfully
 * @return false if there was an I2C error
 */
bool MCP4728::generalCallReset(void) {
  // General Call commands use address 0x00
  Wire.beginTransmission(0x00);
  Wire.write(MCP4728_GENERAL_CALL_RESET);
  uint8_t result = Wire.endTransmission();
  
  if (result == 0) {
    delay(10);  // Wait for reset to complete
    _readRegisters();  // Update cached values
    return true;
  }
  
  return false;
}

/**
 * @brief General Call Wake-up - wakes all devices from power-down mode
 * 
 * @return true if command was sent successfully
 * @return false if there was an I2C error
 */
bool MCP4728::generalCallWakeup(void) {
  Wire.beginTransmission(0x00);
  Wire.write(MCP4728_GENERAL_CALL_WAKEUP);
  uint8_t result = Wire.endTransmission();
  
  return (result == 0);
}

/**
 * @brief General Call Software Update - updates all DAC outputs simultaneously
 * 
 * @return true if command was sent successfully
 * @return false if there was an I2C error
 */
bool MCP4728::generalCallUpdate(void) {
  Wire.beginTransmission(0x00);
  Wire.write(MCP4728_GENERAL_CALL_UPDATE);
  uint8_t result = Wire.endTransmission();
  
  return (result == 0);
}

/**
 * @brief Set voltage reference for a single channel
 * 
 * @param channel The channel to configure
 * @param vref Voltage reference setting (VDD or INTERNAL)
 * @return true if successful
 * @return false if there was an I2C error
 */
bool MCP4728::setVref(MCP4728_channel_t channel, MCP4728_vref_t vref) {
  _vref[channel] = vref;
  
  uint8_t cmd = MCP4728_VREF_WRITE_CMD;
  cmd |= (_vref[0] << 3) | (_vref[1] << 2) | (_vref[2] << 1) | _vref[3];
  
  return i2c_dev->write(&cmd, 1);
}

/**
 * @brief Set voltage reference for all channels
 * 
 * @param vref_a Channel A voltage reference
 * @param vref_b Channel B voltage reference
 * @param vref_c Channel C voltage reference
 * @param vref_d Channel D voltage reference
 * @return true if successful
 * @return false if there was an I2C error
 */
bool MCP4728::setAllVref(MCP4728_vref_t vref_a, MCP4728_vref_t vref_b,
                         MCP4728_vref_t vref_c, MCP4728_vref_t vref_d) {
  _vref[0] = vref_a;
  _vref[1] = vref_b;
  _vref[2] = vref_c;
  _vref[3] = vref_d;
  
  uint8_t cmd = MCP4728_VREF_WRITE_CMD;
  cmd |= (vref_a << 3) | (vref_b << 2) | (vref_c << 1) | vref_d;
  
  return i2c_dev->write(&cmd, 1);
}

/**
 * @brief Set gain for a single channel
 * 
 * @param channel The channel to configure
 * @param gain Gain setting (1X or 2X)
 * @return true if successful
 * @return false if there was an I2C error
 */
bool MCP4728::setGain(MCP4728_channel_t channel, MCP4728_gain_t gain) {
  _gain[channel] = gain;
  
  uint8_t cmd = MCP4728_GAIN_WRITE_CMD;
  cmd |= (_gain[0] << 3) | (_gain[1] << 2) | (_gain[2] << 1) | _gain[3];
  
  return i2c_dev->write(&cmd, 1);
}

/**
 * @brief Set gain for all channels
 * 
 * @param gain_a Channel A gain
 * @param gain_b Channel B gain
 * @param gain_c Channel C gain
 * @param gain_d Channel D gain
 * @return true if successful
 * @return false if there was an I2C error
 */
bool MCP4728::setAllGain(MCP4728_gain_t gain_a, MCP4728_gain_t gain_b,
                         MCP4728_gain_t gain_c, MCP4728_gain_t gain_d) {
  _gain[0] = gain_a;
  _gain[1] = gain_b;
  _gain[2] = gain_c;
  _gain[3] = gain_d;
  
  uint8_t cmd = MCP4728_GAIN_WRITE_CMD;
  cmd |= (gain_a << 3) | (gain_b << 2) | (gain_c << 1) | gain_d;
  
  return i2c_dev->write(&cmd, 1);
}

/**
 * @brief Set power-down mode for a single channel
 * 
 * @param channel The channel to configure
 * @param pd_mode Power-down mode (NORMAL, GND_1K, GND_100K, or GND_500K)
 * @return true if successful
 * @return false if there was an I2C error
 */
bool MCP4728::setPowerDown(MCP4728_channel_t channel, MCP4728_pd_mode_t pd_mode) {
  _pd[channel] = pd_mode;
  
  uint8_t buffer[2];
  buffer[0] = MCP4728_PD_WRITE_CMD;
  buffer[0] |= (_pd[0] << 2) | _pd[1];
  buffer[1] = (_pd[2] << 6) | (_pd[3] << 4);
  
  return i2c_dev->write(buffer, 2);
}

/**
 * @brief Set power-down mode for all channels
 * 
 * @param pd_a Channel A power-down mode
 * @param pd_b Channel B power-down mode
 * @param pd_c Channel C power-down mode
 * @param pd_d Channel D power-down mode
 * @return true if successful
 * @return false if there was an I2C error
 */
bool MCP4728::setAllPowerDown(MCP4728_pd_mode_t pd_a, MCP4728_pd_mode_t pd_b,
                              MCP4728_pd_mode_t pd_c, MCP4728_pd_mode_t pd_d) {
  _pd[0] = pd_a;
  _pd[1] = pd_b;
  _pd[2] = pd_c;
  _pd[3] = pd_d;
  
  uint8_t buffer[2];
  buffer[0] = MCP4728_PD_WRITE_CMD;
  buffer[0] |= (pd_a << 2) | pd_b;
  buffer[1] = (pd_c << 6) | (pd_d << 4);
  
  return i2c_dev->write(buffer, 2);
}

/**
 * @brief Get current voltage reference setting for a channel
 * 
 * @param channel The channel to query
 * @return Current Vref setting
 */
MCP4728_vref_t MCP4728::getVref(MCP4728_channel_t channel) {
  return _vref[channel];
}

/**
 * @brief Get current gain setting for a channel
 * 
 * @param channel The channel to query
 * @return Current gain setting
 */
MCP4728_gain_t MCP4728::getGain(MCP4728_channel_t channel) {
  return _gain[channel];
}

/**
 * @brief Get current power-down mode for a channel
 * 
 * @param channel The channel to query
 * @return Current power-down mode
 */
MCP4728_pd_mode_t MCP4728::getPowerDown(MCP4728_channel_t channel) {
  return _pd[channel];
}

/**
 * @brief Get EEPROM voltage reference setting for a channel
 * 
 * @param channel The channel to query
 * @return EEPROM Vref setting
 */
MCP4728_vref_t MCP4728::getVrefEEPROM(MCP4728_channel_t channel) {
  return _vref_eeprom[channel];
}

/**
 * @brief Get EEPROM gain setting for a channel
 * 
 * @param channel The channel to query
 * @return EEPROM gain setting
 */
MCP4728_gain_t MCP4728::getGainEEPROM(MCP4728_channel_t channel) {
  return _gain_eeprom[channel];
}

/**
 * @brief Get EEPROM power-down mode for a channel
 * 
 * @param channel The channel to query
 * @return EEPROM power-down mode
 */
MCP4728_pd_mode_t MCP4728::getPowerDownEEPROM(MCP4728_channel_t channel) {
  return _pd_eeprom[channel];
}

// I2C bit-bang timing constant (10us = 100kHz)
#define MCP4728_I2C_DELAY_US 10

/**
 * @brief Write a new I2C address to the MCP4728's EEPROM
 * 
 * This function writes a new I2C address to the MCP4728. The address change
 * is permanent and stored in EEPROM. This operation requires precise timing
 * control of the LDAC pin, so it uses bit-banged I2C instead of the hardware
 * I2C peripheral.
 * 
 * IMPORTANT: The LDAC pin must be connected and controllable for this function
 * to work. If LDAC is permanently pulled low, the address cannot be changed.
 * 
 * @param current_address The current I2C address of the device (0x60-0x67)
 * @param new_address The desired new I2C address (0x60-0x67)
 * @param sda_pin GPIO pin number for I2C SDA
 * @param scl_pin GPIO pin number for I2C SCL
 * @param ldac_pin GPIO pin number for LDAC control
 * 
 * @return true if the address was successfully written
 * @return false if there was an error during the write operation
 */
bool MCP4728::writeI2CAddress(uint8_t current_address, uint8_t new_address,
                              uint8_t sda_pin, uint8_t scl_pin, 
                              uint8_t ldac_pin) {
  
  // Validate address range (0x60-0x67)
  if (current_address < 0x60 || current_address > 0x67 ||
      new_address < 0x60 || new_address > 0x67) {
    return false;
  }
  
  // Extract address bits (lower 3 bits)
  uint8_t current_addr_bits = current_address & 0x07;
  uint8_t new_addr_bits = new_address & 0x07;
  
  // Build command bytes per datasheet Figure 5-11
  // Format: 011 A2 A1 A0 XX
  uint8_t byte2 = 0b01100001 | (current_addr_bits << 2);  // 011 A2 A1 A0 01
  uint8_t byte3 = 0b01100010 | (new_addr_bits << 2);      // 011 A2 A1 A0 10
  uint8_t byte4 = 0b01100011 | (new_addr_bits << 2);      // 011 A2 A1 A0 11
  
  // Initialize LDAC pin
  pinMode(ldac_pin, OUTPUT);
  digitalWrite(ldac_pin, HIGH);
  delay(10);
  
  // Initialize bit-bang I2C
  _i2c_bb_init(sda_pin, scl_pin);
  
  // Start I2C transaction
  _i2c_bb_start(sda_pin, scl_pin);
  
  // Send address byte with write bit
  uint8_t addr_byte = (current_address << 1) | 0x00;
  if (!_i2c_bb_write_byte(addr_byte, sda_pin, scl_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return false;
  }
  
  // Send byte 2 with LDAC toggle (critical timing)
  if (!_i2c_bb_write_byte_ldac(byte2, sda_pin, scl_pin, ldac_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return false;
  }
  
  // Send byte 3 (LDAC stays LOW)
  if (!_i2c_bb_write_byte(byte3, sda_pin, scl_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return false;
  }
  
  // Send byte 4
  if (!_i2c_bb_write_byte(byte4, sda_pin, scl_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return false;
  }
  
  // Return LDAC to HIGH
  digitalWrite(ldac_pin, HIGH);
  
  // Stop I2C transaction
  _i2c_bb_stop(sda_pin, scl_pin);
  
  // Wait for EEPROM write to complete (25-50ms per datasheet)
  delay(100);
  
  return true;
}

/**
 * @brief Read the current I2C address from an MCP4728
 * 
 * This function reads the current I2C address using the General Call Read
 * command. Like writeI2CAddress, this requires precise LDAC timing control.
 * 
 * @param sda_pin GPIO pin number for I2C SDA
 * @param scl_pin GPIO pin number for I2C SCL  
 * @param ldac_pin GPIO pin number for LDAC control
 * 
 * @return The current I2C address (0x60-0x67), or 0xFF on error
 */
uint8_t MCP4728::readI2CAddress(uint8_t sda_pin, uint8_t scl_pin,
                                uint8_t ldac_pin) {
  
  // Initialize LDAC pin
  pinMode(ldac_pin, OUTPUT);
  digitalWrite(ldac_pin, HIGH);
  delay(10);
  
  // Initialize bit-bang I2C
  _i2c_bb_init(sda_pin, scl_pin);
  
  // Start I2C transaction to general call address
  _i2c_bb_start(sda_pin, scl_pin);
  
  // Send general call address (0x00) with write bit
  if (!_i2c_bb_write_byte(0x00, sda_pin, scl_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return 0xFF;
  }
  
  // Send read address command with LDAC toggle
  if (!_i2c_bb_write_byte_ldac(0x0C, sda_pin, scl_pin, ldac_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return 0xFF;
  }
  
  // Restart to read
  _i2c_bb_start(sda_pin, scl_pin);
  
  // Send general call address with read bit
  if (!_i2c_bb_write_byte(0xC1, sda_pin, scl_pin)) {
    _i2c_bb_stop(sda_pin, scl_pin);
    digitalWrite(ldac_pin, HIGH);
    return 0xFF;
  }
  
  // Read address byte
  uint8_t addr_data = _i2c_bb_read_byte(false, sda_pin, scl_pin); // NAK
  
  // Return LDAC to HIGH
  digitalWrite(ldac_pin, HIGH);
  
  // Stop I2C transaction
  _i2c_bb_stop(sda_pin, scl_pin);
  
  // Extract address bits (bits 3:1)
  uint8_t address = ((addr_data & 0x0E) >> 1);
  
  return 0x60 | address;
}

// Private bit-bang I2C helper functions

void MCP4728::_i2c_bb_init(uint8_t sda_pin, uint8_t scl_pin) {
  pinMode(scl_pin, OUTPUT);
  pinMode(sda_pin, OUTPUT);
  digitalWrite(scl_pin, HIGH);
  digitalWrite(sda_pin, HIGH);
}

void MCP4728::_i2c_bb_start(uint8_t sda_pin, uint8_t scl_pin) {
  digitalWrite(sda_pin, HIGH);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  digitalWrite(sda_pin, LOW);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  digitalWrite(scl_pin, LOW);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
}

void MCP4728::_i2c_bb_stop(uint8_t sda_pin, uint8_t scl_pin) {
  digitalWrite(sda_pin, LOW);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  digitalWrite(sda_pin, HIGH);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
}

bool MCP4728::_i2c_bb_write_byte(uint8_t data, uint8_t sda_pin, 
                                  uint8_t scl_pin) {
  // Write 8 data bits
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    digitalWrite(sda_pin, (mask & data) ? HIGH : LOW);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(MCP4728_I2C_DELAY_US);
    digitalWrite(scl_pin, LOW);
  }
  
  // Read ACK
  digitalWrite(sda_pin, HIGH);
  pinMode(sda_pin, INPUT);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  bool ack = (digitalRead(sda_pin) == LOW);
  digitalWrite(scl_pin, LOW);
  pinMode(sda_pin, OUTPUT);
  
  return ack;
}

bool MCP4728::_i2c_bb_write_byte_ldac(uint8_t data, uint8_t sda_pin,
                                       uint8_t scl_pin, uint8_t ldac_pin) {
  // Write 8 data bits
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    digitalWrite(sda_pin, (mask & data) ? HIGH : LOW);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(MCP4728_I2C_DELAY_US);
    digitalWrite(scl_pin, LOW);
  }
  
  // CRITICAL: Pull LDAC LOW after data bits but before ACK
  digitalWrite(ldac_pin, LOW);
  
  // Read ACK
  digitalWrite(sda_pin, HIGH);
  pinMode(sda_pin, INPUT);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  bool ack = (digitalRead(sda_pin) == LOW);
  digitalWrite(scl_pin, LOW);
  pinMode(sda_pin, OUTPUT);
  
  return ack;
}

uint8_t MCP4728::_i2c_bb_read_byte(bool ack, uint8_t sda_pin, 
                                    uint8_t scl_pin) {
  uint8_t data = 0;
  
  // Make SDA input and enable pullup
  digitalWrite(sda_pin, HIGH);
  pinMode(sda_pin, INPUT);
  
  // Read 8 bits
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    delayMicroseconds(MCP4728_I2C_DELAY_US);
    digitalWrite(scl_pin, HIGH);
    if (digitalRead(sda_pin)) {
      data |= 1;
    }
    digitalWrite(scl_pin, LOW);
  }
  
  // Send ACK or NAK
  pinMode(sda_pin, OUTPUT);
  digitalWrite(sda_pin, ack ? LOW : HIGH);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(MCP4728_I2C_DELAY_US);
  digitalWrite(scl_pin, LOW);
  digitalWrite(sda_pin, HIGH);
  
  return data;
}