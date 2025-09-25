/**
 * @file scd30.cpp
 * @author Daan Pape <daan@dptechnics.com> Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 25 September 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains drivers for the Walter Feels carrier board
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <cstdint>
#include <scd30.h>

static const char* LOGTAG = "[SCD30]";

SCD30::SCD30(void)
{
  // Constructor
}

// Initialize the Serial port
bool SCD30::begin(i2c_port_t i2cPort, bool autoCalibrate, bool measBegin)
{
  _i2c_port = i2cPort;

  // Check if device is connected
  if(!isConnected()) {
    ESP_LOGE(LOGTAG, "SCD30 not detected on I2C bus");
    return false;
  }

  if(!measBegin) {
    return true;
  }

  // Start continuous measurements
  if(beginMeasuring()) {
    setMeasurementInterval(2);
    setAutoSelfCalibration(autoCalibrate);
    return true;
  }

  return false;
}

// Returns true if device responds to a firmware request
bool SCD30::isConnected()
{
  uint16_t fwVer;
  if(getFirmwareVersion(&fwVer) ==
     false) // Read the firmware version. Return false if the CRC check fails.
    return (false);

  ESP_LOGD(LOGTAG, "Firmware version 0x%u", fwVer);

  return (true);
}

// Returns the latest available CO2 level
// If the current level has already been reported, trigger a new read
uint16_t SCD30::getCO2(void)
{
  if(_co2_has_been_reported == true) // Trigger a new read
  {
    if(readMeasurement() == false) // Pull in new co2, humidity, and temp into global vars
      if(!_use_stale_data)
        _co2 = 0; // Failed to read sensor
  }

  _co2_has_been_reported = true;

  return (uint16_t) _co2; // Cut off decimal as co2 is 0 to 10,000
}

// Returns the latest available humidity
// If the current level has already been reported, trigger a new read
float SCD30::getHumidity(void)
{
  if(_humidity_has_been_reported == true) // Trigger a new read
    if(readMeasurement() == false)        // Pull in new co2, humidity, and temp into global vars
      if(!_use_stale_data)
        _humidity = 0; // Failed to read sensor

  _humidity_has_been_reported = true;

  return _humidity;
}

// Returns the latest available temperature
// If the current level has already been reported, trigger a new read
float SCD30::getTemperature(void)
{
  if(_temperature_has_been_reported == true) // Trigger a new read
    if(readMeasurement() == false)           // Pull in new co2, humidity, and temp into global vars
      if(!_use_stale_data)
        _temperature = 0; // Failed to read sensor

  _temperature_has_been_reported = true;

  return _temperature;
}

// Enables or disables the ASC
bool SCD30::setAutoSelfCalibration(bool enable)
{
  if(enable)
    return sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION,
                       1); // Activate continuous ASC
  else
    return sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION,
                       0); // Deactivate continuous ASC
}

// Set the forced recalibration factor. See 1.3.7.
// The reference CO2 concentration has to be within the range 400 ppm ≤
// cref(CO2) ≤ 2000 ppm.
bool SCD30::setForcedRecalibrationFactor(uint16_t concentration)
{
  if(concentration < 400 || concentration > 2000) {
    return false; // Error check.
  }
  return sendCommand(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, concentration);
}

// Get the temperature offset. See 1.3.8.
float SCD30::getTemperatureOffset(void)
{
  uint16_t response = readRegister(COMMAND_SET_TEMPERATURE_OFFSET);

  union {
    int16_t signed16;
    uint16_t unsigned16;
  } signedUnsigned; // Avoid any ambiguity casting int16_t to uint16_t
  signedUnsigned.unsigned16 = response;

  return (((float) signedUnsigned.signed16) / 100.0);
}

// Set the temperature offset to remove module heating from temp reading
bool SCD30::setTemperatureOffset(float tempOffset)
{
  // Temp offset is only positive. See:
  // https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library/issues/27#issuecomment-971986826
  //"The SCD30 offset temperature is obtained by subtracting the reference
  // temperature from the SCD30 output temperature"
  // https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9.5_CO2/Sensirion_CO2_Sensors_SCD30_Low_Power_Mode.pdf

  if(tempOffset < 0.0)
    return (false);

  uint16_t value = tempOffset * 100;

  return sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, value);
}

// Get the altitude compenstation. See 1.3.9.
uint16_t SCD30::getAltitudeCompensation(void)
{
  return readRegister(COMMAND_SET_ALTITUDE_COMPENSATION);
}

// Set the altitude compenstation. See 1.3.9.
bool SCD30::setAltitudeCompensation(uint16_t altitude)
{
  return sendCommand(COMMAND_SET_ALTITUDE_COMPENSATION, altitude);
}

// Set the pressure compenstation. This is passed during measurement startup.
// mbar can be 700 to 1200
bool SCD30::setAmbientPressure(uint16_t pressure_mbar)
{
  if(pressure_mbar < 700 || pressure_mbar > 1200) {
    return false;
  }
  return sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressure_mbar);
}

// SCD30 soft reset
void SCD30::reset()
{
  sendCommand(COMMAND_RESET);
}

// Get the current ASC setting
bool SCD30::getAutoSelfCalibration()
{
  uint16_t response = readRegister(COMMAND_AUTOMATIC_SELF_CALIBRATION);
  if(response == 1) {
    return true;
  } else {
    return false;
  }
}

// Begins continuous measurements
// Continuous measurement status is saved in non-volatile memory. When the
// sensor is powered down while continuous measurement mode is active SCD30 will
// measure continuously after repowering without sending the measurement
// command. Returns true if successful
bool SCD30::beginMeasuring(uint16_t pressureOffset)
{
  return (sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressureOffset));
}

// Overload - no pressureOffset
bool SCD30::beginMeasuring(void)
{
  return (beginMeasuring(0));
}

// Stop continuous measurement
bool SCD30::StopMeasurement(void)
{
  return (sendCommand(COMMAND_STOP_MEAS));
}

// Sets interval between measurements
// 2 seconds to 1800 seconds (30 minutes)
bool SCD30::setMeasurementInterval(uint16_t interval)
{
  return sendCommand(COMMAND_SET_MEASUREMENT_INTERVAL, interval);
}

// Gets interval between measurements
// 2 seconds to 1800 seconds (30 minutes)
uint16_t SCD30::getMeasurementInterval(void)
{
  uint16_t interval = 0;
  getSettingValue(COMMAND_SET_MEASUREMENT_INTERVAL, &interval);
  return (interval);
}

// Returns true when data is available
bool SCD30::dataAvailable()
{
  uint16_t response = readRegister(COMMAND_GET_DATA_READY);

  if(response == 1)
    return (true);
  return (false);
}

// Get 18 bytes from SCD30
// Updates global variables with floats
// Returns true if success
bool SCD30::readMeasurement()
{
  // Verify we have data from the sensor
  if(dataAvailable() == false)
    return (false);

  uint8_t command[2] = { (uint8_t) (COMMAND_READ_MEASUREMENT >> 8),
                         (uint8_t) (COMMAND_READ_MEASUREMENT & 0xFF) };
  uint8_t buffer[18]; // Buffer to store sensor response

  // Send read measurement command
  esp_err_t err =
      i2c_master_write_to_device(_i2c_port, SCD30_ADDRESS, command, 2, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to send read command: %s", esp_err_to_name(err));
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(3)); // Wait before reading

  // Read 18 bytes of measurement data
  err = i2c_master_read_from_device(_i2c_port, SCD30_ADDRESS, buffer, 18, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to read data: %s", esp_err_to_name(err));
    return false;
  }

  bool error = false;
  uint8_t bytesToCrc[2];

  // Convert received bytes to float values
  ByteToFl tempCO2, tempHumidity, tempTemperature;
  tempCO2.value = tempHumidity.value = tempTemperature.value = 0;

  for(uint8_t x = 0; x < 18; x++) {
    uint8_t incoming = buffer[x];

    switch(x) {
    case 0:
    case 1:
    case 3:
    case 4:
      tempCO2.array[x < 3 ? 3 - x : 4 - x] = incoming;
      bytesToCrc[x % 3] = incoming;
      break;
    case 6:
    case 7:
    case 9:
    case 10:
      tempTemperature.array[x < 9 ? 9 - x : 10 - x] = incoming;
      bytesToCrc[x % 3] = incoming;
      break;
    case 12:
    case 13:
    case 15:
    case 16:
      tempHumidity.array[x < 15 ? 15 - x : 16 - x] = incoming;
      bytesToCrc[x % 3] = incoming;
      break;
    default:
      // Validate CRC
      uint8_t foundCrc = computeCRC8(bytesToCrc, 2);
      if(foundCrc != incoming) {
        ESP_LOGW(LOGTAG, "CRC error at byte %d: expected 0x%02X, got 0x%02X", x, foundCrc,
                 incoming);
        error = true;
      }
      break;
    }
  }

  if(error) {
    ESP_LOGE(LOGTAG, "readMeasurement: CRC error detected.");
    return false;
  }

  // Store new sensor values
  _co2 = tempCO2.value;
  _temperature = tempTemperature.value;
  _humidity = tempHumidity.value;

  // Mark values as fresh
  _co2_has_been_reported = false;
  _humidity_has_been_reported = false;
  _temperature_has_been_reported = false;

  return true;
}

// Gets a setting by reading the appropriate register.
// Returns true if the CRC is valid.
bool SCD30::getSettingValue(uint16_t registerAddress, uint16_t* val)
{
  uint8_t command[2] = { (uint8_t) (registerAddress >> 8), (uint8_t) (registerAddress & 0xFF) };
  uint8_t buffer[3];

  esp_err_t err =
      i2c_master_write_to_device(_i2c_port, SCD30_ADDRESS, command, 2, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to send register read command: %s", esp_err_to_name(err));
    return false;
  }

  vTaskDelay(pdMS_TO_TICKS(3));

  err = i2c_master_read_from_device(_i2c_port, SCD30_ADDRESS, buffer, 3, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to read register: %s", esp_err_to_name(err));
    return false;
  }

  *val = (uint16_t) buffer[0] << 8 | buffer[1];
  uint8_t expectedCRC = computeCRC8(buffer, 2);
  if(buffer[2] != expectedCRC) {
    ESP_LOGW(LOGTAG, "CRC mismatch: expected 0x%02X, got 0x%02X", expectedCRC, buffer[2]);
    return false;
  }
  return true;
}

// Reads two bytes from a register
uint16_t SCD30::readRegister(uint16_t registerAddress)
{
  uint8_t command[2] = { (uint8_t) (registerAddress >> 8), (uint8_t) (registerAddress & 0xFF) };
  uint8_t buffer[2];

  esp_err_t err =
      i2c_master_write_to_device(_i2c_port, SCD30_ADDRESS, command, 2, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to send register read command: %s", esp_err_to_name(err));
    return 0;
  }

  vTaskDelay(pdMS_TO_TICKS(3));

  err = i2c_master_read_from_device(_i2c_port, SCD30_ADDRESS, buffer, 2, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to read register: %s", esp_err_to_name(err));
    return 0;
  }

  return (uint16_t) buffer[0] << 8 | buffer[1];
}

// Sends a command with arguments and CRC
bool SCD30::sendCommand(uint16_t command, uint16_t arguments)
{
  uint8_t data[5];
  data[0] = command >> 8;
  data[1] = command & 0xFF;
  data[2] = arguments >> 8;
  data[3] = arguments & 0xFF;
  data[4] = computeCRC8(&data[2], 2);

  esp_err_t err = i2c_master_write_to_device(_i2c_port, SCD30_ADDRESS, data, 5, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to send command: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

// Sends a command without arguments
bool SCD30::sendCommand(uint16_t command)
{
  uint8_t data[2] = { (uint8_t) (command >> 8), (uint8_t) (command & 0xFF) };

  esp_err_t err = i2c_master_write_to_device(_i2c_port, SCD30_ADDRESS, data, 2, pdMS_TO_TICKS(100));
  if(err != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to send command: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

// Given an array and a number of bytes, this calculate CRC8 for those bytes
// CRC is only calc'd on the data portion (two bytes) of the four bytes being
// sent From:
// http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html Tested
// with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
// x^8+x^5+x^4+1 = 0x31
uint8_t SCD30::computeCRC8(uint8_t data[], uint8_t len)
{
  uint8_t crc = 0xFF; // Init with 0xFF

  for(uint8_t x = 0; x < len; x++) {
    crc ^= data[x]; // XOR-in the next input byte

    for(uint8_t i = 0; i < 8; i++) {
      if((crc & 0x80) != 0)
        crc = (uint8_t) ((crc << 1) ^ 0x31);
      else
        crc <<= 1;
    }
  }

  return crc; // No output reflection
}
