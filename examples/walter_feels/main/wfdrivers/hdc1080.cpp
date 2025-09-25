/**
 * @file hdc1080.cpp
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

#include <WalterFeels.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <hdc1080.h>
#include <esp_mac.h>
#include <stdint.h>
#include <math.h>

static const char* LOGTAG = "[HDC1080]";

HDC1080::HDC1080()
{
}

void HDC1080::begin()
{
  setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
}

void HDC1080::setResolution(HDC1080_MeasurementResolution humidity,
                            HDC1080_MeasurementResolution temperature)
{
  HDC1080_Registers reg;
  reg.HumidityMeasurementResolution = 0;
  reg.TemperatureMeasurementResolution = 0;

  if(temperature == HDC1080_RESOLUTION_11BIT)
    reg.TemperatureMeasurementResolution = 0x01;

  switch(humidity) {
  case HDC1080_RESOLUTION_8BIT:
    reg.HumidityMeasurementResolution = 0x02;
    break;
  case HDC1080_RESOLUTION_11BIT:
    reg.HumidityMeasurementResolution = 0x01;
    break;
  default:
    break;
  }

  writeRegister(reg);
}

HDC1080_SerialNumber HDC1080::readSerialNumber()
{
  HDC1080_SerialNumber sernum;
  sernum.serialFirst = readData(HDC1080_SERIAL_ID_FIRST);
  sernum.serialMid = readData(HDC1080_SERIAL_ID_MID);
  sernum.serialLast = readData(HDC1080_SERIAL_ID_LAST);
  return sernum;
}

HDC1080_Registers HDC1080::readRegister()
{
  HDC1080_Registers reg;
  reg.rawData = (readData(HDC1080_CONFIGURATION) >> 8);
  return reg;
}

// Write to register (Configuration register write)
void HDC1080::writeRegister(HDC1080_Registers reg)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // Begin I²C transaction
  i2c_master_start(cmd);
  // Write device address with write bit
  i2c_master_write_byte(cmd, (HDC1080_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  // Write the pointer/register: typically the configuration register
  i2c_master_write_byte(cmd, HDC1080_CONFIGURATION, true);
  // Write the two bytes of configuration data
  i2c_master_write_byte(cmd, reg.rawData, true);
  i2c_master_write_byte(cmd, 0x00, true);
  // End transmission
  i2c_master_stop(cmd);

  // Send the command
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10)));
  i2c_cmd_link_delete(cmd);

  // Arduino code had a delay here
  vTaskDelay(pdMS_TO_TICKS(10));
}

// Heat up the sensor
void HDC1080::heatUp(uint8_t seconds)
{
  // Read the current configuration
  HDC1080_Registers reg = readRegister();
  // Enable heater and set mode of acquisition
  reg.Heater = 1;
  reg.ModeOfAcquisition = 1;
  writeRegister(reg);

  uint8_t buf[4];
  // The Arduino code loops (seconds*66) times with a write then a read
  for(int i = 1; i < (seconds * 66); i++) {
    // ---- Transaction 1: Write the pointer (0x00) ----
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HDC1080_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(20)));
    i2c_cmd_link_delete(cmd);

    // Give sensor time to process (mimics Arduino delay(20))
    vTaskDelay(pdMS_TO_TICKS(20));

    // ---- Transaction 2: Read 4 bytes from the sensor ----
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HDC1080_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    // For proper I²C protocol, send ACK for all but the last byte
    i2c_master_read_byte(cmd, &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[2], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[3], I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(20)));
    i2c_cmd_link_delete(cmd);
  }

  // Turn off heater and reset mode of acquisition
  reg.Heater = 0;
  reg.ModeOfAcquisition = 0;
  writeRegister(reg);
}

// Read 16-bit data from a given pointer/register
uint16_t HDC1080::readData(uint8_t pointer)
{
  // ---- Transaction 1: Write the pointer to the sensor ----
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (HDC1080_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, pointer, true);
  i2c_master_stop(cmd);
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10)));
  i2c_cmd_link_delete(cmd);

  // Delay to allow sensor to prepare the data
  vTaskDelay(pdMS_TO_TICKS(15));

  // ---- Transaction 2: Read 2 bytes of data ----
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (HDC1080_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
  // Read first byte with ACK...
  uint8_t msb, lsb;
  i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
  // ...and second byte with NACK
  i2c_master_read_byte(cmd, &lsb, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10)));
  i2c_cmd_link_delete(cmd);

  return (msb << 8) | lsb;
}

double HDC1080::readT()
{
  return readTemperature();
}

double HDC1080::readTemperature()
{
  uint16_t rawT = readData(HDC1080_TEMPERATURE);
  return (rawT / pow(2, 16)) * 165.0 - 40.0;
}

double HDC1080::readH()
{
  return readHumidity();
}

double HDC1080::readHumidity()
{
  uint16_t rawH = readData(HDC1080_HUMIDITY);
  return (rawH / pow(2, 16)) * 100.0;
}

uint16_t HDC1080::readManufacturerId()
{
  return readData(HDC1080_MANUFACTURER_ID);
}

uint16_t HDC1080::readDeviceId()
{
  return readData(HDC1080_DEVICE_ID);
}