/**
 * @file lps22hb.cpp
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
#include <esp_mac.h>
#include <lps22hb.h>
#include <stdint.h>

static const char* LOGTAG = "[LPS22HB]";

// Constructor
LPS22HB::LPS22HB(i2c_port_t i2cPort)
{
  _i2c_port = i2cPort, _initialized = false;
}

// I2C initialization
esp_err_t LPS22HB::begin()
{
  // Check sensor ID
  uint8_t id;
  if(_i2cRead(LPS22HB_WHO_AM_I_REG, &id, 1) == ESP_OK && id == 0xB1) {
    _initialized = true;
    return ESP_OK;
  }

  return ESP_FAIL;
}

// Read pressure
float LPS22HB::readPressure(int units)
{
  if(!_initialized)
    return 0;

  uint8_t data[3];
  _i2cWrite(LPS22HB_CTRL2_REG, 0x01); // Trigger one-shot measurement

  // Wait until ONE_SHOT bit is cleared
  uint8_t status;
  do {
    _i2cRead(LPS22HB_CTRL2_REG, &status, 1);
  } while(status & 0x01);

  // Read pressure registers
  _i2cRead(LPS22HB_PRESS_OUT_XL_REG, &data[0], 1);
  _i2cRead(LPS22HB_PRESS_OUT_L_REG, &data[1], 1);
  _i2cRead(LPS22HB_PRESS_OUT_H_REG, &data[2], 1);

  int32_t raw_pressure = (data[2] << 16) | (data[1] << 8) | data[0];
  float pressure_kpa = raw_pressure / 40960.0;

  if(units == MILLIBAR) {
    return pressure_kpa * 10;
  } else if(units == PSI) {
    return pressure_kpa * 0.145038;
  } else {
    return pressure_kpa;
  }
}

// Read temperature
float LPS22HB::readTemperature()
{
  uint8_t data[2];
  _i2cRead(LPS22HB_TEMP_OUT_L_REG, &data[0], 1);
  _i2cRead(LPS22HB_TEMP_OUT_H_REG, &data[1], 1);

  int16_t raw_temp = (data[1] << 8) | data[0];
  return raw_temp / 100.0;
}

// I2C read function
esp_err_t LPS22HB::_i2cRead(uint8_t reg, uint8_t* data, size_t len)
{
  return i2c_master_write_read_device(_i2c_port, LPS22HB_ADDRESS, &reg, 1, data, len,
                                      1000 / portTICK_PERIOD_MS);
}

// I2C write function
esp_err_t LPS22HB::_i2cWrite(uint8_t reg, uint8_t val)
{
  uint8_t data[2] = { reg, val };
  return i2c_master_write_to_device(_i2c_port, LPS22HB_ADDRESS, data, 2, 1000 / portTICK_PERIOD_MS);
}