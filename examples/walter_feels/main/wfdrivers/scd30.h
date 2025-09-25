/**
 * @file scd30.h
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

#include <driver/i2c.h>

#ifndef _SCD30_H_
#define _SCD30_H_

#define CO2_EN_PIN 13
#define CO2_SDA_PIN 12
#define CO2_SCL_PIN 11

// The default I2C address for the SCD30 is 0x61.
#define SCD30_ADDRESS 0x61

// Available commands

#define COMMAND_CONTINUOUS_MEASUREMENT 0x0010
#define COMMAND_SET_MEASUREMENT_INTERVAL 0x4600
#define COMMAND_GET_DATA_READY 0x0202
#define COMMAND_READ_MEASUREMENT 0x0300
#define COMMAND_AUTOMATIC_SELF_CALIBRATION 0x5306
#define COMMAND_SET_FORCED_RECALIBRATION_FACTOR 0x5204
#define COMMAND_SET_TEMPERATURE_OFFSET 0x5403
#define COMMAND_SET_ALTITUDE_COMPENSATION 0x5102
#define COMMAND_RESET 0xD304 // Soft reset
#define COMMAND_STOP_MEAS 0x0104
#define COMMAND_READ_FW_VER 0xD100

typedef union {
  uint8_t array[4];
  float value;
} ByteToFl; // paulvha

class SCD30
{
public:
  SCD30(void);

  static bool begin(bool autoCalibrate) { return begin(I2C_NUM_0, autoCalibrate); }
  static bool begin(i2c_port_t i2cPort = I2C_NUM_0, bool autoCalibrate = false,
                    bool measBegin = true);

  static bool isConnected();

  static bool beginMeasuring(uint16_t pressureOffset);
  static bool beginMeasuring(void);
  static bool StopMeasurement(void); // paulvha

  static bool setAmbientPressure(uint16_t pressure_mbar);

  static bool getSettingValue(uint16_t registerAddress, uint16_t* val);
  static bool getFirmwareVersion(uint16_t* val)
  {
    return (getSettingValue(COMMAND_READ_FW_VER, val));
  }
  static uint16_t getCO2(void);
  static float getHumidity(void);
  static float getTemperature(void);

  static uint16_t getMeasurementInterval(void);
  static bool getMeasurementInterval(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_MEASUREMENT_INTERVAL, val));
  }
  static bool setMeasurementInterval(uint16_t interval);

  static uint16_t getAltitudeCompensation(void);
  static bool getAltitudeCompensation(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_ALTITUDE_COMPENSATION, val));
  }
  static bool setAltitudeCompensation(uint16_t altitude);

  static bool getAutoSelfCalibration(void);
  static bool setAutoSelfCalibration(bool enable);

  static bool getForcedRecalibration(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, val));
  }
  static bool setForcedRecalibrationFactor(uint16_t concentration);

  static float getTemperatureOffset(void);
  static bool getTemperatureOffset(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_TEMPERATURE_OFFSET, val));
  }
  static bool setTemperatureOffset(float tempOffset);

  static bool dataAvailable();
  static bool readMeasurement();

  static void reset();

  static bool sendCommand(uint16_t command, uint16_t arguments);
  static bool sendCommand(uint16_t command);

  static uint16_t readRegister(uint16_t registerAddress);

  static uint8_t computeCRC8(uint8_t data[], uint8_t len);

  static void useStaleData(bool enable) { _use_stale_data = enable; }

private:
  // Variables
  static inline i2c_port_t _i2c_port;
  // Global main datums
  static inline float _co2 = 0;
  static inline float _temperature = 0;
  static inline float _humidity = 0;
  static inline bool _use_stale_data = false; // If true, stale data is returned instead of zeros

  // These track the staleness of the current data
  // This allows us to avoid calling readMeasurement() every time individual
  // datums are requested
  static inline bool _co2_has_been_reported = true;
  static inline bool _humidity_has_been_reported = true;
  static inline bool _temperature_has_been_reported = true;
};
#endif
