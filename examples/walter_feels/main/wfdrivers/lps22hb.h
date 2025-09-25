/*
  This file is part of the Arduino_LPS22HB library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _BARO_H_
#define _BARO_H_

#include <driver/i2c.h>

#define LPS22HB_ADDRESS 0x5C

// Define register addresses
#define LPS22HB_WHO_AM_I_REG 0x0f
#define LPS22HB_CTRL2_REG 0x11
#define LPS22HB_PRESS_OUT_XL_REG 0x28
#define LPS22HB_PRESS_OUT_L_REG 0x29
#define LPS22HB_PRESS_OUT_H_REG 0x2a
#define LPS22HB_TEMP_OUT_L_REG 0x2b
#define LPS22HB_TEMP_OUT_H_REG 0x2c

enum { PSI, MILLIBAR, KILOPASCAL };

class LPS22HB
{
public:
  LPS22HB(i2c_port_t i2cPort = I2C_NUM_0);

  static int begin();
  static void end();

  static float readPressure(int units = KILOPASCAL);
  static float readTemperature(void);

private:
  static inline esp_err_t _i2cRead(uint8_t reg, uint8_t* data, size_t len);
  static inline esp_err_t _i2cWrite(uint8_t reg, uint8_t val);

  static inline i2c_port_t _i2c_port;
  static inline bool _initialized;
};

#endif
