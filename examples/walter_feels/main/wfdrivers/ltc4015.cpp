/**
 * @file ltc4015.cpp
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
#include <esp_log.h>
#include <ltc4015.h>

static const char* LOGTAG = "[LTC4015]";

const char* bat_chem_str[] = { "Li Ion", "LiFePO4", "Lead Acid" };

void LTC4015::initialize(unsigned char _Rsnsi, unsigned char _Rsnsb)
{
  _rsnsi = _Rsnsi;
  _rsnsb = _Rsnsb;
  _is_programmable = false;
  suspend_charging();
  printinfo();
}

void LTC4015::printinfo()
{
  get_battery_info();
  ESP_LOGI(LOGTAG, "Battery : %s", bat_chem_str[_chemistry]);
  ESP_LOGI(LOGTAG, "Cells : %u", _cell_count);

  int vcharge = read_word(LTC4015_REG_VCHARGE_SETTING);
  ESP_LOGI(LOGTAG, "Charge voltage target : %.2fV\n", ((float) vcharge / 80) + 3.4125);

  int icharge = read_word(LTC4015_REG_ICHARGE_TARGET);
  ESP_LOGI(LOGTAG, "Charge current target : %.2fA\n", ((float) icharge + 1) * 1.0 / _rsnsb);
}

void LTC4015::set_input_voltage_limit(float hi, float lo)
{
  _write_word(LTC4015_REG_VIN_LO_ALERT_LIMIT, 0);
  _write_word(LTC4015_REG_VIN_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_system_voltage_limit(float hi, float lo)
{
  _write_word(LTC4015_REG_VSYS_LO_ALERT_LIMIT, 0);
  _write_word(LTC4015_REG_VSYS_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_battery_voltage_limit(float hi, float lo)
{
  _write_word(LTC4015_REG_VBAT_LO_ALERT_LIMIT, 0);
  _write_word(LTC4015_REG_VBAT_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_input_current_limit(float hi)
{
  _write_word(LTC4015_REG_IIN_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_battery_current_limit(float lo)
{
  _write_word(LTC4015_REG_IBAT_LO_ALERT_LIMIT, 0);
}

void LTC4015::set_die_temp_limit(float hi)
{
  _write_word(LTC4015_REG_DIE_TEMP_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_bsr_limit(float hi)
{
  _write_word(LTC4015_REG_BSR_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_ntc_limit(float hi, float lo)
{
  _write_word(LTC4015_REG_NTC_RATIO_HI_ALERT_LIMIT, 0);
  _write_word(LTC4015_REG_NTC_RATIO_LO_ALERT_LIMIT, 0);
}

void LTC4015::set_qcount(uint16_t qcount_val)
{
  _write_word(LTC4015_REG_QCOUNT, qcount_val);
}

uint16_t LTC4015::get_qcount()
{
  return (uint16_t) read_word(LTC4015_REG_QCOUNT);
}

void LTC4015::set_coulomb_counter(uint16_t qcount_init, uint16_t prescale_factor,
                                  uint16_t lo_alert_lim, uint16_t hi_alert_lim)
{
  _write_word(LTC4015_REG_QCOUNT, qcount_init);
  _write_word(LTC4015_REG_QCOUNT_PRESCALE_FACTOR, prescale_factor);
  _write_word(LTC4015_REG_QCOUNT_LO_ALERT_LIMIT, lo_alert_lim);
  _write_word(LTC4015_REG_QCOUNT_HI_ALERT_LIMIT, hi_alert_lim);
}

void LTC4015::en_meas_sys_valid_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, meas_sys_valid);
}
void LTC4015::dis_meas_sys_valid_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, meas_sys_valid);
}
void LTC4015::en_qcount_lo_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, qcount_lo);
}
void LTC4015::dis_qcount_lo_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, qcount_lo);
}
void LTC4015::en_qcount_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, qcount_hi);
}
void LTC4015::dis_qcount_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, qcount_hi);
}
void LTC4015::en_vbat_lo_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, vbat_lo);
}
void LTC4015::dis_vbat_lo_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, vbat_lo);
}
void LTC4015::en_vbat_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, vbat_hi);
}
void LTC4015::dis_vbat_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, vbat_hi);
}
void LTC4015::en_vin_lo_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, vin_lo);
}
void LTC4015::dis_vin_lo_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, vin_lo);
}
void LTC4015::en_vin_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, vin_hi);
}
void LTC4015::dis_vin_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, vin_hi);
}
void LTC4015::en_vsys_lo_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, vsys_lo);
}
void LTC4015::dis_vsys_lo_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, vsys_lo);
}
void LTC4015::en_vsys_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, vsys_hi);
}
void LTC4015::dis_vsys_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, vsys_hi);
}
void LTC4015::en_iin_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, iin_hi);
}
void LTC4015::dis_iin_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, iin_hi);
}
void LTC4015::en_ibat_lo_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, ibat_lo);
}
void LTC4015::dis_ibat_lo_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, ibat_lo);
}
void LTC4015::en_die_temp_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, die_temp_hi);
}
void LTC4015::dis_die_temp_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, die_temp_hi);
}
void LTC4015::en_bsr_hi_alert()
{
  _set_bit(LTC4015_REG_EN_LIMIT_ALERTS, bsr_hi);
}
void LTC4015::dis_bsr_hi_alert()
{
  _clr_bit(LTC4015_REG_EN_LIMIT_ALERTS, bsr_hi);
}
void LTC4015::en_ntc_ratio_hi_alert()
{
}
void LTC4015::dis_ntc_ratio_hi_alert()
{
}
void LTC4015::en_ntc_ratio_lo_alert()
{
}
void LTC4015::dis_ntc_ratio_lo_alert()
{
}
void LTC4015::enable_limit_alerts(unsigned short alert)
{
  _write_word(LTC4015_REG_EN_LIMIT_ALERTS, alert);
}

void LTC4015::enable_charger_state_alerts(unsigned short alert)
{
  _write_word(LTC4015_REG_EN_CHARGER_STATE_ALERTS, alert);
}

void LTC4015::en_uvcl_limit_alert()
{
  _set_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, vin_uvcl_active);
}
void LTC4015::dis_uvcl_limit_alert()
{
  _clr_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, vin_uvcl_active);
}
void LTC4015::en_iin_limit_alert()
{
  _set_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, iin_limit_active);
}
void LTC4015::dis_iin_limit_alert()
{
  _clr_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, iin_limit_active);
}
void LTC4015::en_cc_alert()
{
  _set_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, constant_current);
}
void LTC4015::dis_cc_alert()
{
  _clr_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, constant_current);
}
void LTC4015::en_cv_alert()
{
  _set_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, constant_voltage);
}
void LTC4015::dis_cv_alert()
{
  _clr_bit(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, constant_voltage);
}
void LTC4015::enable_charger_status_alerts(unsigned short alert)
{
  _write_word(LTC4015_REG_EN_CHARGE_STATUS_ALERTS, alert);
}

void LTC4015::suspend_charging()
{
  _set_bit(LTC4015_REG_CONFIG_BITS, suspend_charger);
}
void LTC4015::start_charging()
{
  _clr_bit(LTC4015_REG_CONFIG_BITS, suspend_charger);
}
void LTC4015::enable_mppt()
{
  _set_bit(LTC4015_REG_CONFIG_BITS, mppt_en_i2c);
}
void LTC4015::disable_mppt()
{
  _clr_bit(LTC4015_REG_CONFIG_BITS, mppt_en_i2c);
}
void LTC4015::enable_force_telemetry()
{
  _set_bit(LTC4015_REG_CONFIG_BITS, force_meas_sys_on);
}
void LTC4015::disable_force_telemetry()
{
  _clr_bit(LTC4015_REG_CONFIG_BITS, force_meas_sys_on);
}
void LTC4015::enable_coulomb_counter()
{
  _set_bit(LTC4015_REG_CONFIG_BITS, en_qcount);
}
void LTC4015::disable_coulomb_counter()
{
  _clr_bit(LTC4015_REG_CONFIG_BITS, en_qcount);
}
void LTC4015::config(unsigned short setting)
{
  _write_word(LTC4015_REG_CONFIG_BITS, setting);
}

void LTC4015::set_input_current_max(float current)
{
  unsigned short iin = (unsigned short) ((2 * current * _rsnsi) - 1);
  if(iin > 63) {
    return;
  }
  _write_word(LTC4015_REG_ICHARGE_TARGET, iin);
}

void LTC4015::set_input_voltage_min(float voltage)
{
}

void LTC4015::set_charge_current(float current)
{
  unsigned short icharge = (unsigned short) ((current * _rsnsb) - 1);
  if(icharge > 31) {
    return;
  }
  _write_word(LTC4015_REG_ICHARGE_TARGET, icharge);
}

void LTC4015::set_charge_voltage(float voltage)
{
  float v_cell;

  v_cell = (voltage / _cell_count);
  if(_chemistry == LI_ION) {
    v_cell -= 3.8125;
    _vcharge = (unsigned char) (v_cell * 80);
    if(_vcharge > 31) {
      return;
    }
  }

  else if(_chemistry == LIFEPO4) {
    v_cell -= 3.4125;
    _vcharge = (unsigned char) (v_cell * 80);
    if(_vcharge > 31) {
      return;
    }
  }

  else if(_chemistry == LEAD_ACID) {
    v_cell -= 2.0;
    _vcharge = (unsigned char) (v_cell * 105.0);
    if(_vcharge > 63) {
      return;
    }
  }
  _write_word(LTC4015_REG_VCHARGE_SETTING, _vcharge);
}

void LTC4015::arm_ship_mode()
{
}

void LTC4015::charger_status()
{
}

void LTC4015::charger_state()
{
}

void LTC4015::charger_limit_alert()
{
}

void LTC4015::get_battery_info()
{
  unsigned short tmp = read_word(LTC4015_REG_CHEM_CELLS);
  unsigned char t_chem = (tmp >> chem) & 0x0F;
  _cell_count = tmp & 0x0F;
  if(t_chem < 4) {
    _chemistry = LI_ION;
  } else if(t_chem < 7) {
    _chemistry = LIFEPO4;
  } else if(t_chem < 9) {
    _chemistry = LEAD_ACID;
  } else {
    _chemistry = INVALID;
  }
  if((t_chem == 0) || (t_chem == 4) || (t_chem == 8)) {
    _is_programmable = true;
  }
}

float LTC4015::get_input_voltage()
{
  return (float) ((signed short) read_word(LTC4015_REG_VIN) * 0.001648);
}

float LTC4015::get_input_current()
{
  return (float) ((signed short) read_word(LTC4015_REG_IIN) * 0.00146487 / _rsnsi);
}

float LTC4015::get_system_voltage()
{
  return (float) ((signed short) read_word(LTC4015_REG_VSYS) * 0.001648);
}

float LTC4015::get_battery_voltage()
{
  short v_bat_cell = read_word(LTC4015_REG_VBAT);
  float v_bat = 0.0;
  if(_chemistry == LEAD_ACID) {
    v_bat = v_bat_cell * 0.000128176;
  } else if(_chemistry != INVALID) {
    v_bat = v_bat_cell * 0.0001922;
  } else {
  }
  return (v_bat * _cell_count);
}

float LTC4015::get_charge_current()
{
  return (float) ((signed short) read_word(LTC4015_REG_IBAT) * 0.00146487 / _rsnsb);
}

float LTC4015::get_die_temp()
{
  short rawdie = read_word(LTC4015_REG_DIE_TEMP);
  return (float) ((rawdie - 12010) / 45.6);
}

void LTC4015::_write_word(uint8_t sub_address, uint16_t word)
{
  const uint8_t slave_addr = WFEELS_ADDR_LTC4015 >> 1; // Convert to 7-bit address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t ret;

  // Start I2C command sequence
  ret = i2c_master_start(cmd);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to initiate start condition");
  }

  // Write the slave address with the write bit
  ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write slave address for write");
  }

  // Write the sub-address (register address)
  ret = i2c_master_write_byte(cmd, sub_address, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write sub-address");
  }

  // Write the low byte of the word
  ret = i2c_master_write_byte(cmd, word & 0xFF, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write low byte");
  }

  // Write the high byte of the word
  ret = i2c_master_write_byte(cmd, (word >> 8) & 0xFF, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write high byte");
  }

  // End the command sequence
  i2c_master_stop(cmd);

  // Execute the I2C command
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "I2C command failed (write): 0x%x", ret);
  }

  // Clean up
  i2c_cmd_link_delete(cmd);
}

uint16_t LTC4015::read_word(uint8_t sub_address)
{
  uint8_t data[2] = { 0 };
  const uint8_t slave_addr = WFEELS_ADDR_LTC4015 >> 1; // Convert to 7-bit address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t ret;

  // Write the sub-address first to set the register pointer
  ret = i2c_master_start(cmd);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to initiate start condition for read");
  }

  ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write slave address for write (read sequence)");
  }

  ret = i2c_master_write_byte(cmd, sub_address, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write sub-address for read");
  }

  // Initiate a repeated start and switch to read mode
  ret = i2c_master_start(cmd);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to initiate repeated start for read");
  }

  ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to write slave address for read");
  }

  // Read two bytes:
  // - For multiple bytes, ACK every byte except the last one, which is NACKed.
  ret = i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to read low byte");
  }

  ret = i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "Failed to read high byte");
  }

  // End the command sequence
  i2c_master_stop(cmd);

  // Execute the I2C command
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  if(ret != ESP_OK) {
    ESP_LOGE(LOGTAG, "I2C command failed (read): 0x%x", ret);
  }

  // Clean up
  i2c_cmd_link_delete(cmd);

  // Combine the two bytes (low byte first, then high byte) to form the word
  return (uint16_t) (data[0] | (data[1] << 8));
}

void LTC4015::_clr_bit(unsigned char sub_address, unsigned short new_word)
{
  unsigned short old_word = read_word(sub_address);
  old_word &= ~new_word;
  _write_word(sub_address, old_word);
}

void LTC4015::_set_bit(unsigned char sub_address, unsigned short new_word)
{
  unsigned short old_word = read_word(sub_address);
  old_word |= new_word;
  _write_word(sub_address, old_word);
}