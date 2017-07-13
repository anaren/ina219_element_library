#include "ina219.h"

#include "../i2c/i2c.h"
#include "ina219_config.h"
#include <stdbool.h>

#define NUM_IDS 4 // 0x40 through 0x45. Only 0x40, 0x41, 0x44, and 0x45 are actually available

static ina219_config_t ina219_config[NUM_IDS] = {0};

static int8_t ina219_addr_to_index(uint8_t addr)
{
  switch(id)
  {
    case 0x40:
      return 0;
    case 0x41:
      return 1;
    case 0x44:
      return 2;
    case 0x45:
      return 3;
    default:
      return -1;
  }
}

static bool ina219_is_valid_addr(uint8_t addr)
{
  return ina219_id_to_index(addr) != -1;
}

ina219_result_t INA219_Init(uint8_t addr)
{
  // NOTE: Default configuration assumes .1 Ohm shunt
  // 32V 2A Range
  // 10mA LSB for current readings
  ina219_config_t default_config;
  default_config.config_reg_val = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  default_config.calibration_reg_val = 4096;
  default_config.current_lsb_ma = 10;
  ina219_result_t ret_val = INA219_SetConfig(addr, &default_config );
  return ret_val;
}

ina219_result_t INA219_WriteReg(uint8_t addr, uint8_t reg, uint16_t value)
{
  if( !ina219_is_valid_addr(addr) )
    return;

  uint8_t writeBytes[3];

  writeBytes[0] = reg;
  writeBytes[1] = value >> 8;
  writeBytes[2] = value & 0xFF;
  AIR_I2C_Write(addr, writeBytes, 3);
  return INA219_SUCCESS;
}

ina219_result_t INA219_ReadReg(uint8_t addr, uint8_t reg, uint16_t *value)
{
  if( !ina219_is_valid_addr(addr) )
    return 0;

  uint8_t write_bytes[1] = {0};
  uint8_t read_bytes[2] = {0};
  uint16_t read_data = 0;

  write_bytes[0] = addr;
  AIR_I2C_ComboRead(addr, write_bytes, 1, read_bytes, 2);
  read_data = (unsigned int)read_bytes[0] << 8;
  read_data |= read_bytes[1];
  *value = read_data;
  return INA219_SUCCESS;
}

ina219_result_t INA219_SetConfig(uint8_t addr, const ina219_config_t *config)
{
  if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  ina219_config[ina219_addr_to_index(addr)] = *config;
  ina219_result_t ret_val = INA219_WriteReg(addr, INA219_REG_CONFIG, config->config_reg_val);

  if( ret_val == INA219_SUCCESS )
  {
    ret_val = INA219_SetCalibrationRaw(addr, config->calibration_reg_val);
  }

  return ret_val;
}


ina219_result_t INA219_SetCalibrationRaw(uint8_t addr, uint16_t value)
{
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  ina219_config[ina219_addr_to_index(addr)].calibration_reg_val = value;
  return INA219_WriteReg(addr, INA219_REG_CALIBRATION, value);
}

ina219_result_t INA219_GetBusVoltageRaw(uint8_t addr, uin16_t *voltage) {
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  uint16_t value = 0;

  ina219_result_t ret_val = INA219_ReadReg(addr, INA219_REG_BUSVOLTAGE, &value);

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  *voltage = (int16_t)((value >> 3) * 4);

  return ret_val;
}

ina219_result_t INA219_GetShuntVoltageRaw(uint8_t addr, int16_t *voltage) {
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  return INA219_ReadReg(addr, INA219_REG_SHUNTVOLTAGE, voltage);
}

/**************************************************************************/
/*! 
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
ina219_result_t INA219_GetCurrentRaw(uint8_t addr, int16_t *current) {
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina219_result_t ret_val = INA219_WriteReg(addr, INA219_REG_CALIBRATION, ina219_calValue);

  if( ret_val == INA219_SUCCESS )
  {
    ret_val = INA219_ReadReg(addr, INA219_REG_CURRENT, current);
  }
  
  return ret_val;
}
 
ina219_result_t INA219_GetShuntVoltageMv(uint8_t addr, float *voltage) {
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  int16_t value;
  ina219_result_t ret_val = INA219_GetShuntVoltageRaw(addr, &value);
  *voltage = value * 0.01;
  return ret_val;
}

ina219_result_t INA219_GetBusVoltageV(uint8_t addr, float *voltage) {
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  int16_t value;
  ina219_result_t ret_val = INA219_GetBusVoltageRaw(addr, &value);
  *voltage = value * 0.001;
  return ret_val;
}

ina219_result_t INA219_GetCurrentMa(uint8_t addr, float *current) {
if( !ina219_is_valid_addr(addr) )
    return INA219_INVALID_ADDR;

  int16_t value_dec;
  ina219_result_t ret_val = INA219_GetCurrentRaw(addr, &value_dec);
  *current = value_dec / ina219_config[ina219_addr_to_index(addr)].current_lsb_ma;
  return ret_val;
}