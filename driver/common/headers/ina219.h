#pragma once

#include <stdint.h>
#include "ina219_config.h"

#define INA219_SLAVE_BASE_ADDR 0x40

/*=========================================================================
    CONFIG REGISTER (R/W)
    -----------------------------------------------------------------------*/
#define INA219_REG_CONFIG                      (0x00)
/*---------------------------------------------------------------------*/
#define INA219_CONFIG_RESET                    (0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       (0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V        (0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        (0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                (0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              (0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              (0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             (0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             (0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             (0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             (0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            (0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            (0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            (0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             (0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     (0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   (0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   (0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   (0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  (0x0048)  // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  (0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  (0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US (0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   (0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   (0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  (0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                (0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           (0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     (0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     (0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED (0x0003)
#define INA219_CONFIG_MODE_ADCOFF              (0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    (0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    (0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)
/*=========================================================================*/

/*=========================================================================
    SHUNT VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA219_REG_SHUNTVOLTAGE                (0x01)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA219_REG_BUSVOLTAGE                  (0x02)
/*=========================================================================*/

/*=========================================================================
    POWER REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA219_REG_POWER                       (0x03)
/*=========================================================================*/

/*=========================================================================
    CURRENT REGISTER (R)
    -----------------------------------------------------------------------*/
#define INA219_REG_CURRENT                     (0x04)
/*=========================================================================*/

/*=========================================================================
    CALIBRATION REGISTER (R/W)
    -----------------------------------------------------------------------*/
#define INA219_REG_CALIBRATION                 (0x05)
/*=========================================================================*/

typedef struct ina219_config_t
{
    uint32_t shunt_ohms;
    uint16_t voltage_range;
    uint16_t gain;
    uint16_t bus_adc_resolution;
    uint16_t shunt_adc_resolution;
} ina219_config_t;

typedef enum ina219_result_t
{
    INA219_SUCCESS = 0,
    INA219_INVALID_ADDR = 1,
    INA219_I2C_WRITE = 2,
    INA219_I2C_READ = 3,
} ina219_result_t;

/*
* Initialize INA219
* - Set Configuration Register
*
* NOTE: Default configuration assumed .1 Ohm Shunt. 32V 2A range with current LSB 10mA
*
* @param[in] addr - I2C Address of device
*
* @return ina219_result_t
*/
ina219_result_t INA219_Init( uint8_t addr );

/**
* Write a raw INA219 Register
*
* @param[in] addr - I2C Address of device
* @param[in] reg - register address
* @param[in] value - value to write
*/
ina219_result_t INA219_WriteReg( uint8_t addr, uint8_t reg, uint16_t value );

/**
* Read a raw INA219 Register
*
* @param[in] addr - I2C Address of device
* @param[in] reg - register address
* @param[out] value - value to read
*/
ina219_result_t INA219_ReadReg( uint8_t addr, uint8_t reg, uint16_t *value );

/**
* Set INA219 Configuration
*
* Modifies the INA219_REG_CALIBRATION and INA219_REG_CONFIG registers to provide accurate current readings.
*
* @param[in] addr - I2C Address of device
* @param[in] shunt_milli_ohms - Shunt resistor value in milli-ohms
* @param[in] voltage_range - INA219_CONFIG_BVOLTAGERANGE_16V or INA219_CONFIG_BVOLTAGERANGE_32V
* @param[in] gain - INA219_CONFIG_GAIN_1_40MV, INA219_CONFIG_GAIN_2_80MV, INA219_CONFIG_GAIN_4_160MV, or INA219_CONFIG_GAIN_8_320MV
* @param[in] bus_adc_resolution - INA219_CONFIG_BADCRES_9BIT, INA219_CONFIG_BADCRES_10BIT, INA219_CONFIG_BADCRES_11BIT, INA219_CONFIG_BADCRES_12BIT
* @param[in] shunt_adc_resolution - INA219_CONFIG_SADCRES_9BIT_1S_84US, INA219_CONFIG_SADCRES_10BIT_1S_148US, INA219_CONFIG_SADCRES_11BIT_1S_276US,
*                                   INA219_CONFIG_SADCRES_12BIT_1S_532US, INA219_CONFIG_SADCRES_12BIT_2S_1060US, INA219_CONFIG_SADCRES_12BIT_4S_2130US,
*                                   INA219_CONFIG_SADCRES_12BIT_8S_4260US, INA219_CONFIG_SADCRES_12BIT_16S_8510US, INA219_CONFIG_SADCRES_12BIT_32S_17MS,
*                                   INA219_CONFIG_SADCRES_12BIT_64S_34MS, INA219_CONFIG_SADCRES_12BIT_128S_69MS
*/
ina219_result_t INA219_Configure( uint8_t addr, uint32_t shunt_milli_ohms, uint16_t voltage_range, uint16_t gain, uint16_t bus_adc_resolution, uint16_t shunt_adc_resolution );

/**
* Set INA219 Calibration
*
* Modifies the INA219_REG_CALIBRATION register. Read the INA219 datasheet for information on how to determine this value.
*
* http://www.ti.com/lit/ds/symlink/ina219.pdf
*
* @param[in] addr - I2C Address of device
* @param[in] value - calculated calibration value
*/
ina219_result_t INA219_SetCalibrationRaw( uint8_t addr, uint16_t value );

/**
* Get direct reading of INA219_REG_BUSVOLTAGE register
*
* @param[in] addr - I2C Address of device
* @param[out] voltage
*/
ina219_result_t INA219_GetBusVoltageRaw( uint8_t addr, int16_t *voltage );

/**
* Get direct reading of INA219_REG_SHUNTVOLTAGE register
*
* @param[in] addr - I2C Address of device
* @param[out] voltage
*/
ina219_result_t INA219_GetShuntVoltageRaw( uint8_t addr, int16_t *voltage );

/**
* Get direct reading of INA219_REG_CURRENT register
*
* @param[in] addr - I2C Address of device
* @param[out] current
*/
ina219_result_t INA219_GetCurrentRaw( uint8_t addr, int16_t *current );

/**
* Get direct reading of INA219_POWER register
*
* @param[in] addr - I2C address of device
* @param[out] power
*/
ina219_result_t INA219_GetPowerRaw( uint8_t addr, int16_t *power );

/**
* Get Shunt Voltage in MicroVolts (uV)
*
* @param[in] addr - I2C Address of device
* @param[out] voltage
*/
ina219_result_t INA219_GetShuntVoltage( uint8_t addr, int32_t *voltage );

/**
* Get Bus Voltage in MilliVolts (mV)
*
* @param[in] addr - I2C Address of device
* @param[out] voltage
*/
ina219_result_t INA219_GetBusVoltage( uint8_t addr, int32_t *voltage );

/**
* Get Current in MicroAmps (uA)
*
* @param[in] addr - I2C Address of device
* @param[out] current
*/
ina219_result_t INA219_GetCurrent( uint8_t addr, int32_t *current );

/**
* Get Power in MicroWatts (uW)
*
* @param[in] addr - I2C Address of device
* @param[out] power
*/
ina219_result_t INA219_GetPower( uint8_t addr, int32_t *power );

/**
* Perform a reset (Bit 15 of configuration register)
*
* @param[in] addr - I2C Address of device
*/
ina219_result_t INA219_Reset( uint8_t addr );
