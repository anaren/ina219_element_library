#include "ina219.h"

#include "../i2c/i2c.h"
#include "ina219_config.h"
#include <stdbool.h>

#define NUM_IDS 4 // 0x40 through 0x45. Only 0x40, 0x41, 0x44, and 0x45 are actually available

typedef struct ina219_calibration_t
{
    uint16_t current_lsb_ma;
    uint16_t power_lsb_mW;
} ina219_calibration_t;

static ina219_calibration_t ina219_cal[NUM_IDS] = {0};

#define CALIBRATION_FACTOR (.04096)
#define MAX_CALIBRATION_VALUE (0xFFFE)

// In the spec (p17) the current LSB factor for the minimum LSB is
// documented as 32767, but a larger value (100.1% of 32767) is used
// to guarantee that current overflow can always be detected.
#define CURRENT_LSB_FACTOR (32800)

static int8_t ina219_addr_to_index( uint8_t addr )
{
    switch( addr )
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

static bool ina219_is_valid_addr( uint8_t addr )
{
    return ina219_addr_to_index( addr ) != -1;
}

static float calculate_min_current_lsb( float shunt_ohms )
{
    return CALIBRATION_FACTOR / (shunt_ohms * MAX_CALIBRATION_VALUE);
}

ina219_result_t INA219_Init( uint8_t addr )
{
    // NOTE: Default configuration assumes .1 Ohm shunt
    // 32V 2A Range
    // 10mA LSB for current readings
    // ina219_result_t INA219_Configure( uint8_t addr, uint32_t shunt_ohms, uint16_t voltage_range, uint16_t gain, uint16_t bus_adc_resolution, uint16_t shunt_adc_resolution )
    return INA219_Configure( addr, INA219_SHUNT_MILLIOHMS, INA219_VRANGE, INA219_GAIN, INA219_BUS_ADC_RES, INA219_SHUNT_ADC_RES);
}

ina219_result_t INA219_WriteReg( uint8_t addr, uint8_t reg, uint16_t value )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    uint8_t writeBytes[3];

    writeBytes[0] = reg;
    writeBytes[1] = value >> 8;
    writeBytes[2] = value & 0xFF;
    AIR_I2C_Write( addr, writeBytes, 3 );
    return INA219_SUCCESS;
}

ina219_result_t INA219_ReadReg( uint8_t addr, uint8_t reg, uint16_t *value )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    uint8_t write_bytes[1] = {0};
    uint8_t read_bytes[2] = {0};
    uint16_t read_data = 0;

    write_bytes[0] = addr;
    AIR_I2C_ComboRead( addr, write_bytes, 1, read_bytes, 2 );
    read_data = ( unsigned int )read_bytes[0] << 8;
    read_data |= read_bytes[1];
    *value = read_data;
    return INA219_SUCCESS;
}

static float get_max_gain_voltage(uint16_t gain)
{
    switch(gain)
    {
        case INA219_CONFIG_GAIN_1_40MV:
        {
            return 0.04;
        }
        case INA219_CONFIG_GAIN_2_80MV:
        {
            return 0.08;
        }
        case INA219_CONFIG_GAIN_4_160MV:
        {
            return 0.16;
        }
        case INA219_CONFIG_GAIN_8_320MV:
        {
            return 0.32;
        }
        default:
        {
            return 0.04;
        }
    }
}

ina219_result_t INA219_Configure( uint8_t addr, uint32_t shunt_milli_ohms, uint16_t voltage_range, uint16_t gain, uint16_t bus_adc_resolution, uint16_t shunt_adc_resolution )
{
    // Calibrate
    if( !ina219_is_valid_addr(addr) )
    {
        return INA219_INVALID_ADDR;
    }

    int8_t index = ina219_addr_to_index(addr);

    float shunt_ohms = shunt_milli_ohms / 1000.0;

    // Maximum current across the shunt
    float max_possible_current = get_max_gain_voltage(gain) / shunt_ohms;

    float current_lsb = max_possible_current / CURRENT_LSB_FACTOR; // In Amps

    float min_current_lsb = calculate_min_current_lsb( shunt_ohms );

    if( current_lsb < min_current_lsb )
    {
        current_lsb = min_current_lsb;
    }

    float power_lsb = current_lsb * 20; // In Watts

    uint16_t calibration = (uint16_t)(CALIBRATION_FACTOR/(current_lsb * shunt_ohms));

    ina219_cal[index].current_lsb_ma = (uint16_t)(current_lsb * 1000);
    ina219_cal[index].power_lsb_mW = (uint16_t)(power_lsb * 1000);

    ina219_result_t ret_val = INA219_WriteReg( addr, INA219_REG_CALIBRATION, calibration );

    if( ret_val == INA219_SUCCESS )
    {
        ret_val = INA219_WriteReg( addr, INA219_REG_CONFIG, voltage_range | gain | bus_adc_resolution | shunt_adc_resolution | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS );
    }

    return ret_val;
}

ina219_result_t INA219_SetCalibrationRaw( uint8_t addr, uint16_t value )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    return INA219_WriteReg( addr, INA219_REG_CALIBRATION, value );
}

ina219_result_t INA219_GetBusVoltageRaw( uint8_t addr, int16_t *voltage )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    uint16_t value = 0;

    ina219_result_t ret_val = INA219_ReadReg( addr, INA219_REG_BUSVOLTAGE, &value );

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    *voltage = ( int16_t )( ( value >> 3 ) * 4 );

    return ret_val;
}

ina219_result_t INA219_GetShuntVoltageRaw( uint8_t addr, int16_t *voltage )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    return INA219_ReadReg( addr, INA219_REG_SHUNTVOLTAGE, ( uint16_t * )voltage );
}

/**************************************************************************/
/*!
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
ina219_result_t INA219_GetCurrentRaw( uint8_t addr, int16_t *current )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    return INA219_ReadReg( addr, INA219_REG_CURRENT, ( uint16_t * )current );
}

ina219_result_t INA219_GetPowerRaw( uint8_t addr, int16_t *power )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    return INA219_ReadReg( addr, INA219_REG_POWER, ( uint16_t * )power );
}


ina219_result_t INA219_GetShuntVoltageMv( uint8_t addr, uint32_t *voltage )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    int16_t value;
    ina219_result_t ret_val = INA219_GetShuntVoltageRaw( addr, &value );
    *voltage = (uint32_t)(value * 0.01);
    return ret_val;
}

ina219_result_t INA219_GetBusVoltageMv( uint8_t addr, uint32_t *voltage )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    int16_t value;
    ina219_result_t ret_val = INA219_GetBusVoltageRaw( addr, &value );
    *voltage = (uint32_t)value;
    return ret_val;
}

ina219_result_t INA219_GetCurrentMa( uint8_t addr, uint32_t *current )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    int16_t value_dec;
    ina219_result_t ret_val = INA219_GetCurrentRaw( addr, &value_dec );
    *current = (uint32_t)(value_dec / ina219_cal[ina219_addr_to_index( addr )].current_lsb_ma);
    return ret_val;
}

ina219_result_t INA219_GetPowerMw( uint8_t addr, uint32_t *power )
{
    if( !ina219_is_valid_addr( addr ) )
        return INA219_INVALID_ADDR;

    int16_t value_dec;
    ina219_result_t ret_val = INA219_GetPowerRaw( addr, &value_dec );
    *power = (uint32_t)(value_dec / ina219_cal[ina219_addr_to_index( addr )].power_lsb_mW);
    return ret_val;  
}

ina219_result_t INA219_Reset( uint8_t addr )
{
    return INA219_WriteReg( addr, INA219_REG_CONFIG, INA219_CONFIG_RESET );
}