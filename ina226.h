#ifndef _INA226_H_
#define _INA226_H_

#if !defined BIT
#define BIT(n) (1 << n)
#endif

typedef size_t (*INAWriteBytes)(int address, const uint8_t *Buffer, size_t BytesToWrite);
typedef size_t (*INAReadBytes)(int address, uint8_t *Buffer, size_t BufferMaxLen);

typedef enum
{
    INA226_REG_CFG = 0x00,
    INA226_REG_SHUNT_VOLTAGE,
    INA226_REG_BUS_VOLTAGE,
    INA226_REG_POWER,
    INA226_REG_CURRENT,
    INA226_REG_CALIBRATION,
    INA226_REG_ALERT_MASK,
    INA226_REG_ALERT_LIMIT,
    INA226_REG_MANUFACTURER_ID = 0xFE,
    INA226_REG_DIE_ID
} ina226_reg_t;

typedef enum
{
    INA226_AVERAGES_1 = 0,
    INA226_AVERAGES_4,
    INA226_AVERAGES_16,
    INA226_AVERAGES_64,
    INA226_AVERAGES_128,
    INA226_AVERAGES_256,
    INA226_AVERAGES_512,
    INA226_AVERAGES_1024,
    INA226_NUM_AVERAGES = 7
} ina226_averaging_mode_t;

typedef enum
{
    INA226_CONVERSION_TIME_140_US = 0,
    INA226_CONVERSION_TIME_204_US,
    INA226_CONVERSION_TIME_332_US,
    INA226_CONVERSION_TIME_588_US,
    INA226_CONVERSION_TIME_1_MS,
    INA226_CONVERSION_TIME_116_MS,
    INA226_CONVERSION_TIME_156_MS,
    INA226_CONVERSION_TIME_244_MS,
    INA226_NUM_CONVERSION_TIME = 7
} ina226_conversion_time_t;

typedef enum
{
    INA226_MODE_SHUTDOWN = 0,
    INA226_MODE_SHUNT_VOLTAGE_TRIGGERED,
    INA226_MODE_BUS_VOLTAGE_TRIGGERED,
    INA226_MODE_SHUNT_AND_BUS_TRIGGERED,
    INA226_MODE_SHUTDOWN_2,
    INA226_MODE_SHUNT_VOLTAGE_CONTINUOUS,
    INA226_MODE_BUS_VOLTAGE_CONTINUOUS,
    INA226_MODE_SHUNT_AND_BUS_CONTINUOUS,
    INA226_NUM_MODES = 7
} ina226_mode_t;

typedef enum
{
    INA226_ALERT_SHUNT_OVER_VOLTAGE = BIT(15),
    INA226_ALERT_SHUNT_UNDER_VOLTAGE = BIT(14),
    INA226_ALERT_BUS_OVER_VOLTAGE = BIT(13),
    INA226_ALERT_BUS_UNDER_VOLTAGE = BIT(12),
    INA226_ALERT_POWER_OVER_LIMIT = BIT(11),
    INA226_ALERT_CONVERSION_READY = BIT(10),
    INA226_ALERT_FUNCTION_FLAG = BIT(4),
    INA226_ALERT_CONVERSION_READY_FLAG = BIT(3),
    INA226_ALERT_MATH_OVERFLOW_FLAG = BIT(2),
    INA226_ALERT_POLARITY = BIT(1),
    INA226_ALERT_LATCH_ENABLE = BIT(0)
} ina226_alert_t;

struct ina226_device_t
{
    float shunt_voltage_lsb;
    float bus_voltage_lsb;

    float calibration_value;
    float current_lsb;

    INAWriteBytes WriteBytesFn;
    INAReadBytes ReadBytesFn;

    int address;
};

#define INA226_CFG_Reset BIT(15)

#define INA226_CFG_AveragingMask (BIT(9) | BIT(10) | BIT(11))
#define INA226_CFG_AveragingOffset 9

#define INA226_CFG_BusVoltageTimeMask (BIT(6) | BIT(7) | BIT(8))
#define INA226_CFG_BusVoltageTimeOffset 6

#define INA226_CFG_ShuntVoltageTimeMask (BIT(3) | BIT(4) | BIT(5))
#define INA226_CFG_ShuntVoltageTimeOffset 3

#define INA226_CFG_ModeMask (BIT(0) | BIT(1) | BIT(2))

bool ina226_write_reg(struct ina226_device_t *Device, ina226_reg_t Register, uint16_t Value);
uint16_t ina226_read_reg16(struct ina226_device_t *Device, ina226_reg_t Register);

uint16_t ina226_get_manufacturer_id(struct ina226_device_t *Device);
uint16_t ina226_get_die_id(struct ina226_device_t *Device);

uint16_t ina226_read_config(struct ina226_device_t *Device);
void ina226_write_config(struct ina226_device_t *Device, uint16_t Config);

ina226_averaging_mode_t ina226_get_averaging_mode(struct ina226_device_t *Device);
void ina226_set_averaging_mode(struct ina226_device_t *Device, ina226_averaging_mode_t Mode);

ina226_conversion_time_t ina226_get_bus_voltage_conversation_time(struct ina226_device_t *Device);
void ina226_set_bus_voltage_conversation_time(struct ina226_device_t *Device, ina226_conversion_time_t ConversionTime);

ina226_conversion_time_t ina226_get_shunt_voltage_conversation_time(struct ina226_device_t *Device);
void ina226_set_shunt_voltage_conversation_time(struct ina226_device_t *Device, ina226_conversion_time_t ConversionTime);

ina226_mode_t ina226_get_operation_mode(struct ina226_device_t *Device);
void ina226_set_operation_mode(struct ina226_device_t *Device, ina226_mode_t Mode);

float ina226_get_shunt_voltage(struct ina226_device_t *Device);
float ina226_get_bus_voltage(struct ina226_device_t *Device);
float ina226_get_current(struct ina226_device_t *Device);
float ina226_get_power(struct ina226_device_t *Device);

bool ina226_init(struct ina226_device_t *Device, int I2CAddress, int RShuntInMilliOhms, int MaxCurrentInAmps, INAWriteBytes WriteBytesFn, INAReadBytes ReadBytesFn);
void ina226_reset(struct ina226_device_t *Device);
void ina226_calibrate(struct ina226_device_t *Device, int RShunt, int MaxCurrentInMilliamps);

ina226_alert_t ina226_get_alert_mask(struct ina226_device_t *INADevice);
ina226_alert_t ina226_set_alert_mask(struct ina226_device_t *INADevice, ina226_alert_t AlertMask);

float ina226_set_alert_limit_bus_voltage(struct ina226_device_t *Device, float BusVoltageInMV);

#endif
