/**
 * Copyright (c) 2017 Tara Keeling
 *
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "ina226.h"

static SemaphoreHandle_t INALock = NULL;

#define NullCheck(ptr, retexpr)                             \
    {                                                       \
        if (ptr == NULL)                                    \
        {                                                   \
            printf("%s: %s == NULL\n", __FUNCTION__, #ptr); \
            retexpr;                                        \
        };                                                  \
    }

#define RangeCheck(value, min, max, retexpr)                                                                           \
    {                                                                                                                  \
        if (value < min || value > max)                                                                                \
        {                                                                                                              \
            printf("ERROR %s: %s out of range. Got %d, expected [%d to %d]\n", __FUNCTION__, #value, value, min, max); \
            retexpr;                                                                                                   \
        }                                                                                                              \
    }

static bool INA226_SetRegisterPointer(struct ina226_device_t *Device, ina226_reg_t Register)
{
    uint8_t BReg = (uint8_t)Register;
    bool Result = false;

    NullCheck(Device, return false);
    NullCheck(Device->WriteBytesFn, return false);

    Result = (Device->WriteBytesFn(Device->address, (const uint8_t *)&BReg, 1) == sizeof(uint8_t)) ? true : false;
    return Result;
}

bool ina226_write_reg(struct ina226_device_t *Device, ina226_reg_t Register, uint16_t Value)
{
    uint8_t Command[] = {
        (uint8_t)Register,
        Value >> 8,
        Value & 0xFF};
    bool Result = false;

    NullCheck(Device, return false);
    NullCheck(Device->WriteBytesFn, return false);

    if (xSemaphoreTake(INALock, portMAX_DELAY) == pdTRUE)
    {
        Result = (Device->WriteBytesFn(Device->address, (const uint8_t *)Command, sizeof(Command)) == sizeof(Command)) ? true : false;
        xSemaphoreGive(INALock);
    }

    return Result;
}

uint16_t ina226_read_reg16(struct ina226_device_t *Device, ina226_reg_t Register)
{
    uint16_t Value = 0;

    NullCheck(Device, return 0);
    NullCheck(Device->WriteBytesFn, return 0);
    NullCheck(Device->ReadBytesFn, return 0);

    if (xSemaphoreTake(INALock, portMAX_DELAY) == pdTRUE)
    {
        if (INA226_SetRegisterPointer(Device, Register) == true)
        {
            /* Other thread could interrupt right here and cause shit */
            if (Device->ReadBytesFn(Device->address, (uint8_t *)&Value, sizeof(uint16_t)) != sizeof(uint16_t))
            {
                Value = 0;
            }
        }

        xSemaphoreGive(INALock);
    }

    return (Value >> 8) | (Value << 8);
}

uint16_t ina226_get_manufacturer_id(struct ina226_device_t *Device)
{
    return ina226_read_reg16(Device, INA226_REG_MANUFACTURER_ID);
}

uint16_t ina226_get_die_id(struct ina226_device_t *Device)
{
    return ina226_read_reg16(Device, INA226_REG_DIE_ID);
}

uint16_t ina226_read_config(struct ina226_device_t *Device)
{
    return ina226_read_reg16(Device, INA226_REG_CFG);
}

void ina226_write_config(struct ina226_device_t *Device, uint16_t Config)
{
    ina226_write_reg(Device, INA226_REG_CFG, Config);
}

ina226_averaging_mode_t ina226_get_averaging_mode(struct ina226_device_t *Device)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return -1);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig >>= INA226_CFG_AveragingOffset;
    CurrentConfig &= 0x07;

    return (ina226_averaging_mode_t)CurrentConfig;
}

void ina226_set_averaging_mode(struct ina226_device_t *Device, ina226_averaging_mode_t Mode)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return);
    RangeCheck(Mode, 0, INA226_NUM_AVERAGES, return);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig &= ~INA226_CFG_AveragingMask;
    CurrentConfig |= (Mode << INA226_CFG_AveragingOffset);

    ina226_write_reg(Device, INA226_REG_CFG, CurrentConfig);
}

ina226_conversion_time_t ina226_get_bus_voltage_conversation_time(struct ina226_device_t *Device)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return -1);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig >>= INA226_CFG_BusVoltageTimeOffset;
    CurrentConfig &= 0x07;

    return (ina226_conversion_time_t)CurrentConfig;
}

void ina226_set_bus_voltage_conversation_time(struct ina226_device_t *Device, ina226_conversion_time_t ConversionTime)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return);
    RangeCheck(ConversionTime, 0, INA226_NUM_CONVERSION_TIME, return);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig &= ~INA226_CFG_BusVoltageTimeMask;
    CurrentConfig |= (ConversionTime << INA226_CFG_BusVoltageTimeOffset);

    ina226_write_reg(Device, INA226_REG_CFG, CurrentConfig);
}

ina226_conversion_time_t ina226_get_shunt_voltage_conversation_time(struct ina226_device_t *Device)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return -1);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig >>= INA226_CFG_ShuntVoltageTimeOffset;
    CurrentConfig &= 0x07;

    return (ina226_conversion_time_t)CurrentConfig;
}

void ina226_set_shunt_voltage_conversation_time(struct ina226_device_t *Device, ina226_conversion_time_t ConversionTime)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return);
    RangeCheck(ConversionTime, 0, INA226_NUM_CONVERSION_TIME, return);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig &= ~INA226_CFG_ShuntVoltageTimeMask;
    CurrentConfig |= (ConversionTime << INA226_CFG_ShuntVoltageTimeOffset);

    ina226_write_reg(Device, INA226_REG_CFG, CurrentConfig);
}

ina226_mode_t ina226_get_operation_mode(struct ina226_device_t *Device)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return -1);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig &= 0x07;

    return (ina226_mode_t)CurrentConfig;
}

void ina226_set_operation_mode(struct ina226_device_t *Device, ina226_mode_t Mode)
{
    uint16_t CurrentConfig = 0;

    NullCheck(Device, return);
    RangeCheck(Mode, 0, INA226_NUM_MODES, return);

    CurrentConfig = ina226_read_config(Device);
    CurrentConfig &= ~INA226_CFG_ModeMask;
    CurrentConfig |= Mode;

    ina226_write_reg(Device, INA226_REG_CFG, CurrentConfig);
}

/* Returns the shunt voltage in millivolts */
float ina226_get_shunt_voltage(struct ina226_device_t *Device)
{
    float Result = 0.0f;

    Result = (float)((int16_t)ina226_read_reg16(Device, INA226_REG_SHUNT_VOLTAGE));
    Result = Result * Device->shunt_voltage_lsb;

    return Result;
}

/* Returns the voltage (in millivolts) of VBUS */
float ina226_get_bus_voltage(struct ina226_device_t *Device)
{
    float Data = 0.0f;

    Data = (float)ina226_read_reg16(Device, INA226_REG_BUS_VOLTAGE);
    Data = Data * Device->bus_voltage_lsb;

    return Data;
}

/* Returns the current flowing in microamps */
float ina226_get_current(struct ina226_device_t *Device)
{
    float Data = 0.0f;

    Data = (float)((int16_t)ina226_read_reg16(Device, INA226_REG_CURRENT));
    Data = Data * Device->current_lsb;

    return Data;
}

/* Returns the power flowing in microwatts */
float ina226_get_power(struct ina226_device_t *Device)
{
    float Data = 0.0f;

    Data = (float)((uint16_t)ina226_read_reg16(Device, INA226_REG_POWER));
    Data = (Data * (Device->current_lsb * 25.0f));

    return Data;
}

void ina226_reset(struct ina226_device_t *Device)
{
    NullCheck(Device, return);
    ina226_write_config(Device, ina226_read_config(Device) | ina226_cfg_reset);
}

static void INA226_Calibrate_FP(struct ina226_device_t *Device, int RShuntInMilliOhms, int MaxCurrentInAmps)
{
    float RShunt = ((float)RShuntInMilliOhms) / 1000.0f;
    float current_lsb = 0.0f;
    float Cal = 0.0f;

    /* Somehow converting amperes to microamperes makes this work.
     * At least at the current point in time my head is going to explode figuring this out
     * but for now "Just Works(tm)" is good enough.
     */
    current_lsb = ((float)MaxCurrentInAmps * 1000000) / 32768.0f;
    Cal = (0.00512f / (current_lsb * RShunt)) * 1000000;

    Device->current_lsb = current_lsb;
    Device->calibration_value = Cal;
    Device->shunt_voltage_lsb = 2.5f;
    Device->bus_voltage_lsb = 1.25f;

    ina226_write_reg(Device, INA226_REG_CALIBRATION, (uint16_t)Cal);
}

void ina226_calibrate(struct ina226_device_t *Device, int RShuntInMilliOhms, int MaxCurrentInAmps)
{
    NullCheck(Device, return);
    INA226_Calibrate_FP(Device, RShuntInMilliOhms, MaxCurrentInAmps);
}

bool ina226_init(struct ina226_device_t *Device, int I2CAddress, int RShuntInMilliOhms, int MaxCurrentInAmps, INAWriteBytes WriteBytesFn, INAReadBytes ReadBytesFn)
{
    const uint16_t ConfigRegisterAfterReset = 0x4127;

    NullCheck(WriteBytesFn, return false);
    NullCheck(ReadBytesFn, return false);
    NullCheck(Device, return false);

    memset(Device, 0, sizeof(struct ina226_device_t));

    if (I2CAddress > 0)
    {
        INALock = xSemaphoreCreateMutex();

        Device->WriteBytesFn = WriteBytesFn;
        Device->ReadBytesFn = ReadBytesFn;
        Device->address = I2CAddress;

        ina226_reset(Device);

        /* Check to see if we can actually talk to the device, if we can then the initial
         * value for the config register should be 0x4127 after a reset.
         */
        if (ina226_read_config(Device) == ConfigRegisterAfterReset)
        {
            ina226_calibrate(Device, RShuntInMilliOhms, MaxCurrentInAmps);
            return true;
        }
    }

    return false;
}

ina226_alert_t ina226_get_alert_mask(struct ina226_device_t *Device)
{
    return (ina226_alert_t)ina226_read_reg16(Device, INA226_REG_ALERT_MASK);
}

ina226_alert_t ina226_set_alert_mask(struct ina226_device_t *Device, ina226_alert_t AlertMask)
{
    ina226_alert_t Old = ina226_get_alert_mask(Device);

    ina226_write_reg(Device, INA226_REG_ALERT_MASK, AlertMask);
    return Old;
}

static uint16_t INA226_SetAlertLimit(struct ina226_device_t *Device, float Value)
{
    uint16_t Old = ina226_read_reg16(Device, INA226_REG_ALERT_LIMIT);

    NullCheck(Device, return 0);
    ina226_write_reg(Device, INA226_REG_ALERT_LIMIT, (uint16_t)Value);

    return Old;
}

float ina226_set_alert_limit_bus_voltage(struct ina226_device_t *Device, float BusVoltageInMV)
{
    float OldLimit = 0.0f;

    NullCheck(Device, return 0.0f);

    OldLimit = (float)INA226_SetAlertLimit(Device, BusVoltageInMV * Device->bus_voltage_lsb);
    OldLimit /= Device->bus_voltage_lsb;

    return OldLimit;
}
