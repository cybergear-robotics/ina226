#ifndef _INA226_H_
#define _INA226_H_

#include "esp_err.h"
#include "driver/i2c.h"

#define INA226_I2C_ADDR 0x41

typedef enum
{
    INA226_AVERAGES_1             = 0b000,
    INA226_AVERAGES_4             = 0b001,
    INA226_AVERAGES_16            = 0b010,
    INA226_AVERAGES_64            = 0b011,
    INA226_AVERAGES_128           = 0b100,
    INA226_AVERAGES_256           = 0b101,
    INA226_AVERAGES_512           = 0b110,
    INA226_AVERAGES_1024          = 0b111
} ina226_averages_t;

typedef enum
{
    INA226_BUS_CONV_TIME_140_US    = 0b000,
    INA226_BUS_CONV_TIME_204_US    = 0b001,
    INA226_BUS_CONV_TIME_332_US    = 0b010,
    INA226_BUS_CONV_TIME_588_US    = 0b011,
    INA226_BUS_CONV_TIME_1100_US   = 0b100,
    INA226_BUS_CONV_TIME_2116_US   = 0b101,
    INA226_BUS_CONV_TIME_4156_US   = 0b110,
    INA226_BUS_CONV_TIME_8244_US   = 0b111
} ina226_bus_conv_time_t;


typedef enum
{
    INA226_SHUNT_CONV_TIME_140_US   = 0b000,
    INA226_SHUNT_CONV_TIME_204_US   = 0b001,
    INA226_SHUNT_CONV_TIME_332_US   = 0b010,
    INA226_SHUNT_CONV_TIME_588_US   = 0b011,
    INA226_SHUNT_CONV_TIME_1100_US  = 0b100,
    INA226_SHUNT_CONV_TIME_2116_US  = 0b101,
    INA226_SHUNT_CONV_TIME_4156_US  = 0b110,
    INA226_SHUNT_CONV_TIME_8244_US  = 0b111
} ina226_shunt_conv_time_t;

typedef enum
{

    INA226_MODE_POWER_DOWN      = 0b000,
    INA226_MODE_SHUNT_TRIG      = 0b001,
    INA226_MODE_BUS_TRIG        = 0b010,
    INA226_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA226_MODE_ADC_OFF         = 0b100,
    INA226_MODE_SHUNT_CONT      = 0b101,
    INA226_MODE_BUS_CONT        = 0b110,
    INA226_MODE_SHUNT_BUS_CONT  = 0b111,
} ina226_mode_t;

typedef enum
{
    INA226_ALERT_SHUNT_OVER_VOLTAGE     = 0xf,
    INA226_ALERT_SHUNT_UNDER_VOLTAGE    = 0xe,
    INA226_ALERT_BUS_OVER_VOLTAGE       = 0xd,
    INA226_ALERT_BUS_UNDER_VOLTAGE      = 0xc,
    INA226_ALERT_POWER_OVER_LIMIT       = 0xb,
    INA226_ALERT_CONVERSION_READY       = 0xa,
    INA226_ALERT_FUNCTION_FLAG          = 0x4,
    INA226_ALERT_CONVERSION_READY_FLAG  = 0x3,
    INA226_ALERT_MATH_OVERFLOW_FLAG     = 0x2,
    INA226_ALERT_POLARITY               = 0x1,
    INA226_ALERT_LATCH_ENABLE           = 0x0
} ina226_alert_t;

typedef struct
{
    i2c_port_t i2c_port;
    int i2c_addr;
    uint32_t timeout_ms;
    ina226_averages_t averages;
    ina226_bus_conv_time_t bus_conv_time;
    ina226_shunt_conv_time_t shunt_conv_time;
    ina226_mode_t mode;
    float r_shunt; /* ohm */
    float max_current; /* amps */
} ina226_config_t;


typedef struct
{
    float current_lsb;
    float power_lsb;
    ina226_config_t *config;
} ina226_device_t;


/**
 * @brief get manufacturer ID
 * 
 * @param device pointer to device handle
 * @param manufacturer_id store result to this memory
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_manufacturer_id(ina226_device_t *device, uint16_t *manufacturer_id);

/**
 * @brief get ID of the die.
 * 
 * @param device pointer to device handle
 * @param die_id store result to this memory
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_die_id(ina226_device_t *device, uint16_t *die_id);

/**
 * @brief get voltage from the shunt.
 * 
 * this function is mostly not required. You might look for `ina226_get_bus_voltage`.
 * @param device pointer to device handle
 * @param voltage in Volt (V)
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_shunt_voltage(ina226_device_t *device, float *voltage);

/**
 * @brief get measured voltage
 * 
 * @param device pointer to device handle
 * @param voltage in Volt (V)
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_bus_voltage(ina226_device_t *device, float *voltage);

/**
 * @brief get measured current
 * 
 * @param device pointer to device handle
 * @param current in Amber (A)
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_current(ina226_device_t *device, float *current);

/**
 * @brief get measured power
 * 
 * @param device pointer to device handle
 * @param power in Watt (W)
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_power(ina226_device_t *device, float *power);

/**
 * @brief initialize INA226 with provided config
 * 
 * This will set the configuration register and setup the calibration.
 * 
 * @param device pointer to device handle
 * @param config configuration
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_init(ina226_device_t *device, ina226_config_t *config);

/**
 * @brief get alert mask
 * 
 * @param device pointer to device handle
 * @param alert_mask mask
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_get_alert_mask(ina226_device_t *device, ina226_alert_t *alert_mask);

/**
 * @brief set alert mask
 * 
 * @param device pointer to device handle
 * @param alert_mask mask
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_set_alert_mask(ina226_device_t *device, ina226_alert_t alert_mask);

/**
 * @brief set alert limit
 * 
 * @param device pointer to device handle
 * @param voltage the alert limit in V
 * @return esp_err_t returns ESP_OK on success
 */
esp_err_t ina226_set_alert_limit(ina226_device_t *device, float voltage);


#endif
