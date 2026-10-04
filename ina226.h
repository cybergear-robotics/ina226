#ifndef _INA226_H_
#define _INA226_H_

#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#define INA226_I2C_ADDR 0x41
#define INA226_BUS_VOLTAGE_LSB 0.00125f

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

typedef uint16_t ina226_alert_mask_t;

#define INA226_ALERT_SHUNT_OVER_VOLTAGE    (1U << 15)
#define INA226_ALERT_SHUNT_UNDER_VOLTAGE   (1U << 14)
#define INA226_ALERT_BUS_OVER_VOLTAGE      (1U << 13)
#define INA226_ALERT_BUS_UNDER_VOLTAGE     (1U << 12)
#define INA226_ALERT_POWER_OVER_LIMIT      (1U << 11)
#define INA226_ALERT_CONVERSION_READY      (1U << 10)
#define INA226_ALERT_FUNCTION_FLAG         (1U << 4)
#define INA226_ALERT_CONVERSION_READY_FLAG (1U << 3)
#define INA226_ALERT_MATH_OVERFLOW_FLAG    (1U << 2)
#define INA226_ALERT_POLARITY              (1U << 1)
#define INA226_ALERT_LATCH_ENABLE          (1U << 0)

typedef struct
{
    i2c_master_dev_handle_t i2c_dev;
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
    const ina226_config_t *config;
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
 * @brief Configures the INA226 measurement and calibration registers.
 *
 * @param device INA226 device handle.
 * @param config INA226 bus and measurement configuration.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if device or config is NULL.
 * @return ESP_ERR_TIMEOUT if an I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_init(ina226_device_t *device, const ina226_config_t *config);

/**
 * @brief Gets the Mask/Enable Register.
 *
 * Reading this register clears a latched INA226_ALERT_FUNCTION_FLAG.
 *
 * @param device INA226 device handle initialized with ina226_init().
 * @param alert_mask Destination for the complete Mask/Enable Register value.
 * @return ESP_OK on success.
 * @return ESP_ERR_TIMEOUT if the I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_get_alert_mask(ina226_device_t *device, ina226_alert_mask_t *alert_mask);

/**
 * @brief Sets the complete Mask/Enable Register value.
 *
 * Combine INA226_ALERT_* bit masks as required. Only one limit function from
 * bits 15 through 11 can control the Alert pin at a time.
 *
 * @param device INA226 device handle initialized with ina226_init().
 * @param alert_mask Mask/Enable Register value to write.
 * @return ESP_OK on success.
 * @return ESP_ERR_TIMEOUT if the I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_set_alert_mask(ina226_device_t *device, ina226_alert_mask_t alert_mask);

/**
 * @brief Sets the raw Alert Limit Register value.
 *
 * Use this for shunt-voltage and power alerts, whose register encoding is not
 * a bus voltage. For bus-voltage alerts, prefer
 * ina226_set_bus_voltage_alert_limit().
 *
 * @param device INA226 device handle initialized with ina226_init().
 * @param limit Raw 16-bit Alert Limit Register value.
 * @return ESP_OK on success.
 * @return ESP_ERR_TIMEOUT if the I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_set_alert_limit_raw(ina226_device_t *device, uint16_t limit);

/**
 * @brief Gets the raw Alert Limit Register value.
 *
 * @param device INA226 device handle initialized with ina226_init().
 * @param limit Destination for the raw 16-bit Alert Limit Register value.
 * @return ESP_OK on success.
 * @return ESP_ERR_TIMEOUT if the I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_get_alert_limit_raw(ina226_device_t *device, uint16_t *limit);

/**
 * @brief Sets a bus-voltage alert limit in volts.
 *
 * Use this only with INA226_ALERT_BUS_OVER_VOLTAGE or
 * INA226_ALERT_BUS_UNDER_VOLTAGE.
 *
 * @param device INA226 device handle initialized with ina226_init().
 * @param voltage Bus-voltage limit in V, from 0 to 81.91875 V.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if voltage is outside the supported range.
 * @return ESP_ERR_TIMEOUT if the I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_set_bus_voltage_alert_limit(ina226_device_t *device, float voltage);

/**
 * @brief Gets the bus-voltage alert limit in volts.
 *
 * @param device INA226 device handle initialized with ina226_init().
 * @param voltage Destination for the bus-voltage limit in V.
 * @return ESP_OK on success.
 * @return ESP_ERR_TIMEOUT if the I2C transaction times out.
 * @return ESP_FAIL if the I2C transaction fails.
 */
esp_err_t ina226_get_bus_voltage_alert_limit(ina226_device_t *device, float *voltage);


#endif
