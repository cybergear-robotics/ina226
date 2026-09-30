#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "unity.h"

#include <math.h>

#include "ina226.h"

static const i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = CONFIG_INA226_TEST_I2C_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = CONFIG_INA226_TEST_I2C_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
};

static const ina226_config_t ina226_config = {
    .i2c_port = I2C_NUM_0,
    .i2c_addr = INA226_I2C_ADDR,
    .timeout_ms = 100,
    .averages = INA226_AVERAGES_16,
    .bus_conv_time = INA226_BUS_CONV_TIME_1100_US,
    .shunt_conv_time = INA226_SHUNT_CONV_TIME_1100_US,
    .mode = INA226_MODE_SHUNT_BUS_CONT,
    .r_shunt = 0.0005f,
    .max_current = 10.0f,
};

#define INA226_MANUFACTURER_ID 0x5449
#define INA226_DIE_ID 0x2260

static ina226_device_t ina226;

void setUp(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, i2c_param_config(ina226_config.i2c_port, &i2c_config));
    TEST_ASSERT_EQUAL(ESP_OK, i2c_driver_install(ina226_config.i2c_port, i2c_config.mode, 0, 0, 0));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_init(&ina226, &ina226_config));
}

void tearDown(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, i2c_driver_delete(ina226_config.i2c_port));
}

static void test_bus_voltage_alert(ina226_alert_mask_t alert, float limit, int expect_alert)
{
    ina226_alert_mask_t previous_mask;
    ina226_alert_mask_t triggered_mask;
    uint16_t previous_limit;

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_mask(&ina226, &previous_mask));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_limit_raw(&ina226, &previous_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_bus_voltage_alert_limit(&ina226, limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_mask(&ina226, alert | INA226_ALERT_LATCH_ENABLE));

    /* 16 averaged 1.1 ms conversions require less than this delay. */
    vTaskDelay(pdMS_TO_TICKS(50));

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_mask(&ina226, &triggered_mask));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_mask(&ina226, 0));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_limit_raw(&ina226, previous_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_mask(&ina226, previous_mask));
    TEST_ASSERT_BITS(
        INA226_ALERT_FUNCTION_FLAG,
        expect_alert ? INA226_ALERT_FUNCTION_FLAG : 0,
        triggered_mask);
}

TEST_CASE("INA226 reports its manufacturer and die IDs", "[ina226][hardware]")
{
    uint16_t manufacturer_id;
    uint16_t die_id;

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_manufacturer_id(&ina226, &manufacturer_id));
    TEST_ASSERT_EQUAL_HEX16(INA226_MANUFACTURER_ID, manufacturer_id);
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_die_id(&ina226, &die_id));
    TEST_ASSERT_EQUAL_HEX16(INA226_DIE_ID, die_id);
}

TEST_CASE("INA226 reads bus, shunt, current, and power", "[ina226][hardware]")
{
    float bus_voltage;
    float shunt_voltage;
    float current;
    float power;

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_bus_voltage(&ina226, &bus_voltage));
    TEST_ASSERT_FLOAT_WITHIN(
        CONFIG_INA226_TEST_BUS_VOLTAGE_TOLERANCE_MV / 1000.0f,
        CONFIG_INA226_TEST_EXPECTED_BUS_VOLTAGE_MV / 1000.0f,
        bus_voltage);
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_shunt_voltage(&ina226, &shunt_voltage));
    TEST_ASSERT_TRUE(isfinite(shunt_voltage));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_current(&ina226, &current));
    TEST_ASSERT_TRUE(isfinite(current));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_power(&ina226, &power));
    TEST_ASSERT_TRUE(isfinite(power));
}

TEST_CASE("INA226 updates and restores the alert mask", "[ina226][hardware]")
{
    const ina226_alert_mask_t test_mask =
        INA226_ALERT_CONVERSION_READY | INA226_ALERT_LATCH_ENABLE;
    ina226_alert_mask_t previous_mask;
    ina226_alert_mask_t updated_mask;

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_mask(&ina226, &previous_mask));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_mask(&ina226, test_mask));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_mask(&ina226, &updated_mask));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_mask(&ina226, previous_mask));
    TEST_ASSERT_EQUAL_UINT16(test_mask, updated_mask & test_mask);
}

TEST_CASE("INA226 updates and restores the raw alert limit", "[ina226][hardware]")
{
    const uint16_t test_limit = 0x1234;
    uint16_t previous_limit;
    uint16_t updated_limit;

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_limit_raw(&ina226, &previous_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_limit_raw(&ina226, test_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_alert_limit_raw(&ina226, &updated_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_alert_limit_raw(&ina226, previous_limit));
    TEST_ASSERT_EQUAL_HEX16(test_limit, updated_limit);
}

TEST_CASE("INA226 updates and restores the bus voltage alert limit", "[ina226][hardware]")
{
    const float test_limit = 24.0f;
    float previous_limit;
    float updated_limit;

    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_bus_voltage_alert_limit(&ina226, &previous_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_bus_voltage_alert_limit(&ina226, test_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_get_bus_voltage_alert_limit(&ina226, &updated_limit));
    TEST_ASSERT_EQUAL(ESP_OK, ina226_set_bus_voltage_alert_limit(&ina226, previous_limit));
    TEST_ASSERT_FLOAT_WITHIN(INA226_BUS_VOLTAGE_LSB, test_limit, updated_limit);
}

TEST_CASE("INA226 triggers the under-voltage alert above the bus voltage", "[ina226][hardware]")
{
    test_bus_voltage_alert(INA226_ALERT_BUS_UNDER_VOLTAGE, 25.0f, 1);
}

TEST_CASE("INA226 triggers the over-voltage alert below the bus voltage", "[ina226][hardware]")
{
    test_bus_voltage_alert(INA226_ALERT_BUS_OVER_VOLTAGE, 23.0f, 1);
}

TEST_CASE("INA226 does not trigger the under-voltage alert below the bus voltage", "[ina226][hardware]")
{
    test_bus_voltage_alert(INA226_ALERT_BUS_UNDER_VOLTAGE, 23.0f, 0);
}

TEST_CASE("INA226 does not trigger the over-voltage alert above the bus voltage", "[ina226][hardware]")
{
    test_bus_voltage_alert(INA226_ALERT_BUS_OVER_VOLTAGE, 25.0f, 0);
}
