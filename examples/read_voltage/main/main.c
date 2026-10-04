#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"

#include "ina226.h"

#define TAG "read_voltage"

static const i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = CONFIG_I2C_MASTER_SDA,
    .scl_io_num = CONFIG_I2C_MASTER_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

static const i2c_device_config_t ina226_i2c_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = INA226_I2C_ADDR,
    .scl_speed_hz = 400000,
};

static ina226_config_t ina_config = {
	.timeout_ms = 100, /* wait up to 100 ms on writes */
    .averages = INA226_AVERAGES_16,
    .bus_conv_time = INA226_BUS_CONV_TIME_1100_US,
    .shunt_conv_time = INA226_SHUNT_CONV_TIME_1100_US,
    .mode = INA226_MODE_SHUNT_BUS_CONT,
	.r_shunt = 0.0005, /* rshunt is 0.5 milli ohms */
	.max_current = 10 /* up to max 10 amps */
};


void app_main(void)
{
	ina226_device_t ina;
	static i2c_master_bus_handle_t i2c_bus;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &ina226_i2c_config, &ina_config.i2c_dev));
	
	/* Setup INA226 device. */
	ESP_ERROR_CHECK(ina226_init(&ina, &ina_config));

    /* loop */
	float voltage;
	float power;
	float current;
    while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		ESP_ERROR_CHECK(ina226_get_bus_voltage(&ina, &voltage));
		ESP_ERROR_CHECK(ina226_get_power(&ina, &power));
		ESP_ERROR_CHECK(ina226_get_current(&ina, &current));
		ESP_LOGI(TAG, "%f V | %f W | %f A", voltage, power, current);
    }
}
