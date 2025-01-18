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

/* i2c bus configuration */
i2c_config_t conf = {
	.mode = I2C_MODE_MASTER,
	.sda_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SDA,
	.sda_pullup_en = GPIO_PULLUP_ENABLE,
	.scl_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SCL,
	.scl_pullup_en = GPIO_PULLUP_ENABLE,
	.master.clk_speed = 400000,
	.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

ina226_config_t ina_config = {
	.i2c_port = I2C_NUM_0,	/* use i2c port 0 */
	.i2c_addr = INA226_I2C_ADDR, /* use default i2c address of INA226*/
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

	/* setup i2c */
	ESP_ERROR_CHECK(i2c_param_config(ina_config.i2c_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(ina_config.i2c_port, conf.mode, 0, 0, 0));
	
	/* setup ICM20948 device */
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
