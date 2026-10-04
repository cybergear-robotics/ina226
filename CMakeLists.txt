set(ina226_driver_components driver)
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    list(APPEND ina226_driver_components esp_driver_i2c)
endif()

idf_component_register(SRCS "ina226.c"
    INCLUDE_DIRS "."
    REQUIRES ${ina226_driver_components}
)
