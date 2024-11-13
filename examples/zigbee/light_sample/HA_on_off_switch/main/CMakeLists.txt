idf_component_register(
    SRCS
    "esp_zb_switch.c"
    "switch_driver.c"
    "blink_led.c"
    INCLUDE_DIRS "."
)

# target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11 -fno-rtti -fno-exceptions)
