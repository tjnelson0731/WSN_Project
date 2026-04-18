idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES espressif__esp-zigbee-lib espressif__esp-zboss-lib nvs_flash
)