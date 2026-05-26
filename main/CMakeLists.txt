idf_component_register(
    SRCS "stadia_dev.c" "main.c" "bridge.c" "usb_stadia.c" "ble_central.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_timer driver
)
