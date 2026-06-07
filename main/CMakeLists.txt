idf_component_register(
    SRCS "main.c" "bridge.c" "usb_stadia.c" "ble_central.c" "usb_xbox.c" "xbox_dev.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_timer driver
)
