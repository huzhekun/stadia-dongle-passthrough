#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Set to 1 to enable verbose HID report logging on BLE and USB paths.
#define DONGLE_DEBUG 0

// ble_to_usb: 11-byte Stadia USB HID input reports translated from BLE notifications
// usb_to_ble: 4-byte Stadia rumble payloads from USB HID output report 0x05
// battery_to_usb: 4-byte vendor status packets, "BAT" + percentage/0xFF unknown
extern QueueHandle_t ble_to_usb_queue;
extern QueueHandle_t usb_to_ble_queue;
extern QueueHandle_t battery_to_usb_queue;

void bridge_init(void);
void bridge_send_neutral(void);
void stadia_ble_to_usb_hid(const uint8_t *stadia_ble, uint16_t len, uint8_t *stadia_usb);
void bridge_set_battery_level(uint8_t percent);
void bridge_clear_battery_level(void);
