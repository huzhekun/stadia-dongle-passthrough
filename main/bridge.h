#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Set to 1 to enable verbose HID report logging on BLE and USB paths.
#define DONGLE_DEBUG 0

// ---- Feature toggle -------------------------------------------------------
// Set to 1 to emulate Xbox 360 controller (XInput, VID 045E PID 028E).
// Set to 0 to emulate Stadia controller (HID, VID 18D1 PID 9400).
#define STADIA_EMULATE_XBOX360 1

// ---- Queue item sizes -----------------------------------------------------
#if STADIA_EMULATE_XBOX360
 #define BLE_TO_USB_ITEM_SIZE   20   // Xbox 360 input report
 #define USB_TO_BLE_ITEM_SIZE    4   // Stadia-format rumble (xbox_dev converts)
 #define BATTERY_ITEM_SIZE       4   // "BAT" + percent (still used internally)
#else
 #define BLE_TO_USB_ITEM_SIZE   11   // Stadia USB HID input report
 #define USB_TO_BLE_ITEM_SIZE    4   // Stadia rumble payload (4 bytes, no ID)
 #define BATTERY_ITEM_SIZE       4   // "BAT" + percent
#endif

// ble_to_usb: input reports translated from BLE notifications
// usb_to_ble: rumble payloads from USB output
// battery_to_usb: battery status packets ("BAT" + percentage/0xFF unknown)
extern QueueHandle_t ble_to_usb_queue;
extern QueueHandle_t usb_to_ble_queue;
extern QueueHandle_t battery_to_usb_queue;

void bridge_init(void);
void bridge_send_neutral(void);
void stadia_ble_to_usb_hid(const uint8_t *stadia_ble, uint16_t len, uint8_t *stadia_usb);
void bridge_set_battery_level(uint8_t percent);
void bridge_clear_battery_level(void);

#if STADIA_EMULATE_XBOX360
void stadia_to_xbox360(const uint8_t *s, uint8_t *x);
#endif