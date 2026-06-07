#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Set to 1 to enable verbose HID report logging on BLE and USB paths.
#define DONGLE_DEBUG 0

// ---- Feature toggle -------------------------------------------------------
// 0 = Stadia HID (VID 18D1 PID 9400)
// 1 = XInputHID (Xbox Series BLE HID, VID 045E PID 02FF)
#define STADIA_EMULATE_XINPUT 1

// ---- Queue item sizes -----------------------------------------------------
#if STADIA_EMULATE_XINPUT
 #define BLE_TO_USB_ITEM_SIZE   32   // XInputHID input report
 #define USB_TO_BLE_ITEM_SIZE    4   // Stadia-format rumble
 #define BATTERY_ITEM_SIZE       4
#else
 #define BLE_TO_USB_ITEM_SIZE   11   // Stadia USB HID input report
 #define USB_TO_BLE_ITEM_SIZE    4   // Stadia rumble payload
 #define BATTERY_ITEM_SIZE       4
#endif

extern QueueHandle_t ble_to_usb_queue;
extern QueueHandle_t usb_to_ble_queue;
extern QueueHandle_t battery_to_usb_queue;

void bridge_init(void);
void bridge_send_neutral(void);
void stadia_ble_to_usb_hid(const uint8_t *stadia_ble, uint16_t len, uint8_t *stadia_usb);
void bridge_set_battery_level(uint8_t percent);
void bridge_clear_battery_level(void);

#if STADIA_EMULATE_XINPUT
void stadia_to_xinputhid(const uint8_t *s, uint8_t *x);
#endif