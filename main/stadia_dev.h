#pragma once
#include <stdint.h>

#define STADIA_HID_REPORT_DESC_LEN 204

// Custom TinyUSB class driver for the Stadia USB HID interface.
int stadia_usb_send_report(const uint8_t *report);
int stadia_usb_send_battery_status(const uint8_t *status);
