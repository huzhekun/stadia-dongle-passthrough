#pragma once
#include <stdint.h>

#define XBOX_HID_REPORT_DESC_LEN 252

int xbox_send_report(const uint8_t *report);