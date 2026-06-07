#pragma once
#include <stdbool.h>

// Shared by all emulation modes (Xbox 360, XInputHID)
void usb_xbox_init(void);
void usb_xbox_task(void *arg);
void usb_xbox_set_connected(bool connected);