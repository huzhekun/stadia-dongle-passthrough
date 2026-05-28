#pragma once
#include <stdbool.h>

void usb_xbox_init(void);
void usb_xbox_task(void *arg);
void usb_xbox_set_connected(bool connected);