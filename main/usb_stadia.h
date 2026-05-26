#pragma once
#include <stdbool.h>

void usb_stadia_init(void);
void usb_stadia_task(void *arg);
void usb_stadia_set_connected(bool connected);
