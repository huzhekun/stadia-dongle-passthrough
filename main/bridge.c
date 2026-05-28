/*
 * bridge.c — Translation between Stadia BLE input reports and USB reports.
 *
 * Supports two output modes controlled by STADIA_EMULATE_XBOX360:
 *   Stadia  HID (0): 11-byte Stadia USB HID report (Report ID 0x03)
 *   Xbox 360     (1): 20-byte Xbox 360 input report (vendor-specific)
 *
 * Stadia BLE notification payload (10 bytes, no Report ID):
 *   [0] D-pad hat (0=Up … 7=Up-L, >7=neutral)
 *   [1] bits 7=RS, 6=OPTIONS, 5=MENU, 4=STADIA_BTN
 *   [2] bits 6=A, 5=B, 4=X, 3=Y, 2=LB, 1=RB, 0=LS
 *   [3] Left stick X  (0–255)
 *   [4] Left stick Y  (0–255, 0=up)
 *   [5] Right stick X (0–255)
 *   [6] Right stick Y (0–255, 0=up)
 *   [7] Left trigger  (0–255)
 *   [8] Right trigger (0–255)
 *   [9] consumer/system buttons
 */

#include "bridge.h"
#include <string.h>

#define STADIA_BLE_INPUT_LEN 10
#define STADIA_INPUT_REPORT_ID 0x03

QueueHandle_t ble_to_usb_queue;
QueueHandle_t usb_to_ble_queue;
QueueHandle_t battery_to_usb_queue;

void bridge_init(void)
{
    ble_to_usb_queue = xQueueCreate(8, BLE_TO_USB_ITEM_SIZE);
    usb_to_ble_queue = xQueueCreate(4, USB_TO_BLE_ITEM_SIZE);
    battery_to_usb_queue = xQueueCreate(2, BATTERY_ITEM_SIZE);
}

/* ---- Stadia HID mode --------------------------------------------------- */

void bridge_send_neutral(void)
{
#if STADIA_EMULATE_XBOX360
    uint8_t neutral[20] = {
        0x00, 0x14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
#else
    uint8_t neutral[11] = {
        STADIA_INPUT_REPORT_ID,
        0x08,             // neutral hat
        0x00, 0x00,       // buttons
        0x80, 0x80,       // left stick center
        0x80, 0x80,       // right stick center
        0x00, 0x00,       // triggers
        0x00,             // consumer/system buttons
    };
#endif
    xQueueReset(ble_to_usb_queue);
    xQueueSendToBack(ble_to_usb_queue, neutral, 0);
}

void stadia_ble_to_usb_hid(const uint8_t *stadia_ble, uint16_t len, uint8_t *stadia_usb)
{
    uint8_t neutral_payload[STADIA_BLE_INPUT_LEN] = {
        0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00
    };

    stadia_usb[0] = STADIA_INPUT_REPORT_ID;

    if (len >= STADIA_BLE_INPUT_LEN) {
        memcpy(&stadia_usb[1], stadia_ble, STADIA_BLE_INPUT_LEN);
    } else {
        memcpy(&stadia_usb[1], neutral_payload, STADIA_BLE_INPUT_LEN);
        memcpy(&stadia_usb[1], stadia_ble, len);
    }
}

#if STADIA_EMULATE_XBOX360

/* ---- Xbox 360 mode ----------------------------------------------------- */

/* Stadia hat-switch (0–7) → Xbox 360 d-pad bitmask
 * bits 0=Up, 1=Down, 2=Left, 3=Right */
static const uint8_t dpad_map[8] = {
    0x01, // Up
    0x09, // Up+Right
    0x08, // Right
    0x0A, // Down+Right
    0x02, // Down
    0x06, // Down+Left
    0x04, // Left
    0x05, // Up+Left
};

/* Map 0–255 unsigned stick value to -32767..+32767 signed, optionally inverting axis.
 * Center dead-zone: raw 128 → 0, raw 0/255 → ±32767. */
static int16_t map_stick(uint8_t v, int invert)
{
    int c = (int)v - 128;
    if (c < -127) c = -127;
    if (invert) c = -c;
    return (int16_t)(32767 * c / 127);
}

void stadia_to_xbox360(const uint8_t *s, uint8_t *x)
{
    memset(x, 0, 20);
    x[0] = 0x00; // packet type
    x[1] = 0x14; // packet length = 20

    // Byte 2: d-pad + Start / Back / LS / RS
    uint8_t b2 = (s[0] < 8) ? dpad_map[s[0]] : 0;
    if (s[1] & (1 << 5)) b2 |= (1 << 4); // MENU    → Start
    if (s[1] & (1 << 6)) b2 |= (1 << 5); // OPTIONS → Back
    if (s[2] & (1 << 0)) b2 |= (1 << 6); // LS
    if (s[1] & (1 << 7)) b2 |= (1 << 7); // RS
    x[2] = b2;

    // Byte 3: LB / RB / Guide / A / B / X / Y
    uint8_t b3 = 0;
    if (s[2] & (1 << 2)) b3 |= (1 << 0); // LB
    if (s[2] & (1 << 1)) b3 |= (1 << 1); // RB
    if (s[1] & (1 << 4)) b3 |= (1 << 2); // STADIA_BTN → Guide
    if (s[2] & (1 << 6)) b3 |= (1 << 4); // A
    if (s[2] & (1 << 5)) b3 |= (1 << 5); // B
    if (s[2] & (1 << 4)) b3 |= (1 << 6); // X
    if (s[2] & (1 << 3)) b3 |= (1 << 7); // Y
    x[3] = b3;

    x[4] = s[7]; // left trigger
    x[5] = s[8]; // right trigger

    int16_t lx = map_stick(s[3], 0);
    int16_t ly = map_stick(s[4], 1); // Stadia Y=0 is up → invert for Xbox (positive=up)
    int16_t rx = map_stick(s[5], 0);
    int16_t ry = map_stick(s[6], 1);

    x[6]  = (uint8_t)(lx & 0xFF); x[7]  = (uint8_t)(lx >> 8);
    x[8]  = (uint8_t)(ly & 0xFF); x[9]  = (uint8_t)(ly >> 8);
    x[10] = (uint8_t)(rx & 0xFF); x[11] = (uint8_t)(rx >> 8);
    x[12] = (uint8_t)(ry & 0xFF); x[13] = (uint8_t)(ry >> 8);
    // bytes 14–19 already zero
}

#endif /* STADIA_EMULATE_XBOX360 */

/* ---- Battery (shared between modes) ------------------------------------ */

static void queue_battery_status(uint8_t percent)
{
    uint8_t status[BATTERY_ITEM_SIZE] = { 'B', 'A', 'T', percent };
    if (xQueueSendToBack(battery_to_usb_queue, status, 0) != pdTRUE) {
        uint8_t dummy[BATTERY_ITEM_SIZE];
        xQueueReceive(battery_to_usb_queue, dummy, 0);
        xQueueSendToBack(battery_to_usb_queue, status, 0);
    }
}

void bridge_set_battery_level(uint8_t percent)
{
    if (percent > 100) percent = 100;
    queue_battery_status(percent);
}

void bridge_clear_battery_level(void)
{
    queue_battery_status(0xFF);
}