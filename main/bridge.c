/*
 * bridge.c — Translation between Stadia BLE input reports and USB reports.
 *
 * Supports two output modes controlled by STADIA_EMULATE_XINPUT:
 *   0: Stadia HID  (11-byte Stadia USB HID report, Report ID 0x03)
 *   1: XInputHID   (18-byte standard HID report, VID 045E PID 02FF)
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

void bridge_send_neutral(void)
{
#if STADIA_EMULATE_XINPUT
    uint8_t neutral[32] = {0};
    neutral[0] = 0xFF; neutral[1] = 0x7F; // X = 32767
    neutral[2] = 0xFF; neutral[3] = 0x7F; // Y = 32767
    neutral[4] = 0xFF; neutral[5] = 0x7F; // Rx = 32767
    neutral[6] = 0xFF; neutral[7] = 0x7F; // Ry = 32767
#else
    uint8_t neutral[11] = {
        STADIA_INPUT_REPORT_ID,
        0x08, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00
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

#if STADIA_EMULATE_XINPUT

/* ---- XInputHID mode (Xbox Series Bluetooth HID) ------------------------ */

static const uint8_t dpad_map_xh[8] = {
    1, 2, 3, 4, 5, 6, 7, 8  // Up, Up-Right, Right, Down-Right, Down, Down-Left, Left, Up-Left
};

void stadia_to_xinputhid(const uint8_t *s, uint8_t *x)
{
    memset(x, 0, 18);

    // 16-bit sticks: map 0-255 (unsigned) to 0-65534 (center=32767)
    int lx_i = (int)s[3] * 65534 / 255;
    int ly_i = (int)(255 - s[4]) * 65534 / 255; // invert Y
    int rx_i = (int)s[5] * 65534 / 255;
    int ry_i = (int)(255 - s[6]) * 65534 / 255; // invert Y

    x[0] = (uint8_t)(lx_i & 0xFF);
    x[1] = (uint8_t)(lx_i >> 8);
    x[2] = (uint8_t)(ly_i & 0xFF);
    x[3] = (uint8_t)(ly_i >> 8);
    x[4] = (uint8_t)(rx_i & 0xFF);
    x[5] = (uint8_t)(rx_i >> 8);
    x[6] = (uint8_t)(ry_i & 0xFF);
    x[7] = (uint8_t)(ry_i >> 8);

    // 10-bit triggers: Stadia 0-255 → XInputHID 0-1023
    int lt_i = (int)s[7] * 1023 / 255;
    int rt_i = (int)s[8] * 1023 / 255;
    x[8]  = (uint8_t)(lt_i & 0xFF);
    x[9]  = (uint8_t)((lt_i >> 8) & 0x03);
    x[10] = (uint8_t)(rt_i & 0xFF);
    x[11] = (uint8_t)((rt_i >> 8) & 0x03);

    // Buttons: 16 bits
    uint16_t btns = 0;
    if (s[2] & (1 << 6)) btns |= (1 << 0);  // A
    if (s[2] & (1 << 5)) btns |= (1 << 1);  // B
    if (s[2] & (1 << 4)) btns |= (1 << 2);  // X
    if (s[2] & (1 << 3)) btns |= (1 << 3);  // Y
    if (s[2] & (1 << 2)) btns |= (1 << 4);  // LB
    if (s[2] & (1 << 1)) btns |= (1 << 5);  // RB
    if (s[2] & (1 << 0)) btns |= (1 << 6);  // LS click
    if (s[1] & (1 << 7)) btns |= (1 << 7);  // RS click
    if (s[1] & (1 << 5)) btns |= (1 << 8);  // MENU → Start
    if (s[1] & (1 << 6)) btns |= (1 << 9);  // OPTIONS → Back
    if (s[1] & (1 << 4)) btns |= (1 << 10); // STADIA_BTN → extra button 11
    x[12] = (uint8_t)(btns & 0xFF);
    x[13] = (uint8_t)(btns >> 8);

    // Hat switch
    if (s[0] < 8) {
        x[14] = dpad_map_xh[s[0]];
    } else {
        x[14] = 0;
    }

    // Record (Share) — 0 for now
    x[15] = 0;

    // Battery = unknown until BLE reports
    x[17] = 0xFF;
}

#endif /* STADIA_EMULATE_XINPUT */

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