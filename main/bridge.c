/*
 * bridge.c - Minimal Stadia BLE HID to Stadia USB HID bridging.
 *
 * Stadia HOGP notifications arrive without the Report ID. The USB HID report
 * for the same input state is Report ID 0x03 followed by the 10-byte payload:
 *   [0] 0x03 report ID
 *   [1] hat switch (0..7, 8/other = neutral)
 *   [2..3] button bitfield
 *   [4] left stick X
 *   [5] left stick Y
 *   [6] right stick X
 *   [7] right stick Y
 *   [8] left trigger
 *   [9] right trigger
 *   [10] consumer/system buttons
 *
 * USB rumble comes back as Report ID 0x05 plus two little-endian 16-bit motor
 * magnitudes. BLE HOGP writes use the same four payload bytes without the ID.
 */

#include "bridge.h"
#include <string.h>

#define STADIA_USB_INPUT_LEN 11
#define STADIA_BLE_INPUT_LEN 10
#define STADIA_INPUT_REPORT_ID 0x03

QueueHandle_t ble_to_usb_queue;
QueueHandle_t usb_to_ble_queue;
QueueHandle_t battery_to_usb_queue;

void bridge_init(void)
{
    ble_to_usb_queue = xQueueCreate(8, STADIA_USB_INPUT_LEN);
    usb_to_ble_queue = xQueueCreate(4, 4);
    battery_to_usb_queue = xQueueCreate(2, 4);
}

void bridge_send_neutral(void)
{
    uint8_t neutral[STADIA_USB_INPUT_LEN] = {
        STADIA_INPUT_REPORT_ID,
        0x08,             // neutral hat
        0x00, 0x00,       // buttons
        0x80, 0x80,       // left stick center
        0x80, 0x80,       // right stick center
        0x00, 0x00,       // triggers
        0x00,             // consumer/system buttons
    };
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

static void queue_battery_status(uint8_t percent)
{
    uint8_t status[4] = { 'B', 'A', 'T', percent };
    if (xQueueSendToBack(battery_to_usb_queue, status, 0) != pdTRUE) {
        uint8_t dummy[4];
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
