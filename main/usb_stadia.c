/*
 * usb_stadia.c - Stadia-compatible USB descriptors and input task.
 *
 * The USB host sees a Google Stadia Controller-shaped composite device:
 *   VID:PID 18D1:9400
 *   Interface 0: vendor-specific bulk endpoints (present for descriptor parity)
 *   Interface 1: HID gamepad, input report 0x03 and output report 0x05
 */

#include "usb_stadia.h"
#include "stadia_dev.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "USB";

#define STADIA_USB_INPUT_LEN 11

static bool s_usb_attached = false;

static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xEF,
    .bDeviceSubClass    = 0x02,
    .bDeviceProtocol    = 0x01,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x18D1, // Google
    .idProduct          = 0x9400, // Stadia Controller
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

#define STADIA_CFG_LEN 80

static const uint8_t s_cfg_desc[STADIA_CFG_LEN] = {
    // Configuration Descriptor (9)
    0x09, 0x02,
    STADIA_CFG_LEN & 0xFF, STADIA_CFG_LEN >> 8,
    0x02,       // bNumInterfaces
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration
    0x80,       // bmAttributes: bus-powered
    0xFA,       // bMaxPower: 500 mA

    // Interface Association Descriptor: vendor interface (8)
    0x08, 0x0B,
    0x00,       // bFirstInterface
    0x01,       // bInterfaceCount
    0xFF,       // bFunctionClass: vendor-specific
    0x00,       // bFunctionSubClass
    0x00,       // bFunctionProtocol
    0x00,       // iFunction

    // Interface 0: vendor-specific bulk pipe (9)
    0x09, 0x04,
    0x00,       // bInterfaceNumber
    0x00,       // bAlternateSetting
    0x02,       // bNumEndpoints
    0xFF,       // bInterfaceClass
    0x00,       // bInterfaceSubClass
    0x00,       // bInterfaceProtocol
    0x00,       // iInterface

    // EP7 IN: Bulk, 64 bytes
    0x07, 0x05, 0x87, 0x02, 0x40, 0x00, 0x00,

    // EP7 OUT: Bulk, 64 bytes
    0x07, 0x05, 0x07, 0x02, 0x40, 0x00, 0x00,

    // Interface Association Descriptor: HID interface (8)
    0x08, 0x0B,
    0x01,       // bFirstInterface
    0x01,       // bInterfaceCount
    0x03,       // bFunctionClass: HID
    0x00,       // bFunctionSubClass
    0x00,       // bFunctionProtocol
    0x00,       // iFunction

    // Interface 1: HID gamepad (9)
    0x09, 0x04,
    0x01,       // bInterfaceNumber
    0x00,       // bAlternateSetting
    0x02,       // bNumEndpoints
    0x03,       // bInterfaceClass: HID
    0x00,       // bInterfaceSubClass
    0x00,       // bInterfaceProtocol
    0x00,       // iInterface

    // HID descriptor (9)
    0x09, 0x21,
    0x11, 0x01, // bcdHID 1.11
    0x00,       // bCountryCode
    0x01,       // bNumDescriptors
    0x22,       // Report descriptor
    STADIA_HID_REPORT_DESC_LEN & 0xFF, STADIA_HID_REPORT_DESC_LEN >> 8,

    // EP3 IN: Interrupt, 64 bytes, 6 ms
    0x07, 0x05, 0x83, 0x03, 0x40, 0x00, 0x06,

    // EP3 OUT: Interrupt, 64 bytes, 6 ms
    0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x06,
};

static const char *s_str_desc[] = {
    (char[]){0x09, 0x04}, // [0] Language: English (0x0409)
    "Google Inc.",
    "Stadia Controller",
    "ESP32STADIA001",
};

void usb_stadia_task(void *arg)
{
    uint8_t report[STADIA_USB_INPUT_LEN];
    uint8_t battery_status[4];
    bool have_report = false;
    bool have_battery_status = false;

    while (1) {
        if (!have_battery_status && xQueueReceive(battery_to_usb_queue, battery_status, 0)) {
            have_battery_status = true;
        }
        while (xQueueReceive(battery_to_usb_queue, battery_status, 0)) {
            have_battery_status = true;
        }
        if (have_battery_status) {
            int battery_res = stadia_usb_send_battery_status(battery_status);
            if (battery_res != 0) {
                have_battery_status = false;
            }
        }

        if (!have_report) {
            if (!xQueueReceive(ble_to_usb_queue, report, pdMS_TO_TICKS(4))) continue;
            have_report = true;
        }

        while (xQueueReceive(ble_to_usb_queue, report, 0)) {
            have_report = true;
        }

        int res = stadia_usb_send_report(report);
        if (res == 1) {
            #if DONGLE_DEBUG
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, report, STADIA_USB_INPUT_LEN, ESP_LOG_INFO);
            #endif
            have_report = false;
        } else if (res == -1) {
            have_report = false;
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            #if DONGLE_DEBUG
            ESP_LOGW(TAG, "EP busy, retrying");
            #endif
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void usb_stadia_init(void)
{
    tinyusb_config_t cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy  = { .skip_setup = false, .self_powered = false },
        .task = { .size = 4096, .priority = 4, .xCoreID = 1 },
        .descriptor = {
            .device            = &s_device_desc,
            .qualifier         = NULL,
            .string            = s_str_desc,
            .string_count      = 4,
            .full_speed_config = s_cfg_desc,
            .high_speed_config = NULL,
        },
        .event_cb  = NULL,
        .event_arg = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));
    tud_disconnect();
    s_usb_attached = false;
    ESP_LOGI(TAG, "Stadia USB HID initialised detached (VID=18D1 PID=9400)");
}

void usb_stadia_set_connected(bool connected)
{
    if (connected == s_usb_attached) return;

    if (connected) {
        tud_connect();
        s_usb_attached = true;
        ESP_LOGI(TAG, "USB Stadia device attached");
    } else {
        tud_disconnect();
        s_usb_attached = false;
        ESP_LOGI(TAG, "USB Stadia device detached");
    }
}
