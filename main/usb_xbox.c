/*
 * usb_xbox.c — XInputHID (Xbox Series Bluetooth HID) USB descriptor and task.
 *
 * Presents as an Xbox-compatible HID gamepad (VID 045E PID 02FF).
 * Windows loads xinputhid.sys natively for this standard HID descriptor.
 *
 * USB stays detached until a Stadia controller pairs over BLE.
 */

#include "usb_xbox.h"
#include "xbox_dev.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "USB";

static bool s_usb_attached = false;
static volatile bool s_disconnect_pending = false;

static const tusb_desc_device_t s_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x045E, // Microsoft
    .idProduct          = 0x02FF, // XInputHID compatible
    .bcdDevice          = 0x0503,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

#define XBOX_CFG_LEN 41

static const uint8_t s_cfg_desc[XBOX_CFG_LEN] = {
    // Configuration Descriptor (9)
    0x09, 0x02,
    XBOX_CFG_LEN & 0xFF, XBOX_CFG_LEN >> 8,
    0x01,       // bNumInterfaces
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration
    0xA0,       // bmAttributes: bus-powered, remote-wakeup
    0xFA,       // bMaxPower: 500 mA

    // Interface 0: HID (9)
    0x09, 0x04,
    0x00,       // bInterfaceNumber
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
    XBOX_HID_REPORT_DESC_LEN & 0xFF, XBOX_HID_REPORT_DESC_LEN >> 8,

    // EP1 IN: Interrupt, 64 bytes, 4 ms
    0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x04,

    // EP2 OUT: Interrupt, 64 bytes, 8 ms
    0x07, 0x05, 0x02, 0x03, 0x40, 0x00, 0x08,
};

static const char *s_str_desc[] = {
    (char[]){0x09, 0x04}, // [0] Language: English (0x0409)
    "Microsoft",          // [1] iManufacturer
    "Controller",         // [2] iProduct
    "000000000001",       // [3] iSerialNumber
};

/* ---- USB task ----------------------------------------------------------- */

void usb_xbox_task(void *arg)
{
    uint8_t report[BLE_TO_USB_ITEM_SIZE];
    bool have_report = false;
    while (1) {
        if (s_disconnect_pending) {
            while (xQueueReceive(ble_to_usb_queue, report, 0)) {
                have_report = true;
            }
            if (have_report && s_usb_attached && s_disconnect_pending) {
                xbox_send_report(report);
            }
            have_report = false;
            {
                uint8_t dummy[4];
                while (xQueueReceive(battery_to_usb_queue, dummy, 0)) {}
            }
            if (s_usb_attached) {
                tud_disconnect();
                s_usb_attached = false;
                ESP_LOGI(TAG, "USB device detached");
            }
            s_disconnect_pending = false;
        }

        if (!have_report) {
            if (!xQueueReceive(ble_to_usb_queue, report, pdMS_TO_TICKS(4))) continue;
            have_report = true;
        }
        while (xQueueReceive(ble_to_usb_queue, report, 0)) {
            have_report = true;
        }

        int res = xbox_send_report(report);
        if (res == 1) {
            #if DONGLE_DEBUG
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, report, BLE_TO_USB_ITEM_SIZE, ESP_LOG_INFO);
            #endif
            have_report = false;
        } else if (res == -1) {
            have_report = false;
            vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void usb_xbox_init(void)
{
    tinyusb_config_t cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy  = { .skip_setup = false, .self_powered = false },
        .task = { .size = 4096, .priority = 4, .xCoreID = 1 },
        .descriptor = {
            .device           = &s_device_desc,
            .qualifier        = NULL,
            .string           = s_str_desc,
            .string_count     = 4,
            .full_speed_config = s_cfg_desc,
            .high_speed_config = NULL,
        },
        .event_cb  = NULL,
        .event_arg = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));
    tud_disconnect();
    s_usb_attached = false;
    ESP_LOGI(TAG, "XInputHID initialised detached (VID=045E PID=02FF)");
}

void usb_xbox_set_connected(bool connected)
{
    if (connected) {
        s_disconnect_pending = false;
        tud_connect();
        s_usb_attached = true;
        ESP_LOGI(TAG, "USB device attached");
    } else {
        s_disconnect_pending = true;
    }
}