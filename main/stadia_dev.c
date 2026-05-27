/*
 * stadia_dev.c - Custom TinyUSB class driver for Stadia-compatible USB HID.
 *
 * Single HID interface with two top-level Application Collections:
 *   Collection 1 — gamepad (Report ID 0x03 input, Report ID 0x05 output)
 *   Collection 2 — battery  (Report ID 0x06 input)
 */

#include "stadia_dev.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include <string.h>

static const char *TAG = "STADIA_DEV";

#define STADIA_HID_ITF      0x00
#define STADIA_EP_IN        0x81
#define STADIA_EP_OUT       0x01
#define STADIA_EP_SIZE      64
#define STADIA_INPUT_LEN    11
#define STADIA_BATTERY_LEN   2
#define STADIA_OUTPUT_LEN    5
#define STADIA_REPORT_INPUT  0x03
#define STADIA_REPORT_RUMBLE 0x05
#define STADIA_REPORT_BATTERY 0x06

#define HID_DESC_TYPE_HID       0x21
#define HID_DESC_TYPE_REPORT    0x22
#define HID_REQ_GET_REPORT      0x01
#define HID_REQ_GET_IDLE        0x02
#define HID_REQ_GET_PROTOCOL    0x03
#define HID_REQ_SET_REPORT      0x09
#define HID_REQ_SET_IDLE        0x0A
#define HID_REQ_SET_PROTOCOL    0x0B
#define HID_REPORT_TYPE_OUTPUT  0x02

static uint8_t s_in_buf[STADIA_EP_SIZE] TU_ATTR_ALIGNED(4);
static uint8_t s_out_buf[STADIA_EP_SIZE] TU_ATTR_ALIGNED(4);
static uint8_t s_ctrl_buf[STADIA_OUTPUT_LEN] TU_ATTR_ALIGNED(4);
static uint16_t s_ctrl_len = 0;
static volatile bool s_rhport_ready = false;
static uint8_t s_rhport = 0;
static uint8_t s_idle_rate = 0;
static uint8_t s_protocol = 1;

/*
 * HID report descriptor: real Stadia Controller gamepad (182 bytes)
 * plus battery level collection (20 bytes) = 202 bytes total.
 */
const uint8_t stadia_hid_report_desc[] = {
    /* ========== Gamepad (182 bytes, exact real Stadia bytes) ========== */
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x25, 0x07,        //   Logical Maximum (7)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit
    0x09, 0x39,        //   Usage (Hat switch)
    0x81, 0x42,        //   Input (Data,Var,Abs,Null)
    0x45, 0x00,        //   Physical Maximum (0)
    0x65, 0x00,        //   Unit (None)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x01,        //   Input (Const)
    0x05, 0x09,        //   Usage Page (Button)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0F,        //   Report Count (15)
    0x09, 0x12,        //   Usage (0x12)
    0x09, 0x11,        //   Usage (0x11)
    0x09, 0x14,        //   Usage (0x14)
    0x09, 0x13,        //   Usage (0x13)
    0x09, 0x0D,        //   Usage (0x0D)
    0x09, 0x0C,        //   Usage (0x0C)
    0x09, 0x0B,        //   Usage (0x0B)
    0x09, 0x0F,        //   Usage (0x0F)
    0x09, 0x0E,        //   Usage (0x0E)
    0x09, 0x08,        //   Usage (0x08)
    0x09, 0x07,        //   Usage (0x07)
    0x09, 0x05,        //   Usage (0x05)
    0x09, 0x04,        //   Usage (0x04)
    0x09, 0x02,        //   Usage (0x02)
    0x09, 0x01,        //   Usage (0x01)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Const)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x15, 0x01,        //   Logical Minimum (1)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0xC0,              //   End Collection
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x32,        //     Usage (Z)
    0x09, 0x35,        //     Usage (Rz)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0xC0,              //   End Collection
    0x05, 0x02,        //   Usage Page (Sim Ctrls)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x09, 0xC5,        //   Usage (Brake)
    0x09, 0xC4,        //   Usage (Accelerator)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x05, 0x0C,        //   Usage Page (Consumer)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x09, 0xCD,        //   Usage (Play/Pause)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x05,        //   Report Count (5)
    0x81, 0x01,        //   Input (Const)
    0x85, 0x05,        //   Report ID (5)
    0x06, 0x0F, 0x00,  //   Usage Page (PID Page)
    0x09, 0x97,        //   Usage (0x97)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x02,        //   Report Count (2)
    0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65535)
    0x91, 0x02,        //   Output (Data,Var,Abs)
    0xC0,              // End Collection

    /* ========== Battery level (20 bytes — separate Application Collection) ========== */
    0x05, 0x06,        // Usage Page (Generic Device Controls)
    0x09, 0x20,        // Usage (Battery Strength)  ← collection purpose
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x06,        //   Report ID (6)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x09, 0x20,        //   Usage (Battery Strength)  ← for the data item
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0xC0,              // End Collection
};

_Static_assert(sizeof(stadia_hid_report_desc) == STADIA_HID_REPORT_DESC_LEN,
               "Stadia HID report descriptor length changed");

static const uint8_t s_hid_desc[] = {
    0x09, HID_DESC_TYPE_HID,
    0x11, 0x01,       // bcdHID 1.11
    0x00,             // bCountryCode
    0x01,             // bNumDescriptors
    HID_DESC_TYPE_REPORT,
    (uint8_t)(sizeof(stadia_hid_report_desc) & 0xFF),
    (uint8_t)(sizeof(stadia_hid_report_desc) >> 8),
};

static void queue_rumble_payload(const uint8_t *payload)
{
    uint8_t rumble[4] = { payload[0], payload[1], payload[2], payload[3] };
    if (xQueueSendToBack(usb_to_ble_queue, rumble, 0) != pdTRUE) {
        uint8_t dummy[4];
        xQueueReceive(usb_to_ble_queue, dummy, 0);
        xQueueSendToBack(usb_to_ble_queue, rumble, 0);
    }
}

static void stadia_init(void)
{
    s_rhport_ready = false;
}

static bool stadia_deinit(void)
{
    s_rhport_ready = false;
    return true;
}

static void stadia_reset(uint8_t rhport)
{
    (void)rhport;
    s_rhport_ready = false;
}

static uint16_t stadia_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                            uint16_t max_len)
{
    if (itf_desc->bInterfaceClass != TUSB_CLASS_HID ||
        itf_desc->bInterfaceNumber != STADIA_HID_ITF) {
        return 0;
    }

    uint16_t const drv_len = (uint16_t)(sizeof(tusb_desc_interface_t)
                             + sizeof(s_hid_desc)
                             + itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    TU_VERIFY(max_len >= drv_len, 0);

    s_rhport = rhport;
    uint8_t const *p_desc = tu_desc_next(itf_desc);
    uint8_t found = 0;

    while (found < itf_desc->bNumEndpoints && p_desc < ((uint8_t const *)itf_desc + max_len)) {
        if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
            tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p_desc;
            TU_ASSERT(usbd_edpt_open(rhport, ep), 0);
            ESP_LOGI(TAG, "Opened EP 0x%02x", ep->bEndpointAddress);

            if (ep->bEndpointAddress == STADIA_EP_OUT) {
                if (usbd_edpt_claim(rhport, STADIA_EP_OUT)) {
                    if (!usbd_edpt_xfer(rhport, STADIA_EP_OUT, s_out_buf, STADIA_EP_SIZE)) {
                        usbd_edpt_release(rhport, STADIA_EP_OUT);
                    }
                }
            }
            found++;
        }
        p_desc = tu_desc_next(p_desc);
    }

    s_rhport_ready = true;
    ESP_LOGI(TAG, "Opened HID interface (%u bytes claimed)", drv_len);
    return drv_len;
}

static bool stadia_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                   tusb_control_request_t const *request)
{
    if (stage == CONTROL_STAGE_ACK &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
        request->bRequest == HID_REQ_SET_REPORT &&
        (uint8_t)(request->wValue >> 8) == HID_REPORT_TYPE_OUTPUT) {
        if (s_ctrl_len >= STADIA_OUTPUT_LEN && s_ctrl_buf[0] == STADIA_REPORT_RUMBLE) {
            queue_rumble_payload(&s_ctrl_buf[1]);
        }
        return true;
    }

    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE ||
        request->wIndex != STADIA_HID_ITF) {
        return false;
    }

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bRequest == TUSB_REQ_GET_DESCRIPTOR) {
        uint8_t desc_type = (uint8_t)(request->wValue >> 8);
        if (desc_type == HID_DESC_TYPE_HID) {
            uint16_t len = request->wLength < sizeof(s_hid_desc)
                         ? request->wLength : sizeof(s_hid_desc);
            return tud_control_xfer(rhport, request, (void *)s_hid_desc, len);
        }
        if (desc_type == HID_DESC_TYPE_REPORT) {
            uint16_t len = request->wLength < sizeof(stadia_hid_report_desc)
                         ? request->wLength : sizeof(stadia_hid_report_desc);
            return tud_control_xfer(rhport, request, (void *)stadia_hid_report_desc, len);
        }
    }

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) {
        return false;
    }

    switch (request->bRequest) {
    case HID_REQ_GET_REPORT: {
        uint8_t report_id = (uint8_t)request->wValue;
        if (report_id == STADIA_REPORT_BATTERY) {
            static uint8_t no_battery[STADIA_BATTERY_LEN] = { STADIA_REPORT_BATTERY, 0xFF };
            uint16_t len = request->wLength < sizeof(no_battery)
                         ? request->wLength : sizeof(no_battery);
            return tud_control_xfer(rhport, request, no_battery, len);
        }
        static uint8_t neutral[STADIA_INPUT_LEN] = {
            STADIA_REPORT_INPUT, 0x08, 0x00, 0x00, 0x80, 0x80,
            0x80, 0x80, 0x00, 0x00, 0x00
        };
        uint16_t len = request->wLength < sizeof(neutral)
                     ? request->wLength : sizeof(neutral);
        return tud_control_xfer(rhport, request, neutral, len);
    }
    case HID_REQ_SET_REPORT: {
        s_ctrl_len = request->wLength < sizeof(s_ctrl_buf)
                   ? request->wLength : sizeof(s_ctrl_buf);
        return tud_control_xfer(rhport, request, s_ctrl_buf, s_ctrl_len);
    }
    case HID_REQ_GET_IDLE:
        return tud_control_xfer(rhport, request, &s_idle_rate, 1);
    case HID_REQ_SET_IDLE:
        s_idle_rate = (uint8_t)(request->wValue >> 8);
        return tud_control_status(rhport, request);
    case HID_REQ_GET_PROTOCOL:
        return tud_control_xfer(rhport, request, &s_protocol, 1);
    case HID_REQ_SET_PROTOCOL:
        s_protocol = (uint8_t)request->wValue;
        return tud_control_status(rhport, request);
    default:
        return false;
    }
}

static bool stadia_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                           xfer_result_t result, uint32_t xferred_bytes)
{
    (void)result;

    if (ep_addr == STADIA_EP_OUT) {
        if (xferred_bytes >= STADIA_OUTPUT_LEN && s_out_buf[0] == STADIA_REPORT_RUMBLE) {
            queue_rumble_payload(&s_out_buf[1]);
        } else if (xferred_bytes == 4) {
            queue_rumble_payload(s_out_buf);
        }

        if (usbd_edpt_claim(rhport, STADIA_EP_OUT)) {
            if (!usbd_edpt_xfer(rhport, STADIA_EP_OUT, s_out_buf, STADIA_EP_SIZE)) {
                usbd_edpt_release(rhport, STADIA_EP_OUT);
            }
        }
    }
    return true;
}

static usbd_class_driver_t const s_stadia_driver = {
    .name            = "STADIA",
    .init            = stadia_init,
    .deinit          = stadia_deinit,
    .reset           = stadia_reset,
    .open            = stadia_open,
    .control_xfer_cb = stadia_control_xfer_cb,
    .xfer_cb         = stadia_xfer_cb,
    .xfer_isr        = NULL,
    .sof             = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &s_stadia_driver;
}

int stadia_usb_send_report(const uint8_t *report)
{
    if (!tud_connected() || !s_rhport_ready) return -1;
    if (usbd_edpt_busy(s_rhport, STADIA_EP_IN)) return 0;

    memcpy(s_in_buf, report, STADIA_INPUT_LEN);
    if (usbd_edpt_claim(s_rhport, STADIA_EP_IN)) {
        bool ok = usbd_edpt_xfer(s_rhport, STADIA_EP_IN, s_in_buf, STADIA_INPUT_LEN);
        if (!ok) {
            usbd_edpt_release(s_rhport, STADIA_EP_IN);
        }
        return ok ? 1 : 0;
    }
    return 0;
}

int stadia_usb_send_battery_status(const uint8_t *status)
{
    /* status[3] = battery percentage (0xFF = unavailable) */
    if (!tud_connected() || !s_rhport_ready) return -1;
    if (usbd_edpt_busy(s_rhport, STADIA_EP_IN)) return 0;

    uint8_t report[STADIA_BATTERY_LEN] = { STADIA_REPORT_BATTERY, status[3] };
    memcpy(s_in_buf, report, STADIA_BATTERY_LEN);
    if (usbd_edpt_claim(s_rhport, STADIA_EP_IN)) {
        bool ok = usbd_edpt_xfer(s_rhport, STADIA_EP_IN, s_in_buf, STADIA_BATTERY_LEN);
        if (!ok) {
            usbd_edpt_release(s_rhport, STADIA_EP_IN);
        }
        return ok ? 1 : 0;
    }
    return 0;
}