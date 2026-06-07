/*
 * xbox_dev.c — Custom TinyUSB class driver for XInputHID controller endpoints.
 *
 * Emulates an Xbox Series X|S controller via XInputHID (standard HID over USB).
 * Windows loads xinputhid.sys when the device matches VID 045E and the HID
 * report descriptor declares the expected Xbox collections.
 *
 * HID report layout:
 *   Input (no Report ID): 18 bytes — 4×16-bit sticks, 2×10-bit triggers,
 *                          16 buttons, 4-bit hat, 1-bit Record (Share),
 *                          7-bit padding
 *   Output (Report ID 0x03): rumble + force-feedback parameters
 */

#include "xbox_dev.h"
#include "bridge.h"

#include "esp_log.h"
#include "tinyusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include <string.h>

static const char *TAG = "XBOX_DEV";

#define XBOX_HID_ITF      0x00
#define XBOX_EP_IN        0x81
#define XBOX_EP_OUT       0x01
#define XBOX_EP_SIZE      64
#define XBOX_INPUT_LEN    18
#define XBOX_OUTPUT_LEN    8
#define XBOX_REPORT_RUMBLE 0x03

#define HID_DESC_TYPE_HID       0x21
#define HID_DESC_TYPE_REPORT    0x22
#define HID_REQ_GET_REPORT      0x01
#define HID_REQ_GET_IDLE        0x02
#define HID_REQ_GET_PROTOCOL    0x03
#define HID_REQ_SET_REPORT      0x09
#define HID_REQ_SET_IDLE        0x0A
#define HID_REQ_SET_PROTOCOL    0x0B
#define HID_REPORT_TYPE_OUTPUT  0x02

static uint8_t s_in_buf[XBOX_EP_SIZE]    TU_ATTR_ALIGNED(4);
static uint8_t s_out_buf[XBOX_EP_SIZE]   TU_ATTR_ALIGNED(4);
static uint8_t s_ctrl_buf[16]            TU_ATTR_ALIGNED(4);
static uint16_t s_ctrl_len = 0;
static volatile bool s_rhport_ready = false;
static uint8_t s_rhport = 0;
static uint8_t s_idle_rate = 0;
static uint8_t s_protocol = 1;

/*
 * XInputHID report descriptor (252 bytes).
 * USB variant for VID 045E PID 02FF — xinputhid.sys loads this natively.
 * Source: DsHidMini docs (nefarius/DsHidMini/blob/master/docs/XINPUTHID.md)
 */
const uint8_t xbox_hid_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x00,        //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65534)
    0x95, 0x02,        //     Report Count (2)
    0x75, 0x10,        //     Report Size (16)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0xC0,              //   End Collection
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x33,        //     Usage (Rx)
    0x09, 0x34,        //     Usage (Ry)
    0x15, 0x00,        //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //     Logical Maximum (65534)
    0x95, 0x02,        //     Report Count (2)
    0x75, 0x10,        //     Report Size (16)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0xC0,              //   End Collection
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x32,        //   Usage (Z)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x0A,        //   Report Size (10)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x00,        //   Logical Maximum (0)
    0x75, 0x06,        //   Report Size (6)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Const)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x35,        //   Usage (Rz)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x0A,        //   Report Size (10)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x00,        //   Logical Maximum (0)
    0x75, 0x06,        //   Report Size (6)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Const)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (0x01)
    0x29, 0x10,        //   Usage Maximum (0x10)
    0x95, 0x10,        //   Report Count (16)
    0x75, 0x01,        //   Report Size (1)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x01,        //   Logical Minimum (1)
    0x25, 0x08,        //   Logical Maximum (8)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x66, 0x14, 0x00,  //   Unit
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (Data,Var,Abs,Null)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x00,        //   Logical Maximum (0)
    0x35, 0x00,        //   Physical Minimum (0)
    0x45, 0x00,        //   Physical Maximum (0)
    0x65, 0x00,        //   Unit (None)
    0x81, 0x03,        //   Input (Const)
    0xA1, 0x02,        //   Collection (Logical)
    0x05, 0x0F,        //     Usage Page (PID Page)
    0x09, 0x97,        //     Usage (0x97)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x04,        //     Report Size (4)
    0x95, 0x01,        //     Report Count (1)
    0x91, 0x02,        //     Output (Data,Var,Abs,Non-volatile)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x00,        //     Logical Maximum (0)
    0x91, 0x03,        //     Output (Const)
    0x09, 0x70,        //     Usage (0x70)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x64,        //     Logical Maximum (100)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x04,        //     Report Count (4)
    0x91, 0x02,        //     Output (Data,Var,Abs,Non-volatile)
    0x09, 0x50,        //     Usage (0x50)
    0x66, 0x01, 0x10,  //     Unit
    0x55, 0x0E,        //     Unit Exponent
    0x26, 0xFF, 0x00,  //     Logical Maximum (255)
    0x95, 0x01,        //     Report Count (1)
    0x91, 0x02,        //     Output (Data,Var,Abs,Non-volatile)
    0x09, 0xA7,        //     Usage (0xA7)
    0x91, 0x02,        //     Output (Data,Var,Abs,Non-volatile)
    0x65, 0x00,        //     Unit (None)
    0x55, 0x00,        //     Unit Exponent (0)
    0x09, 0x7C,        //     Usage (0x7C)
    0x91, 0x02,        //     Output (Data,Var,Abs,Non-volatile)
    0xC0,              //   End Collection
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x80,        //   Usage (Sys Control)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x85,        //     Usage (Sys Main Menu)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x00,        //     Logical Maximum (0)
    0x75, 0x07,        //     Report Size (7)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x03,        //     Input (Const)
    0xC0,              //   End Collection
    0x05, 0x06,        //   Usage Page (Generic Dev Ctrls)
    0x09, 0x20,        //   Usage (Battery Strength)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0xC0,              // End Collection
};

_Static_assert(sizeof(xbox_hid_report_desc) == XBOX_HID_REPORT_DESC_LEN,
               "XInputHID report descriptor length changed");

static const uint8_t s_hid_desc[] = {
    0x09, HID_DESC_TYPE_HID,
    0x11, 0x01,       // bcdHID 1.11
    0x00,             // bCountryCode
    0x01,             // bNumDescriptors
    HID_DESC_TYPE_REPORT,
    (uint8_t)(sizeof(xbox_hid_report_desc) & 0xFF),
    (uint8_t)(sizeof(xbox_hid_report_desc) >> 8),
};

static void queue_rumble_payload(const uint8_t *payload)
{
    // XInputHID rumble: Report ID 0x03, followed by up to 8 bytes
    // Bytes 1-4: actuator levels (0-100), byte 5: duration, byte 6: loop count, byte 7: misc
    // Map to Stadia format: 2 motors × 8-bit magnitude
    if (payload[0] != XBOX_REPORT_RUMBLE) return;
    uint8_t rumble[4] = {
        0x00,
        payload[1],  // left large motor
        0x00,
        payload[2],  // right small motor (or index 3 for quad-motor)
    };
    if (xQueueSendToBack(usb_to_ble_queue, rumble, 0) != pdTRUE) {
        uint8_t dummy[4];
        xQueueReceive(usb_to_ble_queue, dummy, 0);
        xQueueSendToBack(usb_to_ble_queue, rumble, 0);
    }
}

static void xbox_init(void) {
    s_rhport_ready = false;
}

static bool xbox_deinit(void) {
    s_rhport_ready = false;
    return true;
}

static void xbox_reset(uint8_t rhport) {
    (void)rhport;
    s_rhport_ready = false;
}

static uint16_t xbox_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc,
                           uint16_t max_len)
{
    if (itf_desc->bInterfaceClass != TUSB_CLASS_HID ||
        itf_desc->bInterfaceNumber != XBOX_HID_ITF) {
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

            if (ep->bEndpointAddress == XBOX_EP_OUT) {
                if (usbd_edpt_claim(rhport, XBOX_EP_OUT)) {
                    if (!usbd_edpt_xfer(rhport, XBOX_EP_OUT, s_out_buf, XBOX_EP_SIZE)) {
                        usbd_edpt_release(rhport, XBOX_EP_OUT);
                    }
                }
            }
            found++;
        }
        p_desc = tu_desc_next(p_desc);
    }

    s_rhport_ready = true;
    ESP_LOGI(TAG, "Opened XInputHID interface (%u bytes claimed)", drv_len);
    return drv_len;
}

static bool xbox_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                  tusb_control_request_t const *request)
{
    if (stage == CONTROL_STAGE_ACK &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
        request->bRequest == HID_REQ_SET_REPORT &&
        (uint8_t)(request->wValue >> 8) == HID_REPORT_TYPE_OUTPUT) {
        if (s_ctrl_len >= 4 && s_ctrl_buf[0] == XBOX_REPORT_RUMBLE) {
            queue_rumble_payload(s_ctrl_buf);
        }
        return true;
    }

    if (stage != CONTROL_STAGE_SETUP) return true;

    if (request->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE ||
        request->wIndex != XBOX_HID_ITF) {
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
            uint16_t len = request->wLength < sizeof(xbox_hid_report_desc)
                         ? request->wLength : sizeof(xbox_hid_report_desc);
            return tud_control_xfer(rhport, request, (void *)xbox_hid_report_desc, len);
        }
    }

    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS) {
        return false;
    }

    switch (request->bRequest) {
    case HID_REQ_GET_REPORT: {
        static uint8_t neutral[XBOX_INPUT_LEN] = {
            0xFF, 0x7F, 0xFF, 0x7F, // X=32767, Y=32767
            0xFF, 0x7F, 0xFF, 0x7F, // Rx=32767, Ry=32767
            0, 0, 0, 0,              // triggers=0, buttons=0
            0, 0,                     // hat=0, Record=0, padding
            0, 0xFF                   // Guide, Battery=unknown
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

static bool xbox_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                          xfer_result_t result, uint32_t xferred_bytes)
{
    (void)result;

    if (ep_addr == XBOX_EP_OUT) {
        if (xferred_bytes >= 4 && s_out_buf[0] == XBOX_REPORT_RUMBLE) {
            queue_rumble_payload(s_out_buf);
        }
        if (usbd_edpt_claim(rhport, XBOX_EP_OUT)) {
            if (!usbd_edpt_xfer(rhport, XBOX_EP_OUT, s_out_buf, XBOX_EP_SIZE)) {
                usbd_edpt_release(rhport, XBOX_EP_OUT);
            }
        }
    }
    return true;
}

static usbd_class_driver_t const s_xbox_driver = {
    .name             = "XINPUT_HID",
    .init             = xbox_init,
    .deinit           = xbox_deinit,
    .reset            = xbox_reset,
    .open             = xbox_open,
    .control_xfer_cb  = xbox_control_xfer_cb,
    .xfer_cb          = xbox_xfer_cb,
    .xfer_isr         = NULL,
    .sof              = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &s_xbox_driver;
}

int xbox_send_report(const uint8_t *report) {
    if (!tud_connected() || !s_rhport_ready) return -1;
    if (usbd_edpt_busy(s_rhport, XBOX_EP_IN)) return 0;

    memcpy(s_in_buf, report, XBOX_INPUT_LEN);
    if (usbd_edpt_claim(s_rhport, XBOX_EP_IN)) {
        bool ok = usbd_edpt_xfer(s_rhport, XBOX_EP_IN, s_in_buf, XBOX_INPUT_LEN);
        if (!ok) {
            usbd_edpt_release(s_rhport, XBOX_EP_IN);
        }
        return ok ? 1 : 0;
    }
    return 0;
}