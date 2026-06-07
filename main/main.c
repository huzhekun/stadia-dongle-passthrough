/*
 * main.c - app_main for the Stadia BT dongle.
 *
 * Task layout:
 *   Core 0, pri 5 - nimble_host_task    (NimBLE host stack)
 *   Core 0, pri 3 - ble_rumble_task     (drain usb_to_ble -> BLE rumble)
 *   Core 1, pri 4 - tinyusb device task (created by tinyusb_driver_install)
 *   Core 1, pri 4 - usb_stadia_task / usb_xbox_task (drain ble_to_usb -> USB)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "bridge.h"
#include "ble_central.h"

#if STADIA_EMULATE_XINPUT
#include "usb_xbox.h"
#else
#include "usb_stadia.h"
#endif

static const char *TAG = "MAIN";

/* ---- Rumble relay task (Core 0, same core as NimBLE) -------------------- */

static void ble_rumble_task(void *arg)
{
    uint8_t payload[4];
    while (1) {
        if (xQueueReceive(usb_to_ble_queue, payload, portMAX_DELAY) == pdTRUE) {
            ble_central_send_rumble(payload);
        }
    }
}

#define NVS_RESET_GPIO GPIO_NUM_0   // BOOT button on ESP32-S3 devkits

/* ---- Entry point --------------------------------------------------------- */

void app_main(void)
{
    // Check for NVS reset request: hold BOOT (GPIO0) low at power-on
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(NVS_RESET_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    vTaskDelay(pdMS_TO_TICKS(10));

    if (gpio_get_level(NVS_RESET_GPIO) == 0) {
        ESP_LOGW("MAIN", "BOOT button held — erasing NVS (all BLE bonds cleared)");
        nvs_flash_erase();
        // Wait for button release to avoid re-triggering
        while (gpio_get_level(NVS_RESET_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        ESP_LOGI("MAIN", "NVS erased, continuing boot");
    }

    // NVS is required for BLE bonding persistence
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    bridge_init();

    // USB: install TinyUSB driver (also starts the USB device task on Core 1)
#if STADIA_EMULATE_XINPUT
    usb_xbox_init();
#else
    usb_stadia_init();
#endif

    // Initialise NimBLE port before ble_central_init(), so the default event
    // queue exists before we attach BLE callouts to it.
    nimble_port_init();

    ble_central_init();

#if STADIA_EMULATE_XINPUT
    xTaskCreatePinnedToCore(usb_xbox_task, "usb_xbox", 4096, NULL, 4, NULL, 1);
#else
    xTaskCreatePinnedToCore(usb_stadia_task, "usb_stadia", 4096, NULL, 4, NULL, 1);
#endif
    xTaskCreatePinnedToCore(ble_rumble_task, "ble_rumble", 4096, NULL, 3, NULL, 0);

    nimble_port_freertos_init(nimble_host_task);

#if STADIA_EMULATE_XINPUT
    ESP_LOGI(TAG, "Stadia BT dongle started (XInput HID emulation)");
#else
    ESP_LOGI(TAG, "Stadia BT dongle started (Stadia HID)");
#endif
}