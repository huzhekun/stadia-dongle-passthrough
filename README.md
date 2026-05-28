# Stadia Controller USB Passthrough Dongle

Firmware for an ESP32-S3 dongle that connects to a Google Stadia Controller over
Bluetooth LE and exposes it to the USB host as a Stadia Controller HID device.

The USB side uses Google VID/PID `18D1:9400`, input report ID `0x03`, and output
report ID `0x05`. The BLE side keeps ownership of the controller's HID-over-GATT
service, so rumble is forwarded with the write-with-response behavior the Stadia
firmware accepts.

### Why a dongle?

After Google released the Bluetooth firmware for the Stadia Controller, normal
input works on Windows but rumble does not. The debug work in
`../stadia-controller-debug` found the reason: Windows' BLE HID-over-GATT driver
rejects the Stadia output report characteristic because it advertises
`Read | Write` but not `WriteWithoutResponse`. Windows then never registers
output report `0x05` in its live HOGP report map, so HID output calls fail before
any Bluetooth write happens.

The ESP32-S3 sidesteps that Windows limitation. It owns the BLE connection,
subscribes to Stadia input report `0x03`, writes Stadia output report `0x05` with
response, and presents the controller over USB.

### USB behavior

The USB device stays detached until a Stadia controller pairs over BLE and
reaches HID-ready state. When the controller disconnects, the dongle soft-detaches
from USB so the host sees the controller disappear instead of keeping a stale
gamepad. A 5-second GATT keep-alive read detects unresponsive controllers
(e.g. powered off while charging) and triggers disconnection.

### Erasing pairing data

Hold the **BOOT** button while powering on the ESP32-S3 to erase all NVS data
(clearing BLE bonds). Release the button to continue normal boot.

### Hardware

Any ESP32-S3 board with native USB (USB OTG on GPIO19/20). The USB port connected
to the host must be the native USB port, not UART.

### Building and flashing

Requires [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html)
or the PlatformIO VS Code extension.

```sh
pio run
pio run -t upload --upload-port <PORT>
```

After flashing through the COM/UART USB port, move the board to the native USB
port for controller passthrough.

The default PlatformIO environment is `esp32s3` and uses the existing ESP-IDF
`main/` component layout. A successful build also writes a merged web-installer
image to:

```text
.pio/build/esp32s3/firmware-merged.bin
```

### Pairing

The dongle bonds to the first Stadia controller it sees. The bond is stored in
NVS and survives power cycles. To pair a different controller, either:

- Hold BOOT while powering on (wipes all NVS data), or
- Erase flash: `pio run -t erase --upload-port <PORT>`

### Battery status

The dongle reads the controller's BLE Battery Service (`0x180F` / `0x2A19`) after
the HID connection is ready. Battery level is exposed as HID Report ID `0x06` —
a 2-byte report containing the battery percentage (0–100, or 0xFF when
unknown/disconnected).

### HID Report IDs

The single HID interface exposes three report IDs:

| Report ID | Direction | Size | Description |
|-----------|-----------|------|-------------|
| `0x03` | Input | 11 bytes | Gamepad state (real Stadia HID descriptor) |
| `0x05` | Output | 5 bytes | Rumble (two 16-bit little-endian magnitudes) |
| `0x06` | Input | 2 bytes | Battery level (percentage, 0xFF = unknown) |

### Debug logging

In `main/bridge.h`, set `DONGLE_DEBUG 1` to enable raw HID report hex dumps over
UART. Set it to `0` for production builds.

### Xbox 360 Emulation

An alternative branch (`xbox-emulation`) emulates an Xbox 360 wired controller
(VID `045E:028E`) for native XInput support in games. Toggle with the
`STADIA_EMULATE_XBOX360` flag in `bridge.h`.

### USB Descriptor Dumper

`scripts/dump_stadia_descriptors.py` can dump the full USB descriptors from a
real Stadia Controller for comparison. Requires `pyusb` and libusb:

```sh
pip install pyusb
python scripts/dump_stadia_descriptors.py