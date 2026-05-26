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

The USB device stays detached until the Bluetooth controller reaches HID-ready
state. If the controller disconnects, the dongle soft-detaches from USB so the
host sees the Stadia Controller disappear instead of keeping a stale gamepad.

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
NVS and survives power cycles. To pair a different controller, erase NVS:

```sh
pio run -t erase --upload-port <PORT>
```

### Battery status

The dongle reads the controller's BLE Battery Service (`0x180F` / `0x2A19`) after
the HID connection is ready. Battery updates are emitted on the USB vendor
interface as a 4-byte packet:

```text
42 41 54 <percent>
```

The first three bytes are ASCII `BAT`; `<percent>` is `0..100`, or `0xFF` when
unknown/disconnected. A host can also write `42 41 54 3F` (`BAT?`) to the vendor
OUT endpoint to request the cached value.

### Debug logging

In `main/bridge.h`, set `DONGLE_DEBUG 1` to enable raw HID report hex dumps over
UART. Set it to `0` for production builds.
