#!/usr/bin/env python3
"""
Dump USB descriptors from a Google Stadia Controller (VID 18D1:PID 9400).

Requires: pyusb + libusb
  pip install pyusb
  (also install libusb: https://github.com/libusb/libusb/releases)

Usage:  python dump_stadia_descriptors.py
"""

import sys

try:
    import usb.core
    import usb.util
except ImportError:
    print("ERROR: pyusb not installed. Run: pip install pyusb")
    print("Also install libusb from https://github.com/libusb/libusb/releases")
    sys.exit(1)

VID = 0x18D1
PID = 0x9400


def get_string(dev, index):
    if index == 0:
        return "<none>"
    try:
        return usb.util.get_string(dev, index)
    except Exception:
        return f"<error reading index {index}>"


def hexdump(data, indent=4):
    prefix = " " * indent
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{prefix}{i:04X}:  {hex_str:<48s}  {ascii_str}")


def parse_config_descriptor(raw):
    """Minimal config descriptor walker."""
    off = 0
    while off < len(raw):
        length = raw[off]
        if length < 2:
            break
        desc_type = raw[off + 1]
        chunk = raw[off:off + length]

        TYPE_NAMES = {
            2: "CONFIGURATION",
            4: "INTERFACE",
            5: "ENDPOINT",
            11: "IAD (Interface Association)",
            33: "HID",
            34: "REPORT",
            15: "BOS",
            16: "DEVICE CAPABILITY",
        }
        name = TYPE_NAMES.get(desc_type, f"TYPE 0x{desc_type:02X}")
        print(f"\n  [{name}] length={length}")
        hexdump(chunk, indent=4)

        if desc_type == 33:  # HID
            wDescriptorLength = chunk[7] | (chunk[8] << 8)
            print(f"    HID Report Descriptor length: {wDescriptorLength} bytes")

        off += length


def main():
    print(f"Looking for Stadia Controller (VID={VID:04X} PID={PID:04X})...")

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("ERROR: Stadia Controller not found!")
        print("Make sure the controller is connected via USB-C cable.")
        print("Make sure libusb/WinUSB driver is installed (use Zadig if needed).")
        sys.exit(1)

    print("Found Stadia Controller!\n")

    # ========== DEVICE DESCRIPTOR ==========
    print("=" * 60)
    print("DEVICE DESCRIPTOR")
    print("=" * 60)
    raw = dev.bus.read(dev.address, 0, 18)  # read raw descriptor
    hexdump(raw)
    print()
    print(f"  bLength:          {dev.bLength}")
    print(f"  bDescriptorType:  {dev.bDescriptorType}")
    print(f"  bcdUSB:           {dev.bcdUSB:04X}")
    print(f"  bDeviceClass:     {dev.bDeviceClass}")
    print(f"  bDeviceSubClass:  {dev.bDeviceSubClass}")
    print(f"  bDeviceProtocol:  {dev.bDeviceProtocol}")
    print(f"  bMaxPacketSize0:  {dev.bMaxPacketSize0}")
    print(f"  idVendor:         {dev.idVendor:04X}")
    print(f"  idProduct:        {dev.idProduct:04X}")
    print(f"  bcdDevice:        {dev.bcdDevice:04X}")
    print(f"  iManufacturer:    {dev.iManufacturer}  ->  \"{get_string(dev, dev.iManufacturer)}\"")
    print(f"  iProduct:         {dev.iProduct}  ->  \"{get_string(dev, dev.iProduct)}\"")
    print(f"  iSerialNumber:    {dev.iSerialNumber}  ->  \"{get_string(dev, dev.iSerialNumber)}\"")
    print(f"  bNumConfigurations: {dev.bNumConfigurations}")

    # ========== CONFIGURATION DESCRIPTOR ==========
    print("\n" + "=" * 60)
    print("CONFIGURATION DESCRIPTOR")
    print("=" * 60)
    try:
        cfg = dev.get_active_configuration()
        raw_cfg = dev.bus.read(dev.address, 0, cfg.wTotalLength)
        # read full config descriptor raw
        print(f"\nTotal length: {len(raw_cfg)} bytes (0x{len(raw_cfg):04X})")
        hexdump(raw_cfg)
        print()
        parse_config_descriptor(raw_cfg)
    except Exception as e:
        print(f"ERROR reading config descriptor: {e}")

    # ========== BOS DESCRIPTOR ==========
    print("\n" + "=" * 60)
    print("BOS (Binary Object Store) DESCRIPTOR")
    print("=" * 60)
    try:
        if dev.bcdUSB >= 0x0201:
            bos_raw = dev.ctrl_transfer(
                bmRequestType=0x80,  # Device-to-Host, Standard, Device
                bRequest=6,          # GET_DESCRIPTOR
                wValue=0x0F00,       # BOS descriptor type
                wIndex=0,
                data_or_wLength=256,
                timeout=1000,
            )
            print(f"Length: {len(bos_raw)} bytes")
            hexdump(bos_raw)
            # Parse BOS
            if len(bos_raw) >= 5:
                wTotalLength = bos_raw[2] | (bos_raw[3] << 8)
                bNumDeviceCaps = bos_raw[4]
                print(f"\n  wTotalLength: {wTotalLength}")
                print(f"  bNumDeviceCaps: {bNumDeviceCaps}")
                off = 5
                for i in range(bNumDeviceCaps):
                    if off + 3 > len(bos_raw):
                        break
                    cap_len = bos_raw[off]
                    cap_type = bos_raw[off + 1]
                    cap_data = bos_raw[off:off + cap_len]
                    print(f"\n  [DEVICE CAPABILITY {i}] length={cap_len}, type=0x{cap_type:02X}")
                    hexdump(cap_data, indent=4)
                    off += cap_len
        else:
            print("Device is USB 2.0 (no BOS required) — trying anyway...")
            try:
                bos_raw = dev.ctrl_transfer(0x80, 6, 0x0F00, 0, 256)
                print(f"BOS present! Length: {len(bos_raw)} bytes")
                hexdump(bos_raw)
            except Exception:
                print("No BOS descriptor (as expected for bcdUSB 2.00)")
    except Exception as e:
        print(f"No BOS descriptor or error: {e}")

    # ========== HID REPORT DESCRIPTOR ==========
    print("\n" + "=" * 60)
    print("HID REPORT DESCRIPTOR")
    print("=" * 60)
    try:
        # Request from interface 1
        report_raw = dev.ctrl_transfer(
            bmRequestType=0x81,  # Device-to-Host, Standard, Interface
            bRequest=6,          # GET_DESCRIPTOR
            wValue=0x2200,       # Report descriptor (type 0x22, index 0x00)
            wIndex=1,            # Interface 1 (HID)
            data_or_wLength=256,
            timeout=1000,
        )
        print(f"Length: {len(report_raw)} bytes")
        print()
        hexdump(report_raw)

        # Also print as C array for easy comparison
        print(f"\n--- C array (total {len(report_raw)} bytes) ---")
        print("const uint8_t stadia_hid_report_desc[] = {")
        for i in range(0, len(report_raw), 8):
            line = ", ".join(f"0x{b:02X}" for b in report_raw[i:i + 8])
            print(f"    {line},")
        print("};")

    except Exception as e:
        print(f"ERROR reading HID report descriptor: {e}")

    # ========== STRING DESCRIPTORS ==========
    print("\n" + "=" * 60)
    print("STRING DESCRIPTORS")
    print("=" * 60)
    for idx in range(1, 6):
        try:
            s = usb.util.get_string(dev, idx)
            print(f"  Index {idx}: \"{s}\"")
        except Exception:
            print(f"  Index {idx}: <not available>")

    print("\nDone.")


if __name__ == "__main__":
    main()