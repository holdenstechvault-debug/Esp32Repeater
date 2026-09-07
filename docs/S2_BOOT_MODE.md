# Adafruit Feather ESP32-S2 boot and recovery

The ESP32-S2 bootloader mode is selected by hardware at reset. The running repeater sketch cannot force ROM bootloader mode before startup.

## Normal repeater boot

For normal use, do **not** hold BOOT/DFU. Power the board or press **RESET** once.

Expected normal behavior:

- the firmware starts setup Wi-Fi before Preferences, UART, or CC1101 initialization
- SSID: `HOLDEN-LORA-S2-xxxxxx`
- password: `repeater-setup`
- setup page: `http://192.168.4.1`
- serial monitor: 115200 baud

## Force ROM bootloader mode

Use this for the first Arduino IDE upload or recovery if the normal application does not enumerate correctly.

1. Connect the Feather with a USB **data** cable.
2. In Arduino IDE select **Adafruit Feather ESP32-S2**.
3. Set **USB CDC On Boot = Enabled**.
4. Set **Upload Mode = Internal USB**.
5. Hold **BOOT/DFU**.
6. While still holding BOOT/DFU, press and release **RESET**.
7. Release BOOT/DFU.
8. Re-open **Tools > Port** and select the newly appearing ESP32-S2 bootloader port.
9. Upload the S2 firmware.
10. When the upload finishes, press **RESET** once for normal operation.

The PlatformIO S2 target also enables native USB CDC on boot with:

```ini
-DARDUINO_USB_CDC_ON_BOOT=1
-DARDUINO_USB_MODE=0
```

## Boot-safe wiring

GPIO0 is the ESP32-S2 BOOT strap. Nothing in this repeater design is connected to GPIO0.

### CC1101 to ESP32-S2 Feather

| CC1101 | Feather |
|---|---|
| GDO0 | GPIO5 |
| CSN | GPIO10 |
| SCK | SCK |
| MOSI | MO |
| MISO | MI |
| GDO2 | GPIO6 |

### UART to XIAO ESP32-C3

| C3 | S2 Feather |
|---|---|
| D6 TX | RX |
| D7 RX | TX |
| GND | GND |

A common ground is required.

## If the S2 Wi-Fi AP still does not appear

1. Disconnect the CC1101 and the C3 UART wires from the S2.
2. Leave only USB connected.
3. Press RESET once and scan for `HOLDEN-LORA-S2-xxxxxx`.
4. If it is still missing, enter ROM bootloader mode with BOOT/DFU + RESET and re-upload.
5. Open Serial Monitor at 115200 and check the boot output.

Because setup Wi-Fi starts before radio initialization, a bad CC1101 connection should not suppress the setup AP in the current firmware.
