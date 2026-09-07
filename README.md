# Holden RF Repeater + Messenger

This repository contains the current firmware and browser flasher for the Holden repeater and the two messenger endpoints.

## Current repeater architecture

The repeater uses two ESP32 boards and two radios:

`endpoint FSK TX -> CC1101 -> ESP32-S2 Feather -> UART -> XIAO ESP32-C3 -> DX-LR20 LoRa TX`

The reverse user experience is handled by the endpoint messenger software: after an endpoint sends its authenticated FSK uplink, it switches back to LoRa receive mode and waits for the repeater's authenticated LoRa retransmission.

### ESP32-S2 Feather role

The S2 owns the **CC1101 receiver**.

Confirmed CC1101 wiring:

| CC1101 | ESP32-S2 Feather |
|---|---|
| GDO0 | GPIO5 |
| CSN | GPIO10 |
| SCK | SCK |
| MOSI | MO |
| MISO | MI |
| GDO2 | GPIO6 |

The S2 uses its pins labeled **RX** and **TX** for the UART link to the C3.

### XIAO ESP32-C3 role

The C3 owns the **DX-LR20 / LLCC68** and transmits the repeated LoRa packet.

Confirmed DX-LR20 wiring:

| DX-LR20 | XIAO ESP32-C3 |
|---|---|
| NSS / CS | D0 |
| RESET | D1 |
| DIO1 | D2 |
| BUSY | D3 |
| TXEN | D4 |
| RXEN | D5 |
| SCK | D8 |
| MISO | D9 |
| MOSI | D10 |

UART:

| C3 | S2 |
|---|---|
| D6 TX | RX |
| D7 RX | TX |
| GND | GND |

A common ground is required.

## Self-receive protection

The repeater prevents a feedback loop in two layers:

1. As soon as the S2 receives a CC1101 packet, it puts the CC1101 into standby.
2. The C3 sends `TX_BEGIN` before the DX-LR20 transmits and `TX_END` afterward. The S2 waits a short guard time before returning the CC1101 to RX.

The C3 and S2 also exchange heartbeats. Their setup pages show `CONNECTED` or `DISCONNECTED` for the UART link.

## Repeated-only receiving endpoints

The messenger endpoints intentionally display **only the repeater output**, not the original direct uplink.

A received message is added to the UI only after the endpoint accepts a valid repeated frame with:

- outer magic `HL`
- correct outer version
- matching Network ID
- repeat marker
- valid HMAC-SHA256 tag
- newer monotonic repeat counter
- valid authenticated inner `HM` message

Pressing **Send** does not immediately create a local chat message. The endpoint transmits the authenticated FSK uplink, returns to LoRa RX, and the message appears only after a valid repeated LoRa frame comes back from the repeater.

## Firmware source

Current source files:

- `src/repeater_v3.cpp` — repeater C3 + S2 firmware
- `src/endpoint_messenger.cpp` — Heltec V3 + home DX-LR20 messenger firmware

PlatformIO targets are defined in `platformio.ini`:

- `seeed_xiao_esp32c3` — repeater C3
- `adafruit_feather_esp32s2` — repeater S2
- `heltec_wifi_lora_32_V3` — Heltec V3 messenger
- `home_xiao_esp32c3` — home DX-LR20 messenger

The S2 PlatformIO target enables native USB CDC on boot.

## Arduino IDE

Arduino IDE instructions are in:

- `docs/ARDUINO_IDE.md`
- `docs/S2_BOOT_MODE.md`

For the ESP32-S2 Feather, the documented recovery sequence is:

**hold BOOT/DFU -> tap RESET -> release BOOT/DFU**, select the new bootloader port, upload, then press RESET once for normal operation.

Nothing in the current CC1101/UART wiring uses GPIO0, so the BOOT strap is left free.

## Setup Wi-Fi

Repeater C3:

- SSID: `HOLDEN-LORA-C3-xxxxxx`
- password: `repeater-setup`
- page: `http://192.168.4.1`

Repeater S2:

- SSID: `HOLDEN-LORA-S2-xxxxxx`
- password: `repeater-setup`
- page: `http://192.168.4.1`

The S2 setup AP starts before Preferences, UART, or CC1101 initialization so radio/wiring failures should not suppress recovery Wi-Fi.

Messenger endpoints:

- Heltec SSID: `HOLDEN-MSG-HELTEC-xxxxxx`
- Home SSID: `HOLDEN-MSG-HOME-xxxxxx`
- password: `holden-messenger`
- page: `http://192.168.4.1`

## Default radio settings

Repeated LoRa output:

- 915.000 MHz
- 125 kHz bandwidth
- SF9
- coding rate 4/7
- sync word `0x12`
- repeater TX power +22 dBm
- preamble 12 symbols

FSK uplink between the messenger endpoint and CC1101:

- 915.000 MHz
- 4.8 kbps
- 5.0 kHz deviation
- CC1101 receive bandwidth approximately 58 kHz
- sync word `12 AD`

The endpoint and repeater settings must match.

## Security

The system uses HMAC-SHA256 authentication with a 256-bit shared key, truncated tags, monotonic counters, and replay rejection. The repeated LoRa frame contains a separate authenticated outer wrapper around the endpoint's authenticated inner message.

Copy the C3 repeater's shared key and Network ID into both messenger endpoints.

## Web flasher

The GitHub Actions workflow builds the firmware and publishes the browser flasher to GitHub Pages:

https://holdenstechvault-debug.github.io/Esp32Repeater/

Use a desktop browser with Web Serial support such as Chrome or Edge.

Connect an antenna appropriate for the configured band before transmitting, and use radio settings that comply with the rules for your location.
