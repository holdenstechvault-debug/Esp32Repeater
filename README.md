# Holden LoRa Repeater Web Flasher

This project builds browser-flashable firmware for a **Seeed Studio XIAO ESP32-C3 + DX-LR20 (LLCC68)** LoRa repeater.

## Important architecture correction

The original plan used a CC1101 as the repeater receiver. That will not work for packets sent by a DX-LR20 or Heltec V3 in LoRa mode: CC1101 does not demodulate LoRa. This version instead uses the DX-LR20 itself as a half-duplex LoRa receiver/transmitter. One XIAO C3 + one DX-LR20 is enough for the repeater.

## Security model

Frames are accepted only when all of these are true:

- magic/version match
- network ID matches
- HMAC-SHA256 (truncated to 128 bits) validates with the 256-bit network key
- sender counter is newer than the last accepted counter
- TTL is non-zero

The repeater decrements TTL, recomputes the HMAC, and retransmits. Old counters are stored in ESP32 NVS to reject replayed packets across reboots.

This is intentionally not a home-made cipher. HMAC-SHA256 provides authentication; the rolling counter provides replay protection. Payload encryption can be added later at the endpoints if desired.

## Default XIAO C3 -> DX-LR20 logical wiring

The setup page lets you change every pin. Defaults are:

| DX-LR20 signal | XIAO pin | ESP32-C3 GPIO |
|---|---|---:|
| SCK | D8 | 8 |
| MISO | D9 | 9 |
| MOSI | D10 | 10 |
| NSS/CS | D3 | 5 |
| DIO1 | D2 | 4 |
| NRST | D1 | 3 |
| BUSY | D6 | 21 |
| RXEN | D4 | 6 |
| TXEN | D5 | 7 |
| VCC | 3V3 | - |
| GND | GND | - |

**Do not assume the physical order of the header from this table. Match the silk-screened signal names on your DX-LR20 adapter.**

Connect the antenna before powering/transmitting. The DX-LR20 is a 3.3 V logic/power device.

## Get the web flasher online

1. Create a new GitHub repository.
2. Upload this project's contents to the repository root.
3. In GitHub, open **Settings -> Pages** and set **Source** to **GitHub Actions**.
4. Push to `main` or manually run the `Build firmware and publish web flasher` workflow.
5. Open the Pages URL in desktop Chrome or Edge.
6. Plug in the XIAO C3 and press **Install repeater firmware**.

ESP Web Tools requires HTTPS (GitHub Pages supplies this) or localhost because Web Serial is a secure-context browser API.

## First boot

The repeater starts a setup access point:

- SSID: `HOLDEN-LORA-xxxxxx`
- password: `repeater-setup`
- setup page: `http://192.168.4.1`

The firmware generates a random 256-bit network key on first boot. Copy that key into the home endpoint and girlfriend endpoint firmware/config. All three radios must also use matching LoRa PHY settings.

## Default LoRa PHY

- 915.000 MHz
- 125 kHz bandwidth
- SF9
- coding rate 4/7
- sync word 0x12
- TX power +22 dBm
- preamble 12 symbols

Change these only in a way that is legal for your region and compatible with both the DX-LR20 and Heltec V3.

## Packet format v1

All integers are big-endian.

| Offset | Length | Field |
|---:|---:|---|
| 0 | 2 | magic `HL` |
| 2 | 1 | version = 1 |
| 3 | 2 | network ID |
| 5 | 4 | sender ID |
| 9 | 8 | monotonically increasing counter |
| 17 | 1 | TTL |
| 18 | 1 | payload length |
| 19 | N | payload |
| 19+N | 16 | HMAC-SHA256 tag, truncated to 16 bytes |

The HMAC covers everything from offset 0 through the end of the payload.

## What still needs endpoint firmware

This package is the grandparents' repeater only. The home DX-LR20 endpoint and girlfriend's Heltec V3 still need matching sender/receiver firmware that creates/verifies this packet format and maintains sender counters.
