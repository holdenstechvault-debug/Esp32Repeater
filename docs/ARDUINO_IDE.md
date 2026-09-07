# Arduino IDE notes

The current source files are already in this repository:

- repeater firmware: `src/repeater_v3.cpp`
- endpoint messenger firmware: `src/endpoint_messenger.cpp`

The PlatformIO build selects the board role with compile-time defines. If you want to use Arduino IDE instead, create a sketch for the target board and use the same source with the matching define at the very top.

## S2 repeater

Board: **Adafruit Feather ESP32-S2**

At the top of the sketch, before the repeater source:

```cpp
#define HOLDEN_TARGET_S2 1
```

Then use the code from `src/repeater_v3.cpp`.

Recommended Arduino IDE settings:

- USB CDC On Boot: **Enabled**
- Upload Mode: **Internal USB**
- Serial Monitor: **115200 baud**

See `docs/S2_BOOT_MODE.md` for the exact BOOT/DFU + RESET procedure.

## C3 repeater

Board: **Seeed XIAO ESP32-C3**

Use `src/repeater_v3.cpp` without `HOLDEN_TARGET_S2` defined.

C3 UART and DX-LR20 wiring:

- D0 = NSS
- D1 = RESET
- D2 = DIO1
- D3 = BUSY
- D4 = TXEN
- D5 = RXEN
- D6 = UART TX to S2 RX
- D7 = UART RX from S2 TX
- D8 = SCK
- D9 = MISO
- D10 = MOSI

## Heltec V3 messenger

Board: **Heltec WiFi LoRa 32 V3**

At the top of the endpoint sketch:

```cpp
#define HOLDEN_ENDPOINT_HELTEC 1
```

Then use the code from `src/endpoint_messenger.cpp`.

## Home DX-LR20 messenger

Board: **Seeed XIAO ESP32-C3**

At the top of the endpoint sketch:

```cpp
#define HOLDEN_ENDPOINT_HOME 1
```

Then use the code from `src/endpoint_messenger.cpp`.

## Library

Install **RadioLib** by Jan Gromes from Arduino Library Manager. The PlatformIO build pins RadioLib 7.7.1.

## Receiver behavior

Both messenger endpoints stay in LoRa receive mode when idle and only add a message to the UI after a valid repeated frame passes all checks:

- `HL` outer magic/version
- matching Network ID
- repeat marker
- valid HMAC-SHA256 tag
- newer monotonic repeat counter
- valid authenticated inner `HM` message

Pressing Send does not immediately add the message locally. The endpoint first transmits the authenticated FSK uplink, switches back to LoRa RX, and the message appears only after the repeater sends back the authenticated repeated LoRa frame.
