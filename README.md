# NavPlotter for the 2.8" ESP32-32E display board

Nautical chart plotter on the "2.8 inch ESP32-32E" board (ESP32, ILI9341 320x240 display, resistive touch, SD card slot) with an external GPS module. It is the touch-only sibling of the [T-Deck plotter](https://github.com/digitalimpresys/Navplotter-tdeck-plus): the board has no keys, so everything is done on the screen, including typing names on an on-screen keyboard.

User interface in English and Italian (Setup page).

**Not for navigation.** The charts are drawn from OpenStreetMap data, which has no depths.

## Hardware

| Part | Connection |
|---|---|
| GPS module (NEO-M8 or similar) | TX to IO35, power from 3.3 V and GND on the small connector. The GPS RX pin is not used. |
| SD card | built in. Charts in `/tiles/z/x/y.jpg`, the same card layout as the T-Deck plotter |
| LoRa radio: RFM95W 868 MHz (SX1276), for example on a Dragino LoRa Shield v1.4 | SCK IO18, MISO IO19, MOSI IO23, NSS IO27 (shield pins 13, 12, 11, 10), 3.3 V to the shield 3V3 pin, GND. **Never 5 V**, never without antenna. DIO0 and RESET are not wired: the firmware polls the radio. On the Dragino shield the SV2-SV4 jumpers must route SPI to pins 11-13. |
| Speaker | optional, on the SPEAKER connector |

The board was identified with the small firmware in `probe/`: ESP32-D0WD-V3 with 4 MB flash and no PSRAM, ILI9341 display on HSPI, XPT2046 touch on IO25/32/39/33, SD card on VSPI.

## Using it

- **First start:** touch calibration, four crosses to press. It can be repeated from Setup.
- **Map:** drag to move, `+` and `-` to zoom, long press to add a waypoint. The button in the top left corner keeps the boat in the middle of the map (follow mode); dragging the map turns it off.
- **WPT:** waypoint list. Go to (sets the destination: a red line on the map, bearing and distance in the status bar), Show, Name, Delete.
- **NEAR:** the other NavPlotter nodes received over LoRa (T-Deck plotter, Heltec node), with distance, bearing and closest approach. They are drawn on the map with a symbol for their type (boat, person, object). When one is predicted to pass closer than 0.1 NM within 5 minutes a red band appears on the map and the speaker beeps; a tap on the band silences it for a minute.
- **COMP:** heading-up compass with the bearing of the destination. Heading is the GPS course.
- **SET:** boat name, language, sound, follow mode, touch calibration, position sharing (off = the boat is hidden from the others but still receives and relays them), status.

Waypoints are stored on the SD card in the same file as the T-Deck plotter (`waypoints.txt`); settings are kept in the board's own memory.

## How the screen is drawn

A full-screen 16-bit buffer does not fit in memory on this board, and drawing straight to the display flickers. The screen is therefore drawn off-screen in three 320x80 strips, which do fit, and then pushed to the display.

## Building

```
~/.platformio/penv/bin/pio run -t upload
```

## Still to do

Routes. Radio interoperability between this SX1276 and the SX1262 of the T-Deck and Heltec nodes still has to be confirmed on the air.

`include/traffic_packet.h` (packet, radio settings, mesh) is shared with the other NavPlotter firmwares and must stay identical to theirs; its comments are in Italian because it was first written there.
