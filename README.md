# EasyWallbox Fiat 500e

ESP32 controller for a Fiat 500e and an EasyWallbox. It lets the user enter the starting battery SoC, estimates charging progress, displays DPM and house power in a local web page, and sends a stop command at the configured target (80% by default).

## Current firmware status

The sketch `esp32_easywallbox_v3.ino` is **REV 3.2 WEB**:

- BLE pairing and application authentication with the EasyWallbox.
- Polling of `$DATA,READ,AD` every 10 seconds after authentication.
- OLED progress and power/energy display.
- Local web server on port 80, with no HTTP login.
- Web control for SoC selection/start, native stop, DPM ON and DPM OFF 15 minutes.
- Graphical confirmation of the SoC accepted by the ESP32.
- Native stop command `$CMD,CHARGE,STOP,0`.
- DPM enable/disable through EEPROM index 178.

The native stop and DPM commands are test features: **successful GATT writes do not prove that the wallbox executed the command**. The AD telemetry and subsequent charging current/state must confirm the result.

## Security

The web page has no HTTP authentication. Use it only on a trusted local network and do not expose the ESP32 to the Internet. BLE authentication is still required for wallbox commands.

## Hardware and software

- NodeMCU ESP-32S / ESP-WROOM-32E.
- Arduino IDE: ESP32 Dev Module.
- SSD1306 OLED 128x64 at I2C address `0x3C`.
- EC11 rotary encoder.
- Libraries: NimBLE-Arduino 2.5.2, Adafruit GFX, Adafruit SSD1306.
- ESP32 pins: I2C SDA/SCL 21/22, encoder CLK/DT/SW 32/33/25.

## Wi-Fi setup

Set `WIFI_SSID` and `WIFI_PASS` near the top of the sketch. The ESP32 should join a 2.4 GHz Wi-Fi network. After boot, open the IP printed by the Serial Monitor at `http://<ESP32-IP>/`.

## Wallbox commands

- Login: `$BLE,AUTH,{pin}`
- Native stop: `$CMD,CHARGE,STOP,0`
- Read app data: `$DATA,READ,AD`
- DPM off: `$EEP,WRITE,IDX,178,0`
- DPM on: `$EEP,WRITE,IDX,178,1`

## Upload and test

1. Configure Wi-Fi, wallbox BLE address and application PIN.
2. Select **ESP32 Dev Module**, the correct serial port, and upload `esp32_easywallbox_v3.ino`.
3. Open Serial Monitor at 115200 baud.
4. Confirm pairing, `$BLE,AUTH,OK`, and `$DATA,READ,AD` responses.
5. Open the web page and test the SoC confirmation first.
6. Test DPM controls while observing AD telemetry.
7. Test stop under supervision. Watch for the wallbox state, offered/current draw, and stop ACK; do not rely only on `-> OK` or a web success message.

## Safety and disclaimer

This is an independent project and is not affiliated with or endorsed by Fiat or EasyWallbox. Charging equipment involves hazardous voltages and currents. Do not open or modify charging equipment or mains wiring. DPM estimates are not electrical protection and must not replace suitable charger, cable and installation protection.
