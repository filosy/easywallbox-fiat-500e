# EasyWallbox Fiat 500e

ESP32 controller for a Fiat 500e and an EasyWallbox. It lets the user enter the starting battery SoC, estimates charging progress, and sends a stop command at the configured target (80% by default). It also reads EasyWallbox telemetry over BLE and displays estimated household and total power on the OLED.

> **Firmware status:** The repository contains the working sketch `esp32_easywallbox_v3.ino` (REV 3.1) and this documentation. BLE pairing, application authentication, and `$DATA,READ,AD` telemetry reception have been observed working on the author's EasyWallbox. Power values in kW are estimates derived from current and a configured nominal voltage; verify them against the wallbox app and an independent meter before relying on them for load-limit decisions.

## Features

- BLE connection and pairing with an ESP32, followed by EasyWallbox application authentication.
- Periodic `$DATA,READ,AD` request every 10 seconds after authentication.
- OLED pages for SoC/charging progress and power/energy information.
- Encoder selection of starting SoC (5–79%); target SoC defaults to 80%.
- Long encoder-button press toggles the sketch's temporary DPM-off timer (15 minutes).
- Automatic charge stop at the target, with elapsed-time fallback if live energy telemetry is unavailable.
- OLED automatic standby after 20 minutes without user activity.

## Hardware and software

- NodeMCU ESP-32S / ESP-WROOM-32E.
- In Arduino IDE, select **ESP32 Dev Module** for this board.
- 128x64 SSD1306 OLED at I2C address `0x3C`.
- EC11 rotary encoder.
- Arduino IDE with the ESP32 Arduino core, NimBLE-Arduino 2.5.2, Adafruit GFX, and Adafruit SSD1306 libraries.

The sketch uses GPIO 21/22 for I2C and GPIO 32, 33, and 25 for the encoder signals. Check your board wiring and pinout before connecting peripherals. Do not connect ESP32 pins directly to mains or charging conductors.

## BLE setup

The sketch identifies the wallbox by its configured BLE address and uses these BLE characteristics:

- Service: `331a36f5-2459-45ea-9d95-6142f0c4b307`
- Command writes: `a9da6040-0823-4995-94ec-9ce41ca28833`
- Notifications: `a73e9a10-628f-4494-a099-12efaf72258f`
- Additional status notifications: `75a9f022-af03-4e41-b4bc-9de90a47d50b`

The pairing passkey and the EasyWallbox application authentication PIN are different credentials. The current sketch provides pairing passkey `01234` (represented as integer `1234` in NimBLE) and sends the application PIN configured in `EWB_BLE_PIN`. Set the latter to the PIN for your wallbox; do not publish personal credentials in shared copies of the sketch.

Only one BLE controller may be able to connect to the wallbox at a time. Close the manufacturer's app while testing if the ESP32 cannot connect.

## DPM power reading and voltage setting

After receiving `$BLE,AUTH,OK`, the ESP32 polls `$DATA,READ,AD`. In the observed AD response, the DPM-current value is reported in tenths of an ampere. The sketch converts it to an estimated power using:

`estimated kW = DPM current in dA × DPM_NOMINAL_VOLTAGE_V / 10000`

The setting is near the top of `esp32_easywallbox_v3.ino`:

```cpp
const float DPM_NOMINAL_VOLTAGE_V = 230.0f;  // da regolare in base alla tensione domestica
```

Change `230.0f` to a representative measured line voltage if you want to adjust the estimate. This is a fixed assumption, **not an automatic voltage measurement**, so the displayed kW will not follow voltage fluctuations. For example, 25 dA at 230 V gives an estimated 0.575 kW.

The sketch displays `DPM total` from the DPM sensor estimate and `House est.` calculated by subtracting the wallbox charging power when the car is charging. The field interpretation is based on observed AD data and is not guaranteed for every EasyWallbox firmware/model. Compare both values with the app while the vehicle is charging and with a known household load before using them to judge proximity to the configured maximum. Avoid adding the vehicle power to a DPM reading if that reading already represents the combined load, as that would double-count the car.

## Upload and test

1. Configure the wallbox BLE address and application PIN in the sketch.
2. Select **ESP32 Dev Module**, the correct serial port, and upload `esp32_easywallbox_v3.ino`.
3. Open Serial Monitor at 115200 baud.
4. Check for pairing success, `[EWB RX] $BLE,AUTH,OK`, and `$DATA,READ,AD` responses.
5. Compare the OLED/serial DPM reading with the EasyWallbox app at idle and while charging; switch a known household load on and off to check that the reading responds plausibly.
6. Test charge-stop behavior under supervision before relying on it. Do not use the estimated kW value as an electrical protection or a substitute for the wallbox DPM.

The sketch reports successful BLE writes separately from wallbox replies: `[EWB TX] ... -> OK` means the GATT write succeeded, not that the requested operation was acknowledged. Look for corresponding `[EWB RX]` data.

## Safety and disclaimer

This is an independent project and is not affiliated with or endorsed by Fiat or EasyWallbox. Charging equipment involves hazardous voltages and currents. Do not open or modify charging equipment or mains wiring unless qualified to do so. The ESP32, its firmware, and estimated telemetry are not safety devices and must not replace the EasyWallbox's correctly installed protections or DPM. Verify behavior in a supervised, safe setup and retain the manufacturer's protections.

## License

No license has been specified. Until a license is added, the repository's contents are not explicitly licensed for reuse.
