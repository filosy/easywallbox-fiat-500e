# EasyWallbox Fiat 500e

A lightweight ESP32-based project to address the lack of automatic charge stopping at a user-defined battery state of charge (SoC) when charging a Fiat 500e with an EasyWallbox.

## What it does

The project is intended to stop charging when the battery reaches a selected SoC limit—typically 80%. Before charging, the user selects the battery's current SoC; the ESP32 then monitors the charge progression and stops the charging process when the configured limit is reached.

## Hardware and software

- ESP32 development board
- EasyWallbox and Fiat 500e
- Project firmware (see the source code in this repository)

## How to use

1. Set up the ESP32 and connect it to the charging-control arrangement implemented by this project.
2. Configure or select the vehicle's current battery SoC.
3. Set the desired charge limit (usually 80%).
4. Start charging. The project is designed to stop the charge when the selected limit is reached.

> **Note:** Wiring, configuration, and operating details depend on the implementation in the source code. Review the code and verify the setup carefully before connecting or using it with a vehicle or charging equipment.

## Disclaimer

This is an independent project and is not affiliated with or endorsed by Fiat or EasyWallbox. Use it at your own risk. Incorrect wiring or control of charging equipment can cause damage or create safety hazards. Follow the manufacturers' instructions and applicable electrical-safety requirements.

## License

No license has been specified yet. Until a license is added, the repository's contents are not explicitly licensed for reuse.
