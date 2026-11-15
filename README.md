# EasyWallbox Fiat 500e

An ESP32-based project designed to stop a Fiat 500e charging session at a user-selected battery state of charge (SoC), typically 80%, when using an EasyWallbox.

> **Project status:** This repository currently contains documentation only. Firmware, wiring diagrams, supported ESP32 boards, and the exact charging-control method have not yet been added. The details below describe the intended behavior based on the project description.

## The problem

The charging setup does not provide the desired automatic stop at a selected battery percentage. This project aims to provide that behavior with a simple ESP32 and a small amount of code.

## Intended behavior

1. The user selects or enters the battery's current SoC before charging.
2. A target SoC is selected, usually 80%.
3. The ESP32 uses those values to determine when the target is reached and stop the charging session.

The exact way the current SoC is entered, how charging progress is determined, and how the ESP32 stops charging depend on the implementation and are not documented yet.

## Hardware and software

- An ESP32 development board
- A Fiat 500e and EasyWallbox
- Firmware and any required interface components (details to be documented)

## Setup and use

The project-specific installation and operating instructions are not available in the repository yet. Before using this project, document and verify at least:

- The supported ESP32 board and required software/toolchain
- How to configure the current SoC and target percentage
- The connection/interface used to control charging
- Wiring, power, and enclosure requirements
- How to test the setup safely before connecting it to charging equipment

Do not infer wiring or connect an ESP32 directly to mains or charging conductors based on this README. Use an appropriately designed and isolated interface, and follow the equipment manufacturers' instructions and applicable electrical-safety requirements.

## Safety and disclaimer

This is an independent project and is not affiliated with or endorsed by Fiat or EasyWallbox. Charging equipment involves hazardous voltages and currents. Incorrect wiring or control can cause electric shock, fire, injury, or damage to the vehicle or equipment. Use at your own risk and have the installation checked by a qualified professional.

## License

No license has been specified. Until a license is added, the repository's contents are not explicitly licensed for reuse.
