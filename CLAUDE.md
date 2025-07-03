# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ZMK (Zephyr Mechanical Keyboard) configuration repository for the Sage60 electro-capacitive keyboard. The project includes custom keyboard shields, board definitions, and drivers for a split 60% keyboard that uses capacitive switches with ADC-based key detection.

## Build Commands

### Primary Build Commands
Use the `west` build system with these commands from `/workspaces/zmk/app`:

```bash
# Seeed XIAO BLE with EC (Electro-Capacitive) - Left side with logging
west build -s /workspaces/zmk/app -d build/xiao_ec_left -b seeeduino_xiao_ble -S zmk-usb-logging -- -DSHIELD=sage60_ec_left -DZMK_CONFIG="/workspaces/zmk-config/" -DZMK_EXTRA_MODULES="/workspaces/zmk-config"

# Akizuki NRF52840 board - EC version
west build -p always -s /workspaces/zmk/app -d build/ec_left -b ae_nrf52840 -S studio-rpc-usb-uart -- -DSHIELD=sage60_ec_left -DZMK_CONFIG="/workspaces/zmk-config/config" -DCONFIG_ZMK_STUDIO=y -DCONFIG_ZMK_STUDIO_LOCKING=n -DZMK_EXTRA_MODULES="/workspaces/zmk-config/config/sage60_drivers"

# Regular MX switch version
west build -s /workspaces/zmk/app -d build/left -b ae_nrf52840 -S studio-rpc-usb-uart -- -DSHIELD=sage60_left -DZMK_CONFIG="/workspaces/zmk-config/config" -DCONFIG_ZMK_STUDIO=y -DCONFIG_ZMK_STUDIO_LOCKING=n
```

### Build Matrix (GitHub Actions)
The `build.yaml` file defines the build matrix for automated builds:
- `ae_nrf52840` board with `sage60_left/right` shields (MX switches)
- `seeeduino_xiao_ble` board with `sage60_ec_left/right` shields (EC switches)
- Special calibrator build for EC tuning
- Settings reset utility

## Architecture

### Repository Structure
```
├── boards/
│   ├── arm/ae_nrf52840/           # Akizuki Electronics NRF52840 board definition
│   └── shields/
│       ├── sage60/                # Standard MX switch version
│       └── sage60_ec/             # Electro-capacitive version
├── config/
│   └── west.yml                   # West manifest for dependencies
├── build.txt                     # Build command reference
└── build.yaml                    # GitHub Actions build matrix
```

### Key Components

#### Electro-Capacitive System
- **Custom EC Matrix Driver**: Uses ADC to detect capacitive changes
- **GPIO Multiplexer**: Switches between capacitive plates using `zmk,gpio-mux`
- **ADC Configuration**: 8-bit resolution, internal reference, specific timing
- **Calibration Support**: Special build target for threshold tuning

#### Board Support
- **Akizuki NRF52840**: Custom board definition with Arduino Pro Micro pin compatibility
- **Seeed XIAO BLE**: Native support with overlay configurations
- **Split Architecture**: Central-peripheral BLE communication

#### Configuration System
- **Devicetree Overlays**: Hardware-specific pin mappings and ADC settings
- **Kconfig Files**: Feature flags and driver configurations
- **Shield Definitions**: Keyboard-specific layouts and matrix configurations

### Physical Layout
The Sage60 is a 60% split keyboard with:
- 62 keys total (31 per side)
- Column-staggered layout with thumb clusters
- Physical layout definition for ZMK Studio compatibility
- Support for both MX mechanical and EC switches

## Development Workflow

### Working with EC (Electro-Capacitive) Features
- EC builds require `DZMK_EXTRA_MODULES` pointing to custom drivers
- Use `-S zmk-usb-logging` snippet for debugging EC sensor values
- Calibration builds help determine optimal thresholds for key detection

### Testing and Debugging
- Enable USB logging with `-S zmk-usb-logging` for real-time debug output
- ZMK Studio support for live configuration (when enabled with `CONFIG_ZMK_STUDIO=y`)
- Use calibrator build to tune EC sensor thresholds

### Hardware Variants
- **MX Version**: Standard mechanical switches with basic GPIO matrix
- **EC Version**: Capacitive switches requiring ADC and custom driver
- **XIAO Version**: Optimized for Seeed XIAO BLE's smaller form factor
- **Akizuki Version**: Uses custom board definition for NRF52840 breakout

## Important Notes

- This is a user configuration repository that depends on the main ZMK firmware
- EC functionality requires the `ec-support-zmk-module` from petejohanson
- The repository uses Japanese documentation but code/configs are in English
- GPIO multiplexer enables scanning multiple capacitive inputs with single ADC channel
- Power management includes dedicated power GPIO for EC system