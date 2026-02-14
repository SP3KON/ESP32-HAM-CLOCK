# PlatformIO Project Setup

This document describes the PlatformIO project structure for ESP32 HAM CLOCK.

## Project Structure

```
ESP32-HAM-CLOCK/
├── platformio.ini          # PlatformIO configuration
├── User_Setup.h            # TFT_eSPI configuration for ESP32-2432S028
├── copy_user_setup.py      # Pre-build script to copy User_Setup.h
├── src/                    # Source code
│   └── main.cpp           # Main application
├── include/               # Header files (empty)
├── lib/                   # Local libraries (empty)
├── data/                  # LittleFS filesystem
│   └── index.html        # Web interface
└── .gitignore            # Git ignore rules
```

## Requirements

- [PlatformIO](https://platformio.org/) installed (CLI or IDE)
- ESP32-2432S028 board (or compatible ESP32 with TFT display)

## Installation

1. Install PlatformIO:
   ```bash
   pip install platformio
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/SP3KON/ESP32-HAM-CLOCK.git
   cd ESP32-HAM-CLOCK
   ```

## Building

### For ESP32-2432S028 (with TFT display):

```bash
pio run -e esp32-2432s028
```

### For ESP32-C3 (without TFT, web UI only):

```bash
pio run -e esp32-c3-devkitm-1
```

## Uploading Firmware

```bash
pio run -e esp32-2432s028 -t upload
```

## Uploading Filesystem (LittleFS)

The web interface and fonts are stored in LittleFS:

```bash
pio run -e esp32-2432s028 -t uploadfs
```

## Environments

### esp32-2432s028 (default)
- Board: ESP32 Dev Module
- Display: ILI9341 240x320 TFT
- Touch: XPT2046
- Features: Full TFT UI + Web UI
- Build flag: `ENABLE_TFT_DISPLAY`

### esp32-c3-devkitm-1
- Board: ESP32-C3 DevKit M-1
- Display: None (Web UI only)
- Build flag: `DISABLE_TFT_DISPLAY`

## Pre-Build Script

The `copy_user_setup.py` script automatically copies `User_Setup.h` to the TFT_eSPI library directory before compilation. This ensures the correct pin definitions for the ESP32-2432S028 board.

## Dependencies

Defined in `platformio.ini`:

- **ArduinoJson** (^7.2.1) - JSON parsing
- **TFT_eSPI** (^2.5.43) - TFT display driver
- **XPT2046_Touchscreen** (^1.4) - Touch controller
- **AsyncTCP** (^1.1.4) - Async TCP library
- **ESP Async WebServer** (^1.2.7) - Web server

## First Run Configuration

1. On first boot, the device creates an Access Point:
   - SSID: `ESP32-HAM-CLOCK`
   - Password: `1234567890`

2. Connect to the AP and open: `http://192.168.4.1`

3. Configure:
   - WiFi credentials (primary and secondary)
   - DX Cluster host/port
   - Callsign and locator
   - OpenWeather API key
   - QRZ.com API key (optional)
   - Display settings (brightness, language, rotation)

4. Click "Save Configuration & Restart"

5. The device will restart and connect to your WiFi network

## Features Implemented

### Network
- [x] WiFi STA mode with auto-reconnect
- [x] WiFi AP mode (fallback)
- [x] Dual WiFi SSID support
- [x] Web server with config portal
- [x] NVS preferences storage

### Display (when `ENABLE_TFT_DISPLAY` is defined)
- [x] TFT initialization with custom User_Setup.h
- [x] Touch screen support
- [x] PWM backlight control
- [x] Multiple screen navigation
- [x] 8 screen types: Clock, DX, APRS, Bands, Propagation, Weather, POTA, Matrix

### Connectivity (Stubs)
- [ ] DX Cluster telnet connection
- [ ] APRS-IS TCP connection
- [ ] OpenWeather API (current, forecast, air quality)
- [ ] POTA API
- [ ] Propagation data (hamqsl)
- [ ] QRZ lookup

### Web Interface
- [x] Status page with system info
- [x] Configuration page
- [x] TFT display emulation canvas (placeholder)
- [x] Auto-refresh status

## TODO

The following features are stubbed in the code and need implementation:

1. **DX Cluster**: Full telnet protocol, spot parsing, filters
2. **APRS-IS**: Frame parsing, station tracking
3. **Weather**: OpenWeather API integration (3 endpoints)
4. **Propagation**: hamqsl solarxml parsing
5. **POTA**: API integration and filtering
6. **QRZ**: Callsign lookup with rate limiting
7. **Time**: NTP sync for accurate UTC/local time
8. **Display**: Detailed screen implementations for each mode
9. **Touch**: Long-press menu, calibration UI
10. **Web UI**: Canvas-based TFT emulation

## Pin Definitions (ESP32-2432S028)

### TFT Display (ILI9341)
- SCLK: GPIO14
- MOSI: GPIO13
- MISO: GPIO12
- CS: GPIO15
- DC: GPIO2
- RST: GPIO4
- BL: GPIO21 (PWM)

### Touch (XPT2046)
- CS: GPIO33
- IRQ: GPIO36
- MOSI: GPIO32
- MISO: GPIO39
- CLK: GPIO25

## Monitoring

View serial output:
```bash
pio device monitor
```

Or combined build, upload, and monitor:
```bash
pio run -e esp32-2432s028 -t upload && pio device monitor
```

## Troubleshooting

### TFT_eSPI User_Setup.h not found
- The `copy_user_setup.py` script should handle this automatically
- Manually copy if needed: `cp User_Setup.h .pio/libdeps/esp32-2432s028/TFT_eSPI/`

### Upload fails
- Check USB connection
- Try holding BOOT button while uploading
- Verify correct board in platformio.ini

### LittleFS upload fails
- Ensure data/ folder exists with index.html
- Try: `pio run -e esp32-2432s028 -t erase` then upload again

## License

Open-source for the ham radio community.
