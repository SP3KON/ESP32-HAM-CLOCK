*[Leia em Português](README.pt-BR.md)*

# ESP32-HAM-CLOCK-ptbr

Fork of [SP3KON/ESP32-HAM-CLOCK](https://github.com/SP3KON/ESP32-HAM-CLOCK) with **Portuguese (Brazil) translation** and **MEDIUM table size**.

## What's new in this fork

### Portuguese (Brazil) language
- New `TFT_LANG_PT` with complete PT translation
- Third language option in TFT menu (PL/EN/PT)
- Portuguese date strings, weather/alert screens, HF band info
- OpenWeather API requests use `lang=pt_br`
- Web UI: `indexPT.html`, `manual_pt.txt` with PT routes
- Accent handling (sanitizes Latin-1 to ASCII for display)

### MEDIUM table size
- New layout mode between SMALL and LARGE
- Uses built-in Font 2 (16px), 20px row pitch, 9 rows
- Works with DX, POTA, HamAlert, APRS-IS screens
- Web option and NVS/backup support (`table_size=medium`)

## Original project

This is an ESP32 HAM clock and DX/APRS terminal for ESP32 (mainly CYD ESP32-2432S028) with 2.4" TFT display and parallel web panel.

Features:
- DX Cluster (telnet)
- POTA (API)
- HAMALERT (telnet)
- APRS-IS (RX + optional TX beacon)
- Weather and forecast (OpenWeather)
- Propagation/solar data

See the [original repository](https://github.com/SP3KON/ESP32-HAM-CLOCK) for full documentation.

## Branch

The Portuguese translation and MEDIUM size are in the `feature/pt-br` branch. There's an open PR (#17) to the original project.

## Credits

- Original project: [SP3KON](https://github.com/SP3KON/ESP32-HAM-CLOCK)
- Portuguese translation and MEDIUM size: PP5RD
