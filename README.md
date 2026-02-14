# ESP32-HAM-CLOCK
ESP32 HAM CLOCK to zegar i terminal DX Cluster dla modułu ESP32-2432S028 (CYD) z dotykowym TFT 2.4". Aplikacja łączy się z DX Cluster (telnet), POTA (API), APRS-IS i pobiera pogodę oraz dane propagacyjne. Ma wbudowany interfejs WWW (emulacja TFT), portal konfiguracyjny w trybie AP, obsługę dotyku (menu filtrów, jasność, kalibracja) i zapis ustawień w NVS. Ekrany obejmują zegar (UTC/local), DX, POTA, APRS, pasma HF, propagację, pogodę i matrix clock. Obsługa LittleFS (fonty/HTML), auto-kopiowanie User_Setup.h do TFT_eSPI, oraz cykliczne odświeżanie danych (pogoda, propagacja, POTA, QRZ).


# ESP32 HAM CLOCK / DX Cluster

Zegar i terminal DX Cluster na ESP32 z wyświetlaczem 2.4" (ESP32-2432S028 / CYD) oraz interfejsem WWW (emulacja TFT).

## Funkcje

- DX Cluster przez telnet (filtry, logowanie, keepalive)
- POTA spoty z publicznego API (HTTP)
- APRS-IS odbiór ramek (TCP)
- Pogoda + prognoza + PM2.5/PM10 z OpenWeather
- Dane propagacyjne (hamqsl solarxml)
- Ekrany TFT: Zegar (UTC/local), DX, APRS, pasma HF, propagacja, pogoda, POTA, matrix clock
- Dotyk XPT2046: nawigacja, menu filtrów, długi press = menu jasności; kalibracja i rotacja dotyku/TFT
- Portal konfiguracyjny HTTP (AP fallback gdy brak WiFi); zapis preferencji w NVS
- LittleFS: fonty, pliki HTML/JS/CSS dla interfejsu WWW
- Kopiowanie `User_Setup.h` do TFT_eSPI przed build (extra_script)

## Sprzęt

- Płytka: ESP32-2432S028 (ESP32 WROOM, TFT 240x320 ILI9341, dotyk XPT2046, wbudowany BL)
- Alternatywnie ESP32 (bez TFT; użyj interfejsu WWW) i zewnętrzny wysietlacz ILI9341

### Piny (ESP32-2432S028)

Wyświetlacz ILI9341:
- TFT_SCLK: GPIO14
- TFT_MOSI: GPIO13
- TFT_MISO: GPIO12
- TFT_CS: GPIO15
- TFT_DC: GPIO2
- TFT_RST: GPIO4
- TFT_BL: GPIO21 (PWM, domyślnie 5 kHz, 8 bit)

Dotyk XPT2046:
- TOUCH_CS: GPIO33
- TOUCH_IRQ: GPIO36
- TOUCH_MOSI: GPIO32
- TOUCH_MISO: GPIO39
- TOUCH_CLK: GPIO25

Inne:
- Backlight PWM kanał 0
- Wbudowane przyciski brak; nawigacja dotykiem (dolne narożniki = zmiana ekranów)

## Środowiska PlatformIO

- `esp32-2432s028`: ESP32 Dev + TFT_eSPI + XPT2046; FS: LittleFS; `ENABLE_TFT_DISPLAY`
- `esp32-c3-devkitm-1`: ESP32-C3 (bez TFT); FS: LittleFS

Kluczowe biblioteki (z `platformio.ini`): ArduinoJson 7, TFT_eSPI, XPT2046_Touchscreen, AsyncTCP, ESPAsyncWebServer.

## Jak zbudować (PlatformIO)

1. `platformio run -e esp32-2432s028` (domyślne środowisko) lub wybierz inne środowisko w `platformio.ini`
2. Przed kompilacją `pre:copy_user_setup.py` kopiuje `User_Setup.h` do biblioteki TFT_eSPI
3. Wgraj firmware; opcjonalnie `pio run -t uploadfs` dla plików LittleFS (`data/`)

## Konfiguracja (pierwsze uruchomienie)

1. Jeśli brak zapisanych WiFi → uruchamia się AP: SSID `ESP32-HAM-CLOCK`, hasło `1234567890`
2. Wejdź na `http://192.168.4.1` → zakładka Config
3. Ustaw WiFi (SSID/hasło + opcjonalny drugi zestaw), DX Cluster host/port, znak, locator, klucz OpenWeather, QRZ (opcjonalnie), ustawienia TFT (jasność, język, rotacja, kalibracja)
4. Zapisz, moduł zrestartuje się i połączy z WiFi STA

## Ekrany TFT (skrót)

- Zegar (UTC/local, data, IP, jasność, język)
- DX Cluster (lista spotów, filtr modulacja/pasmo)
- APRS-IS (stacje, sortowanie)
- Band Info (HF dzień/noc)
- Sun Spots / propagacja
- Pogoda (bieżąca + szczegóły, PM2.5/PM10)
- POTA (filtry, kolejność czasowa)
- Matrix Clock (styl matrycy)

## Uwagi dot. sieci

- WiFi: auto-reconnect do ostatniego SSID (tryb STA); AP tylko gdy brak konfiguracji lub start bez WiFi
- DX Cluster reconnect: co 20 s minimalnie, keepalive CRLF co 30 s
- Pogoda: co 10 min; przy błędzie co 2 min; każdy cykl robi 3 żądania HTTPS (pogoda/forecast/air)
- Propagacja: co 60 min; przy błędzie retry po 5 min
- POTA API: co 180 s
- QRZ lookup: co 3 s gdy aktywny ekran DX/POTA, inaczej co 10 s (z limitami prób)

## Licencja

Projekt open-source dla społeczności krótkofalarskiej.
