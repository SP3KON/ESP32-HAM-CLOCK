# Jak dodać folder projektu z PlatformIO? (PL)

## Wykonane działania

Do repozytorium ESP32-HAM-CLOCK została dodana pełna struktura projektu PlatformIO. Wszystkie niezbędne pliki i foldery zostały utworzone zgodnie z dokumentacją z README.md.

## Struktura projektu

```
ESP32-HAM-CLOCK/
├── platformio.ini          # Konfiguracja PlatformIO
├── User_Setup.h            # Konfiguracja TFT_eSPI dla ESP32-2432S028
├── copy_user_setup.py      # Skrypt pre-build do kopiowania User_Setup.h
├── src/                    # Kod źródłowy
│   └── main.cpp           # Główna aplikacja
├── include/               # Pliki nagłówkowe
├── lib/                   # Lokalne biblioteki
├── data/                  # System plików LittleFS
│   └── index.html        # Interfejs WWW
├── .gitignore            # Reguły ignorowania plików
├── README.md             # Dokumentacja główna
└── PLATFORMIO_SETUP.md   # Dokumentacja PlatformIO (EN)
```

## Co zostało dodane?

### 1. platformio.ini
- Konfiguracja dla dwóch środowisk:
  - `esp32-2432s028` (domyślne) - z wyświetlaczem TFT
  - `esp32-c3-devkitm-1` - bez wyświetlacza (tylko WWW)
- Zależności bibliotek: ArduinoJson, TFT_eSPI, XPT2046_Touchscreen, AsyncTCP, ESPAsyncWebServer
- Flagi kompilacji dla pinów i konfiguracji TFT
- System plików: LittleFS

### 2. User_Setup.h
- Konfiguracja TFT_eSPI dla ESP32-2432S028
- Definicje pinów dla wyświetlacza ILI9341 i dotyku XPT2046
- Częstotliwości SPI
- Wsparcie dla fontów

### 3. copy_user_setup.py
- Skrypt Python wykonywany przed kompilacją
- Automatycznie kopiuje User_Setup.h do biblioteki TFT_eSPI
- Zapewnia poprawną konfigurację pinów

### 4. src/main.cpp
Główna aplikacja zawierająca:
- **WiFi**: tryb STA z auto-reconnect, tryb AP (fallback), podwójne SSID
- **Serwer WWW**: portal konfiguracyjny, API statusu i konfiguracji
- **Preferencje**: zapis w NVS
- **TFT**: inicjalizacja, obsługa dotyku, PWM dla podświetlenia
- **Ekrany**: 8 ekranów (Clock, DX, APRS, Bands, Propagation, Weather, POTA, Matrix)
- **Timery**: dla aktualizacji pogody, propagacji, POTA, keepalive DX Cluster
- **Szkielety funkcji**: dla DX Cluster, APRS-IS, Weather API, POTA, QRZ

### 5. data/index.html
Interfejs WWW zawierający:
- Zakładki: Status, Display, Config
- Strona statusu z auto-odświeżaniem (co 5s)
- Formularz konfiguracyjny
- Responsywny design (ciemny motyw)
- API do komunikacji z ESP32

### 6. .gitignore
- Ignorowanie artefaktów kompilacji (.pio/)
- Ignorowanie plików IDE (.vscode/, .idea/)
- Ignorowanie plików systemowych

### 7. PLATFORMIO_SETUP.md
- Kompletna dokumentacja po angielsku
- Instrukcje instalacji i kompilacji
- Opis środowisk
- Konfiguracja pinów
- Rozwiązywanie problemów

## Jak używać?

### Instalacja PlatformIO

```bash
pip install platformio
```

### Kompilacja

Dla ESP32-2432S028 (z TFT):
```bash
pio run -e esp32-2432s028
```

Dla ESP32-C3 (bez TFT):
```bash
pio run -e esp32-c3-devkitm-1
```

### Upload firmware

```bash
pio run -e esp32-2432s028 -t upload
```

### Upload systemu plików (LittleFS)

```bash
pio run -e esp32-2432s028 -t uploadfs
```

### Monitorowanie

```bash
pio device monitor
```

## Pierwsza konfiguracja

1. Po pierwszym uruchomieniu ESP32 tworzy Access Point:
   - SSID: `ESP32-HAM-CLOCK`
   - Hasło: `1234567890`

2. Połącz się z AP i wejdź na: `http://192.168.4.1`

3. W zakładce Config ustaw:
   - WiFi (główne i zapasowe SSID)
   - DX Cluster (host/port)
   - Znak wywoławczy i lokator
   - Klucz API OpenWeather
   - Klucz API QRZ.com (opcjonalnie)
   - Ustawienia wyświetlacza (jasność, język, rotacja)

4. Kliknij "Save Configuration & Restart"

5. Urządzenie zrestartuje się i połączy z WiFi

## Co jest gotowe?

✅ Pełna struktura projektu PlatformIO
✅ Konfiguracja dla dwóch platform ESP32
✅ Konfiguracja TFT_eSPI z automatycznym kopiowaniem
✅ Główna aplikacja ze szkieletem wszystkich funkcji
✅ Interfejs WWW z portalem konfiguracyjnym
✅ Zapis preferencji w NVS
✅ Obsługa WiFi (STA/AP)
✅ Nawigacja po ekranach dotykiem
✅ System plików LittleFS

## Co wymaga implementacji?

Następujące funkcje mają szkielety i wymagają pełnej implementacji:

- [ ] DX Cluster: pełny protokół telnet, parsowanie spotów, filtry
- [ ] APRS-IS: parsowanie ramek, śledzenie stacji
- [ ] Weather: integracja z OpenWeather API (3 endpointy)
- [ ] Propagation: parsowanie hamqsl solarxml
- [ ] POTA: integracja API i filtrowanie
- [ ] QRZ: lookup z ograniczeniem zapytań
- [ ] Time: synchronizacja NTP dla UTC/local
- [ ] Display: szczegółowe implementacje każdego ekranu
- [ ] Touch: długie naciśnięcie, kalibracja UI
- [ ] Web UI: emulacja TFT na canvas

## Struktura kodu

### Główne obiekty globalne
- `preferences` - NVS do zapisu konfiguracji
- `server` - AsyncWebServer na porcie 80
- `tft` - Obiekt TFT_eSPI (gdy ENABLE_TFT_DISPLAY)
- `touch` - Obiekt XPT2046_Touchscreen (gdy ENABLE_TFT_DISPLAY)
- `dxClusterClient` - Klient TCP dla DX Cluster
- `aprsClient` - Klient TCP dla APRS-IS
- `secureClient` - Klient HTTPS dla API

### Główne funkcje
- `setup()` - Inicjalizacja wszystkich komponentów
- `loop()` - Główna pętla z obsługą timerów
- `setupWiFi()` - Konfiguracja WiFi (STA lub AP)
- `setupWebServer()` - Konfiguracja serwera WWW i API
- `setupTFT()` - Inicjalizacja wyświetlacza
- `handleTouch()` - Obsługa dotyku
- `drawScreen()` - Rysowanie aktualnego ekranu
- `connectDXCluster()` - Połączenie z DX Cluster
- `updateWeather()` - Aktualizacja danych pogodowych
- `updatePropagation()` - Aktualizacja danych propagacyjnych
- `updatePota()` - Aktualizacja spotów POTA

## Biblioteki używane

1. **ArduinoJson 7** - Parsowanie JSON dla API i konfiguracji
2. **TFT_eSPI** - Sterownik wyświetlacza TFT ILI9341
3. **XPT2046_Touchscreen** - Sterownik kontrolera dotyku
4. **AsyncTCP** - Asynchroniczna biblioteka TCP
5. **ESP Async WebServer** - Asynchroniczny serwer WWW

## Definicje pinów (ESP32-2432S028)

### Wyświetlacz TFT (ILI9341)
- SCLK: GPIO14
- MOSI: GPIO13
- MISO: GPIO12
- CS: GPIO15
- DC: GPIO2
- RST: GPIO4
- BL: GPIO21 (PWM, 5 kHz, 8-bit)

### Dotyk (XPT2046)
- CS: GPIO33
- IRQ: GPIO36
- MOSI: GPIO32
- MISO: GPIO39
- CLK: GPIO25

## Podsumowanie

Struktura projektu PlatformIO została w pełni dodana do repozytorium. Projekt jest gotowy do kompilacji i zawiera wszystkie niezbędne pliki konfiguracyjne oraz szkielet aplikacji zgodny z opisem w README.md.

Kod zawiera pełną implementację:
- Zarządzania WiFi
- Serwera WWW z portalem konfiguracyjnym
- Obsługi TFT i dotyku
- Zapisu/odczytu preferencji
- Podstawowej nawigacji po ekranach

Funkcje sieciowe (DX Cluster, APRS, Weather, POTA) mają szkielety i są gotowe do rozbudowy.

---

Więcej informacji w plikach:
- **PLATFORMIO_SETUP.md** - szczegółowa dokumentacja po angielsku
- **README.md** - dokumentacja projektu
- **src/main.cpp** - kod źródłowy z komentarzami
