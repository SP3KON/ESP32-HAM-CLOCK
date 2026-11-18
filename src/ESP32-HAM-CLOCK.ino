/*
 * ESP32-HAM-CLOCK - Odbiornik DX Cluster i stacji APRS-IS z interfejsem WWW i opcjonalnym wyświetlaczem TFT
 * 
 * 
  if (now - lastTelnetAttempt < 20000) {
 * - WiFi Manager (AP mode jeśli brak zapisanych danych)
 * - Połączenie z Telnet DX Cluster (TYLKO ODBIĂ“R - nie wysyła spotów)
 * - Parsowanie i przechowywanie spotów DX
 * - Obliczanie odległości (Haversine)
 * - Interfejs WWW z polling (odświeżanie co 2 sekundy)
 * - Wyświetlacz TFT (ESP32-2432S028) - opcjonalnie ESP32 WROOM + TFT ILI9341 
 * 
 * UWAGA: Urządzenie działa TYLKO w trybie odbioru.
 * Nie wysyła żadnych spotów do DX Cluster - tylko odbiera i wyświetla informacje.
 * Tak samo z APRS.fi  - tylko odczyt.
 */

#include <Arduino.h>

// Wczesne forward-deklaracje, aby auto-prototypowanie Arduino znało typy używane w sygnaturach
enum ScreenType : uint8_t;    // pełna definicja niżej
enum Screen6ViewMode : uint8_t;
struct DXSpot;                // pełna definicja niżej
struct APRSStation;           // pełna definicja niżej
struct AprsWxDecoded;         // pełna definicja niżej
struct PropagationData;       // pełna definicja niżej
struct WeatherAlertInfo;      // pełna definicja niżej

// Forward declarations for UnlisHunter state used before global definitions.
extern bool unlisRunning;
extern bool unlisGameOver;

// Zastępuje polskie znaki w opisach, aby uniknąć problemów z fontem
void normalizePolish(String &text) {
  text.replace("ą", "a");
  text.replace("ć", "c");
  text.replace("ę", "e");
  text.replace("ł", "l");
  text.replace("ń", "n");
  text.replace("ó", "o");
  text.replace("ś", "s");
  text.replace("ź", "z");
  text.replace("ż", "z");
  text.replace("Ą", "A");
  text.replace("Ć", "C");
  text.replace("Ę", "E");
  text.replace("Ł", "L");
  text.replace("Ń", "N");
  text.replace("Ó", "O");
  text.replace("Ś", "S");
  text.replace("Ź", "Z");
  text.replace("Ż", "Z");
}

// ========== KONFIGURACJA WYŚWIETLACZA TFT ==========
// Włącz TFT w platformio.ini (build_flags) lub Arduino IDE; guard to avoid redefinition warnings
#ifndef ENABLE_TFT_DISPLAY
#define ENABLE_TFT_DISPLAY
#endif

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include <ArduinoJson.h>
#include "esp_rom_sys.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <FS.h>
#include <LittleFS.h>
#include <math.h>
#include <ctype.h>

// ========== TYPY EKRANÓW (używane w prototypach) ==========
enum ScreenType : uint8_t {
  SCREEN_OFF = 0,
  SCREEN_HAM_CLOCK = 1,
  SCREEN_DX_CLUSTER = 2,
  SCREEN_SUN_SPOTS = 3,
  SCREEN_BAND_INFO = 4,
  SCREEN_WEATHER_DSP = 5,
  SCREEN_APRS_IS = 6,
  SCREEN_POTA_CLUSTER = 7,
  SCREEN_HAMALERT_CLUSTER = 8,
  SCREEN_APRS_RADAR = 9,
  SCREEN_MATRIX_CLOCK = 10,
  SCREEN_UNLIS_HUNTER = 11,
  SCREEN_WEATHER_FORECAST = 12,
  SCREEN_LOKALIZACJA = 13
};

#define LOG_VERBOSE false
#define LOGV_PRINT(x) do { if (LOG_VERBOSE) Serial.print(x); } while (0)
#define LOGV_PRINTLN(x) do { if (LOG_VERBOSE) Serial.println(x); } while (0)
#define LOGV_PRINTF(...) do {} while (0)

// Monitor RAM na Serial (kontrolnie co 1 s).
// Ustaw false, gdy logi nie są potrzebne.
#define RAM_MONITOR_ENABLED false
#define RAM_MONITOR_INTERVAL_MS 1000UL

static void logRamUsage(unsigned long nowMs) {
  if (!RAM_MONITOR_ENABLED) {
    return;
  }

  static unsigned long lastRamLogMs = 0;
  if ((nowMs - lastRamLogMs) < RAM_MONITOR_INTERVAL_MS) {
    return;
  }
  lastRamLogMs = nowMs;

  const uint32_t freeHeap = ESP.getFreeHeap();
  const uint32_t minFreeHeap = ESP.getMinFreeHeap();
  const uint32_t maxAllocHeap = ESP.getMaxAllocHeap();

#if CONFIG_SPIRAM_SUPPORT
  const uint32_t freePsram = ESP.getFreePsram();
  const uint32_t totalPsram = ESP.getPsramSize();
#else
  const uint32_t freePsram = 0;
  const uint32_t totalPsram = 0;
#endif

  Serial.printf("[RAM] free=%u min=%u maxAlloc=%u psram=%u/%u\n",
                freeHeap,
                minFreeHeap,
                maxAllocHeap,
                freePsram,
                totalPsram);
}

// W setup() są długie, blokujące odcinki (splash/delay), więc loop jeszcze nie loguje.
// Ta pomocnicza funkcja utrzymuje periodyczne logi RAM także przed startem loop().
static void delayWithRamLogs(unsigned long delayMs) {
  unsigned long startMs = millis();
  while ((millis() - startMs) < delayMs) {
    logRamUsage(millis());
    delay(50);
    yield();
  }
}

// ========== WYŚWIETLACZ TFT (ESP32-2432S028) ==========
#ifdef ENABLE_TFT_DISPLAY
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
TFT_eSPI tft = TFT_eSPI();

// Piny TFT dla ESP32-2432S028 (CYD - Cheap Yellow Display)
// Zgodnie z dokumentację: https://github.com/rzeldent/platformio-espressif32-sunton
// ILI9341 kontroler, SPI interface
// 
// W Arduino IDE: piny są definiowane w User_Setup.h (skopiuj plik do folderu biblioteki TFT_eSPI)
// W PlatformIO: piny są definiowane w platformio.ini przez build_flags
//
// Fallback dla backlight (jeśli nie zdefiniowane w User_Setup.h lub build_flags)
#ifndef TFT_BL
#define TFT_BL 21  // Backlight pin (GPIO21 dla ESP32-2432S028)
#endif

#define TFT_BL_PIN TFT_BL

#define BACKLIGHT_PWM_CHANNEL 0
#define BACKLIGHT_PWM_FREQ 5000
#define BACKLIGHT_PWM_RES_BITS 8
#define MIN_BACKLIGHT_PERCENT 10
#define TFT_BACKLIGHT 100
int backlightPercent = TFT_BACKLIGHT;
int tftNightDimmingPercent = 0;
int currentBacklightAppliedPercent = TFT_BACKLIGHT;
unsigned long lastBacklightDayNightCheckMs = 0;
const unsigned long BACKLIGHT_DAY_NIGHT_CHECK_INTERVAL_MS = 5UL * 60UL * 1000UL; // 5 min

// ========== DOTYK (XPT2046 - ESP32-2432S028R) ==========
#ifndef TOUCH_CS
#define TOUCH_CS 33
#endif
#ifndef TOUCH_IRQ
//#define TOUCH_IRQ 36 // domyślnie 36 a dla zewnętrznego TFT ILI9341 użyć 35
#define TOUCH_IRQ 35 // dla zewnętrznego TFT ILI9341
#endif
#ifndef TOUCH_MOSI
#define TOUCH_MOSI 32
#endif
#ifndef TOUCH_MISO
//#define TOUCH_MISO 39 // domyślnie 39 a dla zewnętrznego TFT ILI9341 użyć 27
#define TOUCH_MISO 27 // dla zewnętrznego TFT ILI9341
#endif
#ifndef TOUCH_CLK
#define TOUCH_CLK 25
#endif

// Kalibracja dotyku (dopasuj jeśli pozycje są przesunięte)
#define TOUCH_X_MIN 200
#define TOUCH_X_MAX 3800
#define TOUCH_Y_MIN 200
#define TOUCH_Y_MAX 3800
#define TOUCH_SWAP_XY false
#define TOUCH_INVERT_X false
#define TOUCH_INVERT_Y false

int touchXMin = TOUCH_X_MIN;
int touchXMax = TOUCH_X_MAX;
int touchYMin = TOUCH_Y_MIN;
int touchYMax = TOUCH_Y_MAX;
bool touchSwapXY = TOUCH_SWAP_XY;
bool touchInvertX = TOUCH_INVERT_X;
bool touchInvertY = TOUCH_INVERT_Y;
uint8_t touchRotation = 1;
uint8_t tftRotation = 1;
#ifdef TFT_INVERSION_ON
bool tftInvertColors = true;
#else
bool tftInvertColors = false;
#endif

bool tftInitialized = false;
SPIClass touchSPI(VSPI);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

int bootLogY = 2;
const int BOOT_LOG_LINE_HEIGHT = 10;
bool bootSequenceActive = true;
bool littleFsReady = false;

// ========== SYSTEM MENU I NAWIGACJI ==========
const int SCREEN_PAGE_COUNT = 12;
const ScreenType DEFAULT_SCREEN_ORDER[SCREEN_PAGE_COUNT] = {
  SCREEN_HAM_CLOCK,
  SCREEN_DX_CLUSTER,
  SCREEN_APRS_IS,
  SCREEN_APRS_RADAR,
  SCREEN_BAND_INFO,
  SCREEN_SUN_SPOTS,
  SCREEN_WEATHER_DSP,
  SCREEN_WEATHER_FORECAST,
  SCREEN_POTA_CLUSTER,
  SCREEN_HAMALERT_CLUSTER,
  SCREEN_MATRIX_CLOCK,
  SCREEN_LOKALIZACJA
};

ScreenType screenOrder[SCREEN_PAGE_COUNT] = {
  SCREEN_HAM_CLOCK,
  SCREEN_DX_CLUSTER,
  SCREEN_APRS_IS,
  SCREEN_APRS_RADAR,
  SCREEN_BAND_INFO,
  SCREEN_SUN_SPOTS,
  SCREEN_WEATHER_DSP,
  SCREEN_WEATHER_FORECAST,
  SCREEN_POTA_CLUSTER,
  SCREEN_HAMALERT_CLUSTER,
  SCREEN_MATRIX_CLOCK,
  SCREEN_LOKALIZACJA
};
ScreenType currentScreen = SCREEN_OFF;  // Ustawiany po wczytaniu preferencji
bool inMenu = false;    // Czy jesteśmy w menu wewnętrznym strony
int menuOption = 0;     // Aktualna opcja w menu (jeśli inMenu == true)
const int DEFAULT_TFT_SWITCH_TIME_SEC = 30;
bool tftAutoSwitchEnabled = false;
int tftAutoSwitchTimeSec = DEFAULT_TFT_SWITCH_TIME_SEC;
unsigned long tftAutoSwitchLastMs = 0;
ScreenType tftAutoSwitchLastScreen = SCREEN_OFF;
unsigned long autoswitchPausedUntilMs = 0; // pauza autoswitch po dotknięciu środka ekranu
unsigned long lastScreenUpdate = 0;
const unsigned long SCREEN_UPDATE_INTERVAL = 100; // Aktualizuj ekran co 100ms
const unsigned long DX_SCREEN_MIN_REDRAW_MS = 5000; // Ogranicz zapis tabeli DX do max 1 raz/5s
unsigned long lastScreen1UpdateMs = 0;
unsigned long lastScreen3UpdateMs = 0;
unsigned long lastScreen4UpdateMs = 0;
unsigned long lastScreen5UpdateMs = 0;
unsigned long lastScreen7UpdateMs = 0;
unsigned long lastScreen8UpdateMs = 0;
unsigned long lastScreenLocationUpdateMs = 0;
bool screen1HeaderNeedsRedraw = true;

const int TFT_TABLE_TOP = 32;
const int TFT_TABLE_BOTTOM = 220;
const int TFT_TABLE_WIDTH = 320;
const int TFT_TABLE_HEIGHT = TFT_TABLE_BOTTOM - TFT_TABLE_TOP;
// UnlisHunter constants must be visible before touch handling code uses them.
const int UNLIS_CENTER_X = 160;
const int UNLIS_CENTER_Y = 120;
const int UNLIS_OUTER_R = 115; // 2m
const int UNLIS_INNER_R = 60;  // 70cm
const int UNLIS_BTN_SIZE = 100;
const int UNLIS_DRAW_BTN_SIZE = UNLIS_BTN_SIZE / 2;
const int UNLIS_START_X = 0;
const int UNLIS_START_Y = 0;
const int UNLIS_PTT_X = 320 - UNLIS_BTN_SIZE;
const int UNLIS_PTT_Y = 240 - UNLIS_BTN_SIZE;
const int UNLIS_EXIT_X = 0;
const int UNLIS_EXIT_Y = 240 - UNLIS_BTN_SIZE;
const unsigned long UNLIS_FRAME_MS = 40UL;
const float UNLIS_BASE_SCAN_DEG_PER_SEC = 46.0f;
const float UNLIS_ACCEL_PER_CATCH_EARLY = 0.05f;
const float UNLIS_ACCEL_PER_CATCH_LATE = 0.03f;
const float UNLIS_TARGET_LIFE_ROTATIONS = 2.5f;
const unsigned long UNLIS_TARGET_RESPAWN_MIN_MS = 450UL;
const unsigned long UNLIS_TARGET_RESPAWN_MAX_MS = 1200UL;
const unsigned long UNLIS_SECOND_TARGET_DELAY_MS = 60000UL;
const unsigned long UNLIS_SECOND_TARGET_RESPAWN_MIN_MS = 3500UL;
const unsigned long UNLIS_SECOND_TARGET_RESPAWN_MAX_MS = 9500UL;
const unsigned long UNLIS_GREEN_STATION_LIFE_MS = 5000UL;
const unsigned long UNLIS_GREEN_STATION_RESPAWN_MIN_MS = 9000UL;
const unsigned long UNLIS_GREEN_STATION_RESPAWN_MAX_MS = 18000UL;
const unsigned long TABLE_NAV_FOOTER_VISIBLE_MS = 5000UL;
unsigned long tableNavFooterVisibleUntilMs = 0;
#ifndef TFT_TABLE_SPRITE_COLOR_DEPTH
#define TFT_TABLE_SPRITE_COLOR_DEPTH 16
#endif
const uint16_t TFT_TABLE_ALT_ROW_COLOR = 0x0841;
TFT_eSprite sharedTableSprite = TFT_eSprite(&tft);
bool sharedTableSpriteReady = false;
bool sharedTableSpriteInitTried = false;

static bool ensureSharedTableSprite() {
  if (sharedTableSpriteReady) {
    return true;
  }
  if (sharedTableSpriteInitTried) {
    return false;
  }

  sharedTableSpriteInitTried = true;
  sharedTableSprite.setColorDepth(TFT_TABLE_SPRITE_COLOR_DEPTH);
  sharedTableSpriteReady = (sharedTableSprite.createSprite(TFT_TABLE_WIDTH, TFT_TABLE_HEIGHT) != nullptr);
  if (!sharedTableSpriteReady) {
    LOGV_PRINTLN("[TFT] Shared table sprite alloc failed, fallback to direct draw");
    return false;
  }
  return true;
}

static bool isTableFooterScreen(ScreenType screenNum) {
  return (screenNum == SCREEN_DX_CLUSTER ||
          screenNum == SCREEN_POTA_CLUSTER ||
          screenNum == SCREEN_HAMALERT_CLUSTER ||
          screenNum == SCREEN_APRS_IS ||
          screenNum == SCREEN_WEATHER_DSP ||
          screenNum == SCREEN_WEATHER_FORECAST);
}

static bool isTableNavFooterVisible(ScreenType screenNum) {
  return isTableFooterScreen(screenNum) && millis() < tableNavFooterVisibleUntilMs;
}

static int getTableMaxRowsForScreen(ScreenType screenNum) {
  return isTableNavFooterVisible(screenNum) ? 10 : 11;
}

static int getTableBottomForScreen(ScreenType screenNum) {
  return isTableNavFooterVisible(screenNum) ? TFT_TABLE_BOTTOM : 240;
}

extern uint16_t menuThemeColor;

static void drawSwitchScreenFooter() {
  tft.fillTriangle(10, 230, 20, 222, 20, 238, menuThemeColor);
  tft.fillTriangle(310, 230, 300, 222, 300, 238, menuThemeColor);
  tft.setTextColor(0x52AA);
  tft.setTextSize(1);
  tft.setCursor(125, 226);
  tft.print("SWITCH SCREEN");
}

// Ustawienie trybu czasu dla ekranu 1
enum Screen1TimeMode {
  SCREEN1_TIME_UTC = 0,
  SCREEN1_TIME_LOCAL = 1
};
uint8_t screen1TimeMode = SCREEN1_TIME_UTC;

enum TftLang : uint8_t {
  TFT_LANG_PL = 0,
  TFT_LANG_EN = 1,
  TFT_LANG_PT = 2  // Portuguese (Brazil)
};
uint8_t tftLanguage = TFT_LANG_PL;

// Pick a literal/String by the current language: LANG3(polish, english, portuguese)
#define LANG3(pl, en, pt) \
  ((tftLanguage == TFT_LANG_EN) ? (en) : ((tftLanguage == TFT_LANG_PT) ? (pt) : (pl)))

enum TftStyleMode : uint8_t {
  TFT_STYLE_DARK = 0,
  TFT_STYLE_LIGHT = 1
};
uint8_t tftStyleMode = TFT_STYLE_DARK;

struct TftUiPalette {
  uint16_t bg;
  uint16_t fg;
  uint16_t muted;
  uint16_t accent;
  uint16_t rowAlt;
};

TftUiPalette tftUiPalette = {
  TFT_BLACK,
  TFT_WHITE,
  TFT_LIGHTGREY,
  0xFD20,
  TFT_TABLE_ALT_ROW_COLOR
};

enum DxTableSizeMode : uint8_t {
  DX_TABLE_SIZE_NORMAL = 0,
  DX_TABLE_SIZE_ENLARGED = 1,
  DX_TABLE_SIZE_MEDIUM = 2   // normal layout drawn with the 16 px built-in Font 2
};
uint8_t dxTableSizeMode = DX_TABLE_SIZE_NORMAL;

// Font z polskimi znakami dla daty/dnia tygodnia
#define ROBOTO_FONT10_NAME "/fonts/Roboto10"
#define ROBOTO_FONT10_FILE "/fonts/Roboto10.vlw"
#define ROBOTO_FONT12_NAME "/fonts/Roboto12"
#define ROBOTO_FONT12_FILE "/fonts/Roboto12.vlw"
#define ROBOTO_FONT20_NAME "/fonts/Roboto20"
#define ROBOTO_FONT20_FILE "/fonts/Roboto20.vlw"

static bool tryLoadSmoothFont(const char *fontName, const char *fontFile) {
  if (!littleFsReady || !LittleFS.exists(fontFile)) {
    return false;
  }
  if (tft.fontLoaded) {
    tft.unloadFont();
  }
  tft.loadFont(fontName, LittleFS);
  return tft.fontLoaded;
}

// Po aktualizacji frameworku pierwszy loadFont bywa niestabilny.
// Wariant "light" celowo robi tylko pojedynczy warmup fontu startowego,
// aby nie generować burstu otwarć plików LittleFS (FD).
static bool warmupSmoothFonts() {
  if (!littleFsReady) {
    return false;
  }

  if (!LittleFS.exists(ROBOTO_FONT20_FILE)) {
    return false;
  }

  const bool loaded = tryLoadSmoothFont(ROBOTO_FONT20_NAME, ROBOTO_FONT20_FILE);
  if (tft.fontLoaded) {
    tft.unloadFont();
  }
  yield();
  return loaded;
}


enum TrKey : uint8_t {
  TR_TIME_SHORT = 0,
  TR_CALL_SHORT,
  TR_COUNTRY,
  TR_WAITING_SPOTS,
  TR_WEATHER,
  TR_TEMPERATURE,
  TR_HUMIDITY,
  TR_PRESSURE,
  TR_WIND,
  TR_FORECAST_3H,
  TR_FORECAST_TOMORROW,
  TR_NO_DATA,
  TR_ERROR_PREFIX,
  TR_PAGE,
  TR_TFT_CALIBRATION_HINT,
  TR_TFT_CALIBRATE_BTN,
  TR_ROT_90_RIGHT,
  TR_ROT_90_LEFT,
  TR_DISPLAY_SETTINGS,
  TR_BRIGHTNESS,
  TR_THEME_COLOR,
  TR_HOLD_CAL_HINT,
  TR_SAVE,
  TR_DEFAULT,
  TR_CLOSE,
  TR_LANGUAGE,
  TR_KEY_COUNT
};

static const char* TR_PL[TR_KEY_COUNT] = {
  "Czas",
  "Znak",
  "KRAJ",
  "Oczekiwanie na spoty...",
  "POGODA",
  "TEMPERATURA",
  "WILGOTNOŚĆ",
  "CIŚNIENIE",
  "WIATR",
  "Prognoza na 3 godziny:",
  "Prognoza na jutro:",
  "Brak danych",
  "BLAD: ",
  "Strona",
  "Przytrzymaj 5 sek = Kalibracja",
  "TFT Kalibracja",
  "Obrot 90deg w prawo (rot90cw)",
  "Obrot 90deg w lewo (rot90ccw)",
  "USTAWIENIA TFT",
  "JASNOSC:",
  "KOLOR MOTYWU:",
  "Przytrzymaj 3 sek = Kalibracja",
  "ZAPISZ",
  "DOMYSLNE",
  "ZAMKNIJ",
  "JEZYK"
};

static const char* TR_PT[TR_KEY_COUNT] = {
  "Hora",
  "Ind.",
  "PAIS",
  "Aguardando spots...",
  "CLIMA",
  "TEMPERATURA",
  "UMIDADE",
  "PRESS\xC3\x83O",
  "VENTO",
  "Previs\xC3\xA3o para 3 horas:",
  "Previs\xC3\xA3o para amanh\xC3\xA3:",
  "Sem dados",
  "ERRO: ",
  "Pagina",
  "Segure 5 seg = Calibrar",
  "Calibrar TFT",
  "Girar 90 deg p/ direita (rot90cw)",
  "Girar 90 deg p/ esquerda (rot90ccw)",
  "AJUSTES DA TELA",
  "BRILHO:",
  "COR DO TEMA:",
  "Segure 3 seg = Calibrar",
  "SALVAR",
  "PADRAO",
  "FECHAR",
  "IDIOMA"
};

static const char* TR_EN[TR_KEY_COUNT] = {
  "Time",
  "Call",
  "COUNTRY",
  "Waiting for spots...",
  "WEATHER",
  "TEMPERATURE",
  "HUMIDITY",
  "PRESSURE",
  "WIND",
  "Forecast (3 hours):",
  "Forecast for tomorrow:",
  "No data",
  "ERROR: ",
  "Page",
  "Hold 5 sec anywhere = Calibration:",
  "TFT Calibrate",
  "Rotate 90 deg right (rot90cw)",
  "Rotate 90 deg left (rot90ccw)",
  "DISPLAY SETTINGS",
  "BRIGHTNESS:",
  "THEME COLOR:",
  "Hold 3 sec anywhere = Calibration",
  "SAVE",
  "DEFAULT",
  "CLOSE",
  "LANGUAGE"
};

static const char* tr(TrKey key) {
  uint8_t idx = static_cast<uint8_t>(key);
  if (idx >= TR_KEY_COUNT) {
    return "";
  }
  return LANG3(TR_PL[idx], TR_EN[idx], TR_PT[idx]);
}

static const char* tftLangToCode(uint8_t lang) {
  if (lang == TFT_LANG_PT) return "pt";
  return (lang == TFT_LANG_EN) ? "en" : "pl";
}

static uint8_t tftLangFromCode(const String &code) {
  String up = code;
  up.toLowerCase();
  if (up == "en") {
    return TFT_LANG_EN;
  }
  if (up == "pt" || up == "pt-br" || up == "pt_br") {
    return TFT_LANG_PT;
  }
  return TFT_LANG_PL;
}

static const char* tftStyleToCode(uint8_t style) {
  return (style == TFT_STYLE_LIGHT) ? "light" : "dark";
}

static uint8_t tftStyleFromCode(const String &code) {
  String up = code;
  up.toLowerCase();
  if (up == "light" || up == "jasny" || up == "white") {
    return TFT_STYLE_LIGHT;
  }
  return TFT_STYLE_DARK;
}

static const char* dxTableSizeToCode(uint8_t mode) {
  if (mode == DX_TABLE_SIZE_MEDIUM) return "medium";
  return (mode == DX_TABLE_SIZE_ENLARGED) ? "enlarged" : "normal";
}

static uint8_t dxTableSizeFromCode(const String &code) {
  String up = code;
  up.toLowerCase();
  if (up == "enlarged" || up == "large" || up == "big") {
    return DX_TABLE_SIZE_ENLARGED;
  }
  if (up == "medium" || up == "mid") {
    return DX_TABLE_SIZE_MEDIUM;
  }
  return DX_TABLE_SIZE_NORMAL;
}

static bool isDxTableEnlarged() {
  return dxTableSizeMode == DX_TABLE_SIZE_ENLARGED;
}

static bool isDxTableMedium() {
  return dxTableSizeMode == DX_TABLE_SIZE_MEDIUM;
}

// Max rows for the MEDIUM table size (Font 2, 20 px row pitch)
static int getMediumTableMaxRows(ScreenType screenNum) {
  return isTableNavFooterVisible(screenNum) ? 8 : 9;
}

static int getDxTableMaxRows() {
  if (isDxTableMedium()) {
    return getMediumTableMaxRows(SCREEN_DX_CLUSTER);
  }
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_DX_CLUSTER);
  }
  return isTableNavFooterVisible(SCREEN_DX_CLUSTER) ? 6 : 7;
}

static int getPotaTableMaxRows() {
  if (isDxTableMedium()) {
    return getMediumTableMaxRows(SCREEN_POTA_CLUSTER);
  }
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_POTA_CLUSTER);
  }
  return isTableNavFooterVisible(SCREEN_POTA_CLUSTER) ? 6 : 7;
}

static int getHamalertTableMaxRows() {
  if (isDxTableMedium()) {
    return getMediumTableMaxRows(SCREEN_HAMALERT_CLUSTER);
  }
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_HAMALERT_CLUSTER);
  }
  return isTableNavFooterVisible(SCREEN_HAMALERT_CLUSTER) ? 6 : 7;
}

static int getAprsTableMaxRows() {
  if (isDxTableMedium()) {
    return getMediumTableMaxRows(SCREEN_APRS_IS);
  }
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_APRS_IS);
  }
  return isTableNavFooterVisible(SCREEN_APRS_IS) ? 6 : 7;
}

static bool isTftStyleLight() {
  return tftStyleMode == TFT_STYLE_LIGHT;
}

static uint16_t tftHeaderBgColor() {
  return tftUiPalette.accent;
}

static uint16_t tftHeaderTextColor() {
  return isTftStyleLight() ? tftUiPalette.fg : TFT_BLACK;
}

static uint16_t tftTableFreqColor() {
  return isTftStyleLight() ? tftUiPalette.fg : TFT_YELLOW;
}

static uint16_t tftTableModeColor() {
  return isTftStyleLight() ? tftUiPalette.fg : TFT_GREEN;
}

static uint16_t tftTableAccentTextColor() {
  return isTftStyleLight() ? tftUiPalette.fg : menuThemeColor;
}

static uint16_t getAprsCallsignColorForEnlarged(const APRSStation &station) {
  if (isTftStyleLight()) {
    return tftUiPalette.fg;
  }
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "HOUSE") return TFT_YELLOW;
  if (symbolShort == "HUMAN") return 0x3C1F;
  if (symbolShort == "CAR") return TFT_RED;
  if (symbolShort == "HAMCLOCK") return TFT_ORANGE;
  return TFT_WHITE;
}
#endif

// Kolorystyka motywu menu
#define DEFAULT_MENU_THEME_COLOR 0xFD20
#define DEFAULT_MENU_THEME_HUE 20
uint8_t menuThemeHue = DEFAULT_MENU_THEME_HUE;
uint16_t menuThemeColor = DEFAULT_MENU_THEME_COLOR;

static void applyTftUiPalette() {
  tftUiPalette.accent = menuThemeColor;
  if (isTftStyleLight()) {
    tftUiPalette.bg = TFT_WHITE;
    tftUiPalette.fg = TFT_BLACK;
    tftUiPalette.muted = TFT_DARKGREY;
    tftUiPalette.rowAlt = 0xDEFB;  // Jasnoszary dla parzystych wierszy tabel.
  } else {
    tftUiPalette.bg = TFT_BLACK;
    tftUiPalette.fg = TFT_WHITE;
    tftUiPalette.muted = TFT_LIGHTGREY;
    tftUiPalette.rowAlt = TFT_TABLE_ALT_ROW_COLOR;
  }
}

// Filtry ekranu 2 (TFT)
enum Screen2FilterMode {
  FILTER_MODE_ALL = 0,
  FILTER_MODE_CW = 1,
  FILTER_MODE_SSB = 2,
  FILTER_MODE_DIGI = 3
};
const uint8_t FILTER_MODE_BIT_CW = 1 << 0;
const uint8_t FILTER_MODE_BIT_SSB = 1 << 1;
const uint8_t FILTER_MODE_BIT_DIGI = 1 << 2;
uint8_t screen2FilterModeMask = 0;
uint16_t screen2FilterBandMask = 0; // 0 = ALL
const char *SCREEN2_FILTER_BANDS[] = {"ALL", "160m", "80m", "40m", "20m", "17m", "15m", "12m", "10m"};
const int SCREEN2_FILTER_BANDS_COUNT = sizeof(SCREEN2_FILTER_BANDS) / sizeof(SCREEN2_FILTER_BANDS[0]);
const size_t COUNTRY_COL_MAX_LEN = 19;

// Filtry ekranu 7 (POTA)
uint8_t screen7FilterModeMask = 0;
uint16_t screen7FilterBandMask = 0; // 0 = ALL

// Filtry ekranu 8 (HAMALERT)
uint8_t screen8FilterModeMask = 0;
uint16_t screen8FilterBandMask = 0; // 0 = ALL
const char *SCREEN8_FILTER_BANDS[] = {"ALL", "160m", "80m", "40m", "20m", "17m", "15m", "12m", "10m", "VHF", "UHF", "SHF"};
const int SCREEN8_FILTER_BANDS_COUNT = sizeof(SCREEN8_FILTER_BANDS) / sizeof(SCREEN8_FILTER_BANDS[0]);

// Sortowanie ekranu 6 (APRS)
enum Screen6SortMode {
  APRS_SORT_TIME = 0,
  APRS_SORT_CALLSIGN = 1,
  APRS_SORT_DISTANCE = 2
};

enum Screen6ViewMode : uint8_t {
  APRS_VIEW_LIST = 0,
  APRS_VIEW_RADAR = 1
};

Screen6SortMode screen6SortMode = APRS_SORT_TIME;
Screen6ViewMode screen6ViewMode = APRS_VIEW_LIST;
bool screen6MenuBeaconingTemp = true;
bool screen6MenuAprsAlertTemp = true;
bool screen6MenuRangeAlertTemp = true;
bool screen6MenuLedAlertTemp = true;

const int SCREEN6_VIEW_BTN_ICON_X = 294;
const int SCREEN6_VIEW_BTN_ICON_Y = 7;
const int SCREEN6_VIEW_BTN_HIT_X = 276;
const int SCREEN6_VIEW_BTN_HIT_Y = 0;
const int SCREEN6_VIEW_BTN_HIT_W = 44;
const int SCREEN6_VIEW_BTN_HIT_H = 50;
const float SCREEN6_RADAR_ZOOM_MIN = 1.00f;
const float SCREEN6_RADAR_ZOOM_MAX = 3.00f;
const float SCREEN6_RADAR_ZOOM_STEP = 0.25f;
const unsigned long SCREEN6_RADAR_ZOOM_TAP_COOLDOWN_MS = 220UL;
float screen6RadarZoom = 1.00f;
unsigned long screen6RadarLastZoomTapMs = 0;
const unsigned long SCREEN6_RADAR_HINT_DURATION_MS = 3000UL;
unsigned long screen6RadarHintUntilMs = 0;

// Menu jasnosci (wywolywane dlugim przytrzymaniem)
bool brightnessMenuActive = false;
int brightnessMenuValue = 100;
ScreenType brightnessMenuPrevScreen = SCREEN_HAM_CLOCK;
bool brightnessMenuPrevInMenu = false;
uint8_t brightnessMenuPrevThemeHue = DEFAULT_MENU_THEME_HUE;
uint8_t brightnessMenuPrevStyleMode = TFT_STYLE_DARK;
int brightnessMenuPrevBacklight = TFT_BACKLIGHT;
unsigned long brightnessMenuOpenedMs = 0;
unsigned long brightnessMenuTouchStartMs = 0;
bool brightnessMenuLongPressHandled = false;

bool touchCalActive = false;
uint8_t touchCalStep = 0;
int16_t touchCalRawX1 = 0;
int16_t touchCalRawY1 = 0;
int16_t touchCalRawX2 = 0;
int16_t touchCalRawY2 = 0;
int16_t touchCalRawX3 = 0;
int16_t touchCalRawY3 = 0;
int16_t touchCalRawX4 = 0;
int16_t touchCalRawY4 = 0;
int touchCalNewXMin = TOUCH_X_MIN;
int touchCalNewXMax = TOUCH_X_MAX;
int touchCalNewYMin = TOUCH_Y_MIN;
int touchCalNewYMax = TOUCH_Y_MAX;

// ========== KONFIGURACJA ==========
#define AP_SSID "ESP32-HAM-CLOCK"
#define AP_PASSWORD "1234567890"
#define DEFAULT_CLUSTER_HOST "dxspots.com"
#define DEFAULT_CLUSTER_PORT 7300
#define DEFAULT_POTA_CLUSTER_HOST ""
#define DEFAULT_POTA_CLUSTER_PORT 7300
#define DEFAULT_POTA_FILTER_COMMAND "accept/spot comment POTA"
#define DEFAULT_POTA_API_URL "https://api.pota.app/v1/spots"
#define DEFAULT_HAMALERT_HOST "hamalert.org"
#define DEFAULT_HAMALERT_PORT 7300
#define NTP_SERVER "pool.ntp.org"
#define MAX_SPOTS 50  // Bufor 50 ostatnich spotów
#define MAX_POTA_SPOTS 30  // Bufor 30 ostatnich spotów (TFT pokaże max 10)
#define GMT_OFFSET_SEC 0  // UTC
#define DEFAULT_TIMEZONE_HOURS 0.0f
#define DEFAULT_CALLSIGN "SWL"
#define DEFAULT_OPENWEBRX_URL "http://okno.ddns.net:8078"
#define PROPAGATION_URL "https://www.hamqsl.com/solarxml.php"
const unsigned long PROPAGATION_FETCH_INTERVAL_MS = 60UL * 60UL * 1000UL; // 60 min
const unsigned long PROPAGATION_FETCH_RETRY_MS = 5UL * 60UL * 1000UL;      // 5 min retry on error
const unsigned long WEATHER_FETCH_INTERVAL_MS = 10UL * 60UL * 1000UL;      // 10 min
const unsigned long WEATHER_FETCH_RETRY_MS = 2UL * 60UL * 1000UL;          // 2 min retry on error
const unsigned long NETWORK_SERVICES_START_DELAY_MS = 5000UL;               // 5s opóźnienia po WiFi dla stabilizacji LWIP
const unsigned long QRZ_LOOKUP_INTERVAL_DEFAULT_MS = 3000;                 // 3s bazowy interwał lookupów
const unsigned long QRZ_LOOKUP_INTERVAL_DX_MS = 2000;                      // 2s na ekranie DX (tak samo jak POTA)
const unsigned long QRZ_LOOKUP_INTERVAL_POTA_MS = 2000;                    // 2s na ekranie POTA
const unsigned long QRZ_RETRY_DELAY_MS = 5000;                             // 5s retry
const unsigned long QRZ_CACHE_TTL_MS = 15UL * 60UL * 1000UL;               // 15 min cache wpisu
const uint8_t QRZ_RETRY_LIMIT = 2;
const int QRZ_QUEUE_SIZE = 20;

// ========== STRUKTURA DANYCH ==========
struct DXSpot {
  String time;        // Czas UTC
  String spotter;     // Stacja zgłaszająca
  String callsign;    // Znak wywoławczy
  float frequency;    // Częstotliwość (kHz - jak w DX Cluster)
  String comment;     // Komentarz
  float distance;     // Odległość (km)
  String country;     // Kraj (z QRZ, jeśli dostępny)
  String locator;     // Maidenhead Locator (jeśli dostępny)
  float lat;          // Szerokosc geo (jesli znana)
  float lon;          // Dlugosc geo (jesli znana)
  bool hasLatLon;     // Czy lat/lon jest znane
  String band;        // Pasmo (160m, 80m, 40m, etc.)
  String mode;        // Modulacja (CW, SSB, FT8/FT4)
};

// Struktura dla stacji APRS
struct APRSStation {
  String time;        // Czas UTC (timestamp)
  String callsign;    // Znak wywoławczy nadawcy
  String symbol;      // Symbol APRS (raw)
  String symbolTable; // Table symbol (znak przed /)
  float lat;          // Szerokość geograficzna
  float lon;          // Długość geograficzna
  String comment;     // Komentarz
  float freqMHz;      // Częstotliwość z komentarza (MHz)
  float distance;     // Odległość w km (Haversine)
  bool hasLatLon;     // Czy pozycja jest znana
};

// ========== ZMIENNE GLOBALNE ==========
WebServer* server = nullptr;
Preferences* preferences = nullptr;
WiFiClient telnetClient;
WiFiClient potaTelnetClient;
WiFiClient aprsClient;  // Klient dla APRS-IS

struct PropagationData {
  String sfi;
  String kindex;
  String aindex;
  String muf;
  String updated;
  String hfBandLabel[4];
  String hfBandFreq[4];
  String hfBandDay[4];
  String hfBandNight[4];
  bool valid = false;
  String lastError = "";
  unsigned long fetchedAtMs = 0;
};

PropagationData propagationData;
unsigned long lastPropagationFetchMs = 0;
bool lastPropagationFetchOk = true;

struct WeatherData {
  static const uint8_t DETAIL_COLS = 5;
  String cityName;
  String description;
  String iconCode;  // OWM icon code (e.g., 01d/01n) to detect day/night
  int weatherId = 800; // OpenWeatherMap condition ID (default clear sky)
  float tempC = 0.0f;
  int humidity = 0;
  int pressure = 0;
  float windMs = 0.0f;
  float pm25 = 0.0f;  // PM2.5 w µg/m³
  float pm10 = 0.0f;  // PM10 w µg/m³
  // Prognozy
  float forecast3hTempC = 0.0f;
  float forecast3hWindMs = 0.0f;
  String forecast3hDesc;
  bool forecast3hValid = false;
  float forecastNextDayTempC = 0.0f;
  float forecastNextDayWindMs = 0.0f;
  String forecastNextDayDesc;
  bool forecastNextDayValid = false;
  float detailTempC[DETAIL_COLS] = {0, 0, 0, 0, 0};
  int detailHumidity[DETAIL_COLS] = {0, 0, 0, 0, 0};
  float detailWindMs[DETAIL_COLS] = {0, 0, 0, 0, 0};
  int detailWeatherId[DETAIL_COLS] = {800, 800, 800, 800, 800};
  String detailIconCode[DETAIL_COLS];
  bool detailValid[DETAIL_COLS] = {false, false, false, false, false};
  float nightTempC[2] = {0.0f, 0.0f};
  bool nightTempValid[2] = {false, false};
  String forecastError;
  String updated;
  bool valid = false;
  String lastError = "";
  unsigned long fetchedAtMs = 0;
};

WeatherData weatherData;
unsigned long lastWeatherFetchMs = 0;
bool lastWeatherFetchOk = true;

struct PendingQrzLookup {
  String callsign;
  unsigned long nextTryMs = 0;
  uint8_t attempts = 0;
};

PendingQrzLookup qrzQueue[QRZ_QUEUE_SIZE];
int qrzQueueLen = 0;
unsigned long lastQrzLookupMs = 0;

// Zwraca interwał dla kolejki QRZ zależnie od aktywnego ekranu
unsigned long getQrzLookupIntervalMs() {
#ifdef ENABLE_TFT_DISPLAY
  if (tftInitialized && !inMenu) {
    if (currentScreen == SCREEN_DX_CLUSTER) {
      return QRZ_LOOKUP_INTERVAL_DX_MS;
    }
    if (currentScreen == SCREEN_POTA_CLUSTER) {
      return QRZ_LOOKUP_INTERVAL_POTA_MS;
    }
  }
#endif
  return QRZ_LOOKUP_INTERVAL_DEFAULT_MS;
}

DXSpot spots[MAX_SPOTS];
int spotCount = 0;
DXSpot potaSpots[MAX_POTA_SPOTS];
int potaSpotCount = 0;
DXSpot hamalertSpots[MAX_POTA_SPOTS];
int hamalertSpotCount = 0;
SemaphoreHandle_t dxSpotsMutex = nullptr;
SemaphoreHandle_t potaSpotsMutex = nullptr;
SemaphoreHandle_t hamalertSpotsMutex = nullptr;
SemaphoreHandle_t weatherDataMutex = nullptr;

const size_t DX_TIME_MAX_LEN = 24;
const size_t DX_CALLSIGN_MAX_LEN = 16;
const size_t DX_SPOTTER_MAX_LEN = 16;
const size_t DX_COMMENT_MAX_LEN = 96;
const size_t DX_COUNTRY_MAX_LEN = 28;
const size_t DX_LOCATOR_MAX_LEN = 8;
const size_t DX_BAND_MAX_LEN = 12;
const size_t DX_MODE_MAX_LEN = 8;

const size_t APRS_TIME_MAX_LEN = 24;
const size_t APRS_CALLSIGN_MAX_LEN = 16;
const size_t APRS_SYMBOL_MAX_LEN = 12;
const size_t APRS_COMMENT_MAX_LEN = 96;

static inline void clampStringLength(String &value, size_t maxLen) {
  if (value.length() > maxLen) {
    value.remove(maxLen);
  }
}

static void compactDxSpotStrings(DXSpot &spot) {
  clampStringLength(spot.time, DX_TIME_MAX_LEN);
  clampStringLength(spot.callsign, DX_CALLSIGN_MAX_LEN);
  clampStringLength(spot.spotter, DX_SPOTTER_MAX_LEN);
  clampStringLength(spot.comment, DX_COMMENT_MAX_LEN);
  clampStringLength(spot.country, DX_COUNTRY_MAX_LEN);
  clampStringLength(spot.locator, DX_LOCATOR_MAX_LEN);
  clampStringLength(spot.band, DX_BAND_MAX_LEN);
  clampStringLength(spot.mode, DX_MODE_MAX_LEN);
}

static void compactAprsStationStrings(APRSStation &station) {
  clampStringLength(station.time, APRS_TIME_MAX_LEN);
  clampStringLength(station.callsign, APRS_CALLSIGN_MAX_LEN);
  clampStringLength(station.symbol, APRS_SYMBOL_MAX_LEN);
  clampStringLength(station.symbolTable, APRS_SYMBOL_MAX_LEN);
  clampStringLength(station.comment, APRS_COMMENT_MAX_LEN);
}

static inline void lockDxSpots() {
  if (dxSpotsMutex != nullptr) {
    xSemaphoreTake(dxSpotsMutex, portMAX_DELAY);
  }
}

static inline void unlockDxSpots() {
  if (dxSpotsMutex != nullptr) {
    xSemaphoreGive(dxSpotsMutex);
  }
}

static inline void lockPotaSpots() {
  if (potaSpotsMutex != nullptr) {
    xSemaphoreTake(potaSpotsMutex, portMAX_DELAY);
  }
}

static inline void unlockPotaSpots() {
  if (potaSpotsMutex != nullptr) {
    xSemaphoreGive(potaSpotsMutex);
  }
}

static inline void lockHamalertSpots() {
  if (hamalertSpotsMutex != nullptr) {
    xSemaphoreTake(hamalertSpotsMutex, portMAX_DELAY);
  }
}

static inline void unlockHamalertSpots() {
  if (hamalertSpotsMutex != nullptr) {
    xSemaphoreGive(hamalertSpotsMutex);
  }
}

static inline void lockWeatherData() {
  if (weatherDataMutex != nullptr) {
    xSemaphoreTake(weatherDataMutex, portMAX_DELAY);
  }
}

static inline void unlockWeatherData() {
  if (weatherDataMutex != nullptr) {
    xSemaphoreGive(weatherDataMutex);
  }
}

// Formatuje czas spotu do postaci "HH:MM" niezależnie od wejściowego formatu
String formatSpotUtc(String raw) {
  raw.trim();

  // Obsługa ISO 8601 (np. 2024-12-12T12:34Z)
  int tPos = raw.indexOf('T');
  if (tPos >= 0 && (tPos + 5) <= (int)raw.length()) {
    raw = raw.substring(tPos + 1);
  }

  // Usuń ewentualne końcowe "Z"
  if (raw.endsWith("Z") || raw.endsWith("z")) {
    raw.remove(raw.length() - 1);
  }

  // Jeżeli już jest dwukropek, przytnij do HH:MM
  if (raw.length() >= 5 && raw.charAt(2) == ':') {
    return raw.substring(0, 5);
  }

  // Brak dwukropka: próbuj wstawić między HH a MM (np. "1234" -> "12:34")
  if (raw.length() >= 4) {
    return raw.substring(0, 2) + ":" + raw.substring(2, 4);
  }

  // Awaryjnie: tylko godzina -> dodaj ":00"
  if (raw.length() >= 2) {
    return raw.substring(0, 2) + ":00";
  }

  return raw;
}

// POTA API (HTTP)
const unsigned long POTA_API_FETCH_INTERVAL_MS = 180UL * 1000UL; // 180s
unsigned long lastPotaApiFetchMs = 0;

// HAMALERT Telnet
const unsigned long HAMALERT_FETCH_INTERVAL_MS = 60UL * 1000UL; // 60s
unsigned long lastHamalertFetchMs = 0;

// APRS-IS konfiguracja (domyślne wartości)
#define DEFAULT_APRS_IS_HOST "rotate.aprs2.net"
#define DEFAULT_APRS_IS_PORT 14580
#define DEFAULT_APRS_CALLSIGN "nocall"
#define DEFAULT_APRS_PASSCODE 00000
#define DEFAULT_APRS_SSID 0
#define DEFAULT_APRS_FILTER_RADIUS 50  // Promień w km (domyślnie 50, zakres 1-50)
#define MAX_APRS_STATIONS 30  // Maksymalna liczba stacji do wyświetlenia (bufor dla WWW)
#define MAX_APRS_DISPLAY_LCD 11  // Maksymalna liczba stacji na ekranie LCD

// Zmienne konfiguracyjne APRS-IS
String aprsIsHost = DEFAULT_APRS_IS_HOST;
int aprsIsPort = DEFAULT_APRS_IS_PORT;
String aprsCallsign = DEFAULT_APRS_CALLSIGN;
int aprsPasscode = DEFAULT_APRS_PASSCODE;
int aprsSsid = DEFAULT_APRS_SSID;
int aprsFilterRadius = DEFAULT_APRS_FILTER_RADIUS;  // PromieĹ„ w km (0-30)
// Uwaga: APRS uĹĽywa wspĂłĹ‚rzÄ™dnych z sekcji "Moja Stacja" (userLat, userLon)

const unsigned long APRS_POSITION_FIRST_DELAY_MS = 60UL * 1000UL; // pierwszy beacon po 1 minucie
const unsigned long APRS_FILTER_REFRESH_MIN_INTERVAL_MS = 30000UL; // nie odswiezaj filtra czesciej niz co 30s
const float APRS_FILTER_REFRESH_MOVE_KM = 1.0f; // odswiez filtr po przesunieciu pozycji o >= 1 km
const int DEFAULT_APRS_INTERVAL_MIN = 29; // kolejne co 29 minut (domyślnie)
const char *NVS_KEY_APRS_INTERVAL_MIN = "aprs_int_min";
const int DEFAULT_APRS_ALERT_MIN_SEC = 300;
const char *NVS_KEY_APRS_ALERT_MIN_SEC = "aprs_alrt_sec";
const int DEFAULT_APRS_ALERT_SCREEN_SEC = 5;
const char *NVS_KEY_APRS_ALERT_SCREEN_SEC = "alrt_scr_s";
const float DEFAULT_APRS_ALERT_DISTANCE_KM = 1.0f;
const char *NVS_KEY_APRS_ALERT_DISTANCE_KM = "alrt_dst_km";
const char *NVS_KEY_APRS_ALERT_WX_ENABLED = "alrt_wx_en";
const char *NVS_KEY_WEATHER_ALERT_STORM_ENABLED = "wx_al_st";
const char *NVS_KEY_WEATHER_ALERT_WIND_ENABLED = "wx_al_wi";
const char *NVS_KEY_WEATHER_ALERT_WIND_THRESHOLD_MS = "wx_w_thr";
const char *NVS_KEY_TFT_NIGHT_DIMMING = "tft_ndim";
const char *NVS_KEY_USE_GPS_IF_ACTIVE = "gps_auto_pos";
const bool DEFAULT_WEATHER_ALERT_STORM_ENABLED = false;
const bool DEFAULT_WEATHER_ALERT_WIND_ENABLED = false;
const float DEFAULT_WEATHER_ALERT_WIND_THRESHOLD_MS = 12.0f;
const float WEATHER_ALERT_WIND_THRESHOLD_MIN_MS = 1.0f;
const float WEATHER_ALERT_WIND_THRESHOLD_MAX_MS = 60.0f;
const unsigned long WEATHER_ALERT_WIND_REPEAT_MS = 1UL * 60UL * 1000UL;
const unsigned long WEATHER_ALERT_CURRENT_DATA_MAX_AGE_MS = 30UL * 60UL * 1000UL;
const bool WEATHER_ALERT_STORM_TEST_MODE = false; // TEMP: wymusza alert burzy co 1 minute
const bool WEATHER_ALERT_WIND_FORCE_ENABLED = false; // TEMP: wymusza wlaczenie alertu o silnym wietrze
const bool DEFAULT_ENABLE_LED_ALERT = true;
const int DEFAULT_LED_ALERT_DURATION_MS = 5000;
const int DEFAULT_LED_ALERT_BLINK_MS = 500;
const char *NVS_KEY_ENABLE_LED_ALERT = "led_al_en";
const char *NVS_KEY_LED_ALERT_DURATION_MS = "led_al_dur";
const char *NVS_KEY_LED_ALERT_BLINK_MS = "led_al_blk";
unsigned long aprsPositionIntervalMs = (unsigned long)DEFAULT_APRS_INTERVAL_MIN * 60UL * 1000UL;
unsigned long lastAPRSPositionTxMs = 0;
unsigned long nextAPRSPositionDueMs = 0;
unsigned long aprsFilterLastSentMs = 0;
double aprsFilterLastLat = 0.0;
double aprsFilterLastLon = 0.0;
bool aprsFilterLastValid = false;
bool aprsFilterLastGpsBased = false;
int aprsIntervalMinutes = DEFAULT_APRS_INTERVAL_MIN;
const char DEFAULT_APRS_SYMBOL_TABLE = '/';
const char DEFAULT_APRS_SYMBOL_CODE = 'V';
String aprsSymbolTwoChar = "/V";
char aprsSymbolTable = DEFAULT_APRS_SYMBOL_TABLE;
char aprsSymbolCode = DEFAULT_APRS_SYMBOL_CODE;
String aprsUserComment = "";
String aprsAlertCsv = "";
bool aprsAlertEnabled = true;
int aprsAlertMinSeconds = DEFAULT_APRS_ALERT_MIN_SEC;
int aprsAlertScreenSeconds = DEFAULT_APRS_ALERT_SCREEN_SEC;
bool aprsAlertNearbyEnabled = true;
bool aprsAlertWxEnabled = false;
float aprsAlertDistanceKm = DEFAULT_APRS_ALERT_DISTANCE_KM;
bool weatherAlertStormEnabled = DEFAULT_WEATHER_ALERT_STORM_ENABLED;
bool weatherAlertWindEnabled = DEFAULT_WEATHER_ALERT_WIND_ENABLED;
float weatherAlertWindThresholdMs = DEFAULT_WEATHER_ALERT_WIND_THRESHOLD_MS;
bool enableLedAlert = DEFAULT_ENABLE_LED_ALERT;
int ledAlertDurationMs = DEFAULT_LED_ALERT_DURATION_MS;
int ledAlertBlinkMs = DEFAULT_LED_ALERT_BLINK_MS;
int aprsBeaconTxCount = 0;
const char *APRS_POSITION_COMMENT = "https://github.com/SP3KON/ESP32-HAM-CLOCK";
const char *APRS_KISS_RF_PATH1 = "WIDE1-1";
const char *APRS_KISS_RF_PATH2 = "WIDE2-1";
bool aprsBeaconEnabled = true;
const unsigned long APRS_ALERT_FRAME_PULSE_MS = 500UL;
const unsigned long APRS_BEACONING_SCREEN_MS = 2000UL;
const int APRS_ALERT_CLOSE_BTN_X = 286;
const int APRS_ALERT_CLOSE_BTN_Y = 7;
const int APRS_ALERT_CLOSE_BTN_W = 26;
const int APRS_ALERT_CLOSE_BTN_H = 26;
const int APRS_ALERT_CLOSE_HIT_W = 50;
const int APRS_ALERT_CLOSE_HIT_H = 50;

static void applyWeatherAlertWindThresholdMs(float thresholdMs) {
  float t = thresholdMs;
  if (t < WEATHER_ALERT_WIND_THRESHOLD_MIN_MS) t = WEATHER_ALERT_WIND_THRESHOLD_MIN_MS;
  if (t > WEATHER_ALERT_WIND_THRESHOLD_MAX_MS) t = WEATHER_ALERT_WIND_THRESHOLD_MAX_MS;
  weatherAlertWindThresholdMs = t;
}

static float weatherAlertWindGustThresholdMs() {
  return weatherAlertWindThresholdMs + 5.0f;
}

enum AlertScreenKind {
  ALERT_SCREEN_NONE = 0,
  ALERT_SCREEN_APRS = 1,
  ALERT_SCREEN_WEATHER = 2,
  ALERT_SCREEN_BEACONING = 3
};

struct WeatherAlertInfo {
  bool stormPossible = false;
  bool stormNow = false;
  bool windPossible = false;
  uint8_t stormEtaHours = 0;
  float maxWindMs = 0.0f;
  float maxWindGustMs = 0.0f;
};

bool aprsAlertScreenActive = false;
unsigned long aprsAlertScreenUntilMs = 0;
unsigned long aprsAlertFrameLastToggleMs = 0;
bool aprsAlertFramePulseOn = true;
AlertScreenKind activeAlertScreenKind = ALERT_SCREEN_NONE;
APRSStation aprsAlertScreenStation;
volatile bool aprsAlertDrawPending = false;
APRSStation aprsAlertPendingStation;
portMUX_TYPE aprsAlertPendingMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool weatherAlertDrawPending = false;
WeatherAlertInfo weatherAlertPendingInfo;
portMUX_TYPE weatherAlertPendingMux = portMUX_INITIALIZER_UNLOCKED;
WeatherAlertInfo weatherAlertScreenInfo;
bool weatherStormForecastActive = false;
bool weatherWindForecastActive = false;
int weatherStormEtaMinutes = 0;
unsigned long weatherAlertForecastUpdatedMs = 0;
float weatherWindForecastMaxMs = 0.0f;
float weatherWindForecastGustMs = 0.0f;
unsigned long nextWeatherStormAlertMs = 0;
unsigned long nextWeatherWindAlertMs = 0;

struct AprsAlertCooldownEntry {
  String callsign;
  unsigned long lastAlertMs = 0;
};

AprsAlertCooldownEntry aprsAlertCooldown[MAX_APRS_STATIONS];
int aprsAlertCooldownReplaceIdx = 0;

APRSStation aprsStations[MAX_APRS_STATIONS];
int aprsStationCount = 0;
bool aprsConnected = false;
bool aprsLoginSent = false;
unsigned long lastAPRSAttempt = 0;
unsigned long lastAPRSRxMs = 0;
String aprsBuffer = "";
const unsigned long APRS_INACTIVITY_RECONNECT_MS = 5UL * 60UL * 1000UL; // 5 minut

// Konfiguracja WiFi i Cluster
String wifiSSID = "";
String wifiPassword = "";
String wifiSSID2 = "";
String wifiPassword2 = "";
String clusterHost = DEFAULT_CLUSTER_HOST;
int clusterPort = DEFAULT_CLUSTER_PORT;
String potaClusterHost = DEFAULT_POTA_CLUSTER_HOST;
int potaClusterPort = DEFAULT_POTA_CLUSTER_PORT;
String potaFilterCommand = DEFAULT_POTA_FILTER_COMMAND;
String potaApiUrl = DEFAULT_POTA_API_URL;
String hamalertHost = DEFAULT_HAMALERT_HOST;
int hamalertPort = DEFAULT_HAMALERT_PORT;
String hamalertLogin = "";
String hamalertPassword = "";
String userCallsign = "";
String userLocator = "";
double userLat = 0.0;
double userLon = 0.0;
bool userLatLonValid = false;
bool useGpsIfActive = false;

// GPS ATGM336H (UART)
const int GPS_UART_TX_PIN = 22;   // IO22 -> TX ESP32 -> RX GPS
const int GPS_UART_RX_PIN = 27;   // IO27 -> RX ESP32 <- TX GPS
const uint32_t GPS_UART_BAUD = 9600;
const char *GPS_UART_ACTIVATE_COMMAND = "GPS ON";
const char *UART2_APRS_TNC_KISS_ON_COMMAND = "KISS ON";
const char *UART2_APRS_TNC_RESTART_COMMAND = "RESTART";
const char *UART2_BRIDGE_BTSPP_ON_COMMAND = "BTSPP_ON";
const char *UART2_BRIDGE_BTSPP_OFF_COMMAND = "BTSPP_OFF";
const char *UART2_BRIDGE_BT_SCAN_COMMAND = "BT SCAN";
const char *NVS_KEY_UART2_MODE = "uart2_mode";
const char *NVS_KEY_UART2_CONNECTED_MODULE = "uart2_cmod";
const char *NVS_KEY_UART2_BAUD = "uart2_baud";
const char *NVS_KEY_UART2_DATABITS = "uart2_dbits";
const char *NVS_KEY_UART2_PARITY = "uart2_par";
const char *NVS_KEY_UART2_STOP_BITS = "uart2_stop";
const char *NVS_KEY_BRIDGE_PAIR_ADDR = "br_pair_addr";
const char *NVS_KEY_BRIDGE_PAIR_NAME = "br_pair_name";
const uint32_t UART2_BAUD_MIN = 300;
const uint32_t UART2_BAUD_MAX = 2000000;
const uint8_t DEFAULT_UART2_MODE = 0;      // 0=gps, 1=bt, 2=off
const uint32_t DEFAULT_UART2_BAUD = GPS_UART_BAUD;
const uint8_t DEFAULT_UART2_DATA_BITS = 8;
const uint8_t DEFAULT_UART2_PARITY = 0;    // 0=none, 1=even, 2=odd
const uint8_t DEFAULT_UART2_STOP_BITS = 1;
const uint8_t UART2_MODE_GPS = 0;
const uint8_t UART2_MODE_BT = 1;
const uint8_t UART2_MODE_OFF = 2;
const uint8_t UART2_PARITY_NONE = 0;
const uint8_t UART2_PARITY_EVEN = 1;
const uint8_t UART2_PARITY_ODD = 2;
uint8_t uart2Mode = DEFAULT_UART2_MODE;
String uart2ConnectedModule = "none"; // none, gps, aprs_tnc, aprs_tnc_kiss, esp32_bridge
uint32_t uart2BaudRate = DEFAULT_UART2_BAUD;
uint8_t uart2DataBits = DEFAULT_UART2_DATA_BITS;
uint8_t uart2Parity = DEFAULT_UART2_PARITY;
uint8_t uart2StopBits = DEFAULT_UART2_STOP_BITS;
const unsigned long GPS_STATUS_LOG_INTERVAL_MS = 60000UL;
const unsigned long GPS_FIX_FRESHNESS_MS = 15000UL;
const unsigned long GPS_UART_ACTIVE_TIMEOUT_MS = 3000UL;
const unsigned long GPS_RAW_BRIDGE_WINDOW_MS = 60000UL;
const unsigned long GPS_RAW_STATS_INTERVAL_MS = 5000UL;
const unsigned long GPS_GSV_SOURCE_FRESHNESS_MS = 10000UL;

static void processAprsFrameFromLine(const String &line, const char *sourceTag);
bool parseAPRSFrame(String line, APRSStation &station);
void addAPRSStation(APRSStation station);
static bool shouldTriggerAprsAlert(const APRSStation &station);
static bool isLikelyAprsTnc2Frame(const String &line);

const int MAX_BRIDGE_BT_DEVICES = 24;
const unsigned long BRIDGE_UART_COMMAND_COOLDOWN_MS = 250UL;
const unsigned long BRIDGE_BTSPP_ON_RETRY_INTERVAL_MS = 2000UL;
const uint8_t BRIDGE_BTSPP_ON_MAX_TRIES = 3;
const unsigned long BRIDGE_FORCE_GPS_ON_AFTER_BOOT_MS = 60000UL;
const unsigned long BRIDGE_FORCE_BTSPP_ON_DELAY_AFTER_GPS_ON_MS = 10000UL;
const unsigned long APRS_TNC_RESTART_AFTER_BOOT_MS = 30000UL;
const unsigned long APRS_TNC_KISS_RESTART_DELAY_AFTER_KISS_ON_MS = 1000UL;

struct BridgeBtDevice {
  String name;
  String addressNoColon;
};

HardwareSerial gpsSerial(2);
String gpsNmeaLine = "";
double gpsLatitude = 0.0;
double gpsLongitude = 0.0;
bool gpsLocationValid = false;
bool gpsRmcFix = false;
bool gpsGgaFix = false;
int gpsSatellitesUsed = -1;
int gpsSatellitesInView = -1;
int gpsSatellitesInViewGN = -1;
int gpsSatellitesInViewGP = -1;
int gpsSatellitesInViewGL = -1;
int gpsSatellitesInViewGA = -1;
int gpsSatellitesInViewGB = -1;
int gpsSatellitesInViewBD = -1;
int gpsSatellitesInViewGQ = -1;
unsigned long gpsSatellitesInViewLastGNMs = 0;
unsigned long gpsSatellitesInViewLastGPMS = 0;
unsigned long gpsSatellitesInViewLastGLMs = 0;
unsigned long gpsSatellitesInViewLastGAMs = 0;
unsigned long gpsSatellitesInViewLastGBMs = 0;
unsigned long gpsSatellitesInViewLastBDMs = 0;
unsigned long gpsSatellitesInViewLastGQMs = 0;
float gpsSpeedKmh = -1.0f;
unsigned long gpsLastRmcMs = 0;
unsigned long gpsLastGgaMs = 0;
unsigned long gpsLastStatusLogMs = 0;
unsigned long gpsLastRxMs = 0;
unsigned long gpsRawBridgeStartMs = 0;
bool gpsRawBridgeEndLogged = false;
unsigned long gpsRawLastStatsMs = 0;
uint32_t gpsRawBytesTotal = 0;
uint32_t gpsRawBytesSinceStats = 0;
String gpsUtcTime = "";
String gpsUtcDate = "";
String uart2AprsLine = "";
String uart2YaesuUiAprsPrefix = "";
bool uart2KissInFrame = false;
bool uart2KissEscape = false;
uint8_t uart2KissFrame[600];
size_t uart2KissFrameLen = 0;
BridgeBtDevice bridgeBtDevices[MAX_BRIDGE_BT_DEVICES];
int bridgeBtDeviceCount = 0;
bool bridgeBtListCollecting = false;
int bridgeBtListExpected = -1;
bool bridgeBtScanInProgress = false;
bool bridgeBtScanRequested = false;
bool bridgeAwaitBtsppOffAckForScan = false;
String bridgeBtLastError = "";
String bridgeBtLastDiscStatus = "";
String bridgeBtLastAck = "";
unsigned long bridgeBtLastScanMs = 0;
unsigned long bridgeBtLastCommandTxMs = 0;
String bridgeBtPairedAddress = "";
String bridgeBtPairedName = "";
bool bridgeBtAutoConnectPending = false;
bool bridgeBtPairPending = false;
String bridgeBtPairPendingAddress = "";
String bridgeBtPairPendingName = "";
int bridgeBtPairLastResult = 0; // 0=brak, 1=OK, -1=ERR
unsigned long bridgeBtPairLastResultMs = 0;
bool bridgeBtsppOnAwaitAck = false;
uint8_t bridgeBtsppOnRetryCount = 0;
unsigned long bridgeBtsppOnNextRetryMs = 0;
bool bridgeBootForceGpsOnSent = false;
unsigned long bridgeBootForceGpsOnMs = 0;
bool bridgeBootForceBtsppOnSent = false;
bool aprsTncBootRestartSent = false;
bool aprsTncBootKissOnSent = false;
unsigned long aprsTncBootKissOnSentMs = 0;

static String uart2ModeToCode(uint8_t mode) {
  if (mode == UART2_MODE_BT) {
    return "bt";
  }
  if (mode == UART2_MODE_OFF) {
    return "off";
  }
  return "gps";
}

static String sanitizeUart2ConnectedModule(String moduleCode) {
  moduleCode.trim();
  moduleCode.toLowerCase();

  if (moduleCode == "gps") {
    return "gps";
  }
  if (moduleCode == "aprs tnc" || moduleCode == "aprs tnc2 (rx only)" || moduleCode == "aprs_tnc" || moduleCode == "tnc") {
    return "aprs_tnc";
  }
  if (moduleCode == "aprs tnc kiss" || moduleCode == "aprs-tnc-kiss" || moduleCode == "aprs_tnc-kiss" ||
      moduleCode == "aprs_tnc_kiss" || moduleCode == "kiss") {
    return "aprs_tnc_kiss";
  }
  if (moduleCode == "esp32-bridge" || moduleCode == "esp32_bridge" || moduleCode == "bridge") {
    return "esp32_bridge";
  }
  if (moduleCode == "brak" || moduleCode == "none" || moduleCode == "off") {
    return "none";
  }

  return "none";
}

static bool isUart2GpsModuleSelected() {
  return uart2ConnectedModule == "gps";
}

static bool isUart2AprsTncModuleSelected() {
  return uart2ConnectedModule == "aprs_tnc";
}

static bool isUart2AprsTncKissModuleSelected() {
  return uart2ConnectedModule == "aprs_tnc_kiss";
}

static bool isUart2BridgeModuleSelected() {
  return uart2ConnectedModule == "esp32_bridge";
}

static bool isLegacyGpsModeWithoutModule() {
  return (uart2ConnectedModule == "none") && (uart2Mode == UART2_MODE_GPS);
}

static bool isUart2GpsParsingEnabled() {
  return isUart2GpsModuleSelected() || isUart2BridgeModuleSelected() || isLegacyGpsModeWithoutModule();
}

static bool isUart2AprsUartSourceEnabled() {
  return isUart2AprsTncModuleSelected() || isUart2AprsTncKissModuleSelected() || isUart2BridgeModuleSelected();
}

static bool isUart2AprsTextParsingEnabled() {
  if (isUart2AprsTncKissModuleSelected()) {
    return false;
  }
  return isUart2AprsUartSourceEnabled();
}

static bool isUart2AprsKissParsingEnabled() {
  return isUart2AprsTncModuleSelected() || isUart2AprsTncKissModuleSelected();
}

static bool isAprsNetworkFeedEnabled() {
  return !isUart2AprsUartSourceEnabled();
}

static String normalizeBridgeBtAddress(String raw) {
  String cleaned = "";
  cleaned.reserve(12);
  for (size_t i = 0; i < raw.length(); i++) {
    char c = raw.charAt(i);
    bool isDigitChar = (c >= '0' && c <= '9');
    bool isUpperHex = (c >= 'A' && c <= 'F');
    bool isLowerHex = (c >= 'a' && c <= 'f');
    if (isDigitChar || isUpperHex || isLowerHex) {
      cleaned += (char)toupper((unsigned char)c);
      if (cleaned.length() >= 12) {
        break;
      }
    }
  }
  if (cleaned.length() != 12) {
    return "";
  }
  return cleaned;
}

static String formatBridgeBtAddressForDisplay(const String &addrNoColon) {
  String norm = normalizeBridgeBtAddress(addrNoColon);
  if (norm.length() != 12) {
    return "";
  }
  String out = "";
  out.reserve(17);
  for (int i = 0; i < 12; i += 2) {
    if (i > 0) out += ":";
    out += norm.substring(i, i + 2);
  }
  return out;
}

static String sanitizeBridgeTextField(String raw, size_t maxLen) {
  String out = "";
  out.reserve(raw.length());
  for (size_t i = 0; i < raw.length(); i++) {
    char c = raw.charAt(i);
    if (c < 32 || c > 126) continue;
    if (c == '[' || c == ']' || c == '\r' || c == '\n') continue;
    out += c;
    if (out.length() >= maxLen) break;
  }
  out.trim();
  return out;
}

static void clearBridgeBtScanList() {
  for (int i = 0; i < MAX_BRIDGE_BT_DEVICES; i++) {
    bridgeBtDevices[i].name = "";
    bridgeBtDevices[i].addressNoColon = "";
  }
  bridgeBtDeviceCount = 0;
  bridgeBtListExpected = -1;
  bridgeBtListCollecting = false;
}

static bool sendBridgeUartCommandLine(const String &command, bool bypassCooldown = false) {
  if (!isUart2BridgeModuleSelected() || command.length() == 0) {
    return false;
  }

  unsigned long nowMs = millis();
  if (!bypassCooldown && bridgeBtLastCommandTxMs != 0 && (nowMs - bridgeBtLastCommandTxMs) < BRIDGE_UART_COMMAND_COOLDOWN_MS) {
    return false;
  }

  gpsSerial.print(command);
  gpsSerial.print("\r\n");
  Serial.printf("[BRIDGE] TX: %s\\r\\n\n", command.c_str());
  bridgeBtLastCommandTxMs = nowMs;
  return true;
}

static void saveBridgeBtPairingToNvs() {
  Preferences bridgePrefs;
  bridgePrefs.begin("dxcluster", false);
  bridgePrefs.putString(NVS_KEY_BRIDGE_PAIR_ADDR, bridgeBtPairedAddress);
  bridgePrefs.putString(NVS_KEY_BRIDGE_PAIR_NAME, bridgeBtPairedName);
  bridgePrefs.end();
}

static void resetBridgeBtsppOnRetryState() {
  bridgeBtsppOnAwaitAck = false;
  bridgeBtsppOnRetryCount = 0;
  bridgeBtsppOnNextRetryMs = 0;
}

static void triggerBridgeBtsppOnRetryStart() {
  bridgeBtsppOnAwaitAck = true;
  bridgeBtsppOnRetryCount = 0;
  bridgeBtsppOnNextRetryMs = 0;
}

static void serviceBridgeBtsppOnRetry(unsigned long nowMs) {
  if (!bridgeBtsppOnAwaitAck) {
    return;
  }

  if (!isUart2BridgeModuleSelected()) {
    resetBridgeBtsppOnRetryState();
    return;
  }

  if ((long)(nowMs - bridgeBtsppOnNextRetryMs) < 0) {
    return;
  }

  if (bridgeBtsppOnRetryCount >= BRIDGE_BTSPP_ON_MAX_TRIES) {
    bridgeBtsppOnAwaitAck = false;
    bridgeBtLastError = "BTSPP_ON_NO_ACK";
    Serial.println("[BRIDGE] Brak ACK BTSPP_ON po 3 probach");
    return;
  }

  bool sent = sendBridgeUartCommandLine(UART2_BRIDGE_BTSPP_ON_COMMAND, true);
  if (sent) {
    bridgeBtsppOnRetryCount++;
    Serial.printf("[BRIDGE] BTSPP_ON proba %u/%u\n",
                  (unsigned)bridgeBtsppOnRetryCount,
                  (unsigned)BRIDGE_BTSPP_ON_MAX_TRIES);
  } else {
    Serial.println("[BRIDGE] BTSPP_ON send fail, ponowie za 2s");
  }

  bridgeBtsppOnNextRetryMs = nowMs + BRIDGE_BTSPP_ON_RETRY_INTERVAL_MS;
}

static void serviceBridgeBootForceCommands(unsigned long nowMs) {
  if (!isUart2BridgeModuleSelected()) {
    return;
  }

  // Fallback po starcie: po 60 s wymus GPS ON, niezależnie od statusu connect/autoconnect.
  if (!bridgeBootForceGpsOnSent && (long)(nowMs - BRIDGE_FORCE_GPS_ON_AFTER_BOOT_MS) >= 0) {
    if (sendBridgeUartCommandLine(GPS_UART_ACTIVATE_COMMAND, true)) {
      bridgeBootForceGpsOnSent = true;
      bridgeBootForceGpsOnMs = nowMs;
      Serial.println("[BRIDGE] Fallback boot: wysylam GPS ON po 60s");
    }
    return;
  }

  // 10 s po wymuszonym GPS ON wyslij BTSPP_ON bez czekania na odpowiedzi zwrotne.
  if (bridgeBootForceGpsOnSent && !bridgeBootForceBtsppOnSent &&
      (long)(nowMs - (bridgeBootForceGpsOnMs + BRIDGE_FORCE_BTSPP_ON_DELAY_AFTER_GPS_ON_MS)) >= 0) {
    if (sendBridgeUartCommandLine(UART2_BRIDGE_BTSPP_ON_COMMAND, true)) {
      bridgeBootForceBtsppOnSent = true;
      Serial.println("[BRIDGE] Fallback boot: wysylam BTSPP_ON po +10s od GPS ON");
    }
  }
}

static void serviceAprsTncBootRestart(unsigned long nowMs) {
  if (aprsTncBootRestartSent) {
    return;
  }

  if (isUart2AprsTncKissModuleSelected()) {
    // Po 30 s wlacz tryb KISS, a 1 s pozniej zrestartuj modem.
    if (!aprsTncBootKissOnSent) {
      if ((long)(nowMs - APRS_TNC_RESTART_AFTER_BOOT_MS) < 0) {
        return;
      }
      gpsSerial.print(UART2_APRS_TNC_KISS_ON_COMMAND);
      gpsSerial.print("\r\n");
      Serial.printf("[UART2] TX: %s\\r\\n\n", UART2_APRS_TNC_KISS_ON_COMMAND);
      aprsTncBootKissOnSent = true;
      aprsTncBootKissOnSentMs = nowMs;
      return;
    }

    if ((long)(nowMs - (aprsTncBootKissOnSentMs + APRS_TNC_KISS_RESTART_DELAY_AFTER_KISS_ON_MS)) < 0) {
      return;
    }

    gpsSerial.print(UART2_APRS_TNC_RESTART_COMMAND);
    gpsSerial.print("\r\n");
    Serial.printf("[UART2] TX: %s\\r\\n\n", UART2_APRS_TNC_RESTART_COMMAND);
    aprsTncBootRestartSent = true;
    return;
  }

  if (!isUart2AprsTncModuleSelected()) {
    return;
  }

  // Po starcie wymusz restart modemu APRS TNC, aby odswiezyc polaczenie BT.
  if ((long)(nowMs - APRS_TNC_RESTART_AFTER_BOOT_MS) < 0) {
    return;
  }

  gpsSerial.print(UART2_APRS_TNC_RESTART_COMMAND);
  gpsSerial.print("\r\n");
  Serial.printf("[UART2] TX: %s\\r\\n\n", UART2_APRS_TNC_RESTART_COMMAND);
  aprsTncBootRestartSent = true;
}

static void startBridgeScanSequence() {
  if (!isUart2BridgeModuleSelected()) {
    return;
  }

  clearBridgeBtScanList();
  bridgeBtScanInProgress = true;
  bridgeBtScanRequested = true;
  bridgeAwaitBtsppOffAckForScan = true;
  bridgeBtLastError = "";
  bridgeBtLastDiscStatus = "";
  bridgeBtLastAck = "";

  if (!sendBridgeUartCommandLine(UART2_BRIDGE_BTSPP_OFF_COMMAND, true)) {
    bridgeBtLastError = "SEND_BTSPP_OFF_FAIL";
    bridgeBtScanInProgress = false;
    bridgeBtScanRequested = false;
    bridgeAwaitBtsppOffAckForScan = false;
  }
}

static void addOrUpdateBridgeBtDevice(const String &name, const String &addressNoColon) {
  if (addressNoColon.length() != 12) {
    return;
  }

  String safeName = sanitizeBridgeTextField(name, 40);
  if (safeName.length() == 0) {
    safeName = "BT_DEVICE";
  }

  for (int i = 0; i < bridgeBtDeviceCount; i++) {
    if (bridgeBtDevices[i].addressNoColon.equalsIgnoreCase(addressNoColon)) {
      bridgeBtDevices[i].name = safeName;
      return;
    }
  }

  if (bridgeBtDeviceCount >= MAX_BRIDGE_BT_DEVICES) {
    return;
  }

  bridgeBtDevices[bridgeBtDeviceCount].name = safeName;
  bridgeBtDevices[bridgeBtDeviceCount].addressNoColon = addressNoColon;
  bridgeBtDeviceCount++;
}

static bool requestBridgePairConnect(String nameRaw, String addrRaw, String passwordRaw, bool bypassCooldown = false) {
  if (!isUart2BridgeModuleSelected()) {
    return false;
  }

  String addr = normalizeBridgeBtAddress(addrRaw);
  if (addr.length() != 12) {
    bridgeBtLastError = "PAIR_ADDR_INVALID";
    return false;
  }

  String safeName = sanitizeBridgeTextField(nameRaw, 40);
  if (safeName.length() == 0) {
    safeName = "BT_DEVICE";
  }

  String safePassword = sanitizeBridgeTextField(passwordRaw, 16);
  if (safePassword.length() > 16) {
    bridgeBtLastError = "PAIR_PASSWORD_TOO_LONG";
    return false;
  }

  // PIN/hasło jest opcjonalne: nie wymuszaj sztucznego "****", bo to powoduje BTCNT_ERR na części modułów.
  String command = "BTCNT[NAME]" + safeName + "[ADR]" + addr;
  if (safePassword.length() > 0) {
    command += "[PSWD]";
    command += safePassword;
  }
  if (!sendBridgeUartCommandLine(command, bypassCooldown)) {
    bridgeBtLastError = "PAIR_COMMAND_SEND_FAIL";
    return false;
  }

  bridgeBtPairPending = true;
  bridgeBtPairPendingAddress = addr;
  bridgeBtPairPendingName = safeName;
  bridgeBtPairLastResult = 0;
  return true;
}

static uint8_t uart2ModeFromCode(String modeCode) {
  modeCode.trim();
  modeCode.toLowerCase();
  if (modeCode == "bt" || modeCode == "bt_module" || modeCode == "bluetooth") {
    return UART2_MODE_BT;
  }
  if (modeCode == "off" || modeCode == "none" || modeCode == "disabled") {
    return UART2_MODE_OFF;
  }
  return UART2_MODE_GPS;
}

static String uart2ParityToCode(uint8_t parity) {
  if (parity == UART2_PARITY_EVEN) {
    return "even";
  }
  if (parity == UART2_PARITY_ODD) {
    return "odd";
  }
  return "none";
}

static uint8_t uart2ParityFromCode(String parityCode) {
  parityCode.trim();
  parityCode.toLowerCase();
  if (parityCode == "even" || parityCode == "e") {
    return UART2_PARITY_EVEN;
  }
  if (parityCode == "odd" || parityCode == "o") {
    return UART2_PARITY_ODD;
  }
  return UART2_PARITY_NONE;
}

static void applyUart2BaudRate(int candidateBaud) {
  long bounded = candidateBaud;
  if (bounded < (long)UART2_BAUD_MIN) bounded = (long)UART2_BAUD_MIN;
  if (bounded > (long)UART2_BAUD_MAX) bounded = (long)UART2_BAUD_MAX;
  uart2BaudRate = (uint32_t)bounded;
}

static void applyUart2DataBits(int candidateDataBits) {
  uart2DataBits = (candidateDataBits == 7) ? 7 : 8;
}

static void applyUart2StopBits(int candidateStopBits) {
  uart2StopBits = (candidateStopBits == 2) ? 2 : 1;
}

static uint32_t getUart2SerialConfig() {
  if (uart2DataBits == 7) {
    if (uart2Parity == UART2_PARITY_EVEN) {
      return (uart2StopBits == 2) ? SERIAL_7E2 : SERIAL_7E1;
    }
    if (uart2Parity == UART2_PARITY_ODD) {
      return (uart2StopBits == 2) ? SERIAL_7O2 : SERIAL_7O1;
    }
    return (uart2StopBits == 2) ? SERIAL_7N2 : SERIAL_7N1;
  }

  if (uart2Parity == UART2_PARITY_EVEN) {
    return (uart2StopBits == 2) ? SERIAL_8E2 : SERIAL_8E1;
  }
  if (uart2Parity == UART2_PARITY_ODD) {
    return (uart2StopBits == 2) ? SERIAL_8O2 : SERIAL_8O1;
  }
  return (uart2StopBits == 2) ? SERIAL_8N2 : SERIAL_8N1;
}

static void resetGpsRuntimeState() {
  gpsNmeaLine = "";
  gpsLatitude = 0.0;
  gpsLongitude = 0.0;
  gpsLocationValid = false;
  gpsRmcFix = false;
  gpsGgaFix = false;
  gpsSatellitesUsed = -1;
  gpsSatellitesInView = -1;
  gpsSatellitesInViewGN = -1;
  gpsSatellitesInViewGP = -1;
  gpsSatellitesInViewGL = -1;
  gpsSatellitesInViewGA = -1;
  gpsSatellitesInViewGB = -1;
  gpsSatellitesInViewBD = -1;
  gpsSatellitesInViewGQ = -1;
  gpsSatellitesInViewLastGNMs = 0;
  gpsSatellitesInViewLastGPMS = 0;
  gpsSatellitesInViewLastGLMs = 0;
  gpsSatellitesInViewLastGAMs = 0;
  gpsSatellitesInViewLastGBMs = 0;
  gpsSatellitesInViewLastBDMs = 0;
  gpsSatellitesInViewLastGQMs = 0;
  gpsSpeedKmh = -1.0f;
  gpsLastRmcMs = 0;
  gpsLastGgaMs = 0;
  gpsLastStatusLogMs = 0;
  gpsLastRxMs = 0;
  gpsRawBridgeStartMs = millis();
  gpsRawBridgeEndLogged = false;
  gpsRawLastStatsMs = gpsRawBridgeStartMs;
  gpsRawBytesTotal = 0;
  gpsRawBytesSinceStats = 0;
  gpsUtcTime = "";
  gpsUtcDate = "";
  uart2AprsLine = "";
  uart2YaesuUiAprsPrefix = "";
  uart2KissInFrame = false;
  uart2KissEscape = false;
  uart2KissFrameLen = 0;
}

static void configureUart2Port() {
  gpsSerial.end();
  resetGpsRuntimeState();
  resetBridgeBtsppOnRetryState();
  aprsTncBootRestartSent = false;
  aprsTncBootKissOnSent = false;
  aprsTncBootKissOnSentMs = 0;
  bridgeBtScanInProgress = false;
  bridgeBtScanRequested = false;
  bridgeAwaitBtsppOffAckForScan = false;

  uint32_t serialCfg = getUart2SerialConfig();
  gpsSerial.begin(uart2BaudRate, serialCfg, GPS_UART_RX_PIN, GPS_UART_TX_PIN);

  char parityChar = 'N';
  if (uart2Parity == UART2_PARITY_EVEN) parityChar = 'E';
  if (uart2Parity == UART2_PARITY_ODD) parityChar = 'O';
  Serial.printf("[UART2] Start RX=IO%d TX=IO%d BAUD=%lu FORMAT=%u%c%u TRYB=%s\n",
                GPS_UART_RX_PIN,
                GPS_UART_TX_PIN,
                (unsigned long)uart2BaudRate,
                (unsigned)uart2DataBits,
                parityChar,
                (unsigned)uart2StopBits,
                uart2ModeToCode(uart2Mode).c_str());

  if (isUart2GpsParsingEnabled()) {
    // Aktywacja GPS. Dla ATGM336H komenda jest po prostu ignorowana.
    gpsSerial.print(GPS_UART_ACTIVATE_COMMAND);
    gpsSerial.print("\r\n");
    Serial.printf("[UART2] TX: %s\\r\\n\n", GPS_UART_ACTIVATE_COMMAND);

    // Nie uruchamiaj automatycznie skanu BT na starcie.
    // Skan BTSPP ma byc wykonywany tylko recznie z panelu WWW.
    gpsSerial.print(UART2_BRIDGE_BTSPP_OFF_COMMAND);
    gpsSerial.print("\r\n");
    Serial.printf("[UART2] TX: %s\\r\\n\n", UART2_BRIDGE_BTSPP_OFF_COMMAND);

    gpsRawBridgeStartMs = millis();
    gpsRawLastStatsMs = gpsRawBridgeStartMs;
    gpsRawBridgeEndLogged = false;
    Serial.println("[GPS RAW] Start trybu testowego: przez 60 s surowe bajty z GPS beda kopiowane 1:1 na USB Serial");
#ifdef ENABLE_TFT_DISPLAY
    if (TOUCH_MISO == GPS_UART_RX_PIN) {
      Serial.printf("[GPS WARN] MOZLIWY KONFLIKT PINOW: GPS RX=IO%d i TOUCH_MISO=IO%d\n", GPS_UART_RX_PIN, TOUCH_MISO);
    }
#endif
  } else {
    // Gdy GPS parser nie jest aktywny, most BT SPP ma byc wyłączony.
    gpsSerial.print(UART2_BRIDGE_BTSPP_OFF_COMMAND);
    gpsSerial.print("\r\n");
    Serial.printf("[UART2] TX: %s\\r\\n\n", UART2_BRIDGE_BTSPP_OFF_COMMAND);
    gpsRawBridgeEndLogged = true;
    if (isUart2AprsUartSourceEnabled()) {
      Serial.println("[UART2] Nasluch APRS/TNC aktywny (parser GPS wylaczony)");
    } else {
      Serial.println("[UART2] Tryb none: dane z UART2 beda ignorowane");
    }
  }
}

float timezoneHours = DEFAULT_TIMEZONE_HOURS;
String qrzUsername = "";
String qrzPassword = "";
String qrzStatus = "QRZ: not configured";
String weatherApiKey = "";
String openWebRxUrl = DEFAULT_OPENWEBRX_URL;

// Konfiguracja filtrĂłw CC-Cluster (dxspots.com)
bool clusterNoAnnouncements = true;      // set/noann - wyłącz ogłoszenia
bool clusterNoWWV = true;                // set/nowwv - wyłącz WWV
bool clusterNoWCY = true;                // set/nowcy - wyłącz WCY
bool clusterUseFilters = true;            // Czy używać filtrów (set/filter)
String clusterFilterCommands = "set/ft8"; // Domyślna komenda filtra dla nowej konfiguracji
// Preferences/NVS key max length on ESP32 is 15 chars.
const char* NVS_KEY_CLUSTER_USEFILTERS = "cluster_usefl";

// Status poĹ‚Ä…czenia
bool wifiConnected = false;
unsigned long wifiConnectedAtMs = 0;
bool telnetConnected = false;
unsigned long lastTelnetAttempt = 0;
unsigned long lastNTPUpdate = 0;
bool gpsTimeSourceWasActive = false;
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long lastWiFiStaReconnectAttempt = 0;
unsigned long wifiStaRetryWindowStartMs = 0;
unsigned long lastPotaAttempt = 0;
const unsigned long WIFI_AP_RETRY_INTERVAL_MS = 5UL * 60UL * 1000UL; // 5 minut
const unsigned long WIFI_STA_RETRY_INTERVAL_MS = 20UL * 1000UL; // 20 sekund
const unsigned long WIFI_STA_RETRY_WINDOW_MS = 10UL * 60UL * 1000UL; // maks. 10 minut prób w STA
const uint8_t WIFI_STA_RECONNECT_ATTEMPTS = 6; // 6 * 500ms = ~3s na SSID (mniej blokujące)

const uint8_t RGB_LED_RED_PIN = 4;
const uint8_t RGB_LED_GREEN_PIN = 16;
const uint8_t RGB_LED_BLUE_PIN = 17;
const unsigned long RGB_RED_DISCONNECTED_BLINK_MS = 1000UL;
bool rgbLedPrevWifiConnected = false;
unsigned long rgbRedBlinkLastToggleMs = 0;
bool rgbRedBlinkStateOn = false;
bool rgbBlueAprsAlertActive = false;
unsigned long rgbBlueAprsAlertUntilMs = 0;
unsigned long rgbBlueAprsLastToggleMs = 0;
bool rgbBlueAprsStateOn = false;

// Restart (np. po zapisaniu konfiguracji)
bool restartRequested = false;
unsigned long restartAtMs = 0;

// Stan sesji DX Cluster (Telnet)
bool clusterLoginSent = false;
bool clusterLoginScheduled = false;
unsigned long clusterSendLoginAtMs = 0;
unsigned long lastClusterKeepAliveMs = 0;
unsigned long lastTelnetRxMs = 0;
const unsigned long TELNET_INACTIVITY_RECONNECT_MS = 5UL * 60UL * 1000UL; // 5 minut

// Stan sesji POTA Cluster (Telnet)
bool potaTelnetConnected = false;
bool potaLoginSent = false;
bool potaLoginScheduled = false;
unsigned long potaSendLoginAtMs = 0;
unsigned long lastPotaKeepAliveMs = 0;
unsigned long lastPotaRxMs = 0;
const unsigned long POTA_TELNET_INACTIVITY_RECONNECT_MS = 5UL * 60UL * 1000UL; // 5 minut

// Bufor dla danych Telnet
String telnetBuffer = "";
String pendingTelnetLine = "";
unsigned long pendingTelnetDropped = 0;

// Bufor dla danych POTA Telnet
String potaTelnetBuffer = "";
String pendingPotaLine = "";
unsigned long pendingPotaDropped = 0;

// QRZ cache (żeby nie pytaÄ‡ w kółko o te same znaki)
struct QrzCacheEntry {
  String callsign;
  String grid;
  String country;
  float lat;
  float lon;
  bool hasLatLon;
  unsigned long fetchedAtMs;
};
const int QRZ_CACHE_SIZE = 20;
QrzCacheEntry qrzCache[QRZ_CACHE_SIZE];

static bool validateNmeaChecksum(const String &line) {
  if (!line.startsWith("$")) {
    return false;
  }

  int starPos = line.indexOf('*');
  if (starPos < 0) {
    return true;
  }
  if ((starPos + 2) >= (int)line.length()) {
    return false;
  }

  uint8_t calc = 0;
  for (int i = 1; i < starPos; i++) {
    calc ^= (uint8_t)line.charAt(i);
  }

  char expectedHex[3];
  expectedHex[0] = line.charAt(starPos + 1);
  expectedHex[1] = line.charAt(starPos + 2);
  expectedHex[2] = '\0';
  uint8_t expected = (uint8_t)strtoul(expectedHex, nullptr, 16);

  return calc == expected;
}

static String getNmeaField(const String &line, int fieldIndex) {
  int fieldStart = 0;
  int currentField = 0;

  for (int i = 0; i <= (int)line.length(); i++) {
    bool fieldEnd = (i == (int)line.length()) || (line.charAt(i) == ',') || (line.charAt(i) == '*');
    if (!fieldEnd) {
      continue;
    }

    if (currentField == fieldIndex) {
      return line.substring(fieldStart, i);
    }

    currentField++;
    fieldStart = i + 1;

    if (i < (int)line.length() && line.charAt(i) == '*') {
      break;
    }
  }

  return "";
}

static bool parseNmeaCoordinate(const String &rawCoord, char hemisphere, double &outCoord) {
  if (rawCoord.length() < 4) {
    return false;
  }

  int dotPos = rawCoord.indexOf('.');
  int degDigits = (dotPos > 4) ? 3 : 2;
  if (dotPos < 0) {
    degDigits = (rawCoord.length() > 5) ? 3 : 2;
  }
  if ((int)rawCoord.length() <= degDigits) {
    return false;
  }

  double degrees = rawCoord.substring(0, degDigits).toDouble();
  double minutes = rawCoord.substring(degDigits).toDouble();
  if (minutes < 0.0 || minutes >= 60.0) {
    return false;
  }

  outCoord = degrees + (minutes / 60.0);
  hemisphere = (char)toupper((unsigned char)hemisphere);
  if (hemisphere == 'S' || hemisphere == 'W') {
    outCoord = -outCoord;
  }

  return true;
}

static void updateGpsFromRmc(const String &line) {
  String status = getNmeaField(line, 2);
  status.toUpperCase();

  gpsRmcFix = (status.length() > 0 && status.charAt(0) == 'A');
  gpsLastRmcMs = millis();

  String utc = getNmeaField(line, 1);
  if (utc.length() >= 6) {
    gpsUtcTime = utc.substring(0, 6);
  }

  String date = getNmeaField(line, 9);
  if (date.length() >= 6) {
    gpsUtcDate = date.substring(0, 6);
  }

  String speedKnotsField = getNmeaField(line, 7);
  if (speedKnotsField.length() > 0) {
    gpsSpeedKmh = speedKnotsField.toFloat() * 1.852f;
  } else {
    gpsSpeedKmh = -1.0f;
  }

  if (!gpsRmcFix) {
    return;
  }

  String latField = getNmeaField(line, 3);
  String latHemField = getNmeaField(line, 4);
  String lonField = getNmeaField(line, 5);
  String lonHemField = getNmeaField(line, 6);

  if (latField.length() == 0 || latHemField.length() == 0 || lonField.length() == 0 || lonHemField.length() == 0) {
    return;
  }

  double parsedLat = 0.0;
  double parsedLon = 0.0;
  if (!parseNmeaCoordinate(latField, latHemField.charAt(0), parsedLat)) {
    return;
  }
  if (!parseNmeaCoordinate(lonField, lonHemField.charAt(0), parsedLon)) {
    return;
  }

  gpsLatitude = parsedLat;
  gpsLongitude = parsedLon;
  gpsLocationValid = true;
}

static void updateGpsFromGga(const String &line) {
  String fixQualityField = getNmeaField(line, 6);
  int fixQuality = fixQualityField.toInt();
  String satsUsedField = getNmeaField(line, 7);
  if (satsUsedField.length() > 0) {
    gpsSatellitesUsed = satsUsedField.toInt();
  }

  gpsGgaFix = (fixQuality > 0);
  gpsLastGgaMs = millis();

  String utc = getNmeaField(line, 1);
  if (utc.length() >= 6) {
    gpsUtcTime = utc.substring(0, 6);
  }

  if (!gpsGgaFix) {
    return;
  }

  String latField = getNmeaField(line, 2);
  String latHemField = getNmeaField(line, 3);
  String lonField = getNmeaField(line, 4);
  String lonHemField = getNmeaField(line, 5);

  if (latField.length() == 0 || latHemField.length() == 0 || lonField.length() == 0 || lonHemField.length() == 0) {
    return;
  }

  double parsedLat = 0.0;
  double parsedLon = 0.0;
  if (!parseNmeaCoordinate(latField, latHemField.charAt(0), parsedLat)) {
    return;
  }
  if (!parseNmeaCoordinate(lonField, lonHemField.charAt(0), parsedLon)) {
    return;
  }

  gpsLatitude = parsedLat;
  gpsLongitude = parsedLon;
  gpsLocationValid = true;
}

static void updateGpsFromGsv(const String &line) {
  String sentenceType = getNmeaField(line, 0);
  if (sentenceType.startsWith("$")) {
    sentenceType.remove(0, 1);
  }
  sentenceType.toUpperCase();

  String talker = "";
  if (sentenceType.length() >= 2) {
    talker = sentenceType.substring(0, 2);
  }

  String satsInViewField = getNmeaField(line, 3);
  if (satsInViewField.length() == 0) {
    return;
  }

  int satsInView = satsInViewField.toInt();
  if (satsInView < 0) {
    return;
  }

  unsigned long nowMs = millis();

  if (talker == "GN") {
    gpsSatellitesInViewGN = satsInView;
    gpsSatellitesInViewLastGNMs = nowMs;
  } else if (talker == "GP") {
    gpsSatellitesInViewGP = satsInView;
    gpsSatellitesInViewLastGPMS = nowMs;
  } else if (talker == "GL") {
    gpsSatellitesInViewGL = satsInView;
    gpsSatellitesInViewLastGLMs = nowMs;
  } else if (talker == "GA") {
    gpsSatellitesInViewGA = satsInView;
    gpsSatellitesInViewLastGAMs = nowMs;
  } else if (talker == "GB" || talker == "BD") {
    if (talker == "GB") {
      gpsSatellitesInViewGB = satsInView;
      gpsSatellitesInViewLastGBMs = nowMs;
    } else {
      gpsSatellitesInViewBD = satsInView;
      gpsSatellitesInViewLastBDMs = nowMs;
    }
  } else if (talker == "GQ") {
    gpsSatellitesInViewGQ = satsInView;
    gpsSatellitesInViewLastGQMs = nowMs;
  }

  bool gnFresh = (gpsSatellitesInViewGN >= 0) && (gpsSatellitesInViewLastGNMs != 0) && ((nowMs - gpsSatellitesInViewLastGNMs) <= GPS_GSV_SOURCE_FRESHNESS_MS);
  if (gnFresh) {
    // GNGSV zwykle podaje laczna liczbe satelitow we wszystkich konstelacjach.
    gpsSatellitesInView = gpsSatellitesInViewGN;
    return;
  }

  int totalInView = 0;
  bool hasAny = false;

  if (gpsSatellitesInViewGP >= 0 && gpsSatellitesInViewLastGPMS != 0 && (nowMs - gpsSatellitesInViewLastGPMS) <= GPS_GSV_SOURCE_FRESHNESS_MS) {
    totalInView += gpsSatellitesInViewGP;
    hasAny = true;
  }
  if (gpsSatellitesInViewGL >= 0 && gpsSatellitesInViewLastGLMs != 0 && (nowMs - gpsSatellitesInViewLastGLMs) <= GPS_GSV_SOURCE_FRESHNESS_MS) {
    totalInView += gpsSatellitesInViewGL;
    hasAny = true;
  }
  if (gpsSatellitesInViewGA >= 0 && gpsSatellitesInViewLastGAMs != 0 && (nowMs - gpsSatellitesInViewLastGAMs) <= GPS_GSV_SOURCE_FRESHNESS_MS) {
    totalInView += gpsSatellitesInViewGA;
    hasAny = true;
  }
  if (gpsSatellitesInViewGB >= 0 && gpsSatellitesInViewLastGBMs != 0 && (nowMs - gpsSatellitesInViewLastGBMs) <= GPS_GSV_SOURCE_FRESHNESS_MS) {
    totalInView += gpsSatellitesInViewGB;
    hasAny = true;
  }
  if (gpsSatellitesInViewBD >= 0 && gpsSatellitesInViewLastBDMs != 0 && (nowMs - gpsSatellitesInViewLastBDMs) <= GPS_GSV_SOURCE_FRESHNESS_MS) {
    totalInView += gpsSatellitesInViewBD;
    hasAny = true;
  }
  if (gpsSatellitesInViewGQ >= 0 && gpsSatellitesInViewLastGQMs != 0 && (nowMs - gpsSatellitesInViewLastGQMs) <= GPS_GSV_SOURCE_FRESHNESS_MS) {
    totalInView += gpsSatellitesInViewGQ;
    hasAny = true;
  }

  if (hasAny) {
    gpsSatellitesInView = totalInView;
  } else {
    gpsSatellitesInView = satsInView;
  }
}

static bool parseGpsEsp32Frame(const String &line) {
  if (!line.startsWith("$GPSESP32[")) {
    return false;
  }

  int openPos = line.indexOf('[');
  int closePos = line.lastIndexOf(']');
  if (openPos < 0 || closePos <= openPos) {
    return true;
  }

  String payload = line.substring(openPos + 1, closePos);
  String fields[7];
  int fieldCount = 0;
  int start = 0;
  for (int i = 0; i <= (int)payload.length(); i++) {
    bool endField = (i == (int)payload.length()) || (payload.charAt(i) == ',');
    if (!endField) {
      continue;
    }

    if (fieldCount >= 7) {
      return true;
    }

    fields[fieldCount] = payload.substring(start, i);
    fields[fieldCount].trim();
    fieldCount++;
    start = i + 1;
  }

  if (fieldCount != 7) {
    return true;
  }

  bool nmeaValid = (fields[0].toInt() == 1);
  bool hasFix = (fields[1].toInt() == 1);
  int sats = fields[2].toInt();
  if (sats < 0) sats = 0;
  if (sats > 99) sats = 99;

  String hhmmss = "";
  if (fields[5].length() == 8 && fields[5].charAt(2) == ':' && fields[5].charAt(5) == ':') {
    hhmmss = fields[5].substring(0, 2) + fields[5].substring(3, 5) + fields[5].substring(6, 8);
  }

  String ddmmyy = "";
  if (fields[6].length() == 10 && fields[6].charAt(2) == '-' && fields[6].charAt(5) == '-') {
    ddmmyy = fields[6].substring(0, 2) + fields[6].substring(3, 5) + fields[6].substring(8, 10);
  }

  unsigned long nowMs = millis();
  gpsLastRmcMs = nowMs;
  gpsLastGgaMs = nowMs;
  gpsRmcFix = nmeaValid && hasFix;
  gpsGgaFix = nmeaValid && hasFix;
  gpsSatellitesUsed = sats;
  gpsSatellitesInView = sats;
  gpsSatellitesInViewGN = -1;
  gpsSatellitesInViewGP = -1;
  gpsSatellitesInViewGL = -1;
  gpsSatellitesInViewGA = -1;
  gpsSatellitesInViewGB = -1;
  gpsSatellitesInViewBD = -1;
  gpsSatellitesInViewGQ = -1;
  gpsSatellitesInViewLastGNMs = 0;
  gpsSatellitesInViewLastGPMS = 0;
  gpsSatellitesInViewLastGLMs = 0;
  gpsSatellitesInViewLastGAMs = 0;
  gpsSatellitesInViewLastGBMs = 0;
  gpsSatellitesInViewLastBDMs = 0;
  gpsSatellitesInViewLastGQMs = 0;
  gpsSpeedKmh = -1.0f;
  gpsUtcTime = hhmmss;
  gpsUtcDate = ddmmyy;

  if (!(nmeaValid && hasFix)) {
    gpsLocationValid = false;
    gpsLatitude = 0.0;
    gpsLongitude = 0.0;
    return true;
  }

  double lat = fields[3].toDouble();
  double lon = fields[4].toDouble();
  bool latOk = (lat >= -90.0 && lat <= 90.0);
  bool lonOk = (lon >= -180.0 && lon <= 180.0);
  if (latOk && lonOk) {
    gpsLatitude = lat;
    gpsLongitude = lon;
    gpsLocationValid = true;
  } else {
    gpsLocationValid = false;
  }

  return true;
}

static void parseGpsNmeaSentence(const String &line) {
  if (parseGpsEsp32Frame(line)) {
    return;
  }

  if (!validateNmeaChecksum(line)) {
    return;
  }

  String sentenceType = getNmeaField(line, 0);
  if (sentenceType.startsWith("$")) {
    sentenceType.remove(0, 1);
  }
  sentenceType.toUpperCase();

  if (sentenceType.endsWith("RMC")) {
    updateGpsFromRmc(line);
    return;
  }

  if (sentenceType.endsWith("GGA")) {
    updateGpsFromGga(line);
    return;
  }

  if (sentenceType.endsWith("GSV")) {
    updateGpsFromGsv(line);
    return;
  }
}

static String decodeKissPrintableInfo(const uint8_t *payload, size_t len, size_t infoStart) {
  if (len == 0 || infoStart >= len) {
    return "";
  }

  String out = "";
  out.reserve((len - infoStart) + 8);
  for (size_t i = infoStart; i < len; i++) {
    char c = (char)payload[i];
    if (c == '\r' || c == '\n') {
      continue;
    }
    if ((uint8_t)c < 32 || (uint8_t)c > 126) {
      if (c == '\t') {
        out += ' ';
      }
      continue;
    }
    out += c;
  }
  out.trim();
  return out;
}

static String decodeAx25AddressToken(const uint8_t *addr, bool withRepeatedMark, bool *okOut, bool *lastOut = nullptr) {
  bool ok = true;
  String call = "";
  call.reserve(6);

  for (int i = 0; i < 6; i++) {
    char c = (char)(addr[i] >> 1);
    if (c == ' ') {
      continue;
    }
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
      ok = false;
      break;
    }
    call += c;
  }

  call.trim();
  if (call.length() == 0) {
    ok = false;
  }

  if (!ok) {
    if (okOut) *okOut = false;
    if (lastOut) *lastOut = false;
    return "";
  }

  uint8_t ssid = (uint8_t)((addr[6] >> 1) & 0x0F);
  bool isLast = (addr[6] & 0x01) != 0;
  bool hasBeenRepeated = (addr[6] & 0x80) != 0;

  String token = call;
  if (ssid > 0) {
    token += "-";
    token += String(ssid);
  }
  if (withRepeatedMark && hasBeenRepeated) {
    token += "*";
  }

  if (okOut) *okOut = true;
  if (lastOut) *lastOut = isLast;
  return token;
}

static String decodeKissPayloadToTnc2(const uint8_t *payload, size_t len) {
  if (len == 0) {
    return "";
  }

  // Fallback: część modemów zwraca już payload tekstowy.
  if (len < 16) {
    return decodeKissPrintableInfo(payload, len, 0);
  }

  size_t offset = 0;
  bool ok = false;
  bool isLast = false;

  String destination = decodeAx25AddressToken(payload + offset, false, &ok, &isLast);
  if (!ok) {
    return decodeKissPrintableInfo(payload, len, 0);
  }
  offset += 7;

  if ((offset + 7) > len) {
    return decodeKissPrintableInfo(payload, len, 0);
  }

  String source = decodeAx25AddressToken(payload + offset, false, &ok, &isLast);
  if (!ok) {
    return decodeKissPrintableInfo(payload, len, 0);
  }
  offset += 7;

  String path = "";
  uint8_t digiGuard = 0;
  while (!isLast && (offset + 7) <= len && digiGuard < 8) {
    bool digiLast = false;
    String digi = decodeAx25AddressToken(payload + offset, true, &ok, &digiLast);
    if (!ok || digi.length() == 0) {
      return decodeKissPrintableInfo(payload, len, 0);
    }
    if (path.length() > 0) {
      path += ",";
    }
    path += digi;
    isLast = digiLast;
    offset += 7;
    digiGuard++;
  }

  if (!isLast || offset >= len) {
    return decodeKissPrintableInfo(payload, len, 0);
  }

  size_t infoStart = offset;
  if (infoStart < len && payload[infoStart] == 0x03) {
    infoStart++;
  }
  if (infoStart < len && payload[infoStart] == 0xF0) {
    infoStart++;
  }

  String info = decodeKissPrintableInfo(payload, len, infoStart);
  if (info.length() == 0) {
    return "";
  }

  String tnc2 = source;
  tnc2 += ">";
  tnc2 += destination;
  if (path.length() > 0) {
    tnc2 += ",";
    tnc2 += path;
  }
  tnc2 += ":";
  tnc2 += info;
  return tnc2;
}

static bool isAprsCallsignTokenValid(const String &token) {
  if (token.length() < 3 || token.length() > 15) {
    return false;
  }

  for (size_t i = 0; i < token.length(); i++) {
    char c = token.charAt(i);
    bool ok = ((c >= 'A' && c <= 'Z') ||
               (c >= 'a' && c <= 'z') ||
               (c >= '0' && c <= '9') ||
               c == '-');
    if (!ok) {
      return false;
    }
  }
  return true;
}

static bool isAprsSourceChar(char c) {
  return ((c >= 'A' && c <= 'Z') ||
          (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') ||
          c == '-');
}

static int findAprsFrameStartInText(const String &text, int fromPos) {
  int gtPos = text.indexOf('>', fromPos);
  while (gtPos >= 0) {
    int start = gtPos - 1;
    while (start >= 0 && isAprsSourceChar(text.charAt(start))) {
      start--;
    }
    start++;

    int tokenLen = gtPos - start;
    if (tokenLen >= 3 && tokenLen <= 15) {
      String src = text.substring(start, gtPos);
      if (isAprsCallsignTokenValid(src)) {
        return start;
      }
    }

    gtPos = text.indexOf('>', gtPos + 1);
  }

  return -1;
}

static void stripKnownUartNoiseTokens(String &line) {
  // BT stack alert, który bywa doklejany do ramek APRS bez separatora.
  const char *warnToken = "ASSERT_WARN(1 8), in lc_task.c at line 1559";
  while (line.indexOf(warnToken) >= 0) {
    line.replace(warnToken, "");
  }
}

static void removeAllTokenFromLine(String &line, const char *token) {
  while (line.indexOf(token) >= 0) {
    line.replace(token, "");
  }
}

static void stripBridgeControlTokensFromLine(String &line) {
  removeAllTokenFromLine(line, "ACK BTSPP_OFF");
  removeAllTokenFromLine(line, "ACK BTSPP_ON");
  removeAllTokenFromLine(line, "ACK BT SCAN");
  removeAllTokenFromLine(line, "[BT] AUTO CONNECT OK");
  removeAllTokenFromLine(line, "AUTO CONNECT OK");
  removeAllTokenFromLine(line, "BTCNT_OK");
  removeAllTokenFromLine(line, "BTCNT_ERR");
  removeAllTokenFromLine(line, "BTDISC_OK");
  removeAllTokenFromLine(line, "BTDISC_ERR");
  removeAllTokenFromLine(line, "BT[LIST_END]");

  int beginPos = line.indexOf("BT[LIST_BEGIN]:");
  while (beginPos >= 0) {
    int endPos = beginPos + (int)strlen("BT[LIST_BEGIN]:");
    while (endPos < (int)line.length() && isdigit((unsigned char)line.charAt(endPos))) {
      endPos++;
    }
    line.remove(beginPos, endPos - beginPos);
    beginPos = line.indexOf("BT[LIST_BEGIN]:");
  }

  int namePos = line.indexOf("BT[NAME]:");
  while (namePos >= 0) {
    int btPos = line.indexOf("[BT]", namePos + 9);
    if (btPos >= 0) {
      line.remove(namePos, (btPos + 4) - namePos);
    } else {
      line.remove(namePos);
      break;
    }
    namePos = line.indexOf("BT[NAME]:");
  }

  int errPos = line.indexOf("BT[ERROR]:");
  while (errPos >= 0) {
    int btPos = line.indexOf("[BT]", errPos + 10);
    if (btPos >= 0) {
      line.remove(errPos, (btPos + 4) - errPos);
    } else {
      line.remove(errPos);
      break;
    }
    errPos = line.indexOf("BT[ERROR]:");
  }
}

static bool parseBridgeBtNameLine(const String &line, String &outName, String &outAddrNoColon) {
  int namePos = line.indexOf("BT[NAME]:");
  if (namePos < 0) {
    return false;
  }

  String payload = line.substring(namePos + 9);
  int adrPos = payload.indexOf("[ADR]");
  int btPos = payload.indexOf("[BT]", adrPos + 5);
  if (adrPos < 0 || btPos < 0 || btPos <= adrPos) {
    return false;
  }

  String name = payload.substring(0, adrPos);
  String rawAddr = payload.substring(adrPos + 5, btPos);
  name = sanitizeBridgeTextField(name, 40);
  String normAddr = normalizeBridgeBtAddress(rawAddr);
  if (normAddr.length() != 12) {
    return false;
  }
  if (name.length() == 0) {
    name = "BT_DEVICE";
  }

  outName = name;
  outAddrNoColon = normAddr;
  return true;
}

static bool processBridgeControlLine(String rawLine) {
  if (!isUart2BridgeModuleSelected()) {
    return false;
  }

  String line = rawLine;
  line.trim();
  if (line.length() == 0) {
    return false;
  }

  bool handled = false;

  if (line.indexOf("ACK BTSPP_OFF") >= 0) {
    bridgeBtLastAck = "ACK BTSPP_OFF";
    handled = true;
    if (bridgeAwaitBtsppOffAckForScan || bridgeBtScanRequested) {
      bridgeAwaitBtsppOffAckForScan = false;
      bridgeBtScanRequested = false;
      if (!sendBridgeUartCommandLine(UART2_BRIDGE_BT_SCAN_COMMAND, true)) {
        bridgeBtLastError = "SEND_BT_SCAN_FAIL";
        bridgeBtScanInProgress = false;
      }
    }
  }

  if (line.indexOf("ACK BTSPP_ON") >= 0) {
    bridgeBtLastAck = "ACK BTSPP_ON";
    bridgeBtLastError = "";
    resetBridgeBtsppOnRetryState();
    handled = true;
  }

  if (line.indexOf("ACK BT SCAN") >= 0) {
    bridgeBtLastAck = "ACK BT SCAN";
    bridgeBtScanInProgress = true;
    bridgeBtLastScanMs = millis();
    bridgeBtLastError = "";
    handled = true;
  }

  if (line.indexOf("[BT] AUTO CONNECT OK") >= 0 || line.indexOf("AUTO CONNECT OK") >= 0) {
    bridgeBtLastAck = "AUTO CONNECT OK";
    bridgeBtLastError = "";
    handled = true;
    Serial.println("[BRIDGE] AUTO CONNECT OK -> start retry BTSPP_ON (max 3, co 2s)");
    triggerBridgeBtsppOnRetryStart();
    serviceBridgeBtsppOnRetry(millis());
  }

  if (line.indexOf("BTDISC_OK") >= 0) {
    bridgeBtLastDiscStatus = "BTDISC_OK";
    handled = true;
  }
  if (line.indexOf("BTDISC_ERR") >= 0) {
    bridgeBtLastDiscStatus = "BTDISC_ERR";
    handled = true;
  }

  int listBeginPos = line.indexOf("BT[LIST_BEGIN]:");
  if (listBeginPos >= 0) {
    int countStart = listBeginPos + (int)strlen("BT[LIST_BEGIN]:");
    int countEnd = countStart;
    while (countEnd < (int)line.length() && isdigit((unsigned char)line.charAt(countEnd))) {
      countEnd++;
    }
    String countStr = line.substring(countStart, countEnd);
    countStr.trim();
    bridgeBtListExpected = (countStr.length() > 0) ? countStr.toInt() : -1;
    clearBridgeBtScanList();
    bridgeBtListCollecting = true;
    bridgeBtScanInProgress = true;
    handled = true;
  }

  String nameParsed;
  String addrParsed;
  if (parseBridgeBtNameLine(line, nameParsed, addrParsed)) {
    addOrUpdateBridgeBtDevice(nameParsed, addrParsed);
    bridgeBtListCollecting = true;
    bridgeBtScanInProgress = true;
    handled = true;
  }

  if (line.indexOf("BT[LIST_END]") >= 0) {
    bridgeBtListCollecting = false;
    bridgeBtScanInProgress = false;
    bridgeBtLastScanMs = millis();
    handled = true;

    if (bridgeBtAutoConnectPending && bridgeBtPairedAddress.length() == 12) {
      requestBridgePairConnect(bridgeBtPairedName, bridgeBtPairedAddress, "", true);
      bridgeBtAutoConnectPending = false;
    }
  }

  int errPos = line.indexOf("BT[ERROR]:");
  if (errPos >= 0) {
    String errPayload = line.substring(errPos + 10);
    int btEnd = errPayload.indexOf("[BT]");
    if (btEnd >= 0) {
      errPayload = errPayload.substring(0, btEnd);
    }
    errPayload.trim();
    bridgeBtLastError = errPayload;
    bridgeBtScanInProgress = false;
    bridgeBtListCollecting = false;
    handled = true;
  }

  if (line.indexOf("BTCNT_OK") >= 0) {
    bridgeBtPairPending = false;
    bridgeBtPairLastResult = 1;
    bridgeBtPairLastResultMs = millis();
    handled = true;

    if (bridgeBtPairPendingAddress.length() == 12) {
      bridgeBtPairedAddress = bridgeBtPairPendingAddress;
      bridgeBtPairedName = bridgeBtPairPendingName;
      saveBridgeBtPairingToNvs();
    }

    bridgeBtPairPendingAddress = "";
    bridgeBtPairPendingName = "";
    Serial.println("[BRIDGE] BTCNT_OK -> start retry BTSPP_ON (max 3, co 2s)");
    triggerBridgeBtsppOnRetryStart();
    serviceBridgeBtsppOnRetry(millis());
  }

  if (line.indexOf("BTCNT_ERR") >= 0) {
    bridgeBtPairPending = false;
    bridgeBtPairLastResult = -1;
    bridgeBtPairLastResultMs = millis();
    bridgeBtLastError = "BTCNT_ERR";
    bridgeBtPairPendingAddress = "";
    bridgeBtPairPendingName = "";
    handled = true;
  }

  return handled;
}

static bool isLikelyAprsTnc2Frame(const String &line) {
  if (line.length() < 10 || line.length() > 340) {
    return false;
  }

  if (line.startsWith("ASSERT") || line.startsWith("Guru Meditation") || line.startsWith("E (")) {
    return false;
  }

  int gtPos = line.indexOf('>');
  if (gtPos < 3 || gtPos > 15) {
    return false;
  }

  int colonPos = line.indexOf(':', gtPos + 1);
  if (colonPos < 0 || colonPos <= (gtPos + 1) || colonPos >= (int)line.length() - 1) {
    return false;
  }

  String source = line.substring(0, gtPos);
  source.trim();
  if (!isAprsCallsignTokenValid(source)) {
    return false;
  }

  return true;
}

static bool isYaesuUiMonitorHeaderLine(const String &line) {
  if (line.length() < 12) {
    return false;
  }
  if (line.indexOf("] <UI") < 0) {
    return false;
  }
  return line.endsWith(">:");
}

static String normalizeYaesuUiHeaderToAprsPrefix(const String &headerLine) {
  String prefix = headerLine;
  int uiPos = prefix.lastIndexOf(" <UI");
  if (uiPos < 0 || !prefix.endsWith(">:")) {
    return prefix;
  }

  int timestampPos = prefix.lastIndexOf(" [", uiPos);
  if (timestampPos >= 0 && prefix.indexOf("] <UI", timestampPos) >= 0) {
    prefix = prefix.substring(0, timestampPos);
  } else {
    prefix = prefix.substring(0, uiPos);
  }
  prefix += ":";
  return prefix;
}

static void handleUartAprsKissFrame(const uint8_t *frame, size_t len) {
  if (!isUart2AprsKissParsingEnabled()) {
    return;
  }
  if (len < 2) {
    return;
  }

  uint8_t portCmd = frame[0];
  if ((portCmd & 0x0F) != 0x00) {
    return;
  }

  String line = decodeKissPayloadToTnc2(frame + 1, len - 1);
  if (line.length() == 0) {
    return;
  }

  if (!isLikelyAprsTnc2Frame(line)) {
    return;
  }

  processAprsFrameFromLine(line, "UART2/KISS");
}

static void handleUartAprsTextLine(String line) {
  if (!isUart2AprsTextParsingEnabled()) {
    return;
  }

  stripKnownUartNoiseTokens(line);
  processBridgeControlLine(line);
  stripBridgeControlTokensFromLine(line);
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.startsWith("#")) {
    return;
  }

  // Yaesu Packet Monitor: naglowek jest w osobnej linii, payload w nastepnej.
  if (uart2YaesuUiAprsPrefix.length() > 0) {
    String combined = uart2YaesuUiAprsPrefix + line;
    uart2YaesuUiAprsPrefix = "";
    combined.trim();
    if (isLikelyAprsTnc2Frame(combined)) {
      processAprsFrameFromLine(combined, "UART2/TNC2");
      return;
    }
  }

  if (isYaesuUiMonitorHeaderLine(line)) {
    uart2YaesuUiAprsPrefix = normalizeYaesuUiHeaderToAprsPrefix(line);
    return;
  }

  int cursor = 0;
  while (cursor < (int)line.length()) {
    int start = findAprsFrameStartInText(line, cursor);
    if (start < 0) {
      break;
    }

    int gtPos = line.indexOf('>', start);
    if (gtPos < 0) {
      break;
    }

    int colonPos = line.indexOf(':', gtPos + 1);
    if (colonPos < 0) {
      break;
    }

    int nextStart = findAprsFrameStartInText(line, colonPos + 1);
    int end = (nextStart >= 0) ? nextStart : (int)line.length();
    if (end <= start) {
      cursor = start + 1;
      continue;
    }

    String candidate = line.substring(start, end);
    candidate.trim();
    if (isLikelyAprsTnc2Frame(candidate)) {
      processAprsFrameFromLine(candidate, "UART2/TNC2");
    }

    cursor = end;
  }
}

static void processUartAprsByte(uint8_t b) {
  if (!isUart2AprsUartSourceEnabled()) {
    return;
  }

  if (isUart2AprsKissParsingEnabled()) {
    if (b == 0xC0) {
      if (uart2KissInFrame && uart2KissFrameLen > 0) {
        handleUartAprsKissFrame(uart2KissFrame, uart2KissFrameLen);
      }
      uart2KissInFrame = true;
      uart2KissEscape = false;
      uart2KissFrameLen = 0;
      return;
    }

    if (uart2KissInFrame) {
      if (uart2KissEscape) {
        uart2KissEscape = false;
        if (b == 0xDC) {
          b = 0xC0;
        } else if (b == 0xDD) {
          b = 0xDB;
        }
      } else if (b == 0xDB) {
        uart2KissEscape = true;
        return;
      }

      if (uart2KissFrameLen < sizeof(uart2KissFrame)) {
        uart2KissFrame[uart2KissFrameLen++] = b;
      } else {
        uart2KissFrameLen = 0;
        uart2KissInFrame = false;
        uart2KissEscape = false;
      }
      return;
    }
  }

  if (b == '\r' || b == '\n') {
    if (uart2AprsLine.length() > 0) {
      String line = uart2AprsLine;
      uart2AprsLine = "";
      handleUartAprsTextLine(line);
    }
    return;
  }

  if (uart2AprsLine.length() < 512) {
    uart2AprsLine += (char)b;
  } else {
    uart2AprsLine = "";
  }
}

static void pollGpsSerial() {
  if (!isUart2GpsParsingEnabled() && !isUart2AprsUartSourceEnabled()) {
    return;
  }

  unsigned long nowMs = millis();
  bool gpsParsingEnabled = isUart2GpsParsingEnabled();
  bool rawBridgeActive = gpsParsingEnabled && ((nowMs - gpsRawBridgeStartMs) < GPS_RAW_BRIDGE_WINDOW_MS);

  if (rawBridgeActive && (nowMs - gpsRawLastStatsMs) >= GPS_RAW_STATS_INTERVAL_MS) {
    gpsRawLastStatsMs = nowMs;
    Serial.printf("[GPS RAW] Odebrane bajty (ostatnie 5 s): %lu | suma: %lu\n",
                  (unsigned long)gpsRawBytesSinceStats,
                  (unsigned long)gpsRawBytesTotal);
    gpsRawBytesSinceStats = 0;
  }

  if (!rawBridgeActive && !gpsRawBridgeEndLogged) {
    Serial.printf("[GPS RAW] Koniec trybu testowego po 60 s | suma odebranych bajtow: %lu\n",
                  (unsigned long)gpsRawBytesTotal);
    gpsRawBridgeEndLogged = true;
  }

  while (gpsSerial.available() > 0) {
    char c = (char)gpsSerial.read();
    gpsLastRxMs = nowMs;
    gpsRawBytesTotal++;
    gpsRawBytesSinceStats++;

    if (rawBridgeActive) {
      Serial.write((uint8_t)c);
    }

    if (isUart2AprsUartSourceEnabled()) {
      processUartAprsByte((uint8_t)c);
    }

    if (!gpsParsingEnabled) {
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (gpsNmeaLine.length() > 6) {
        parseGpsNmeaSentence(gpsNmeaLine);
      }
      gpsNmeaLine = "";
      continue;
    }

    if (gpsNmeaLine.length() < 160) {
      gpsNmeaLine += c;
    } else {
      gpsNmeaLine = "";
    }
  }
}

static bool isGpsFixActive(unsigned long nowMs) {
  bool rmcFreshFix = gpsRmcFix && gpsLastRmcMs != 0 && (nowMs - gpsLastRmcMs) <= GPS_FIX_FRESHNESS_MS;
  bool ggaFreshFix = gpsGgaFix && gpsLastGgaMs != 0 && (nowMs - gpsLastGgaMs) <= GPS_FIX_FRESHNESS_MS;
  return rmcFreshFix || ggaFreshFix;
}

static bool isGpsUartActive(unsigned long nowMs) {
  return gpsLastRxMs != 0 && (nowMs - gpsLastRxMs) <= GPS_UART_ACTIVE_TIMEOUT_MS;
}

static bool getServiceCoordinatesForServices(double &lat, double &lon) {
  if (useGpsIfActive) {
    unsigned long nowMs = millis();
    if (gpsLocationValid && isGpsFixActive(nowMs)) {
      lat = gpsLatitude;
      lon = gpsLongitude;
      return true;
    }
  }

  if (userLatLonValid) {
    lat = userLat;
    lon = userLon;
    return true;
  }

  if (userLocator.length() >= 4) {
    locatorToLatLon(userLocator, lat, lon);
    return true;
  }

  return false;
}

static String formatGpsUtcForLog() {
  if (gpsUtcTime.length() < 6) {
    return "brak czasu";
  }

  String hhmmss = gpsUtcTime.substring(0, 2) + ":" + gpsUtcTime.substring(2, 4) + ":" + gpsUtcTime.substring(4, 6);
  if (gpsUtcDate.length() < 6) {
    return hhmmss + " UTC";
  }

  String dd = gpsUtcDate.substring(0, 2);
  String mm = gpsUtcDate.substring(2, 4);
  int yy = gpsUtcDate.substring(4, 6).toInt();
  int fullYear = (yy >= 80) ? (1900 + yy) : (2000 + yy);

  return String(fullYear) + "-" + mm + "-" + dd + " " + hhmmss + " UTC";
}

static bool isGpsUtcFieldSixDigits(const String &value) {
  if (value.length() < 6) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    if (!isdigit((unsigned char)value.charAt(i))) {
      return false;
    }
  }
  return true;
}

static bool isGpsTimeSourceActive(unsigned long nowMs) {
  if (!isUart2GpsParsingEnabled()) {
    return false;
  }
  if (!isGpsUartActive(nowMs) || !isGpsFixActive(nowMs)) {
    return false;
  }
  return isGpsUtcFieldSixDigits(gpsUtcTime) && isGpsUtcFieldSixDigits(gpsUtcDate);
}

static int64_t daysFromCivilUtc(int year, unsigned month, unsigned day) {
  year -= (month <= 2);
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy = (153U * (month + (month > 2 ? -3 : 9)) + 2U) / 5U + day - 1U;
  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
}

static bool parseGpsUtcToUnix(time_t &outUtcSeconds) {
  if (!isGpsUtcFieldSixDigits(gpsUtcTime) || !isGpsUtcFieldSixDigits(gpsUtcDate)) {
    return false;
  }

  int day = gpsUtcDate.substring(0, 2).toInt();
  int month = gpsUtcDate.substring(2, 4).toInt();
  int yy = gpsUtcDate.substring(4, 6).toInt();
  int hour = gpsUtcTime.substring(0, 2).toInt();
  int minute = gpsUtcTime.substring(2, 4).toInt();
  int second = gpsUtcTime.substring(4, 6).toInt();

  if (day < 1 || day > 31 || month < 1 || month > 12) {
    return false;
  }
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
    return false;
  }

  int fullYear = (yy >= 80) ? (1900 + yy) : (2000 + yy);
  int64_t daysSinceEpoch = daysFromCivilUtc(fullYear, (unsigned)month, (unsigned)day);
  int64_t unixSeconds = daysSinceEpoch * 86400LL + (int64_t)hour * 3600LL + (int64_t)minute * 60LL + (int64_t)second;
  if (unixSeconds < 100000LL) {
    return false;
  }

  outUtcSeconds = (time_t)unixSeconds;
  return true;
}

static uint64_t makeGpsDateTimeToken() {
  uint64_t token = 0;
  for (int i = 0; i < 6; i++) {
    token = token * 10ULL + (uint64_t)(gpsUtcDate.charAt(i) - '0');
  }
  for (int i = 0; i < 6; i++) {
    token = token * 10ULL + (uint64_t)(gpsUtcTime.charAt(i) - '0');
  }
  return token;
}

static bool syncSystemTimeFromGps(unsigned long nowMs) {
  if (!isGpsTimeSourceActive(nowMs)) {
    return false;
  }

  static uint64_t lastSyncedToken = 0;
  uint64_t currentToken = makeGpsDateTimeToken();
  if (currentToken == lastSyncedToken) {
    return true;
  }

  time_t utcSeconds = 0;
  if (!parseGpsUtcToUnix(utcSeconds)) {
    return false;
  }

  struct timeval tv;
  tv.tv_sec = utcSeconds;
  tv.tv_usec = 0;
  if (settimeofday(&tv, nullptr) != 0) {
    return false;
  }

  lastSyncedToken = currentToken;
  return true;
}

static void logGpsStatusEvery5s(unsigned long nowMs) {
  if (!isUart2GpsParsingEnabled()) {
    return;
  }

  if (gpsLastStatusLogMs != 0 && (nowMs - gpsLastStatusLogMs) < GPS_STATUS_LOG_INTERVAL_MS) {
    return;
  }
  gpsLastStatusLogMs = nowMs;

  bool fix = isGpsFixActive(nowMs);
  String gpsUtc = formatGpsUtcForLog();
  String satsInViewText = (gpsSatellitesInView >= 0) ? String(gpsSatellitesInView) : String("--");
  String satsUsedText = (gpsSatellitesUsed >= 0) ? String(gpsSatellitesUsed) : String("--");

  if (fix && gpsLocationValid) {
    Serial.printf("[GPS] FIX=TAK SAT_VIEW=%s SAT_USED=%s LAT=%.6f LON=%.6f UTC=%s\n",
                  satsInViewText.c_str(),
                  satsUsedText.c_str(),
                  gpsLatitude,
                  gpsLongitude,
                  gpsUtc.c_str());
  } else {
    Serial.printf("[GPS] FIX=NIE SAT_VIEW=%s SAT_USED=%s UTC=%s\n",
                  satsInViewText.c_str(),
                  satsUsedText.c_str(),
                  gpsUtc.c_str());
  }
}

// ========== INICJALIZACJA TFT (ESP32-2432S028) ==========
#ifdef ENABLE_TFT_DISPLAY
void setupBacklightPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino ESP32 core 3.x LEDC API
  ledcAttach(TFT_BL_PIN, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RES_BITS);
#else
  // Arduino ESP32 core 2.x LEDC API
  ledcSetup(BACKLIGHT_PWM_CHANNEL, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RES_BITS);
  ledcAttachPin(TFT_BL_PIN, BACKLIGHT_PWM_CHANNEL);
#endif
}

static void writeBacklightPwmPercent(int percent) {
  if (percent < MIN_BACKLIGHT_PERCENT) percent = MIN_BACKLIGHT_PERCENT;
  if (percent > 100) percent = 100;
  currentBacklightAppliedPercent = percent;
  uint32_t dutyMax = (1U << BACKLIGHT_PWM_RES_BITS) - 1U;
  uint32_t duty = (percent * dutyMax) / 100U;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(TFT_BL_PIN, duty);
#else
  ledcWrite(BACKLIGHT_PWM_CHANNEL, duty);
#endif
}

static bool getWeatherIsNightFromIcon(bool &isNight) {
  String iconCodeLocal;
  bool weatherValidLocal = false;

  lockWeatherData();
  weatherValidLocal = weatherData.valid;
  iconCodeLocal = weatherData.iconCode;
  unlockWeatherData();

  iconCodeLocal.trim();
  iconCodeLocal.toLowerCase();
  if (!weatherValidLocal || iconCodeLocal.length() == 0) {
    return false;
  }
  if (iconCodeLocal.endsWith("n")) {
    isNight = true;
    return true;
  }
  if (iconCodeLocal.endsWith("d")) {
    isNight = false;
    return true;
  }
  return false;
}

static int getTargetBacklightPercentForCurrentDayNight() {
  int targetPercent = backlightPercent;
  if (targetPercent < MIN_BACKLIGHT_PERCENT) targetPercent = MIN_BACKLIGHT_PERCENT;
  if (targetPercent > 100) targetPercent = 100;

  bool isNight = false;
  bool hasWeatherDayNight = getWeatherIsNightFromIcon(isNight);
  if (!hasWeatherDayNight || !isNight || tftNightDimmingPercent <= 0) {
    return targetPercent;
  }

  int dimming = tftNightDimmingPercent;
  if (dimming < 0) dimming = 0;
  if (dimming > 100) dimming = 100;

  int reducedPercent = (targetPercent * (100 - dimming)) / 100;
  if (reducedPercent < MIN_BACKLIGHT_PERCENT) reducedPercent = MIN_BACKLIGHT_PERCENT;
  if (reducedPercent > 100) reducedPercent = 100;
  return reducedPercent;
}

static void applyBacklightByDayNight(bool forceApply) {
  if (!tftInitialized || brightnessMenuActive) {
    return;
  }
  int targetPercent = getTargetBacklightPercentForCurrentDayNight();
  if (!forceApply && targetPercent == currentBacklightAppliedPercent) {
    return;
  }
  writeBacklightPwmPercent(targetPercent);
}

void setBacklightPercent(int percent) {
  if (percent < MIN_BACKLIGHT_PERCENT) percent = MIN_BACKLIGHT_PERCENT;
  if (percent > 100) percent = 100;
  backlightPercent = percent;
  int targetPercent = getTargetBacklightPercentForCurrentDayNight();
  writeBacklightPwmPercent(targetPercent);
}

void applyTouchRotation() {
  Serial.print("*** Applying touch.setRotation(");
  Serial.print(touchRotation);
  Serial.println(") ***");
  touch.setRotation(touchRotation);
}

void applyTftRotation() {
  Serial.print("*** Applying tft.setRotation(");
  Serial.print(tftRotation);
  Serial.println(") ***");
  tft.setRotation(tftRotation);
}

void applyTftInversion() {
  Serial.print("*** Applying tft.invertDisplay(");
  Serial.print(tftInvertColors ? "true" : "false");
  Serial.println(") ***");
  tft.invertDisplay(tftInvertColors);
}

void initTFT() {
  Serial.println("=== Inicjalizacja TFT ===");
  
  // Backlight włączany automatycznie przez tft.begin() (zgodnie z Setup801)
  // Nie trzeba ręcznie włączać backlight przed begin()
  Serial.println("TFT begin()...");
  tft.begin();
  Serial.println("TFT begin() OK");
  setupBacklightPwm();
  setBacklightPercent(backlightPercent);
  
  applyTftRotation();
  applyTftInversion();
  // Dla rotacji 1: szerokości 320, wysokości 240
  tft.fillScreen(TFT_WHITE);

  // Inicjalizacja dotyku (XPT2046, osobny SPI)
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  touch.begin(touchSPI);
  applyTouchRotation();
  
  tftInitialized = true;
  inMenu = false;
  menuOption = 0;
  
  Serial.println("=== TFT zainicjalizowany OK ===");
  
  // Wyświetl ekran startowy (boot log)
  drawBootScreen();
  tftBootPrintLine("=== Inicjalizacja TFT ===");
  tftBootPrintLine("TFT begin()...");
  tftBootPrintLine("TFT begin() OK");
  tftBootPrintLine("=== TFT zainicjalizowany OK ===");
}

void drawBootScreen() {
  if (!tftInitialized) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  bootLogY = 2;
  tft.setCursor(2, bootLogY);
}

void tftBootPrintLine(const String &line) {
  if (!tftInitialized) {
    return;
  }
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.setCursor(2, bootLogY);
  tft.print(line);
  bootLogY += BOOT_LOG_LINE_HEIGHT;
  if (bootLogY > 230) {
    tft.fillScreen(TFT_BLACK);
    bootLogY = 2;
  }
}

void drawWelcomeScreenYellow() {
  if (!tftInitialized) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  // Optional startup background from LittleFS (data/logo.bmp)
  drawBmpFromFS("/logo.bmp", 0, 0);
  tft.setTextSize(1);
  const int centerX = 160;
  const int startY = 40;
  const int lineGap = 16;
  String lines[] = {
    "ESP32-HAM-CLOCK",
    "version 1.3.0",
    "Created by SP3KON",
    "(AI-assisted)",
    "sp3kon@gmail.com",
    ""
  };
  for (int i = 0; i < 6; i++) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(5, startY + i * lineGap);
    tft.print(lines[i]);
  }
}

void drawWelcomeScreenGreen() {
  if (!tftInitialized) {
    return;
  }
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  const int centerX = 160;
  const int startY = 40;
  const int lineGap = 16;
  tft.setTextSize(2);
  String line = "FOLLOW THE PROPAGATION...";
  int textWidth = line.length() * 12;
  int x = centerX - (textWidth / 2);
  tft.setCursor(x, startY + 6 * lineGap + 25);
  tft.print(line);
}

// Aktualizuj wyĹ›wietlacz z aktualnym adresem IP
void updateTFT_IP() {
  if (!tftInitialized) {
    return;
  }
  
  IPAddress ip;
  String modeStr = "";
  
  // SprawdĹş czy jesteĹ›my w trybie STA czy AP
  if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
    modeStr = "WiFi";
  } else if (WiFi.getMode() & WIFI_AP) {
    ip = WiFi.softAPIP();
    modeStr = "AP Mode";
  } else {
    // Brak IP - wyświetl komunikat
    tft.fillRect(10, 40, 300, 20, TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 45);
    tft.print("No connection");
    return;
  }
  
  // WyczyĹ›Ä‡ obszar IP (dla rotacji 1: szerokoĹ›Ä‡ 320)
  tft.fillRect(10, 40, 300, 20, TFT_BLACK);
  
  // WyĹ›wietl tryb i IP w jednej linii (oszczędność miejsca)
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 45);
  tft.print(modeStr);
  tft.print(": ");
  
  // Wyświetl IP
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(ip.toString());
}

// Aktualizuj wyĹ›wietlacz z tabelą spotów (podobnie jak na stronie WWW)
void updateTFT_Spots() {
  static unsigned long lastTFTPrint = 0;
  static int tftCallCount = 0;
  tftCallCount++;
  
  if (!tftInitialized) {
    return;
  }
  
  // Print co 10 wywołań (dla debugowania)
  unsigned long now = millis();
  if (now - lastTFTPrint > 10000) { // Co 10 sekund
    Serial.print("[TFT] updateTFT_Spots wywoĹ‚ane ");
    Serial.print(tftCallCount);
    Serial.print(" razy, spotCount=");
    Serial.println(spotCount);
    lastTFTPrint = now;
    tftCallCount = 0;
  }
  
  // WyczyĹ›Ä‡ obszar tabeli (zostaw nagĹ‚Ăłwek i IP)
  // Dla rotacji 1 (krajobraz): szerokoĹ›Ä‡ 320, wysokoĹ›Ä‡ 240
  // Zostaw miejsce na nagĹ‚Ăłwek (0-45) i tabelÄ™ (65-240)
    tft.fillRect(0, 65, 320, 175, TFT_WHITE);
  
  // WyĹ›wietl nagĹ‚Ăłwek tabeli
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(1);
  
  // Nagłówki kolumn (dla szerokoĹ›ci 320px)
  int yPos = 65;
  tft.setCursor(5, yPos);
  tft.print(tr(TR_TIME_SHORT));
  tft.setCursor(55, yPos);
  tft.print(tr(TR_CALL_SHORT));
  tft.setCursor(130, yPos);
  tft.print("MHz");
  tft.setCursor(200, yPos);
  tft.print("Mode");
  tft.setCursor(260, yPos);
  tft.print("km");
  
  yPos += 15;
  
  // Wyświetl maksymalnie 10 spotĂłw na wyĹ›wietlaczu TFT (ĹĽeby zmieścić się na ekranie)
  int maxDisplaySpots = min(spotCount, 10);
  
  for (int i = 0; i < maxDisplaySpots; i++) {
    if (yPos >= 230) break; // Nie wchodź poza ekran
    
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextSize(1);
    
    // Czas
    tft.setCursor(5, yPos);
    String timeStr = spots[i].time;
    if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
    tft.print(timeStr);
    
    // Znak wywoĹ‚awczy
    tft.setCursor(55, yPos);
    String callStr = spots[i].callsign;
    if (callStr.length() > 10) callStr = callStr.substring(0, 10);
    tft.print(callStr);
    
    // CzÄ™stotliwoĹ›Ä‡ (w MHz)
    tft.setCursor(130, yPos);
    float freqMHz = spots[i].frequency / 1000.0;
    tft.print(freqMHz, 3);
    
    // Mode
    tft.setCursor(200, yPos);
    String modeStr = spots[i].mode;
    if (modeStr.length() > 4) modeStr = modeStr.substring(0, 4);
    tft.print(modeStr);
    
    // OdlegĹ‚oĹ›Ä‡
    tft.setCursor(260, yPos);
    tft.print(formatDistanceOrCountry(spots[i], 6));
    
    yPos += 18; // OdstÄ™p miÄ™dzy wierszami
  }
  
  // Jeśli brak spotów, wyĹ›wietl komunikat
  if (spotCount == 0) {
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 90);
    tft.println(tr(TR_WAITING_SPOTS));
  }
}

// ========== SYSTEM MENU I EKRANĂ“W ==========

extern TaskHandle_t uiTaskHandle;
static void requestUiScreenRedraw(uint8_t pendingScreenId);
void drawLokalizacja();

// GĹ‚Ăłwna funkcja rysujÄ…ca ekrany
void drawScreen(ScreenType screenNum) {
  if (!tftInitialized) {
    return;
  }

  if (uiTaskHandle != nullptr && xTaskGetCurrentTaskHandle() != uiTaskHandle) {
    requestUiScreenRedraw((uint8_t)screenNum);
    return;
  }

  static ScreenType lastDrawnScreen = SCREEN_OFF;
  if (screenNum != lastDrawnScreen) {
    if (isTableFooterScreen(screenNum)) {
      tableNavFooterVisibleUntilMs = millis() + TABLE_NAV_FOOTER_VISIBLE_MS;
    }
    if (screenNum == SCREEN_APRS_RADAR) {
      // On first enter to radar screen, show zoom/info hints for a short time.
      screen6RadarHintUntilMs = millis() + SCREEN6_RADAR_HINT_DURATION_MS;
    }
    lastDrawnScreen = screenNum;
  }

  if (aprsAlertScreenActive) {
    return;
  }
  
  if (screenNum == SCREEN_MATRIX_CLOCK || screenNum == SCREEN_UNLIS_HUNTER) {
    tft.fillScreen(TFT_BLACK);
  } else {
    tft.fillScreen(tftUiPalette.bg);
  }
  
  switch (screenNum) {
    case SCREEN_HAM_CLOCK:
      drawHamClock();
      break;
    case SCREEN_DX_CLUSTER:
      drawDxCluster();
      break;
    case SCREEN_SUN_SPOTS:
      drawSunSpots();
      break;
    case SCREEN_BAND_INFO:
      drawBandInfo();
      break;
    case SCREEN_WEATHER_DSP:
      drawWeather();
      break;
    case SCREEN_WEATHER_FORECAST:
      drawWeatherForecast();
      break;
    case SCREEN_LOKALIZACJA:
      drawLokalizacja();
      break;
    case SCREEN_APRS_IS:
      drawAprsIs();
      break;
    case SCREEN_APRS_RADAR:
      drawAprsRadar();
      break;
    case SCREEN_POTA_CLUSTER:
      drawPotaCluster();
      break;
    case SCREEN_HAMALERT_CLUSTER:
      drawHamalertCluster();
      break;
    case SCREEN_MATRIX_CLOCK:
      drawMatrixClock();
      break;
    case SCREEN_UNLIS_HUNTER:
      drawUnlisHunter();
      break;
    case SCREEN_OFF:
    default:
      drawHamClock();
      break;
  }
}

static void drawLokalizacjaValueField(int x, int y, int width, uint16_t color, const String &value) {
  uint16_t bgColor = isTftStyleLight() ? TFT_WHITE : TFT_BLACK;
  tft.fillRect(x, y - 2, width, 20, bgColor);
  tft.setTextColor(color);
  tft.setCursor(x, y);
  tft.print(value);
}

static const int LOK_SAVE_BTN_X = 246;
static const int LOK_SAVE_BTN_Y = 6;
static const int LOK_SAVE_BTN_W = 68;
static const int LOK_SAVE_BTN_H = 24;

static bool isLokalizacjaSaveButtonHit(uint16_t x, uint16_t y) {
  const int hitPaddingX = 10;
  const int hitPaddingY = 10;
  const int hitX = max(0, LOK_SAVE_BTN_X - hitPaddingX);
  const int hitY = max(0, LOK_SAVE_BTN_Y - hitPaddingY);
  const int hitW = min(320 - hitX, LOK_SAVE_BTN_W + 2 * hitPaddingX);
  const int hitH = min(240 - hitY, LOK_SAVE_BTN_H + 2 * hitPaddingY);
  return isPointInRect(x, y, hitX, hitY, hitW, hitH);
}

static void drawLokalizacjaSaveButton() {
  const char *btnLabel = LANG3("ZAPISZ", "SAVE", "SALVAR");
  uint16_t btnFill = TFT_WHITE;
  uint16_t btnBorder = TFT_BLACK;
  uint16_t btnText = TFT_BLACK;
  int textW = strlen(btnLabel) * 6;

  tft.fillRoundRect(LOK_SAVE_BTN_X, LOK_SAVE_BTN_Y, LOK_SAVE_BTN_W, LOK_SAVE_BTN_H, 4, btnFill);
  tft.drawRoundRect(LOK_SAVE_BTN_X, LOK_SAVE_BTN_Y, LOK_SAVE_BTN_W, LOK_SAVE_BTN_H, 4, btnBorder);
  tft.setTextSize(1);
  tft.setTextColor(btnText);
  tft.setCursor(LOK_SAVE_BTN_X + (LOK_SAVE_BTN_W - textW) / 2, LOK_SAVE_BTN_Y + 8);
  tft.print(btnLabel);
}

static String latLonToMaidenhead6(double lat, double lon) {
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
    return "--";
  }

  if (lat >= 90.0) lat = 89.999999;
  if (lon >= 180.0) lon = 179.999999;

  double lonAdj = lon + 180.0;
  double latAdj = lat + 90.0;

  int fieldLon = (int)(lonAdj / 20.0);
  int fieldLat = (int)(latAdj / 10.0);
  if (fieldLon < 0 || fieldLon > 17 || fieldLat < 0 || fieldLat > 17) {
    return "--";
  }

  double lonAfterField = lonAdj - (fieldLon * 20.0);
  double latAfterField = latAdj - (fieldLat * 10.0);

  int squareLon = (int)(lonAfterField / 2.0);
  int squareLat = (int)(latAfterField / 1.0);
  if (squareLon < 0) squareLon = 0;
  if (squareLon > 9) squareLon = 9;
  if (squareLat < 0) squareLat = 0;
  if (squareLat > 9) squareLat = 9;

  double lonAfterSquare = lonAfterField - (squareLon * 2.0);
  double latAfterSquare = latAfterField - (squareLat * 1.0);

  int subLon = (int)(lonAfterSquare * 12.0);
  int subLat = (int)(latAfterSquare * 24.0);
  if (subLon < 0) subLon = 0;
  if (subLon > 23) subLon = 23;
  if (subLat < 0) subLat = 0;
  if (subLat > 23) subLat = 23;

  String locator = "";
  locator += (char)('A' + fieldLon);
  locator += (char)('A' + fieldLat);
  locator += (char)('0' + squareLon);
  locator += (char)('0' + squareLat);
  locator += (char)('A' + subLon);
  locator += (char)('A' + subLat);
  return locator;
}

static void updateLokalizacjaFields() {
  unsigned long nowMs = millis();
  bool gpsActive = isGpsUartActive(nowMs);
  bool gpsFix = isGpsFixActive(nowMs);
  bool lightTheme = isTftStyleLight();
  uint16_t bgColor = lightTheme ? TFT_WHITE : TFT_BLACK;

  String gpsText = gpsActive ? String(LANG3("aktywny", "ACTIVE", "ativo"))
                             : String(LANG3("nieaktywny", "INACTIVE", "inativo"));
  String fixText = gpsFix ? String(LANG3("TAK", "YES", "SIM"))
                          : String(LANG3("NIE", "NO", "NAO"));
  String satsInView = (gpsSatellitesInView >= 0) ? String(gpsSatellitesInView) : String("--");
  String latText = gpsLocationValid ? String(gpsLatitude, 6) : String("--");
  String lonText = gpsLocationValid ? String(gpsLongitude, 6) : String("--");
  String speedText = (gpsSpeedKmh >= 0.0f) ? (String(gpsSpeedKmh, 1) + " km/h") : String("--");
  String locatorText = gpsLocationValid ? latLonToMaidenhead6(gpsLatitude, gpsLongitude) : String("--");

  uint16_t gpsLabelColor = lightTheme ? TFT_BLACK : TFT_WHITE;
  uint16_t gpsValueColor = lightTheme ? TFT_BLACK : (gpsActive ? TFT_GREEN : TFT_RED);
  uint16_t fixValueColor = lightTheme ? TFT_BLACK : (gpsFix ? TFT_GREEN : TFT_RED);
  uint16_t satsColor = lightTheme ? TFT_BLACK : TFT_CYAN;
  uint16_t latColor = lightTheme ? TFT_BLACK : TFT_YELLOW;
  uint16_t lonColor = lightTheme ? TFT_BLACK : TFT_YELLOW;
  uint16_t speedColor = lightTheme ? TFT_BLACK : TFT_CYAN;
  uint16_t locatorColor = lightTheme ? TFT_BLACK : TFT_CYAN;

  tft.setTextSize(2);
  tft.fillRect(20, 60, 290, 20, bgColor);
  tft.setCursor(20, 62);
  tft.setTextColor(gpsLabelColor);
  tft.print("GPS: ");
  tft.setTextColor(gpsValueColor);
  tft.print(gpsText);
  tft.setTextColor(gpsLabelColor);
  tft.print(" FIX: ");
  tft.setTextColor(fixValueColor);
  tft.print(fixText);
  drawLokalizacjaValueField(130, 88, 170, satsColor, satsInView);
  drawLokalizacjaValueField(130, 114, 170, latColor, latText);
  drawLokalizacjaValueField(130, 140, 170, lonColor, lonText);
  drawLokalizacjaValueField(130, 166, 170, speedColor, speedText);
  drawLokalizacjaValueField(130, 192, 170, locatorColor, locatorText);
}

void drawLokalizacja() {

  bool lightTheme = isTftStyleLight();
  uint16_t pageBgColor = lightTheme ? TFT_WHITE : TFT_BLACK;
  uint16_t labelColor = lightTheme ? TFT_BLACK : TFT_WHITE;
  uint16_t frameOuterColor = TFT_DARKGREY;
  uint16_t frameMidColor = lightTheme ? TFT_BLACK : TFT_LIGHTGREY;
  uint16_t frameInnerColor = lightTheme ? TFT_BLACK : TFT_WHITE;

  tft.fillScreen(pageBgColor);
  tft.fillRect(0, 0, 320, 40, menuThemeColor);
  tft.drawFastHLine(0, 39, 320, 0x8410);

  const String headerText = LANG3("LOKALIZACJA", "LOCATION", "LOCALIZACAO");
  int headerX = (320 - ((int)headerText.length() * 12)) / 2;
  if (headerX < 6) {
    headerX = 6;
  }
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(headerX, 12);
  tft.print(headerText);
  tft.setCursor(headerX + 1, 12);
  tft.print(headerText);
  drawLokalizacjaSaveButton();

  const int frameX = 10;
  const int frameY = 50;
  const int frameW = 300;
  const int frameH = 170;
  const int frameR = 8;
  tft.drawRoundRect(frameX - 2, frameY - 2, frameW + 4, frameH + 4, frameR + 2, frameOuterColor);
  tft.drawRoundRect(frameX - 1, frameY - 1, frameW + 2, frameH + 2, frameR + 1, frameMidColor);
  tft.drawRoundRect(frameX, frameY, frameW, frameH, frameR, frameInnerColor);

  tft.setTextSize(2);
  tft.setTextColor(labelColor);

  int y = 88;
  const int rowH = 26;
  tft.setTextColor(labelColor);
  tft.setCursor(20, y);
  tft.print(LANG3("Satelity:", "SATS:", "Satelites:"));

  y += rowH;
  tft.setTextColor(labelColor);
  tft.setCursor(20, y);
  tft.print("LAT:");

  y += rowH;
  tft.setTextColor(labelColor);
  tft.setCursor(20, y);
  tft.print("LON:");

  y += rowH;
  tft.setTextColor(labelColor);
  tft.setCursor(20, y);
  tft.print(LANG3("Predkosc:", "SPEED:", "Velocidade:"));

  y += rowH;
  tft.setTextColor(labelColor);
  tft.setCursor(20, y);
  tft.print(LANG3("Lokator:", "LOCATOR:", "Locator:"));

  updateLokalizacjaFields();
  drawSwitchScreenFooter();
}

// Ekran 1: Startowy - Info
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return rgb565(255 - pos * 3, 0, pos * 3);
  }
  if (pos < 170) {
    pos -= 85;
    return rgb565(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return rgb565(pos * 3, 255 - pos * 3, 0);
}

void applyMenuThemeFromHue() {
  menuThemeColor = colorWheel(menuThemeHue);
  applyTftUiPalette();
}

// Definicja koloru "Radio Orange" - dynamiczny motyw
#define TFT_RADIO_ORANGE menuThemeColor

// Kolory i tĹ‚o "Matrix" dla ekranu 10
#define MATRIX_DARKGREEN 0x0320
#define MATRIX_BRIGHTGREEN 0x07E0
#define MATRIX_WHITEGREEN 0xAFE5

uint16_t lerpColor565(uint16_t from, uint16_t to, uint8_t t) {
  uint8_t r1 = (from >> 11) & 0x1F;
  uint8_t g1 = (from >> 5) & 0x3F;
  uint8_t b1 = from & 0x1F;
  uint8_t r2 = (to >> 11) & 0x1F;
  uint8_t g2 = (to >> 5) & 0x3F;
  uint8_t b2 = to & 0x1F;
  uint8_t r = r1 + ((r2 - r1) * t) / 255;
  uint8_t g = g1 + ((g2 - g1) * t) / 255;
  uint8_t b = b1 + ((b2 - b1) * t) / 255;
  return (r << 11) | (g << 5) | b;
}

struct MatrixDrop {
  int x;
  int y;
  int speed;
  int len;
  char headChar;
  bool introParticipates;
  int introStartY;
  int introDelayMs;
  float introSpeedPxPerMs;
  bool introActive;
};

const int SCREEN10_WIDTH = 320;
const int MATRIX_COL_SPACING = 7;
const int numDrops = SCREEN10_WIDTH / MATRIX_COL_SPACING; // Tyle kolumn, ile się mieści
const char MATRIX_HEAD_TEXT[] = "ESP32   HAM   CLOCK   by   SP3KON";
const int MATRIX_HEAD_TEXT_LEN = (int)(sizeof(MATRIX_HEAD_TEXT) - 1);
MatrixDrop drops[numDrops];
bool matrixInitialized = false;

static inline char getMatrixHeadCharForColumn(int columnIdx) {
  if (MATRIX_HEAD_TEXT_LEN <= 0) {
    return ' ';
  }
  if (numDrops <= MATRIX_HEAD_TEXT_LEN) {
    return MATRIX_HEAD_TEXT[columnIdx];
  }

  const int freeCols = numDrops - MATRIX_HEAD_TEXT_LEN;
  const int leftPad = freeCols / 2;
  const int rightStart = leftPad + MATRIX_HEAD_TEXT_LEN;

  if (columnIdx < leftPad || columnIdx >= rightStart) {
    return ' ';
  }

  return MATRIX_HEAD_TEXT[columnIdx - leftPad];
}

const int SCREEN10_HEADER_H = 0;
const int SCREEN10_BODY_TOP = SCREEN10_HEADER_H;
const int SCREEN10_BODY_BOTTOM = 240;
const int SCREEN10_BODY_H = SCREEN10_BODY_BOTTOM - SCREEN10_BODY_TOP;
const int CLOCK_TEXT_SIZE = 6;
const int CLOCK_CHAR_W = 6;
const int CLOCK_CHAR_H = 8;
const unsigned long MATRIX_UPDATE_INTERVAL_MS = 120;
const unsigned long MATRIX_INTRO_ALIGN_MS = 3400UL;

unsigned long lastScreen10UpdateMs = 0;
unsigned long lastMatrixUpdateMs = 0;
bool screen10NeedsRedraw = true;
bool matrixIntroActive = false;
unsigned long matrixIntroStartMs = 0;
int clockMaskX = 0;
int clockMaskY = 0;
int clockMaskW = 0;
int clockMaskH = 0;
String lastClockText = "";
bool clockNeedsRedraw = true;

static void resetMatrixDropRandom(int i) {
  drops[i].y = random(-SCREEN10_BODY_H, 0);
  drops[i].speed = random(5, 15);
  drops[i].len = random(4, 17);
  drops[i].headChar = (char)random(33, 126);
  drops[i].introActive = false;
}

static void prepareMatrixIntro() {
  const int charStep = 8;
  const int alignHeadY = SCREEN10_BODY_TOP + (SCREEN10_BODY_H / 2);

  for (int i = 0; i < numDrops; i++) {
    drops[i].headChar = getMatrixHeadCharForColumn(i);
    drops[i].introParticipates = (drops[i].headChar != ' ');
    drops[i].introStartY = random(-140, -40);
    drops[i].introDelayMs = random(0, 1900);
    drops[i].introActive = drops[i].introParticipates;

    int targetDropY = (alignHeadY - SCREEN10_BODY_TOP) - ((drops[i].len - 1) * charStep);
    int travelMs = (int)MATRIX_INTRO_ALIGN_MS - drops[i].introDelayMs;
    if (travelMs < 250) {
      travelMs = 250;
    }

    drops[i].introSpeedPxPerMs = (float)(targetDropY - drops[i].introStartY) / (float)travelMs;
    if (drops[i].introSpeedPxPerMs < 0.03f) {
      drops[i].introSpeedPxPerMs = 0.03f;
    }
  }

  matrixIntroActive = true;
  matrixIntroStartMs = millis();
}

String getUtcTimeString() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[9];
    strftime(timeBuffer, 9, "%H:%M:%S", &timeinfo);
    return String(timeBuffer);
  }
  return "--:--:--";
}

bool getTimeWithTimezone(struct tm *outTm) {
  time_t now = time(nullptr);
  if (now < 100000) {
    return false;
  }
  now += (time_t)(timezoneHours * 3600.0f);
  gmtime_r(&now, outTm);
  return true;
}

String getTimezoneTimeString(const char *fmt, size_t bufSize) {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "--:--";
  }
  char timeBuffer[16];
  strftime(timeBuffer, bufSize, fmt, &timeinfo);
  return String(timeBuffer);
}

String formatAprsTimeWithTimezone(String timeStr) {
  if (timeStr.endsWith("Z")) {
    timeStr.remove(timeStr.length() - 1);
  }
  if (timeStr.length() < 4) {
    return timeStr;
  }
  int hour = timeStr.substring(0, 2).toInt();
  int minute = timeStr.substring(2, 4).toInt();
  int hourLocal = hour + (int)timezoneHours;
  while (hourLocal < 0) hourLocal += 24;
  while (hourLocal >= 24) hourLocal -= 24;
  String hh = (hourLocal < 10 ? "0" : "") + String(hourLocal);
  String mm = (minute < 10 ? "0" : "") + String(minute);
  return hh + ":" + mm;
}

String sanitizePolishToAscii(const String &input) {
  String out;
  out.reserve(input.length());
  const char *s = input.c_str();
  size_t len = input.length();
  for (size_t i = 0; i < len; i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 0x80) {
      out += (char)c;
      continue;
    }
    if (i + 1 >= len) {
      continue;
    }
    uint8_t c2 = (uint8_t)s[i + 1];
    // UTF-8 Polish letters
    if (c == 0xC4 && c2 == 0x84) { out += 'a'; i++; continue; } // Ą
    if (c == 0xC4 && c2 == 0x85) { out += 'a'; i++; continue; } // ą
    if (c == 0xC4 && c2 == 0x86) { out += 'c'; i++; continue; } // Ć
    if (c == 0xC4 && c2 == 0x87) { out += 'c'; i++; continue; } // ć
    if (c == 0xC4 && c2 == 0x98) { out += 'e'; i++; continue; } // Ę
    if (c == 0xC4 && c2 == 0x99) { out += 'e'; i++; continue; } // ę
    if (c == 0xC5 && c2 == 0x81) { out += 'l'; i++; continue; } // Ł
    if (c == 0xC5 && c2 == 0x82) { out += 'l'; i++; continue; } // ł
    if (c == 0xC5 && c2 == 0x83) { out += 'n'; i++; continue; } // Ń
    if (c == 0xC5 && c2 == 0x84) { out += 'n'; i++; continue; } // ń
    if (c == 0xC3 && c2 == 0x93) { out += 'o'; i++; continue; } // Ó
    if (c == 0xC3 && c2 == 0xB3) { out += 'o'; i++; continue; } // ó
    if (c == 0xC5 && c2 == 0x9A) { out += 'S'; i++; continue; } // Ś
    if (c == 0xC5 && c2 == 0x9B) { out += 's'; i++; continue; } // ś
    if (c == 0xC5 && c2 == 0xB9) { out += 'z'; i++; continue; } // Ź
    if (c == 0xC5 && c2 == 0xBA) { out += 'z'; i++; continue; } // ź
    if (c == 0xC5 && c2 == 0xBB) { out += 'z'; i++; continue; } // Ż
    if (c == 0xC5 && c2 == 0xBC) { out += 'z'; i++; continue; } // ż
    // Latin-1 accented letters (Portuguese etc.) -> base ASCII letter
    if (c == 0xC3 && c2 >= 0x80 && c2 <= 0xBF) {
      static const char latin1Base[] =
          "AAAAAAACEEEEIIIIDNOOOOO*OUUUUYTs"
          "aaaaaaaceeeeiiiidnooooo/ouuuuyty";
      out += latin1Base[c2 - 0x80];
      i++;
      continue;
    }
    // Unknown multibyte - skip
  }
  return out;
}

static inline const char* utf8LiteralToChar(const char *s) {
  return s;
}

#if defined(__cpp_char8_t)
static inline const char* utf8LiteralToChar(const char8_t *s) {
  return reinterpret_cast<const char*>(s);
}
#endif

String getPolishDateStringFull() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return "Brak daty";
  }

  #if defined(__cpp_char8_t)
  static const char8_t* const weekdaysFull[] = {
    u8"Niedziela", u8"Poniedzia\u0142ek", u8"Wtorek",
    u8"\u015Aroda", u8"Czwartek", u8"Pi\u0105tek", u8"Sobota"
  };
  static const char8_t* const monthsFull[] = {
    u8"stycznia", u8"lutego", u8"marca", u8"kwietnia",
    u8"maja", u8"czerwca", u8"lipca", u8"sierpnia",
    u8"wrze\u015Bnia", u8"pa\u017Adziernika", u8"listopada", u8"grudnia"
  };
  #else
  static const char* const weekdaysFull[] = {
    u8"Niedziela", u8"Poniedzia\u0142ek", u8"Wtorek",
    u8"\u015Aroda", u8"Czwartek", u8"Pi\u0105tek", u8"Sobota"
  };
  static const char* const monthsFull[] = {
    u8"stycznia", u8"lutego", u8"marca", u8"kwietnia",
    u8"maja", u8"czerwca", u8"lipca", u8"sierpnia",
    u8"wrze\u015Bnia", u8"pa\u017Adziernika", u8"listopada", u8"grudnia"
  };
  #endif

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(utf8LiteralToChar(weekdaysFull[timeinfo.tm_wday])) + " " + dateBuf;
  return full;
}

String getEnglishDateStringFull() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return "No date";
  }

  static const char* weekdaysFullEn[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFullEn[timeinfo.tm_wday]) + " " + dateBuf;
  return full;
}

String getPolishDateStringFullWithTimezone() {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "Brak daty";
  }

  #if defined(__cpp_char8_t)
  static const char8_t* const weekdaysFull[] = {
    u8"Niedziela", u8"Poniedzia\u0142ek", u8"Wtorek",
    u8"\u015Aroda", u8"Czwartek", u8"Pi\u0105tek", u8"Sobota"
  };
  #else
  static const char* const weekdaysFull[] = {
    u8"Niedziela", u8"Poniedzia\u0142ek", u8"Wtorek",
    u8"\u015Aroda", u8"Czwartek", u8"Pi\u0105tek", u8"Sobota"
  };
  #endif

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(utf8LiteralToChar(weekdaysFull[timeinfo.tm_wday])) + " " + dateBuf;
  return full;
}

String getEnglishDateStringFullWithTimezone() {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "No date";
  }

  static const char* weekdaysFullEn[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFullEn[timeinfo.tm_wday]) + " " + dateBuf;
  return full;
}

String getPortugueseDateStringFull() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return "Sem data";
  }

  static const char* weekdaysFullPt[] = {
    "Domingo", "Segunda-feira", "Ter\xC3\xA7" "a-feira", "Quarta-feira",
    "Quinta-feira", "Sexta-feira", "S\xC3\xA1" "bado"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFullPt[timeinfo.tm_wday]) + " " + dateBuf;
  return full;
}

String getPortugueseDateStringFullWithTimezone() {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "Sem data";
  }

  static const char* weekdaysFullPt[] = {
    "Domingo", "Segunda-feira", "Ter\xC3\xA7" "a-feira", "Quarta-feira",
    "Quinta-feira", "Sexta-feira", "S\xC3\xA1" "bado"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFullPt[timeinfo.tm_wday]) + " " + dateBuf;
  return full;
}

// Rysuje linię daty/tygodnia wyśrodkowaną z użyciem polskiej czcionki VLW (jeśli dostępna)
void drawDateLine(const String &dateText) {
  const int frameX = 10;
  const int frameWidth = 300;
  const int frameY = 50;
  const int frameHeight = 155;
  const int dateY = 178;

  bool fontLoaded = false;
  int textW = 0;
  int textH = 0;

  if (tryLoadSmoothFont(ROBOTO_FONT20_NAME, ROBOTO_FONT20_FILE)) {
    fontLoaded = true;
    textW = tft.textWidth(dateText);
    textH = tft.fontHeight();
    // Jeśli linia jest zbyt szeroka, wróć do mniejszej czcionki systemowej
    if (textW > (frameWidth - 10)) {
      tft.unloadFont();
      fontLoaded = false;
    }
  }

  if (!fontLoaded) {
    tft.setTextSize(2);
    textW = dateText.length() * 12;
    textH = 16;
  }

  int dateX = frameX + (frameWidth - textW) / 2;
  if (dateX < frameX + 5) {
    dateX = frameX + 5;
  }

  // Czyści stały obszar: pełna szerokość między ramkami i wysokość
  // od strefy pod napisem Local/UTC do dolnej ramki.
  const int cleanX = frameX + 5;
  const int cleanW = frameWidth - 10;
  const int cleanY = 173;
  const int cleanBottom = (frameY + frameHeight) - 5;
  const int cleanH = cleanBottom - cleanY;

  const bool lightTheme = isTftStyleLight();
  const uint16_t dateBg = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t dateFg = lightTheme ? TFT_BLACK : TFT_WHITE;
  tft.fillRect(cleanX, cleanY, cleanW, cleanH, dateBg);
  tft.setTextColor(dateFg);

  if (fontLoaded) {
    tft.setTextDatum(TL_DATUM);
    tft.drawString(dateText, dateX, dateY);
    tft.unloadFont();
  } else {
    tft.setCursor(dateX, dateY);
    tft.print(sanitizePolishToAscii(dateText));
  }
}

void updateScreen1Header() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  if (!screen1HeaderNeedsRedraw) {
    return;
  }

  const int headerY = 0;
  const int headerH = 40;
  const uint16_t headerBg = TFT_RADIO_ORANGE;
  const uint16_t headerText = TFT_BLACK;
  const uint16_t logoBlockBg = TFT_BLACK;
  const uint16_t logoBlockText = TFT_RADIO_ORANGE;
  const uint16_t lineColor = 0x8410;
  
  // 1. TĹO I RAMKA LOGO
  tft.fillRect(0, headerY, 320, headerH, headerBg);
  // Ciemniejsza krawÄ™dĹş na dole dla efektu 3D
  tft.drawFastHLine(0, headerH - 1, 320, lineColor); 

  // 2. FORMATOWANIE "LOGO"
  
  int startX = 25; // OdstÄ™p od lewej (miejsce na ikonÄ™ menu)
  int textY = 12;

  // CzÄ™Ĺ›Ä‡ 1: "ESP32" w czarnym prostokÄ…cie (negatyw)
  tft.fillRect(startX, textY - 4, 65, 22, logoBlockBg);
  tft.setTextColor(logoBlockText);
  tft.setTextSize(2);
  tft.setCursor(startX + 5, textY);
  tft.print("ESP32");

  // CzÄ™Ĺ›Ä‡ 2: "-HAM-" (standardowo)
  tft.setTextColor(headerText);
  tft.setCursor(startX + 75, textY);
  tft.print("-HAM-");

  // Część 3: "CLOCK" (pogrubione wizualnie przez podwĂłjny druk)
  int clockX = startX + 145;
  tft.setCursor(clockX, textY);
  tft.print("CLOCK");
  tft.setCursor(clockX + 1, textY); // Lekkie przesuniÄ™cie o 1px w bok daje efekt "Bold"
  tft.print("CLOCK");

  // 3. OZDOBNIK GRAFICZNY (Fale radiowe)
  for(int i = 0; i < 3; i++) {
    tft.drawCircle(300, 20, 4 + (i * 4), headerText);
  }

  screen1HeaderNeedsRedraw = false;
}

static const unsigned long SCREEN1_GPS_BADGE_INTERVAL_MS = 4000UL;
static const unsigned long SCREEN1_GPS_BADGE_VISIBLE_MS = 2000UL;

static void drawHamClockConnectionIndicator(bool connected, bool lightTheme) {
  const int indicatorCx = 50;
  const int indicatorCy = 222;
  const int indicatorMaxR = 12;
  const int indicatorClearPad = 2;
  const int indicatorClearX = indicatorCx - indicatorMaxR - indicatorClearPad;
  const int indicatorClearY = indicatorCy - indicatorMaxR - indicatorClearPad;
  const int indicatorClearW = 2 * (indicatorMaxR + indicatorClearPad);
  const int indicatorClearH = 2 * (indicatorMaxR + indicatorClearPad);

  tft.fillRect(indicatorClearX, indicatorClearY, indicatorClearW, indicatorClearH, lightTheme ? TFT_WHITE : TFT_BLACK);

  unsigned long nowMs = millis();
  bool gpsBadgeVisible = false;
  if (isUart2GpsParsingEnabled()) {
    bool gpsActive = isGpsUartActive(nowMs);
    bool gpsFix = isGpsFixActive(nowMs);
    gpsBadgeVisible = gpsActive && gpsFix && ((nowMs % SCREEN1_GPS_BADGE_INTERVAL_MS) < SCREEN1_GPS_BADGE_VISIBLE_MS);
  }

  if (!connected) {
    if (gpsBadgeVisible) {
      tft.fillCircle(indicatorCx, indicatorCy, indicatorMaxR, TFT_GREEN);
      tft.drawCircle(indicatorCx, indicatorCy, indicatorMaxR, lightTheme ? TFT_BLACK : TFT_WHITE);
      tft.setTextSize(1);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.setCursor(indicatorCx - 9, indicatorCy - 3);
      tft.print("GPS");
    } else {
      tft.fillCircle(indicatorCx, indicatorCy, 4, TFT_RED);
    }
    return;
  }

  if (gpsBadgeVisible) {
    tft.fillCircle(indicatorCx, indicatorCy, indicatorMaxR, TFT_GREEN);
    tft.drawCircle(indicatorCx, indicatorCy, indicatorMaxR, lightTheme ? TFT_BLACK : TFT_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.setCursor(indicatorCx - 9, indicatorCy - 3);
    tft.print("GPS");
  } else {
    tft.fillCircle(indicatorCx, indicatorCy, 4, TFT_GREEN);
  }
}

void drawHamClock() {
  const bool lightTheme = isTftStyleLight();
  const uint16_t bgColor = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t fgColor = lightTheme ? TFT_BLACK : TFT_WHITE;
  const uint16_t frameColor = lightTheme ? TFT_BLACK : TFT_DARKGREY;
  const uint16_t accentColor = lightTheme ? TFT_BLACK : TFT_RADIO_ORANGE;
  const uint16_t ipColor = lightTheme ? TFT_BLACK : TFT_LIGHTGREY;
  // Ciemne tĹ‚o - profesjonalny wyglÄ…d i mniejsze zmÄ™czenie oczu
  tft.fillScreen(bgColor);
  screen1HeaderNeedsRedraw = true;
  
  // 1. NAGĹĂ“WEK - PomaraĹ„czowa belka
  updateScreen1Header();

  // 2. RAMKA GĹĂ“WNA
  tft.drawRoundRect(10, 50, 300, 155, 8, frameColor);

  // 3. CALLSIGN (sp3kon) - PrzesuniÄ™ty wyĹĽej
  tft.setTextColor(fgColor);
  tft.setTextSize(4);
  String callsign = (userCallsign.length() > 0) ? userCallsign : DEFAULT_CALLSIGN;
  callsign.toUpperCase();
  int callX = (320 - (callsign.length() * 24)) / 2; // Rozmiar 4 ma ok. 24px szerokoĹ›ci
  tft.setCursor(callX > 15 ? callX : 15, 65);
  tft.print(callsign);

  // Napis "OPERATOR STATION"
  tft.setTextSize(1);
  tft.setTextColor(accentColor);
  tft.setCursor(110, 105);
  tft.print("OPERATOR STATION");

  // 4. ZEGAR UTC - WielkoĹ›Ä‡ taka sama jak Callsign (TextSize 4)
  // Ograniczenie do obszaru ramki gĹ‚Ăłwnej (x=10, szerokoĹ›Ä‡=300)
  tft.setTextColor(fgColor);
  tft.setTextSize(4);
  String timeUTC = (screen1TimeMode == SCREEN1_TIME_LOCAL)
                     ? getTimezoneTimeString("%H:%M:%S", 9)
                     : getUtcTimeString();
  int frameX = 10;        // PoczÄ…tek ramki gĹ‚Ăłwnej
  int frameWidth = 300;   // SzerokoĹ›Ä‡ ramki gĹ‚Ăłwnej
  int timeWidth = timeUTC.length() * 24; // SzerokoĹ›Ä‡ tekstu zegara
  int timeX = frameX + (frameWidth - timeWidth) / 2; // WyĹ›rodkowanie w ramce
  // Ograniczenie do obszaru ramki (minimum frameX + margines)
  timeX = (timeX < frameX + 5) ? frameX + 5 : timeX;
  tft.setCursor(timeX, 125);
  tft.print(timeUTC);

  tft.setTextSize(1);
  const char *timeLabel = "UTC";
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    timeLabel = LANG3("Czas Lokalny", "Local Time", "Hora Local");
  }
  int labelWidth = (int)strlen(timeLabel) * 6;
  int labelX = frameX + (frameWidth - labelWidth) / 2;
  tft.setCursor(labelX, 165);
  tft.print(timeLabel);

  String dateText;
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    dateText = LANG3(getPolishDateStringFullWithTimezone(),
                     getEnglishDateStringFullWithTimezone(),
                     getPortugueseDateStringFullWithTimezone());
  } else {
    dateText = LANG3(getPolishDateStringFull(),
                     getEnglishDateStringFull(),
                     getPortugueseDateStringFull());
  }
  drawDateLine(dateText);

  // 5. DOLNY PASEK - Tylko Adres IP
  IPAddress ip;
  bool connected = false;
  if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
    connected = true;
  } else if (WiFi.getMode() & WIFI_AP) {
    ip = WiFi.softAPIP();
    connected = true;
  }

  if (connected) {
    tft.setTextColor(ipColor);
    tft.setTextSize(2);
    String ipStr = ip.toString();
    int ipX = (320 - (ipStr.length() * 12)) / 2;
    tft.setCursor(ipX, 215);
    tft.print(ipStr);
  } else {
    tft.setTextColor(lightTheme ? TFT_BLACK : TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(80, 215);
    tft.print("NO CONNECTION");
  }
  drawHamClockConnectionIndicator(connected, lightTheme);
    // 4. STRZAĹKI ZMIANY EKRANU (Nawigacja)
  // Lewa strzaĹ‚ka (TrĂłjkÄ…t: x1, y1, x2, y2, x3, y3, kolor)
  tft.fillTriangle(10, 230, 20, 222, 20, 238, accentColor);
  
  // Prawa strzaĹ‚ka
  tft.fillTriangle(310, 230, 300, 222, 300, 238, accentColor);
  

}

void updateScreen1Clock() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  String timeUTC = (screen1TimeMode == SCREEN1_TIME_LOCAL)
                     ? getTimezoneTimeString("%H:%M:%S", 9)
                     : getUtcTimeString();
  
  // UĹĽyj tych samych wspĂłĹ‚rzÄ™dnych co w drawHamClock() - ograniczenie do obszaru ramki
  int frameX = 10;        // PoczÄ…tek ramki gĹ‚Ăłwnej
  int frameWidth = 300;   // SzerokoĹ›Ä‡ ramki gĹ‚Ăłwnej
  int timeWidth = timeUTC.length() * 24; // SzerokoĹ›Ä‡ tekstu zegara
  int timeX = frameX + (frameWidth - timeWidth) / 2; // WyĹ›rodkowanie w ramce
  timeX = (timeX < frameX + 5) ? frameX + 5 : timeX;
  
  // WyczyĹ›Ä‡ tylko obszar tekstu zegara (z maĹ‚ym marginesem), nie caĹ‚y prostokÄ…t
  // TextSize 4 ma wysokoĹ›Ä‡ okoĹ‚o 32px, wiÄ™c wyczyĹ›Ä‡ trochÄ™ wiÄ™cej
  const int timeBoxY = 120;
  const int timeBoxH = 35; // WysokoĹ›Ä‡ tekstu + margines
  const int timeBoxW = timeWidth + 10; // SzerokoĹ›Ä‡ tekstu + margines po bokach
  const int timeBoxX = timeX - 5; // Margines z lewej
  
  // Ograniczenie do obszaru ramki (nie wychodĹş poza ramkÄ™)
  int cleanX = (timeBoxX < frameX + 5) ? frameX + 5 : timeBoxX;
  int cleanW = timeBoxW;
  if (cleanX + cleanW > frameX + frameWidth - 5) {
    cleanW = (frameX + frameWidth - 5) - cleanX;
  }
  
  const bool lightTheme = isTftStyleLight();
  tft.fillRect(cleanX, timeBoxY, cleanW, timeBoxH, lightTheme ? TFT_WHITE : TFT_BLACK);

  // WyĹ›wietl zegar
  tft.setTextColor(lightTheme ? TFT_BLACK : TFT_WHITE);
  tft.setTextSize(4);
  tft.setCursor(timeX, 125);
  tft.print(timeUTC);

  bool connected = false;
  if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    connected = true;
  } else if (WiFi.getMode() & WIFI_AP) {
    connected = true;
  }
  drawHamClockConnectionIndicator(connected, lightTheme);
}

void updateScreen1Date() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  struct tm timeinfo;
  bool gotTime = false;
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    gotTime = getTimeWithTimezone(&timeinfo);
  } else {
    gotTime = getLocalTime(&timeinfo, 1);
  }
  if (!gotTime) {
    return;
  }

  static int lastDay = -1;
  static int lastMonth = -1;
  static int lastYear = -1;
  if (timeinfo.tm_mday == lastDay &&
      timeinfo.tm_mon == lastMonth &&
      timeinfo.tm_year == lastYear) {
    return;
  }
  lastDay = timeinfo.tm_mday;
  lastMonth = timeinfo.tm_mon;
  lastYear = timeinfo.tm_year;

  String dateText;
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    dateText = LANG3(getPolishDateStringFullWithTimezone(),
                     getEnglishDateStringFullWithTimezone(),
                     getPortugueseDateStringFullWithTimezone());
  } else {
    dateText = LANG3(getPolishDateStringFull(),
                     getEnglishDateStringFull(),
                     getPortugueseDateStringFull());
  }
  drawDateLine(dateText);
}

void updateScreen2Clock() {
  if (!tftInitialized || currentScreen != SCREEN_DX_CLUSTER || inMenu) {
    return;
  }

  static unsigned long lastClockRedrawMs = 0;
  unsigned long now = millis();
  if (lastClockRedrawMs != 0 && (now - lastClockRedrawMs) < DX_SCREEN_MIN_REDRAW_MS) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  // WyczyĹ›Ä‡ tylko obszar godziny w nagĹ‚Ăłwku.
  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
  lastClockRedrawMs = now;
}

bool spotMatchesScreen2Filters(const DXSpot &spot) {
  return spotMatchesModeFilter(spot.mode, screen2FilterModeMask) &&
         spotMatchesBandFilter(spot, screen2FilterBandMask, SCREEN8_FILTER_BANDS, SCREEN8_FILTER_BANDS_COUNT);
}

uint8_t getModeFilterBitForIndex(int index) {
  switch (index) {
    case FILTER_MODE_CW:
      return FILTER_MODE_BIT_CW;
    case FILTER_MODE_SSB:
      return FILTER_MODE_BIT_SSB;
    case FILTER_MODE_DIGI:
      return FILTER_MODE_BIT_DIGI;
    default:
      return 0;
  }
}

uint16_t getBandFilterBitForIndex(int index) {
  if (index <= 0 || index >= 16) {
    return 0;
  }
  return (uint16_t)1u << (index - 1);
}

bool isModeFilterTileSelected(uint8_t mask, int index) {
  if (index == FILTER_MODE_ALL) {
    return mask == 0;
  }
  return (mask & getModeFilterBitForIndex(index)) != 0;
}

bool isBandFilterTileSelected(uint16_t mask, int index) {
  if (index == 0) {
    return mask == 0;
  }
  return (mask & getBandFilterBitForIndex(index)) != 0;
}

void toggleModeFilterSelection(uint8_t &mask, int index) {
  if (index == FILTER_MODE_ALL) {
    mask = 0;
    return;
  }

  uint8_t bit = getModeFilterBitForIndex(index);
  if (bit == 0) {
    return;
  }

  mask ^= bit;
}

void toggleBandFilterSelection(uint16_t &mask, int index) {
  if (index == 0) {
    mask = 0;
    return;
  }

  uint16_t bit = getBandFilterBitForIndex(index);
  if (bit == 0) {
    return;
  }

  mask ^= bit;
}

bool spotMatchesModeFilter(const String &modeRaw, uint8_t mask) {
  if (mask == 0) {
    return true;
  }

  String mode = modeRaw;
  mode.toUpperCase();

  if ((mask & FILTER_MODE_BIT_CW) && mode == "CW") {
    return true;
  }
  if ((mask & FILTER_MODE_BIT_SSB) && mode == "SSB") {
    return true;
  }
  if ((mask & FILTER_MODE_BIT_DIGI) && (mode == "FT8" || mode == "FT4")) {
    return true;
  }
  return false;
}

bool spotMatchesBandFilter(const DXSpot &spot, uint16_t mask, const char **bands, int bandCount) {
  if (mask == 0) {
    return true;
  }

  float freqMHz = spot.frequency;
  for (int index = 1; index < bandCount; index++) {
    uint16_t bit = getBandFilterBitForIndex(index);
    if ((mask & bit) == 0) {
      continue;
    }

    String selectedBand = bands[index];
    if (selectedBand == "VHF") {
      if (freqMHz >= 30.0f && freqMHz < 300.0f) {
        return true;
      }
      continue;
    }
    if (selectedBand == "UHF") {
      if (freqMHz >= 300.0f && freqMHz < 3000.0f) {
        return true;
      }
      continue;
    }
    if (selectedBand == "SHF") {
      if (freqMHz >= 3000.0f && freqMHz < 30000.0f) {
        return true;
      }
      continue;
    }
    if (spot.band == selectedBand) {
      return true;
    }
  }

  return false;
}

bool spotMatchesScreen7Filters(const DXSpot &spot) {
  return spotMatchesModeFilter(spot.mode, screen7FilterModeMask) &&
         spotMatchesBandFilter(spot, screen7FilterBandMask, SCREEN2_FILTER_BANDS, SCREEN2_FILTER_BANDS_COUNT);
}

bool spotMatchesScreen8Filters(const DXSpot &spot) {
  return spotMatchesModeFilter(spot.mode, screen8FilterModeMask) &&
         spotMatchesBandFilter(spot, screen8FilterBandMask, SCREEN8_FILTER_BANDS, SCREEN8_FILTER_BANDS_COUNT);
}

float getAprsSortDistance(const APRSStation &s) {
  if (s.hasLatLon && s.distance > 0) {
    return s.distance;
  }
  return 1.0e9f;
}

static int roundAprsDistanceKm(float km) {
  // Odległość APRS jest nieujemna; +0.5f daje zaokrąglenie do pełnego km zamiast obcinania.
  return (int)(km + 0.5f);
}

bool aprsSortLess(int a, int b) {
  if (screen6SortMode == APRS_SORT_CALLSIGN) {
    String ca = aprsStations[a].callsign;
    String cb = aprsStations[b].callsign;
    ca.toUpperCase();
    cb.toUpperCase();
    if (ca == cb) {
      return a < b;
    }
    return ca < cb;
  }
  if (screen6SortMode == APRS_SORT_DISTANCE) {
    float da = getAprsSortDistance(aprsStations[a]);
    float db = getAprsSortDistance(aprsStations[b]);
    if (da == db) {
      String ca = aprsStations[a].callsign;
      String cb = aprsStations[b].callsign;
      ca.toUpperCase();
      cb.toUpperCase();
      return ca < cb;
    }
    return da < db;
  }
  // APRS_SORT_TIME = aktualna kolejność
  return a < b;
}

void buildAprsDisplayOrder(int *order, int &count) {
  count = min(aprsStationCount, MAX_APRS_DISPLAY_LCD);
  for (int i = 0; i < count; i++) {
    order[i] = i;
  }
  if (screen6SortMode == APRS_SORT_TIME) {
    return;
  }
  for (int i = 0; i < count - 1; i++) {
    int best = i;
    for (int j = i + 1; j < count; j++) {
      if (aprsSortLess(order[j], order[best])) {
        best = j;
      }
    }
    if (best != i) {
      int tmp = order[i];
      order[i] = order[best];
      order[best] = tmp;
    }
  }
}

uint32_t computeScreen2Signature() {
  // Prosty hash treĹ›ci tabeli (10 lub 11 wierszy zależnie od paska nawigacji)
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  lockDxSpots();
  hash ^= (uint32_t)spotCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen2FilterModeMask;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen2FilterBandMask;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_DX_CLUSTER) ? 1u : 0u;
  hash *= fnvPrime;

  int maxRows = getDxTableMaxRows();
  int displayCount = 0;
  for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
    const DXSpot &s = spots[i];
    if (!spotMatchesScreen2Filters(s)) {
      continue;
    }
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)(s.frequency * 100);
    hash *= fnvPrime;
    for (size_t j = 0; j < s.mode.length(); j++) {
      hash ^= (uint8_t)s.mode[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.country.length(); j++) {
      hash ^= (uint8_t)s.country[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)s.distance;
    hash *= fnvPrime;
    displayCount++;
  }

  unlockDxSpots();

  return hash;
}

void updateScreen2Data() {
  if (!tftInitialized || currentScreen != SCREEN_DX_CLUSTER || inMenu) {
    return;
  }

  // 1) Zaktualizuj czas w nagĹ‚Ăłwku (bez peĹ‚nego odĹ›wieĹĽania)
  updateScreen2Clock();

  // 2) OdĹ›wieĹĽ tabelÄ™ tylko gdy dane siÄ™ zmieniĹ‚y
  static uint32_t lastSig = 0;
  static unsigned long lastTableRedrawMs = 0;
  uint32_t currentSig = computeScreen2Signature();
  if (currentSig == lastSig) {
    return;
  }

  unsigned long now = millis();
  if (lastTableRedrawMs != 0 && (now - lastTableRedrawMs) < DX_SCREEN_MIN_REDRAW_MS) {
    return;
  }

  lastSig = currentSig;
  lastTableRedrawMs = now;

  // 3) Renderuj tabelÄ™ do bufora i wypchnij jednym ruchem (bez migotania)
  const bool navVisible = isTableNavFooterVisible(SCREEN_DX_CLUSTER);
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  const int maxRows = getDxTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_DX_CLUSTER);
  const int tableHeight = tableBottom - tableTop;
  TFT_eSprite *tableSprite = (navVisible && ensureSharedTableSprite()) ? &sharedTableSprite : nullptr;

  if (tableSprite != nullptr) {
    tableSprite->fillSprite(tftUiPalette.bg);

    int yPos = 8;
    tableSprite->setTextColor(tftUiPalette.muted);
    tableSprite->setTextSize(1);
    if (enlarged) {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(74, yPos);  tableSprite->print("CALL");
      tableSprite->setCursor(191, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(274, yPos); tableSprite->print("MODE");
    } else {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(50, yPos);  tableSprite->print("CALLSIGN");
      tableSprite->setCursor(125, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(182, yPos); tableSprite->print("MODE");
      tableSprite->setCursor(212, yPos); tableSprite->print(tr(TR_COUNTRY));
    }
    tableSprite->drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
    yPos += enlarged ? 20 : 18;

    int displayCount = 0;
    lockDxSpots();
    for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
      if (!spotMatchesScreen2Filters(spots[i])) {
        continue;
      }
      if (yPos >= (tableHeight - 2)) {
        break;
      }
      if (enlarged) {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
        }

        tableSprite->setTextSize(2);
        tableSprite->setTextColor(tftUiPalette.muted);
        tableSprite->setCursor(5, yPos);
        tableSprite->print(formatSpotUtc(spots[i].time));

        tableSprite->setTextColor(tftUiPalette.fg);
        tableSprite->setCursor(74, yPos);
        String callText = spots[i].callsign;
        if (callText.length() > 9) callText = callText.substring(0, 9);
        tableSprite->print(callText);

        tableSprite->setTextColor(tftTableFreqColor());
        tableSprite->setCursor(191, yPos);
        tableSprite->print(spots[i].frequency / 1000.0, 3);

        tableSprite->setTextColor(tftTableModeColor());
        tableSprite->setCursor(274, yPos);
        String modeText = spots[i].mode;
        if (modeText.length() > 4) modeText = modeText.substring(0, 4);
        tableSprite->print(modeText);

        yPos += 27;
      } else {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
        }

        tableSprite->setTextSize(1);

        if (medium) tableSprite->setTextFont(2);

        tableSprite->setTextColor(tftUiPalette.muted);
        tableSprite->setCursor(5, yPos);
        tableSprite->print(formatSpotUtc(spots[i].time));

        tableSprite->setTextColor(tftUiPalette.fg);
        tableSprite->setCursor(50, yPos);
        tableSprite->print(medium ? String(spots[i].callsign).substring(0, 9) : String(spots[i].callsign));

        tableSprite->setTextColor(tftTableFreqColor());
        tableSprite->setCursor(125, yPos);
        tableSprite->print(spots[i].frequency / 1000.0, 3);

        tableSprite->setTextColor(tftTableModeColor());
        tableSprite->setCursor(182, yPos);
        tableSprite->print(spots[i].mode);

        tableSprite->setTextColor(tftTableAccentTextColor());
        tableSprite->setCursor(212, yPos);
        String countryText = formatDistanceOrCountry(spots[i], COUNTRY_COL_MAX_LEN);
        tableSprite->print(countryText);

        if (medium) tableSprite->setTextFont(1);

        yPos += medium ? 20 : 17;
      }
      displayCount++;
    }
    unlockDxSpots();

    if (displayCount == 0) {
      tableSprite->setTextColor(TFT_RED);
      tableSprite->setTextSize(2);
      tableSprite->setCursor(40, 120 - tableTop);
      tableSprite->print("WAITING FOR SPOTS...");
    }

    tableSprite->pushSprite(0, tableTop);
    return;
  }

  // Fallback bez sprite (np. gdy zabraknie RAM)
  tft.fillRect(0, tableTop, 320, tableHeight, tftUiPalette.bg);

  int yPos = 40;
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(191, yPos); tft.print("MHz");
    tft.setCursor(274, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(182, yPos); tft.print("MODE");
    tft.setCursor(212, yPos); tft.print(tr(TR_COUNTRY));
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  int displayCount = 0;
  lockDxSpots();
  for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen2Filters(spots[i])) {
      continue;
    }
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
      }

      tft.setTextSize(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(74, yPos);
      String callText = spots[i].callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(191, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(274, yPos);
      String modeText = spots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
      }

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(spots[i].callsign).substring(0, 9) : String(spots[i].callsign));

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(125, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(182, yPos);
      tft.print(spots[i].mode);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(212, yPos);
      String countryText = formatDistanceOrCountry(spots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
    displayCount++;
  }
  unlockDxSpots();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

// Ekran 7: POTA Cluster (SSB only, 10 spotów)
void drawPotaCluster() {
  // Użyj tych samych współrzędnych co w drawHamClock() - ograniczenie do obszaru ramki

  tft.fillRect(0, 0, 320, 32, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());

  drawHamburgerMenuButton3D(5, 7);

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print("POTA.app");
  

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[10];
    strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  int yPos = 40;
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);

  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(191, yPos); tft.print("MHz");
    tft.setCursor(274, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(182, yPos); tft.print("MODE");
    tft.setCursor(212, yPos); tft.print(tr(TR_COUNTRY));
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  int maxRows = getPotaTableMaxRows();
  int displayCount = 0;
  lockPotaSpots();
  for (int i = 0; i < potaSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
    if (enlarged) {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);

      tft.setTextSize(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(potaSpots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(74, yPos);
      String callText = potaSpots[i].callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(191, yPos);
      tft.print(potaSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(274, yPos);
      String modeText = potaSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      String timeStr = potaSpots[i].time;
      // Obsłuż ISO 8601 z datą: wyciągnij HH:MM
      int tPos = timeStr.indexOf('T');
      if (tPos > 0 && tPos + 5 < (int)timeStr.length()) {
        timeStr = timeStr.substring(tPos + 1, tPos + 6);
      }
      if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
      tft.print(timeStr);

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(potaSpots[i].callsign).substring(0, 9) : String(potaSpots[i].callsign));

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(125, yPos);
      tft.print(potaSpots[i].frequency, 3); // frequency już w MHz

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(182, yPos);
      tft.print(potaSpots[i].mode);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(212, yPos);
      String countryText = formatDistanceOrCountry(potaSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
    displayCount++;
  }
  unlockPotaSpots();

  if (isTableNavFooterVisible(SCREEN_POTA_CLUSTER)) {
    drawSwitchScreenFooter();
  }

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

void updateScreen7Clock() {
  if (!tftInitialized || currentScreen != 7 || inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen7Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  lockPotaSpots();
  hash ^= (uint32_t)potaSpotCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen7FilterModeMask;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen7FilterBandMask;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_POTA_CLUSTER) ? 1u : 0u;
  hash *= fnvPrime;

  int maxDisplaySpots = min(potaSpotCount, getPotaTableMaxRows());
  int displayCount = 0;
  for (int i = 0; i < potaSpotCount && displayCount < maxDisplaySpots; i++) {
    const DXSpot &s = potaSpots[i];
    if (!spotMatchesScreen7Filters(s)) continue;
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }

    hash ^= (uint32_t)(s.frequency * 100);
    hash *= fnvPrime;
    for (size_t j = 0; j < s.mode.length(); j++) {
      hash ^= (uint8_t)s.mode[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.country.length(); j++) {
      hash ^= (uint8_t)s.country[j];
      hash *= fnvPrime;
    }
    displayCount++;
  }
  unlockPotaSpots();

  return hash;
}

void updateScreen7Data() {
  if (!tftInitialized || currentScreen != 7 || inMenu) {
    return;
  }

  updateScreen7Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen7Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  const bool navVisible = isTableNavFooterVisible(SCREEN_POTA_CLUSTER);
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  const int maxRows = getPotaTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_POTA_CLUSTER);
  const int tableHeight = tableBottom - tableTop;
  TFT_eSprite *tableSprite = (navVisible && ensureSharedTableSprite()) ? &sharedTableSprite : nullptr;

  if (tableSprite != nullptr) {
    tableSprite->fillSprite(tftUiPalette.bg);

    int yPos = 8;
    tableSprite->setTextColor(tftUiPalette.muted);
    tableSprite->setTextSize(1);
    if (enlarged) {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(74, yPos);  tableSprite->print("CALL");
      tableSprite->setCursor(191, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(274, yPos); tableSprite->print("MODE");
    } else {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(50, yPos);  tableSprite->print("CALLSIGN");
      tableSprite->setCursor(125, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(182, yPos); tableSprite->print("MODE");
      tableSprite->setCursor(212, yPos); tableSprite->print(tr(TR_COUNTRY));
    }
    tableSprite->drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
    yPos += enlarged ? 20 : 18;

    int displayCount = 0;
    lockPotaSpots();
    for (int i = 0; i < potaSpotCount && displayCount < maxRows; i++) {
      if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
      if (yPos >= (tableHeight - 2)) {
        break;
      }
      if (enlarged) {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
        }

        tableSprite->setTextSize(2);

        tableSprite->setTextColor(tftUiPalette.muted);
        tableSprite->setCursor(5, yPos);
        tableSprite->print(formatSpotUtc(potaSpots[i].time));

        tableSprite->setTextColor(tftUiPalette.fg);
        tableSprite->setCursor(74, yPos);
        String callText = potaSpots[i].callsign;
        if (callText.length() > 9) callText = callText.substring(0, 9);
        tableSprite->print(callText);

        tableSprite->setTextColor(tftTableFreqColor());
        tableSprite->setCursor(191, yPos);
        tableSprite->print(potaSpots[i].frequency, 3);

        tableSprite->setTextColor(tftTableModeColor());
        tableSprite->setCursor(274, yPos);
        String modeText = potaSpots[i].mode;
        if (modeText.length() > 4) modeText = modeText.substring(0, 4);
        tableSprite->print(modeText);

        yPos += 27;
      } else {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
        }

        tableSprite->setTextSize(1);

        if (medium) tableSprite->setTextFont(2);

        tableSprite->setTextColor(tftUiPalette.muted);
        tableSprite->setCursor(5, yPos);
        String timeStr = potaSpots[i].time;
        int tPos = timeStr.indexOf('T');
        if (tPos > 0 && tPos + 5 < (int)timeStr.length()) {
          timeStr = timeStr.substring(tPos + 1, tPos + 6);
        }
        if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
        tableSprite->print(timeStr);

        tableSprite->setTextColor(tftUiPalette.fg);
        tableSprite->setCursor(50, yPos);
        tableSprite->print(medium ? String(potaSpots[i].callsign).substring(0, 9) : String(potaSpots[i].callsign));

        tableSprite->setTextColor(tftTableFreqColor());
        tableSprite->setCursor(125, yPos);
        tableSprite->print(potaSpots[i].frequency, 3);

        tableSprite->setTextColor(tftTableModeColor());
        tableSprite->setCursor(182, yPos);
        tableSprite->print(potaSpots[i].mode);

        tableSprite->setTextColor(tftTableAccentTextColor());
        tableSprite->setCursor(212, yPos);
        String countryText = formatDistanceOrCountry(potaSpots[i], COUNTRY_COL_MAX_LEN);
        tableSprite->print(countryText);

        if (medium) tableSprite->setTextFont(1);

        yPos += medium ? 20 : 17;
      }
      displayCount++;
    }
    unlockPotaSpots();

    if (displayCount == 0) {
      tableSprite->setTextColor(TFT_RED);
      tableSprite->setTextSize(2);
      tableSprite->setCursor(40, 120 - tableTop);
      tableSprite->print("WAITING FOR SPOTS...");
    }

    tableSprite->pushSprite(0, tableTop);
    return;
  }

  // Fallback bez sprite (np. gdy zabraknie RAM)
  tft.fillRect(0, tableTop, 320, tableHeight, tftUiPalette.bg);

  int yPos = 40;
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(191, yPos); tft.print("MHz");
    tft.setCursor(274, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(182, yPos); tft.print("MODE");
    tft.setCursor(212, yPos); tft.print(tr(TR_COUNTRY));
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  int displayCount = 0;
  lockPotaSpots();
  for (int i = 0; i < potaSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
      }

      tft.setTextSize(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(potaSpots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(74, yPos);
      String callText = potaSpots[i].callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(191, yPos);
      tft.print(potaSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(274, yPos);
      String modeText = potaSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
      }

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      String timeStr = potaSpots[i].time;
      int tPos = timeStr.indexOf('T');
      if (tPos > 0 && tPos + 5 < (int)timeStr.length()) {
        timeStr = timeStr.substring(tPos + 1, tPos + 6);
      }
      if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
      tft.print(timeStr);

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(potaSpots[i].callsign).substring(0, 9) : String(potaSpots[i].callsign));

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(125, yPos);
      tft.print(potaSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(182, yPos);
      tft.print(potaSpots[i].mode);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(212, yPos);
      String countryText = formatDistanceOrCountry(potaSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
    displayCount++;
  }
  unlockPotaSpots();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

static String formatHamalertTftTime(const String &rawTime) {
  String t = rawTime;
  t.trim();
  if (t.length() == 0) return "-----";

  int tPos = t.indexOf('T');
  if (tPos > 0 && (tPos + 5) < (int)t.length()) {
    String hhmm = t.substring(tPos + 1, tPos + 6);
    if (hhmm.length() == 5) {
      return hhmm;
    }
  }

  if (t.length() >= 5 && t.charAt(2) == ':') {
    return t.substring(0, 5);
  }

  if (t.length() >= 5 && t.charAt(4) == 'Z' && isDigit((unsigned char)t.charAt(0)) && isDigit((unsigned char)t.charAt(1)) && isDigit((unsigned char)t.charAt(2)) && isDigit((unsigned char)t.charAt(3))) {
    String hhmm = t.substring(0, 4);
    return hhmm.substring(0, 2) + ":" + hhmm.substring(2, 4);
  }

  return t;
}

void drawHamalertCluster() {
  tft.fillScreen(tftUiPalette.bg);

  tft.fillRect(0, 0, 320, 32, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());

  drawHamburgerMenuButton3D(5, 7);

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print("HAMALERT.org");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[10];
    strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  int yPos = 40;
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(191, yPos); tft.print("MHz");
    tft.setCursor(274, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(182, yPos); tft.print("MODE");
    tft.setCursor(212, yPos); tft.print(tr(TR_COUNTRY));
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  const int maxRows = getHamalertTableMaxRows();
  int displayCount = 0;
  lockHamalertSpots();
  for (int i = 0; i < hamalertSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen8Filters(hamalertSpots[i])) {
      continue;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
      }

      tft.setTextSize(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(74, yPos);
      String callText = hamalertSpots[i].callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(191, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(274, yPos);
      String modeText = hamalertSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
      }

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(hamalertSpots[i].callsign).substring(0, 9) : String(hamalertSpots[i].callsign));

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(125, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(182, yPos);
      tft.print(hamalertSpots[i].mode);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(212, yPos);
      String countryText = formatDistanceOrCountry(hamalertSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
    displayCount++;
  }
  unlockHamalertSpots();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

void updateScreen8Clock() {
  if (!tftInitialized || currentScreen != SCREEN_HAMALERT_CLUSTER || inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen8Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  lockHamalertSpots();
  hash ^= (uint32_t)hamalertSpotCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen8FilterModeMask;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen8FilterBandMask;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_HAMALERT_CLUSTER) ? 1u : 0u;
  hash *= fnvPrime;

  int maxRows = getHamalertTableMaxRows();
  int displayCount = 0;
  for (int i = 0; i < hamalertSpotCount && displayCount < maxRows; i++) {
    const DXSpot &s = hamalertSpots[i];
    if (!spotMatchesScreen8Filters(s)) {
      continue;
    }
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)(s.frequency * 100);
    hash *= fnvPrime;
    for (size_t j = 0; j < s.mode.length(); j++) {
      hash ^= (uint8_t)s.mode[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.country.length(); j++) {
      hash ^= (uint8_t)s.country[j];
      hash *= fnvPrime;
    }
    displayCount++;
  }
  unlockHamalertSpots();

  return hash;
}

void updateScreen8Data() {
  if (!tftInitialized || currentScreen != SCREEN_HAMALERT_CLUSTER || inMenu) {
    return;
  }

  updateScreen8Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen8Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  const int maxRows = getHamalertTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_HAMALERT_CLUSTER);
  const int tableHeight = tableBottom - tableTop;

  tft.fillRect(0, tableTop, 320, tableHeight, tftUiPalette.bg);

  int yPos = 40;
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(191, yPos); tft.print("MHz");
    tft.setCursor(274, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(182, yPos); tft.print("MODE");
    tft.setCursor(212, yPos); tft.print(tr(TR_COUNTRY));
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  int displayCount = 0;
  lockHamalertSpots();
  for (int i = 0; i < hamalertSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen8Filters(hamalertSpots[i])) {
      continue;
    }
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
      }

      tft.setTextSize(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(74, yPos);
      String callText = hamalertSpots[i].callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(191, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(274, yPos);
      String modeText = hamalertSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
      }

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(hamalertSpots[i].callsign).substring(0, 9) : String(hamalertSpots[i].callsign));

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(125, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(182, yPos);
      tft.print(hamalertSpots[i].mode);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(212, yPos);
      String countryText = formatDistanceOrCountry(hamalertSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
    displayCount++;
  }
  unlockHamalertSpots();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

bool readRawTouchPoint(int16_t &rx, int16_t &ry) {
  if (!touch.touched()) {
    return false;
  }
  TS_Point p = touch.getPoint();
  rx = p.x;
  ry = p.y;
  return true;
}

void mapRawToScreenWithValues(int16_t rx, int16_t ry,
                              int xmin, int xmax, int ymin, int ymax,
                              bool swapXY, bool invertX, bool invertY,
                              uint16_t &x, uint16_t &y) {
  int16_t mx = rx;
  int16_t my = ry;

  if (swapXY) {
    int16_t tmp = mx;
    mx = my;
    my = tmp;
  }
  if (invertX) {
    mx = xmax - (mx - xmin);
  }
  if (invertY) {
    my = ymax - (my - ymin);
  }

  mx = constrain(mx, xmin, xmax);
  my = constrain(my, ymin, ymax);

  int screenW = tft.width();
  int screenH = tft.height();
  x = map(mx, xmin, xmax, 0, screenW - 1);
  y = map(my, ymin, ymax, 0, screenH - 1);
}

void mapRawToScreen(int16_t rx, int16_t ry, uint16_t &x, uint16_t &y) {
  mapRawToScreenWithValues(rx, ry, touchXMin, touchXMax, touchYMin, touchYMax,
                           touchSwapXY, touchInvertX, touchInvertY, x, y);
}

bool readTouchPoint(uint16_t &x, uint16_t &y) {
  int16_t rx = 0;
  int16_t ry = 0;
  if (!readRawTouchPoint(rx, ry)) {
    return false;
  }
  mapRawToScreen(rx, ry, x, y);
  return true;
}

const int SCREEN_ORDER_COUNT = SCREEN_PAGE_COUNT;

ScreenType normalizeScreenType(uint8_t raw) {
  switch (raw) {
    case SCREEN_HAM_CLOCK:
    case SCREEN_DX_CLUSTER:
    case SCREEN_SUN_SPOTS:
    case SCREEN_BAND_INFO:
    case SCREEN_WEATHER_DSP:
    case SCREEN_WEATHER_FORECAST:
    case SCREEN_LOKALIZACJA:
    case SCREEN_APRS_IS:
    case SCREEN_APRS_RADAR:
    case SCREEN_POTA_CLUSTER:
    case SCREEN_HAMALERT_CLUSTER:
    case SCREEN_MATRIX_CLOCK:
    case SCREEN_UNLIS_HUNTER:
      return (ScreenType)raw;
    default:
      return SCREEN_OFF;
  }
}

const char* screenTypeToCodeStr(ScreenType t) {
  switch (t) {
    case SCREEN_HAM_CLOCK: return "hamclock";
    case SCREEN_DX_CLUSTER: return "dxcluster";
    case SCREEN_APRS_IS: return "aprsis";
    case SCREEN_APRS_RADAR: return "aprsradar";
    case SCREEN_BAND_INFO: return "bandinfo";
    case SCREEN_SUN_SPOTS: return "sunspots";
    case SCREEN_WEATHER_DSP: return "weather";
    case SCREEN_WEATHER_FORECAST: return "weatherforecast";
    case SCREEN_LOKALIZACJA: return "lokalizacja";
    case SCREEN_POTA_CLUSTER: return "pota";
    case SCREEN_HAMALERT_CLUSTER: return "hamalert";
    case SCREEN_MATRIX_CLOCK: return "matrix";
    case SCREEN_UNLIS_HUNTER: return "unlishunter";
    case SCREEN_OFF:
    default:
      return "off";
  }
}

ScreenType screenCodeToType(const String &code) {
  String c = code;
  c.toLowerCase();
  c.trim();
  if (c == "hamclock") return SCREEN_HAM_CLOCK;
  if (c == "dxcluster") return SCREEN_DX_CLUSTER;
  if (c == "aprsis") return SCREEN_APRS_IS;
  if (c == "aprsradar") return SCREEN_APRS_RADAR;
  if (c == "bandinfo") return SCREEN_BAND_INFO;
  if (c == "sunspots") return SCREEN_SUN_SPOTS;
  if (c == "weather") return SCREEN_WEATHER_DSP;
  if (c == "weatherforecast" || c == "forecast") return SCREEN_WEATHER_FORECAST;
  if (c == "lokalizacja" || c == "location" || c == "gps") return SCREEN_LOKALIZACJA;
  if (c == "pota") return SCREEN_POTA_CLUSTER;
  if (c == "hamalert") return SCREEN_HAMALERT_CLUSTER;
  if (c == "matrix") return SCREEN_MATRIX_CLOCK;
  if (c == "unlishunter") return SCREEN_UNLIS_HUNTER;
  return SCREEN_OFF;
}

void loadDefaultScreenOrder() {
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    screenOrder[i] = DEFAULT_SCREEN_ORDER[i];
  }
}

void ensureScreenOrderValid() {
  bool hasActive = false;
  bool seen[14] = {false}; // indeksy odpowiadają ScreenType wartościom (0..13)
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    screenOrder[i] = normalizeScreenType(screenOrder[i]);
    ScreenType t = screenOrder[i];
    if (t != SCREEN_OFF) {
      if (t >= 0 && t <= SCREEN_LOKALIZACJA) {
        if (seen[t]) {
          // drugi raz ten sam ekran – usuń duplikat
          screenOrder[i] = SCREEN_OFF;
          continue;
        }
        seen[t] = true;
      }
      hasActive = true;
    }
  }
  if (!hasActive) {
    loadDefaultScreenOrder();
  }
}

int findScreenOrderIndex(ScreenType screenId) {
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    if (screenOrder[i] == screenId) {
      return i;
    }
  }
  return 0;
}

ScreenType firstActiveScreen() {
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    if (screenOrder[i] != SCREEN_OFF) {
      return screenOrder[i];
    }
  }
  return SCREEN_HAM_CLOCK;
}

ScreenType getNextScreenId(ScreenType currentId) {
  int idx = findScreenOrderIndex(currentId);
  for (int step = 1; step <= SCREEN_ORDER_COUNT; step++) {
    ScreenType candidate = screenOrder[(idx + step) % SCREEN_ORDER_COUNT];
    if (candidate != SCREEN_OFF) {
      return candidate;
    }
  }
  return SCREEN_HAM_CLOCK;
}

ScreenType getPrevScreenId(ScreenType currentId) {
  int idx = findScreenOrderIndex(currentId);
  for (int step = 1; step <= SCREEN_ORDER_COUNT; step++) {
    int prev = idx - step;
    if (prev < 0) {
      prev += SCREEN_ORDER_COUNT;
    }
    ScreenType candidate = screenOrder[prev];
    if (candidate != SCREEN_OFF) {
      return candidate;
    }
  }
  return SCREEN_HAM_CLOCK;
}

static int normalizeTftAutoSwitchTimeSec(int seconds) {
  if (seconds < 1) return 1;
  if (seconds > 3600) return 3600;
  return seconds;
}

static void applyTftAutoSwitchTimeSec(int seconds) {
  tftAutoSwitchTimeSec = normalizeTftAutoSwitchTimeSec(seconds);
}

static void resetTftAutoSwitchTimer() {
  tftAutoSwitchLastMs = millis();
  tftAutoSwitchLastScreen = currentScreen;
}

void handleTouchNavigation() {
  if (!tftInitialized) {
    return;
  }

  const uint16_t menuHitW = 50;
  const uint16_t menuHitH = 50;

  static bool touchActive = false;
  static unsigned long lastTouchMs = 0;
  static unsigned long touchStartMs = 0;
  static uint16_t touchStartX = 0;
  static uint16_t touchStartY = 0;
  static bool longPressHandled = false;
  static bool matrixGameHoldCandidate = false;
  unsigned long now = millis();

  uint16_t x = 0;
  uint16_t y = 0;
  int16_t rawX = 0;
  int16_t rawY = 0;
  if (readRawTouchPoint(rawX, rawY)) {
    mapRawToScreen(rawX, rawY, x, y);
    bool isNewTap = false;
    if (!touchActive) {
      if ((now - lastTouchMs) <= 150) {
        return;
      }
      touchActive = true;
      lastTouchMs = now;
      touchStartMs = now;
      touchStartX = x;
      touchStartY = y;
      longPressHandled = false;
      matrixGameHoldCandidate = (currentScreen == SCREEN_MATRIX_CLOCK && !inMenu && x >= 240 && y <= 80);
      isNewTap = true;
    }

    if (touchCalActive) {
      handleTouchCalibrationTouch(rawX, rawY, x, y, isNewTap);
      return;
    }

    if (aprsAlertScreenActive) {
      if (isNewTap && isAprsAlertCloseButtonHit(x, y)) {
        dismissAprsAlertScreen();
      }
      return;
    }

    if (!brightnessMenuActive && !longPressHandled && matrixGameHoldCandidate) {
      if (currentScreen == SCREEN_MATRIX_CLOCK && !inMenu && (now - touchStartMs) >= 2000) {
        currentScreen = SCREEN_UNLIS_HUNTER;
        drawScreen(currentScreen);
        resetTftAutoSwitchTimer();
        longPressHandled = true;
        matrixGameHoldCandidate = false;
        return;
      }
    }

    if (!brightnessMenuActive && !longPressHandled) {
      if (now - touchStartMs >= 3000) {
        brightnessMenuPrevScreen = currentScreen;
        brightnessMenuPrevInMenu = inMenu;
        brightnessMenuPrevBacklight = backlightPercent;
        brightnessMenuPrevThemeHue = menuThemeHue;
        brightnessMenuPrevStyleMode = tftStyleMode;
        brightnessMenuActive = true;
        brightnessMenuValue = backlightPercent;
        brightnessMenuOpenedMs = now;
        inMenu = true;
        drawBrightnessMenu();
        longPressHandled = true;
        return;
      }
    }

    if (brightnessMenuActive) {
      handleBrightnessMenuTouch(x, y, isNewTap);
      return;
    }

    if (!isNewTap) {
      return;
    }

    if (currentScreen == SCREEN_UNLIS_HUNTER) {
      if (x >= UNLIS_EXIT_X && x < (UNLIS_EXIT_X + UNLIS_BTN_SIZE) &&
          y >= UNLIS_EXIT_Y && y < (UNLIS_EXIT_Y + UNLIS_BTN_SIZE)) {
        currentScreen = SCREEN_MATRIX_CLOCK;
        drawScreen(currentScreen);
        resetTftAutoSwitchTimer();
        return;
      }
      if (x >= UNLIS_START_X && x < (UNLIS_START_X + UNLIS_BTN_SIZE) &&
          y >= UNLIS_START_Y && y < (UNLIS_START_Y + UNLIS_BTN_SIZE)) {
        if (unlisRunning && !unlisGameOver) {
          unlisStopGame();
        } else {
          unlisStartResetGame();
        }
        return;
      }
      if (x >= UNLIS_PTT_X && x < (UNLIS_PTT_X + UNLIS_BTN_SIZE) &&
          y >= UNLIS_PTT_Y && y < (UNLIS_PTT_Y + UNLIS_BTN_SIZE)) {
        unlisHandlePttPress(now);
        return;
      }
      return;
    }

    if (inMenu) {
      if (currentScreen == SCREEN_HAM_CLOCK) {
        handleScreen1MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_DX_CLUSTER) {
        handleScreen2MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_POTA_CLUSTER) {
        handleScreen7MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_HAMALERT_CLUSTER) {
        handleScreen8MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) {
        handleScreen6MenuTouch(x, y);
      }
      return;
    }

    if (currentScreen == SCREEN_DX_CLUSTER) {
      if (x < menuHitW && y < menuHitH) {
          inMenu = true;
          drawDxClusterFilterMenu();
          return;
        }
      }
    if (currentScreen == SCREEN_POTA_CLUSTER) {
        if (x < menuHitW && y < menuHitH) {
          inMenu = true;
          drawPotaFilterMenu();
          return;
        }
      }
    if (currentScreen == SCREEN_HAMALERT_CLUSTER) {
        if (x < menuHitW && y < menuHitH) {
          inMenu = true;
          drawHamalertFilterMenu();
          return;
        }
      }
      if (currentScreen == SCREEN_APRS_IS) {
      if (x < menuHitW && y < menuHitH) {
        inMenu = true;
        screen6MenuBeaconingTemp = aprsBeaconEnabled;
        screen6MenuAprsAlertTemp = aprsAlertEnabled;
        screen6MenuRangeAlertTemp = aprsAlertNearbyEnabled;
        screen6MenuLedAlertTemp = enableLedAlert;
        drawAprsSortMenu();
        return;
      }
    }
      if (currentScreen == SCREEN_APRS_RADAR) {
      if (isScreen6RadarZoomTopHit(x, y)) {
        if ((now - screen6RadarLastZoomTapMs) < SCREEN6_RADAR_ZOOM_TAP_COOLDOWN_MS) {
          return;
        }
        screen6RadarLastZoomTapMs = now;
        float newZoom = min(SCREEN6_RADAR_ZOOM_MAX, screen6RadarZoom + SCREEN6_RADAR_ZOOM_STEP);
        if (newZoom > screen6RadarZoom + 0.0001f) {
          screen6RadarZoom = newZoom;
          triggerScreen6RadarHint();
          drawScreen(SCREEN_APRS_RADAR);
          resetTftAutoSwitchTimer();
        }
        return;
      }
      if (isScreen6RadarZoomBottomHit(x, y)) {
        if ((now - screen6RadarLastZoomTapMs) < SCREEN6_RADAR_ZOOM_TAP_COOLDOWN_MS) {
          return;
        }
        screen6RadarLastZoomTapMs = now;
        float newZoom = max(SCREEN6_RADAR_ZOOM_MIN, screen6RadarZoom - SCREEN6_RADAR_ZOOM_STEP);
        if (newZoom < screen6RadarZoom - 0.0001f) {
          screen6RadarZoom = newZoom;
          triggerScreen6RadarHint();
          drawScreen(SCREEN_APRS_RADAR);
          resetTftAutoSwitchTimer();
        }
        return;
      }
      if (x < menuHitW && y < menuHitH) {
        inMenu = true;
        screen6MenuBeaconingTemp = aprsBeaconEnabled;
        screen6MenuAprsAlertTemp = aprsAlertEnabled;
        screen6MenuRangeAlertTemp = aprsAlertNearbyEnabled;
        screen6MenuLedAlertTemp = enableLedAlert;
        drawAprsSortMenu();
        return;
      }
    }
    if (currentScreen == SCREEN_HAM_CLOCK) {
      if (x > 285 && y < 35) {
        inMenu = true;
        drawHamClockTimeMenu();
        return;
      }
    }

    if (currentScreen == SCREEN_LOKALIZACJA) {
      if (isLokalizacjaSaveButtonHit(x, y)) {
        unsigned long nowGps = millis();
        bool gpsReadyToSave = isGpsUartActive(nowGps) && isGpsFixActive(nowGps) && gpsLocationValid;
        if (gpsReadyToSave) {
          userLat = gpsLatitude;
          userLon = gpsLongitude;
          userLatLonValid = true;
          savePreferences();
          Serial.printf("[GPS] Zapisano pozycje stacji z GPS: LAT=%.6f LON=%.6f\n", userLat, userLon);
        } else {
          Serial.println("[GPS] Nie zapisano pozycji: wymagany aktywny GPS + FIX + poprawne LAT/LON");
        }
        drawLokalizacja();
        resetTftAutoSwitchTimer();
        autoswitchPausedUntilMs = 0;
        return;
      }
    }

    // Nawigacja: dolne obszary dotyku (ok. 80x180)
    const uint16_t cornerY = 60;
    const uint16_t cornerX = 80;
    if (y >= cornerY && x < cornerX) {
      currentScreen = getPrevScreenId(currentScreen);
      drawScreen(currentScreen);
      resetTftAutoSwitchTimer();
      autoswitchPausedUntilMs = 0;
      LOGV_PRINTF("[TOUCH] Ekran zmieniony na: %d (tap left)\n", currentScreen);
    } else if (y >= cornerY && x >= (320 - cornerX)) {
      currentScreen = getNextScreenId(currentScreen);
      drawScreen(currentScreen);
      resetTftAutoSwitchTimer();
      autoswitchPausedUntilMs = 0;
      LOGV_PRINTF("[TOUCH] Ekran zmieniony na: %d (tap right)\n", currentScreen);
    } else if (y >= cornerY && x >= cornerX && x < (320 - cornerX)) {
      // środek ekranu: pauzuj autoswitch na 10 sekund
      autoswitchPausedUntilMs = millis() + 10000UL;
      LOGV_PRINTF("[TOUCH] Autoswitch zapauzowany na 10s (tap center)\n");
    }
  } else {
    touchActive = false;
    longPressHandled = false;
    matrixGameHoldCandidate = false;
    // Reset liczników long-press dla menu jasności po puszczeniu dotyku
    if (brightnessMenuActive) {
      brightnessMenuTouchStartMs = 0;
      brightnessMenuLongPressHandled = false;
    }
  }
}
// Ekran 2: DX Cluster
void drawDxCluster() {
  tft.fillScreen(tftUiPalette.bg);

  // 1. NAGĹĂ“WEK: Belka z menu, nazwÄ… klastra i czasem UTC
  tft.fillRect(0, 0, 320, 32, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());

// IKONA MENU (3D)
drawHamburgerMenuButton3D(5, 7);

// Nazwa klastra - przesuniÄ™ta w prawo (x=35 zamiast 5), by zrobiÄ‡ miejsce na menu
tft.setTextSize(2);
tft.setCursor(35, 8);
tft.print(clusterHost);
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[10];
    strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  // 2. NAGĹĂ“WKI TABELI
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  int yPos = 40;
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);

  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(191, yPos); tft.print("MHz");
    tft.setCursor(274, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(182, yPos); tft.print("MODE");
    tft.setCursor(212, yPos); tft.print(tr(TR_COUNTRY));
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  // 3. LISTA SPOTĂ“W
  int maxRows = getDxTableMaxRows();
  int displayCount = 0;
  lockDxSpots();
  for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen2Filters(spots[i])) {
      continue;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
      tft.setTextSize(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(74, yPos);
      String callText = spots[i].callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(191, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(274, yPos);
      String modeText = spots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);
      yPos += 27;
    } else {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(spots[i].callsign).substring(0, 9) : String(spots[i].callsign));

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(125, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(tftTableModeColor());
      tft.setCursor(182, yPos);
      tft.print(spots[i].mode);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(212, yPos);
      String countryText = formatDistanceOrCountry(spots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
    displayCount++;
  }
  unlockDxSpots();

  // 4. Pasek nawigacji widoczny tylko przez 5 sekund od wejścia na ekran
  if (isTableNavFooterVisible(SCREEN_DX_CLUSTER)) {
    drawSwitchScreenFooter();
  }

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

bool isPointInRect(uint16_t x, uint16_t y, int rx, int ry, int rw, int rh) {
  return (x >= rx && x <= (rx + rw) && y >= ry && y <= (ry + rh));
}

void drawHamburgerMenuButton3D(int x, int y) {
  const int iconW = 20;
  const int iconH = 17;
  const int pad = 3;
  const int btnX = x - pad;
  const int btnY = y - pad;
  const int btnW = iconW + (pad * 2);
  const int btnH = iconH + (pad * 2);
  const uint16_t edgeLight = lerpColor565(TFT_WHITE, TFT_RADIO_ORANGE, 140);
  const uint16_t edgeShadow = lerpColor565(TFT_BLACK, TFT_RADIO_ORANGE, 115);

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 3, TFT_RADIO_ORANGE);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 3, edgeLight);
  tft.drawLine(btnX + 1, btnY + btnH - 1, btnX + btnW - 2, btnY + btnH - 1, edgeShadow);
  tft.drawLine(btnX + btnW - 1, btnY + 1, btnX + btnW - 1, btnY + btnH - 2, edgeShadow);

  tft.fillRect(x, y, iconW, 3, TFT_BLACK);
  tft.fillRect(x, y + 7, iconW, 3, TFT_BLACK);
  tft.fillRect(x, y + 14, iconW, 3, TFT_BLACK);
}

static void drawScreen6ViewToggleButton3D(int x, int y, Screen6ViewMode mode) {
  const int iconW = 20;
  const int iconH = 17;
  const int pad = 3;
  const int btnX = x - pad;
  const int btnY = y - pad;
  const int btnW = iconW + (pad * 2);
  const int btnH = iconH + (pad * 2);
  const uint16_t edgeLight = lerpColor565(TFT_WHITE, TFT_RADIO_ORANGE, 140);
  const uint16_t edgeShadow = lerpColor565(TFT_BLACK, TFT_RADIO_ORANGE, 115);

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 3, TFT_RADIO_ORANGE);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 3, edgeLight);
  tft.drawLine(btnX + 1, btnY + btnH - 1, btnX + btnW - 2, btnY + btnH - 1, edgeShadow);
  tft.drawLine(btnX + btnW - 1, btnY + 1, btnX + btnW - 1, btnY + btnH - 2, edgeShadow);

  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(x + 6, y + 1);
  tft.print(mode == APRS_VIEW_RADAR ? "L" : "R");
}

static bool isScreen6ViewToggleHit(uint16_t x, uint16_t y) {
  return (x >= SCREEN6_VIEW_BTN_HIT_X &&
          x <= (SCREEN6_VIEW_BTN_HIT_X + SCREEN6_VIEW_BTN_HIT_W) &&
          y >= SCREEN6_VIEW_BTN_HIT_Y &&
          y <= (SCREEN6_VIEW_BTN_HIT_Y + SCREEN6_VIEW_BTN_HIT_H));
}

static bool isScreen6RadarZoomTopHit(uint16_t x, uint16_t y) {
  if (currentScreen != SCREEN_APRS_RADAR) {
    return false;
  }
  int w = tft.width();
  int h = tft.height();
  int xMin = w / 4;
  int xMax = (3 * w) / 4;
  return (x >= xMin && x <= xMax && y < (h / 2));
}

static bool isScreen6RadarZoomBottomHit(uint16_t x, uint16_t y) {
  if (currentScreen != SCREEN_APRS_RADAR) {
    return false;
  }
  int w = tft.width();
  int h = tft.height();
  int xMin = w / 4;
  int xMax = (3 * w) / 4;
  return (x >= xMin && x <= xMax && y >= (h / 2));
}

static void triggerScreen6RadarHint() {
  screen6RadarHintUntilMs = millis() + SCREEN6_RADAR_HINT_DURATION_MS;
}

static bool isScreen6RadarHintVisible() {
  return (currentScreen == SCREEN_APRS_RADAR && millis() < screen6RadarHintUntilMs);
}

static void drawScreen6RadarZoomHints() {
  if (!isScreen6RadarHintVisible()) {
    return;
  }

  const bool lightTheme = isTftStyleLight();
  const uint16_t radarBgColor = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t radarTextColor = lightTheme ? tftUiPalette.fg : TFT_LIGHTGREY;
  const uint16_t hintTextColor = lightTheme ? TFT_NAVY : TFT_YELLOW;

  const int screenW = tft.width();
  const int screenH = tft.height();
  const char *topHint = "ZOOM +";
  const char *bottomHint = "ZOOM -";
  const int topW = (int)strlen(topHint) * 12;
  const int bottomW = (int)strlen(bottomHint) * 12;

  tft.setTextSize(2);
  tft.setTextColor(hintTextColor);
  tft.setCursor((screenW - topW) / 2, 6);
  tft.print(topHint);
  tft.setCursor((screenW - bottomW) / 2, screenH - 20);
  tft.print(bottomHint);

  const uint16_t radarAlertColor = rgb565(70, 130, 190);
  int zoomPercent = (int)(screen6RadarZoom * 100.0f + 0.5f);
  float radarVisibleMaxKm = (screen6RadarZoom > 0.0f)
                              ? ((float)aprsFilterRadius / screen6RadarZoom)
                              : (float)aprsFilterRadius;

  String zoomInfo = String("ZOOM:") + String(zoomPercent) + "%";
  String rmaxInfo = String("Rmax:") + String(radarVisibleMaxKm, 1) + "km";
  String alertInfo = String("ALERT:") + String(aprsAlertDistanceKm, 1) + "km";

  const int infoX = 2;
  const int zoomY = screenH - 29;
  const int rmaxY = screenH - 19;
  const int alertY = screenH - 9;

  tft.setTextSize(1);

  int zoomW = (int)zoomInfo.length() * 6;
  int rmaxW = (int)rmaxInfo.length() * 6;
  int alertW = (int)alertInfo.length() * 6;

  tft.fillRect(infoX - 1, zoomY - 1, zoomW + 2, 10, radarBgColor);
  tft.fillRect(infoX - 1, rmaxY - 1, rmaxW + 2, 10, radarBgColor);
  tft.fillRect(infoX - 1, alertY - 1, alertW + 2, 10, radarBgColor);

  tft.setTextColor(radarTextColor);
  tft.setCursor(infoX, zoomY);
  tft.print(zoomInfo);

  tft.setCursor(infoX, rmaxY);
  tft.print(rmaxInfo);

  tft.setTextColor(radarAlertColor);
  tft.setCursor(infoX, alertY);
  tft.print(alertInfo);
}

static uint16_t getAprsRadarColorForStation(const APRSStation &station) {
  String symbolShort = getAPRSSymbolShort(station);
  if (isTftStyleLight()) {
    return (symbolShort == "CAR") ? TFT_RED : TFT_BLACK;
  }
  if (symbolShort == "CAR") return TFT_RED;
  if (symbolShort == "HOUSE") return TFT_YELLOW;
  if (symbolShort == "HUMAN") return 0x3C1F;
  if (symbolShort == "HAMCLOCK") return TFT_ORANGE;
  return TFT_GREEN;
}

static void drawAprsRadarBody() {
  const int screenW = tft.width();
  const int screenH = tft.height();
  const bool lightTheme = isTftStyleLight();
  const uint16_t radarBgColor = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t radarMainRingColor = lightTheme ? TFT_BLACK : TFT_WHITE;
  const uint16_t radarTextColor = lightTheme ? tftUiPalette.fg : TFT_LIGHTGREY;
  const uint16_t crossColor = lightTheme ? TFT_DARKGREY : 0x39C7;
  tft.fillRect(0, 0, screenW, screenH, radarBgColor);

  const int centerX = screenW / 2;
  const int centerY = screenH / 2;
  const int pad = 6;
  int baseOuterRadius = min(min(centerX - pad, screenW - centerX - pad),
                            min(centerY - pad, screenH - centerY - pad));
  if (baseOuterRadius < 20) {
    baseOuterRadius = 20;
  }

  int outerRadius = (int)(baseOuterRadius * screen6RadarZoom);
  if (outerRadius < 20) {
    outerRadius = 20;
  }

  const int fixedOuterRadius = baseOuterRadius; // nowy, stały wizualnie pierścień 100%
  const int fixedHalfRadius = max(1, fixedOuterRadius / 2); // nowy, stały wizualnie pierścień 50%

  const float zoomSafe = (screen6RadarZoom > 0.01f) ? screen6RadarZoom : 0.01f;
  const float fixedOuterKm = (aprsFilterRadius > 0)
                              ? ((float)aprsFilterRadius / zoomSafe)
                              : 0.0f;

  const uint16_t radarCircleBgColor = radarBgColor;
  const uint16_t radarAlertColor = rgb565(70, 130, 190);
  const uint16_t ringColor = TFT_DARKGREY;
  const int innerRadius = (aprsFilterRadius > 0)
                            ? (int)((aprsAlertDistanceKm / (float)aprsFilterRadius) * outerRadius)
                            : 0;
  const int clampedInnerRadius = constrain(innerRadius, 1, outerRadius);

  if (outerRadius <= baseOuterRadius + 2) {
    tft.fillCircle(centerX, centerY, outerRadius - 1, radarCircleBgColor);
  }

  // Dodane stałe pierścienie referencyjne na spodzie (stacje są rysowane później, nad nimi)
  tft.drawCircle(centerX, centerY, fixedOuterRadius, ringColor); // 100%
  tft.drawCircle(centerX, centerY, fixedHalfRadius, ringColor);  // 50%

  // Oryginalne okręgi radaru
  tft.drawCircle(centerX, centerY, outerRadius, radarMainRingColor);
  tft.drawCircle(centerX, centerY, clampedInnerRadius, radarAlertColor);

  tft.drawFastVLine(centerX, centerY - outerRadius, outerRadius * 2, crossColor);
  tft.drawFastHLine(centerX - outerRadius, centerY, outerRadius * 2, crossColor);

  auto kmLabel = [](float km) -> String {
    if (km >= 100.0f) return String((int)(km + 0.5f)) + "km";
    if (km >= 10.0f) return String(km, 1) + "km";
    return String(km, 2) + "km";
  };

  tft.setTextSize(1);
  tft.setTextColor(radarTextColor);
  String ring100Text = kmLabel(fixedOuterKm);
  String ring50Text = kmLabel(fixedOuterKm * 0.5f);

  int ringLabelX = centerX + 4;
  int ring100Y = max(0, centerY - fixedOuterRadius + 2);
  int ring50Y = max(0, centerY - fixedHalfRadius + 2);

  tft.fillRect(ringLabelX - 1, ring100Y - 1, (int)ring100Text.length() * 6 + 2, 10, radarCircleBgColor);
  tft.fillRect(ringLabelX - 1, ring50Y - 1, (int)ring50Text.length() * 6 + 2, 10, radarCircleBgColor);
  tft.setCursor(ringLabelX, ring100Y);
  tft.print(ring100Text);
  tft.setCursor(ringLabelX, ring50Y);
  tft.print(ring50Text);

  tft.fillCircle(centerX, centerY, 4, TFT_RADIO_ORANGE);

  int order[MAX_APRS_DISPLAY_LCD];
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int maxRows = min(displayCount, MAX_APRS_DISPLAY_LCD);

  for (int i = 0; i < maxRows; i++) {
    const APRSStation &station = aprsStations[order[i]];
    if (!station.hasLatLon || station.distance <= 0.0f) {
      continue;
    }
    if (aprsFilterRadius <= 0 || station.distance > (float)aprsFilterRadius) {
      continue;
    }

    double radarUserLat = 0.0;
    double radarUserLon = 0.0;
    if (!getServiceCoordinatesForServices(radarUserLat, radarUserLon)) {
      continue;
    }
    float bearingDeg = calculateBearing(radarUserLat, radarUserLon, (double)station.lat, (double)station.lon);
    float angleRad = bearingDeg * (float)M_PI / 180.0f;
    float scaledRadius = (station.distance / (float)aprsFilterRadius) * outerRadius;
    if (scaledRadius > outerRadius) {
      continue;
    }
    int px = centerX + (int)(sin(angleRad) * scaledRadius);
    int py = centerY - (int)(cos(angleRad) * scaledRadius);

    uint16_t color = getAprsRadarColorForStation(station);
    tft.fillCircle(px, py, 3, color);

    String call = station.callsign;
    if (call.length() > 8) {
      call = call.substring(0, 8);
    }
    int tx = px - ((int)call.length() * 3);
    int ty = py - 10;
    if (tx < 1) tx = 1;
    if (tx > screenW - ((int)call.length() * 6) - 1) tx = screenW - ((int)call.length() * 6) - 1;
    if (ty < 2) ty = py + 6;
    if (ty > screenH - 9) ty = screenH - 9;

    uint16_t labelColor = lightTheme ? TFT_NAVY : color;
    tft.setTextColor(labelColor);
    tft.setTextSize(1);
    tft.setCursor(tx, ty);
    tft.print(call);

    if ((i & 0x03) == 0) {
      yield();
    }
  }

}

void drawFilterTile(int x, int y, int w, int h, const char *label, bool active) {
  uint16_t fill = active ? TFT_RADIO_ORANGE : TFT_DARKGREY;
  uint16_t text = active ? TFT_BLACK : TFT_WHITE;
  tft.fillRect(x, y, w, h, fill);
  tft.drawRect(x, y, w, h, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(text);
  int textW = strlen(label) * 6;
  int textX = x + (w - textW) / 2;
  int textY = y + (h - 8) / 2;
  tft.setCursor(textX, textY);
  tft.print(label);
}

static void drawScreen6MenuCheckButton(int x, int y, const char *label, bool checked) {
  const int boxSize = 18;
  tft.drawRect(x, y, boxSize, boxSize, TFT_WHITE);
  if (checked) {
    tft.fillRect(x + 4, y + 4, boxSize - 8, boxSize - 8, TFT_RADIO_ORANGE);
  } else {
    tft.fillRect(x + 1, y + 1, boxSize - 2, boxSize - 2, TFT_BLACK);
  }

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(x + boxSize + 10, y + 2);
  tft.print(label);
}

void drawDxClusterFilterMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("FILTERS");

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 28);
  tft.print("MODE");

  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (320 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;
  drawFilterTile(modeStartX + 0 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "ALL", isModeFilterTileSelected(screen2FilterModeMask, FILTER_MODE_ALL));
  drawFilterTile(modeStartX + 1 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "CW", isModeFilterTileSelected(screen2FilterModeMask, FILTER_MODE_CW));
  drawFilterTile(modeStartX + 2 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "SSB", isModeFilterTileSelected(screen2FilterModeMask, FILTER_MODE_SSB));
  drawFilterTile(modeStartX + 3 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "DIGI", isModeFilterTileSelected(screen2FilterModeMask, FILTER_MODE_DIGI));

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 82);
  tft.print("BAND");

  const int bandTileW = 72;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (320 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int x = bandStartX + col * (bandTileW + bandGap);
      int y = bandStartY + row * (bandTileH + bandGap);
      drawFilterTile(x, y, bandTileW, bandTileH, SCREEN8_FILTER_BANDS[bandIdx], isBandFilterTileSelected(screen2FilterBandMask, bandIdx));
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen2MenuTouch(uint16_t x, uint16_t y) {
  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (320 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;

  for (int i = 0; i < 4; i++) {
    int rx = modeStartX + i * (modeTileW + modeGap);
    if (isPointInRect(x, y, rx, modeY, modeTileW, modeTileH)) {
      toggleModeFilterSelection(screen2FilterModeMask, i);
      drawDxClusterFilterMenu();
      return;
    }
  }

  const int bandTileW = 72;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (320 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int rx = bandStartX + col * (bandTileW + bandGap);
      int ry = bandStartY + row * (bandTileH + bandGap);
      if (isPointInRect(x, y, rx, ry, bandTileW, bandTileH)) {
        toggleBandFilterSelection(screen2FilterBandMask, bandIdx);
        drawDxClusterFilterMenu();
        return;
      }
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    savePreferences();
    inMenu = false;
    drawScreen(SCREEN_DX_CLUSTER);
    return;
  }
}

void drawAprsSortMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 4);
  tft.print("SORT BY:");

  const int tileW = 90;
  const int tileH = 26;
  const int gap = 6;
  const int startX = (320 - (3 * tileW + 2 * gap)) / 2;
  const int tileY = 32;
  drawFilterTile(startX + 0 * (tileW + gap), tileY, tileW, tileH, "TIME", screen6SortMode == APRS_SORT_TIME);
  drawFilterTile(startX + 1 * (tileW + gap), tileY, tileW, tileH, "CALL", screen6SortMode == APRS_SORT_CALLSIGN);
  drawFilterTile(startX + 2 * (tileW + gap), tileY, tileW, tileH, "DIST", screen6SortMode == APRS_SORT_DISTANCE);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(2);
  tft.setCursor(10, 74);
  tft.print("APRS Setting:");

  const int checkX = 34;
  const int checkRowY0 = 94;
  const int checkRowGap = 28;
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 0 * checkRowGap, "Beaconing", screen6MenuBeaconingTemp);
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 1 * checkRowGap, "APRS Alert", screen6MenuAprsAlertTemp);
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 2 * checkRowGap, "Range Alert", screen6MenuRangeAlertTemp);
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 3 * checkRowGap, "LED / Buzzer", screen6MenuLedAlertTemp);

  const int closeW = 100;
  const int saveW = 100;
  const int closeH = 26;
  const int closeX = 54;
  const int saveX = 166;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
  drawFilterTile(saveX, closeY, saveW, closeH, "SAVE", false);
}

void handleScreen6MenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 90;
  const int tileH = 26;
  const int gap = 6;
  const int startX = (320 - (3 * tileW + 2 * gap)) / 2;
  const int tileY = 32;

  if (isPointInRect(x, y, startX + 0 * (tileW + gap), tileY, tileW, tileH)) {
    screen6SortMode = APRS_SORT_TIME;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, startX + 1 * (tileW + gap), tileY, tileW, tileH)) {
    screen6SortMode = APRS_SORT_CALLSIGN;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, startX + 2 * (tileW + gap), tileY, tileW, tileH)) {
    screen6SortMode = APRS_SORT_DISTANCE;
    drawAprsSortMenu();
    return;
  }

  const int checkX = 34;
  const int checkRowY0 = 94;
  const int checkRowGap = 28;
  const int checkHitW = 250;
  const int checkHitH = 22;

  if (isPointInRect(x, y, checkX, checkRowY0 + 0 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuBeaconingTemp = !screen6MenuBeaconingTemp;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, checkX, checkRowY0 + 1 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuAprsAlertTemp = !screen6MenuAprsAlertTemp;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, checkX, checkRowY0 + 2 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuRangeAlertTemp = !screen6MenuRangeAlertTemp;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, checkX, checkRowY0 + 3 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuLedAlertTemp = !screen6MenuLedAlertTemp;
    drawAprsSortMenu();
    return;
  }

  const int closeW = 100;
  const int saveW = 100;
  const int closeH = 26;
  const int closeX = 54;
  const int saveX = 166;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    savePreferences();
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }

  if (isPointInRect(x, y, saveX, closeY, saveW, closeH)) {
    aprsBeaconEnabled = screen6MenuBeaconingTemp;
    aprsAlertEnabled = screen6MenuAprsAlertTemp;
    aprsAlertNearbyEnabled = screen6MenuRangeAlertTemp;
    enableLedAlert = screen6MenuLedAlertTemp;
    savePreferences();
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }
}

void drawPotaFilterMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("POTA FILTERS");

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 28);
  tft.print("MODE");

  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (320 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;
  drawFilterTile(modeStartX + 0 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "ALL", isModeFilterTileSelected(screen7FilterModeMask, FILTER_MODE_ALL));
  drawFilterTile(modeStartX + 1 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "CW", isModeFilterTileSelected(screen7FilterModeMask, FILTER_MODE_CW));
  drawFilterTile(modeStartX + 2 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "SSB", isModeFilterTileSelected(screen7FilterModeMask, FILTER_MODE_SSB));
  drawFilterTile(modeStartX + 3 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "DIGI", isModeFilterTileSelected(screen7FilterModeMask, FILTER_MODE_DIGI));

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 82);
  tft.print("BAND");

  const int bandTileW = 90;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (320 - (3 * bandTileW + 2 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      if (bandIdx >= SCREEN2_FILTER_BANDS_COUNT) break;
      int x = bandStartX + col * (bandTileW + bandGap);
      int y = bandStartY + row * (bandTileH + bandGap);
      drawFilterTile(x, y, bandTileW, bandTileH, SCREEN2_FILTER_BANDS[bandIdx], isBandFilterTileSelected(screen7FilterBandMask, bandIdx));
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen7MenuTouch(uint16_t x, uint16_t y) {
  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (320 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;

  for (int i = 0; i < 4; i++) {
    int rx = modeStartX + i * (modeTileW + modeGap);
    if (isPointInRect(x, y, rx, modeY, modeTileW, modeTileH)) {
      toggleModeFilterSelection(screen7FilterModeMask, i);
      drawPotaFilterMenu();
      return;
    }
  }

  const int bandTileW = 90;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (320 - (3 * bandTileW + 2 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      if (bandIdx >= SCREEN2_FILTER_BANDS_COUNT) break;
      int rx = bandStartX + col * (bandTileW + bandGap);
      int ry = bandStartY + row * (bandTileH + bandGap);
      if (isPointInRect(x, y, rx, ry, bandTileW, bandTileH)) {
        toggleBandFilterSelection(screen7FilterBandMask, bandIdx);
        drawPotaFilterMenu();
        return;
      }
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    savePreferences();
    inMenu = false;
    drawScreen(SCREEN_POTA_CLUSTER);
    return;
  }
}

void drawHamalertFilterMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("HAMALERT FILTR");

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 28);
  tft.print("MODE");

  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (320 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;
  drawFilterTile(modeStartX + 0 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "ALL", isModeFilterTileSelected(screen8FilterModeMask, FILTER_MODE_ALL));
  drawFilterTile(modeStartX + 1 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "CW", isModeFilterTileSelected(screen8FilterModeMask, FILTER_MODE_CW));
  drawFilterTile(modeStartX + 2 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "SSB", isModeFilterTileSelected(screen8FilterModeMask, FILTER_MODE_SSB));
  drawFilterTile(modeStartX + 3 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "DIGI", isModeFilterTileSelected(screen8FilterModeMask, FILTER_MODE_DIGI));

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 82);
  tft.print("BAND");

  const int bandTileW = 72;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (320 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int x = bandStartX + col * (bandTileW + bandGap);
      int y = bandStartY + row * (bandTileH + bandGap);
      drawFilterTile(x, y, bandTileW, bandTileH, SCREEN8_FILTER_BANDS[bandIdx], isBandFilterTileSelected(screen8FilterBandMask, bandIdx));
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen8MenuTouch(uint16_t x, uint16_t y) {
  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (320 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;

  for (int i = 0; i < 4; i++) {
    int rx = modeStartX + i * (modeTileW + modeGap);
    if (isPointInRect(x, y, rx, modeY, modeTileW, modeTileH)) {
      toggleModeFilterSelection(screen8FilterModeMask, i);
      drawHamalertFilterMenu();
      return;
    }
  }

  const int bandTileW = 72;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (320 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int rx = bandStartX + col * (bandTileW + bandGap);
      int ry = bandStartY + row * (bandTileH + bandGap);
      if (isPointInRect(x, y, rx, ry, bandTileW, bandTileH)) {
        toggleBandFilterSelection(screen8FilterBandMask, bandIdx);
        drawHamalertFilterMenu();
        return;
      }
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    savePreferences();
    inMenu = false;
    drawScreen(SCREEN_HAMALERT_CLUSTER);
    return;
  }
}

void drawHamClockTimeMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("TIME MODE");

  const int tileW = 110;
  const int tileH = 28;
  const int gap = 10;
  const int startX = (320 - (2 * tileW + gap)) / 2;
  const int tileY = 60;
  drawFilterTile(startX, tileY, tileW, tileH, "UTC", screen1TimeMode == SCREEN1_TIME_UTC);
  drawFilterTile(startX + tileW + gap, tileY, tileW, tileH, "LOCAL", screen1TimeMode == SCREEN1_TIME_LOCAL);

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen1MenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 110;
  const int tileH = 28;
  const int gap = 10;
  const int startX = (320 - (2 * tileW + gap)) / 2;
  const int tileY = 60;

  if (isPointInRect(x, y, startX, tileY, tileW, tileH)) {
    screen1TimeMode = SCREEN1_TIME_UTC;
    savePreferences();
    drawHamClockTimeMenu();
    return;
  }
  if (isPointInRect(x, y, startX + tileW + gap, tileY, tileW, tileH)) {
    screen1TimeMode = SCREEN1_TIME_LOCAL;
    savePreferences();
    drawHamClockTimeMenu();
    return;
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (320 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(SCREEN_HAM_CLOCK);
    return;
  }
}

void redrawCurrentMenu() {
  if (currentScreen == SCREEN_HAM_CLOCK) {
    drawHamClockTimeMenu();
    return;
  }
  if (currentScreen == SCREEN_DX_CLUSTER) {
    drawDxClusterFilterMenu();
    return;
  }
  if (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) {
    drawAprsSortMenu();
    return;
  }
}

void restoreAfterBrightnessMenu() {
  inMenu = brightnessMenuPrevInMenu;
  if (brightnessMenuPrevInMenu) {
    redrawCurrentMenu();
    return;
  }
  currentScreen = brightnessMenuPrevScreen;
  drawScreen(currentScreen);
}

void drawBrightnessSlider() {
  const int sliderX = 40;
  const int sliderY = 90;
  const int sliderW = 240;
  const int sliderH = 12;
  const int knobW = 6;

  // Wyczyść obszar suwaka (z zapasem na pokrętło), żeby nie zostawał ślad
  tft.fillRect(sliderX - knobW, sliderY - 6, sliderW + knobW * 2, sliderH + 12, TFT_BLACK);

  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);
  int fillW = (brightnessMenuValue * (sliderW - 2)) / 100;
  tft.fillRect(sliderX + 1, sliderY + 1, sliderW - 2, sliderH - 2, TFT_BLACK);
  tft.fillRect(sliderX + 1, sliderY + 1, fillW, sliderH - 2, TFT_RADIO_ORANGE);

  int knobX = sliderX + (brightnessMenuValue * (sliderW - knobW)) / 100;
  tft.fillRect(knobX, sliderY - 4, knobW, sliderH + 8, TFT_WHITE);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 70);
  tft.print(tr(TR_BRIGHTNESS));
  tft.fillRect(120, 70, 40, 8, TFT_BLACK);
  tft.setCursor(120, 70);
  tft.print(brightnessMenuValue);
  tft.print("%");
}

void drawThemeSlider() {
  const int sliderX = 40;
  const int sliderY = 130;
  const int sliderW = 240;
  const int sliderH = 12;
  const int knobW = 6;

  // Wyczyść obszar suwaka (z zapasem na pokrętło)
  tft.fillRect(sliderX - knobW, sliderY - 6, sliderW + knobW * 2, sliderH + 12, TFT_BLACK);

  for (int i = 0; i < sliderW; i++) {
    uint8_t hue = (uint8_t)((i * 255) / (sliderW - 1));
    tft.drawFastVLine(sliderX + i, sliderY + 1, sliderH - 2, colorWheel(hue));
  }
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);

  int knobX = sliderX + (menuThemeHue * (sliderW - knobW)) / 255;
  tft.fillRect(knobX, sliderY - 4, knobW, sliderH + 8, TFT_WHITE);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 110);
  tft.print(tr(TR_THEME_COLOR));
}

void drawBrightnessMenuHeader() {
  const int themeBtnX = 248;
  const int themeBtnY = 4;
  const int themeBtnW = 68;
  const int themeBtnH = 20;

  tft.fillRect(0, 0, 320, 28, menuThemeColor);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print(tr(TR_DISPLAY_SETTINGS));

  uint16_t btnFill = isTftStyleLight() ? TFT_WHITE : TFT_BLACK;
  uint16_t btnBorder = isTftStyleLight() ? TFT_BLACK : TFT_WHITE;
  uint16_t btnText = isTftStyleLight() ? TFT_BLACK : TFT_WHITE;
  const char *btnLabel = isTftStyleLight()
      ? LANG3("JASNY", "LIGHT", "CLARO")
      : LANG3("CIEMNY", "DARK", "ESCURO");
  int textW = strlen(btnLabel) * 6;

  tft.fillRoundRect(themeBtnX, themeBtnY, themeBtnW, themeBtnH, 4, btnFill);
  tft.drawRoundRect(themeBtnX, themeBtnY, themeBtnW, themeBtnH, 4, btnBorder);
  tft.setTextSize(1);
  tft.setTextColor(btnText);
  tft.setCursor(themeBtnX + (themeBtnW - textW) / 2, themeBtnY + 6);
  tft.print(btnLabel);
}

void drawBrightnessMenu() {
  tft.fillScreen(TFT_BLACK);
  drawBrightnessMenuHeader();

  // Podpowiedź: długi tap 3s w dowolnym miejscu uruchamia kalibrację
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print(tr(TR_TFT_CALIBRATION_HINT));

  drawBrightnessSlider();
  drawThemeSlider();

  const int langLabelY = 150;
  const int langTileW = 70;
  const int langTileH = 22;
  const int langGap = 10;
  const int langStartX = (320 - (3 * langTileW + 2 * langGap)) / 2;
  const int langY = 160;

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, langLabelY);
  tft.print(tr(TR_LANGUAGE));
  drawFilterTile(langStartX, langY, langTileW, langTileH, "PL", tftLanguage == TFT_LANG_PL);
  drawFilterTile(langStartX + langTileW + langGap, langY, langTileW, langTileH, "EN", tftLanguage == TFT_LANG_EN);
  drawFilterTile(langStartX + 2 * (langTileW + langGap), langY, langTileW, langTileH, "PT", tftLanguage == TFT_LANG_PT);

  const int btnW = 90;
  const int btnH = 26;
  const int btnY = 190;
  drawFilterTile(10, btnY, btnW, btnH, tr(TR_SAVE), false);
  drawFilterTile(115, btnY, btnW, btnH, tr(TR_DEFAULT), false);
  drawFilterTile(220, btnY, btnW, btnH, tr(TR_CLOSE), false);
}

void drawTouchCalTarget(int cx, int cy, uint16_t color) {
  tft.drawLine(cx - 10, cy, cx + 10, cy, color);
  tft.drawLine(cx, cy - 10, cx, cy + 10, color);
  tft.drawRect(cx - 12, cy - 12, 24, 24, color);
}

void drawTouchCalibrationScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  int centerY = 120;
  
  if (touchCalStep == 0) {
    tft.setCursor(40, centerY);
    tft.print("Touch target 1/4 (top-left)");
    drawTouchCalTarget(20, 20, TFT_RADIO_ORANGE);
  } else if (touchCalStep == 1) {
    tft.setCursor(35, centerY);
    tft.print("Touch target 2/4 (top-right)");
    drawTouchCalTarget(300, 20, TFT_RADIO_ORANGE);
  } else if (touchCalStep == 2) {
    tft.setCursor(30, centerY);
    tft.print("Touch target 3/4 (bottom-right)");
    drawTouchCalTarget(300, 220, TFT_RADIO_ORANGE);
  } else if (touchCalStep == 3) {
    tft.setCursor(35, centerY);
    tft.print("Touch target 4/4 (bottom-left)");
    drawTouchCalTarget(20, 220, TFT_RADIO_ORANGE);
  } else {
    // touchCalStep >= 4 - Review screen
    tft.setCursor(40, centerY);
    tft.print("Calibration complete - Touch to close");
    
    // Narysuj wszystkie 4 targets jako potwierdzenie
    drawTouchCalTarget(20, 20, TFT_DARKGREY);
    drawTouchCalTarget(300, 20, TFT_DARKGREY);
    drawTouchCalTarget(300, 220, TFT_DARKGREY);
    drawTouchCalTarget(20, 220, TFT_DARKGREY);
    
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setCursor(10, 30);
    tft.print("Raw X: ");
    tft.print(touchCalNewXMin);
    tft.print(" - ");
    tft.print(touchCalNewXMax);
    tft.setCursor(10, 45);
    tft.print("Raw Y: ");
    tft.print(touchCalNewYMin);
    tft.print(" - ");
    tft.print(touchCalNewYMax);
    
    tft.setCursor(10, 65);
    tft.print("Detected: Swap:");
    tft.print(touchSwapXY ? "YES" : "NO");
    tft.print(" InvX:");
    tft.print(touchInvertX ? "YES" : "NO");
    tft.setCursor(10, 80);
    tft.print("          InvY:");
    tft.print(touchInvertY ? "YES" : "NO");
    
    // Podpowiedź do ustawienia w WWW
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 145);
    tft.print("Use WWW panel >");
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 160);
    tft.print(tr(TR_TFT_CALIBRATION_HINT));
    tft.setTextColor(TFT_GREENYELLOW);
    tft.setCursor(10, 180);
    
    // Ustal jaki tryb sugerować
    if (touchSwapXY && touchInvertX && touchInvertY) {
      tft.print("-> SWAP XY + INVERT BOTH");
    } else if (touchSwapXY && touchInvertX) {
      tft.print("-> ");
      tft.print(tr(TR_ROT_90_RIGHT));
    } else if (touchSwapXY && touchInvertY) {
      tft.print("-> ");
      tft.print(tr(TR_ROT_90_LEFT));
    } else if (touchSwapXY) {
      tft.print("-> SWAP XY");
    } else if (touchInvertX && touchInvertY) {
      tft.print("-> INVERT BOTH (180deg)");
    } else if (touchInvertX) {
      tft.print("-> INVERT X");
    } else if (touchInvertY) {
      tft.print("-> INVERT Y");
    } else {
      tft.print("-> NONE (default - OK)");
    }
    
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setCursor(10, 210);
    tft.print("(Touch will NOT save changes)");
  }
}

void startTouchCalibration() {
  touchCalActive = true;
  touchCalStep = 0;
  touchCalRawX1 = 0;
  touchCalRawY1 = 0;
  touchCalRawX2 = 0;
  touchCalRawY2 = 0;
  touchCalRawX3 = 0;
  touchCalRawY3 = 0;
  touchCalRawX4 = 0;
  touchCalRawY4 = 0;
  touchCalNewXMin = touchXMin;
  touchCalNewXMax = touchXMax;
  touchCalNewYMin = touchYMin;
  touchCalNewYMax = touchYMax;
  drawTouchCalibrationScreen();
}

void setBrightnessFromTouch(uint16_t x) {
  const int sliderX = 40;
  const int sliderW = 240;
  int val = (int)((long)(x - sliderX) * 100 / (sliderW - 1));
  if (val < MIN_BACKLIGHT_PERCENT) val = MIN_BACKLIGHT_PERCENT;
  if (val > 100) val = 100;
  if (val != brightnessMenuValue) {
    brightnessMenuValue = val;
    setBacklightPercent(brightnessMenuValue);
    drawBrightnessSlider();
  }
}

void setThemeFromTouch(uint16_t x) {
  const int sliderX = 40;
  const int sliderW = 240;
  int val = (int)((long)(x - sliderX) * 255 / (sliderW - 1));
  if (val < 0) val = 0;
  if (val > 255) val = 255;
  uint8_t hue = (uint8_t)val;
  if (hue != menuThemeHue) {
    menuThemeHue = hue;
    applyMenuThemeFromHue();
    drawBrightnessMenuHeader();
    drawThemeSlider();
    drawBrightnessSlider();
  }
}

void handleTouchCalibrationTouch(int16_t rawX, int16_t rawY, uint16_t x, uint16_t y, bool isNewTap) {
  if (!touchCalActive || !isNewTap) {
    return;
  }

  if (touchCalStep == 0) {
    // Target 1: lewy górny (20, 20)
    touchCalRawX1 = rawX;
    touchCalRawY1 = rawY;
    touchCalStep = 1;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep == 1) {
    // Target 2: prawy górny (300, 20)
    touchCalRawX2 = rawX;
    touchCalRawY2 = rawY;
    touchCalStep = 2;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep == 2) {
    // Target 3: prawy dolny (300, 220)
    touchCalRawX3 = rawX;
    touchCalRawY3 = rawY;
    touchCalStep = 3;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep == 3) {
    // Target 4: lewy dolny (20, 220)
    touchCalRawX4 = rawX;
    touchCalRawY4 = rawY;
    
    // Wszystkie 4 punkty zebrane - oblicz parametry
    // Min/Max ze wszystkich 4 punktów surowych
    touchCalNewXMin = min(min(touchCalRawX1, touchCalRawX2), min(touchCalRawX3, touchCalRawX4));
    touchCalNewXMax = max(max(touchCalRawX1, touchCalRawX2), max(touchCalRawX3, touchCalRawX4));
    touchCalNewYMin = min(min(touchCalRawY1, touchCalRawY2), min(touchCalRawY3, touchCalRawY4));
    touchCalNewYMax = max(max(touchCalRawY1, touchCalRawY2), max(touchCalRawY3, touchCalRawY4));
    
    // Wykryj swap: porównaj rozpiętość surowych wartości
    // Targets rozciągają się w X: 20-300 (280px) i Y: 20-220 (200px)
    // Jeśli deltaRawY > deltaRawX, osie są zamienione
    int deltaRawX = touchCalNewXMax - touchCalNewXMin;
    int deltaRawY = touchCalNewYMax - touchCalNewYMin;
    touchSwapXY = (deltaRawY > deltaRawX);
    
    // Wykryj odwrócenie osi
    // Porównaj punkt 1 (góra-lewo) z punktem 3 (dół-prawo)
    int16_t x1 = touchCalRawX1;
    int16_t y1 = touchCalRawY1;
    int16_t x3 = touchCalRawX3;
    int16_t y3 = touchCalRawY3;
    
    // Jeśli osie są zamienione, swap współrzędne przed sprawdzeniem kierunku
    if (touchSwapXY) {
      int16_t tmp = x1; x1 = y1; y1 = tmp;
      tmp = x3; x3 = y3; y3 = tmp;
    }
    
    // Target 1 (20,20) -> Target 3 (300,220): oczekujemy x3 > x1 i y3 > y1
    touchInvertX = (x3 < x1);
    touchInvertY = (y3 < y1);
    
    touchCalStep = 4;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep >= 4) {
    // Dowolne dotknięcie zamyka ekran kalibracji bez zapisywania
    // Przywróć poprzednie wartości flag
    touchSwapXY = TOUCH_SWAP_XY;
    touchInvertX = TOUCH_INVERT_X;
    touchInvertY = TOUCH_INVERT_Y;
    touchCalActive = false;
    drawBrightnessMenu();
    return;
  }
}

void handleBrightnessMenuTouch(uint16_t x, uint16_t y, bool isNewTap) {
  unsigned long now = millis();

  const int themeBtnHitX = 205;
  const int themeBtnHitY = 0;
  const int themeBtnHitW = 115;
  const int themeBtnHitH = 44;

  // Przycisk zmiany motywu ma priorytet i działa od razu po otwarciu menu.
  if (isNewTap && isPointInRect(x, y, themeBtnHitX, themeBtnHitY, themeBtnHitW, themeBtnHitH)) {
    tftStyleMode = isTftStyleLight() ? TFT_STYLE_DARK : TFT_STYLE_LIGHT;
    applyTftUiPalette();
    drawBrightnessMenu();
    return;
  }
  
  if (now - brightnessMenuOpenedMs < 1500) {
    return;
  }

  // Zawsze aktualizuj znacznik startu dla bieżącego dotknięcia, zanim obsłużymy kafelki
  if (isNewTap) {
    brightnessMenuTouchStartMs = now;
    brightnessMenuLongPressHandled = false;
  }

  // Długi tap (>=5s) w dowolnym miejscu ekranu jasności uruchamia kalibrację dotyku
  if (!brightnessMenuLongPressHandled && brightnessMenuTouchStartMs > 0 && (now - brightnessMenuTouchStartMs) >= 5000) {
    brightnessMenuLongPressHandled = true;
    startTouchCalibration();
    return;
  }

  const int sliderX = 40;
  const int sliderY = 90;
  const int sliderW = 240;
  const int sliderH = 12;
  const int themeSliderY = 130;
  const int themeSliderH = 12;
  const int langTileW = 70;
  const int langTileH = 22;
  const int langGap = 10;
  const int langStartX = (320 - (3 * langTileW + 2 * langGap)) / 2;
  const int langY = 160;

  // Obsługa sliderów
  if (y >= sliderY - 6 && y <= sliderY + sliderH + 6) {
    setBrightnessFromTouch(x);
  }
  if (y >= themeSliderY - 6 && y <= themeSliderY + themeSliderH + 6) {
    setThemeFromTouch(x);
  }

  if (isNewTap) {
    if (isPointInRect(x, y, langStartX, langY, langTileW, langTileH)) {
      tftLanguage = TFT_LANG_PL;
      // Wymuś odświeżenie opisów pogody po zmianie języka
      lastWeatherFetchMs = 0;
      lockWeatherData();
      weatherData.valid = false;
      weatherData.forecast3hValid = false;
      weatherData.forecastNextDayValid = false;
      for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
        weatherData.detailValid[i] = false;
      }
      unlockWeatherData();
      savePreferences();
      drawBrightnessMenu();
      return;
    }
    if (isPointInRect(x, y, langStartX + langTileW + langGap, langY, langTileW, langTileH)) {
      tftLanguage = TFT_LANG_EN;
      // Wymuś odświeżenie opisów pogody po zmianie języka
      lastWeatherFetchMs = 0;
      lockWeatherData();
      weatherData.valid = false;
      weatherData.forecast3hValid = false;
      weatherData.forecastNextDayValid = false;
      for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
        weatherData.detailValid[i] = false;
      }
      unlockWeatherData();
      savePreferences();
      drawBrightnessMenu();
      return;
    }
    if (isPointInRect(x, y, langStartX + 2 * (langTileW + langGap), langY, langTileW, langTileH)) {
      tftLanguage = TFT_LANG_PT;
      // Wymuś odświeżenie opisów pogody po zmianie języka
      lastWeatherFetchMs = 0;
      lockWeatherData();
      weatherData.valid = false;
      weatherData.forecast3hValid = false;
      weatherData.forecastNextDayValid = false;
      for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
        weatherData.detailValid[i] = false;
      }
      unlockWeatherData();
      savePreferences();
      drawBrightnessMenu();
      return;
    }
  }

  if (!isNewTap) {
    return;
  }

  const int btnW = 90;
  const int btnH = 26;
  const int btnY = 190;
  const int saveX = 10;
  const int defaultX = 115;
  const int closeX = 220;

  if (isPointInRect(x, y, saveX, btnY, btnW, btnH)) {
    backlightPercent = brightnessMenuValue;
    savePreferences();
    brightnessMenuActive = false;
    restoreAfterBrightnessMenu();
    return;
  }
  if (isPointInRect(x, y, defaultX, btnY, btnW, btnH)) {
    menuThemeHue = DEFAULT_MENU_THEME_HUE;
    applyMenuThemeFromHue();
    drawBrightnessMenuHeader();
    drawThemeSlider();
    drawBrightnessSlider();
    return;
  }
  if (isPointInRect(x, y, closeX, btnY, btnW, btnH)) {
    backlightPercent = brightnessMenuPrevBacklight;
    setBacklightPercent(backlightPercent);
    menuThemeHue = brightnessMenuPrevThemeHue;
    applyMenuThemeFromHue();
    tftStyleMode = brightnessMenuPrevStyleMode;
    applyTftUiPalette();
    brightnessMenuActive = false;
    restoreAfterBrightnessMenu();
    return;
  }
}

// Ekran 6: APRS-IS
void updateScreen6Clock() {
  if (!tftInitialized || inMenu) {
    return;
  }
  if (currentScreen != SCREEN_APRS_IS && currentScreen != SCREEN_APRS_RADAR) {
    return;
  }

  static char lastDrawnTime[6] = "";
  static bool lastRadarMode = false;

  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return;
  }

  char timeBuffer[6];
  strftime(timeBuffer, 6, "%H:%M", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  bool radarMode = (currentScreen == SCREEN_APRS_RADAR);
  uint16_t topClearColor = radarMode ? (isTftStyleLight() ? tftUiPalette.bg : TFT_BLACK) : tftHeaderBgColor();
  uint16_t timeColor = radarMode ? (isTftStyleLight() ? tftUiPalette.fg : TFT_WHITE) : tftHeaderTextColor();
  int timeY = radarMode ? 5 : 8;

  if (strcmp(lastDrawnTime, timeBuffer) == 0 && lastRadarMode == radarMode) {
    return;
  }

  int timeX = 320 - timeWidth - 4;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, topClearColor);
  tft.setTextColor(timeColor);
  tft.setTextSize(2);
  tft.setCursor(timeX, timeY);
  tft.print(timeBuffer);

  strncpy(lastDrawnTime, timeBuffer, sizeof(lastDrawnTime));
  lastDrawnTime[sizeof(lastDrawnTime) - 1] = '\0';
  lastRadarMode = radarMode;
}

uint32_t computeScreen6Signature() {
  // Prosty hash treĹ›ci tabeli (10 lub 11 stacji APRS zależnie od paska nawigacji)
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)aprsStationCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen6SortMode;
  hash *= fnvPrime;
  hash ^= (uint32_t)(screen6RadarZoom * 100.0f);
  hash *= fnvPrime;
  hash ^= isScreen6RadarHintVisible() ? 1u : 0u;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_APRS_IS) ? 1u : 0u;
  hash *= fnvPrime;

  int order[MAX_APRS_DISPLAY_LCD];
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int maxRows = getAprsTableMaxRows();
  int visibleCount = min(displayCount, maxRows);
  for (int i = 0; i < visibleCount; i++) {
    const APRSStation &s = aprsStations[order[i]];
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)(s.lat * 1000);
    hash *= fnvPrime;
    hash ^= (uint32_t)(s.lon * 1000);
    hash *= fnvPrime;
    hash ^= (uint32_t)s.distance;
    hash *= fnvPrime;
    hash ^= (uint32_t)(s.freqMHz * 1000);
    hash *= fnvPrime;
  }

  return hash;
}

void updateScreen6Data() {
  if (!tftInitialized || inMenu) {
    return;
  }
  if (currentScreen != SCREEN_APRS_IS && currentScreen != SCREEN_APRS_RADAR) {
    return;
  }

  // 1) Zaktualizuj czas w nagĹ‚Ăłwku (bez peĹ‚nego odĹ›wieĹĽania)
  updateScreen6Clock();

  // 2) OdĹ›wieĹĽ tabelÄ™ tylko gdy dane siÄ™ zmieniĹ‚y
  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen6Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  if (currentScreen == SCREEN_APRS_RADAR) {
    drawAprsRadar();
    return;
  }

  // 3) Renderuj tabelÄ™ do bufora i wypchnij jednym ruchem (bez migotania)
  const bool navVisible = isTableNavFooterVisible(SCREEN_APRS_IS);
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  const int maxRows = getAprsTableMaxRows();
  const int aprsKmXEnlarged = 195;
  const int aprsFreqXEnlarged = 234;
  const int aprsKmXNormal = 197;
  const int aprsFreqXNormal = 255;
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_APRS_IS);
  const int tableHeight = tableBottom - tableTop;
  TFT_eSprite *tableSprite = (navVisible && ensureSharedTableSprite()) ? &sharedTableSprite : nullptr;

  if (tableSprite != nullptr) {
    tableSprite->fillSprite(tftUiPalette.bg);

    int yPos = 8;
    tableSprite->setTextColor(tftUiPalette.muted);
    tableSprite->setTextSize(1);
    if (enlarged) {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(74, yPos);  tableSprite->print("CALL");
      tableSprite->setCursor(aprsKmXEnlarged, yPos); tableSprite->print("KM");
      tableSprite->setCursor(aprsFreqXEnlarged, yPos); tableSprite->print("FREQ");
    } else {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(50, yPos);  tableSprite->print("CALLSIGN");
      tableSprite->setCursor(125, yPos); tableSprite->print("SYMBOL");
      tableSprite->setCursor(aprsKmXNormal, yPos); tableSprite->print("KM");
      tableSprite->setCursor(aprsFreqXNormal, yPos); tableSprite->print("FREQ");
    }
    tableSprite->drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
    yPos += enlarged ? 20 : 18;

    int order[MAX_APRS_DISPLAY_LCD];
    int displayCount = 0;
    buildAprsDisplayOrder(order, displayCount);
    int visibleCount = min(displayCount, maxRows);
    for (int i = 0; i < visibleCount; i++) {
      if (yPos >= (tableHeight - 2)) {
        break;
      }
      const APRSStation &station = aprsStations[order[i]];
      if (enlarged) {
        if (i % 2 == 0) {
          tableSprite->fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
        }

        tableSprite->setTextSize(2);

        tableSprite->setTextColor(tftUiPalette.muted);
        tableSprite->setCursor(5, yPos);
        String timeStr = formatAprsTimeWithTimezone(station.time);
        tableSprite->print(timeStr);

        tableSprite->setTextColor(getAprsCallsignColorForEnlarged(station));
        tableSprite->setCursor(74, yPos);
        String callText = station.callsign;
        if (callText.length() > 9) callText = callText.substring(0, 9);
        tableSprite->print(callText);

        tableSprite->setTextColor(tftTableAccentTextColor());
        tableSprite->setCursor(aprsKmXEnlarged, yPos);
        if (station.hasLatLon && station.distance > 0) {
          tableSprite->print(roundAprsDistanceKm(station.distance));
        } else {
          tableSprite->print("-");
        }

        tableSprite->setTextColor(tftTableFreqColor());
        tableSprite->setCursor(aprsFreqXEnlarged, yPos);
        if (station.freqMHz > 0.0f) {
          tableSprite->print(String(station.freqMHz, 3));
        } else {
          tableSprite->print("-");
        }

        yPos += 27;
      } else {
        if (i % 2 == 0) {
          tableSprite->fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
        }

        tableSprite->setTextSize(1);

        if (medium) tableSprite->setTextFont(2);

        tableSprite->setTextColor(tftUiPalette.muted);
        tableSprite->setCursor(5, yPos);
        String timeStr = formatAprsTimeWithTimezone(station.time);
        tableSprite->print(timeStr);

        tableSprite->setTextColor(tftUiPalette.fg);
        tableSprite->setCursor(50, yPos);
        tableSprite->print(medium ? String(station.callsign).substring(0, 9) : String(station.callsign));

        String symbolShort = getAPRSSymbolShort(station);
        uint16_t symbolColor = TFT_GREEN;
        if (isTftStyleLight()) {
          symbolColor = tftUiPalette.fg;
        } else if (symbolShort == "HUMAN") {
          symbolColor = 0x3C1F;
        } else if (symbolShort == "HOUSE") {
          symbolColor = TFT_YELLOW;
        } else if (symbolShort == "CAR") {
          symbolColor = TFT_RED;
        } else if (symbolShort == "HAMCLOCK") {
          symbolColor = TFT_ORANGE;
        }
        tableSprite->setTextColor(symbolColor);
        tableSprite->setCursor(125, yPos);
        tableSprite->print(symbolShort);

        tableSprite->setTextColor(tftTableAccentTextColor());
        tableSprite->setCursor(aprsKmXNormal, yPos);
        if (station.hasLatLon && station.distance > 0) {
          tableSprite->print(roundAprsDistanceKm(station.distance));
          tableSprite->print(" km");
        } else {
          tableSprite->print("-");
        }

        tableSprite->setTextColor(tftTableFreqColor());
        tableSprite->setCursor(aprsFreqXNormal, yPos);
        if (station.freqMHz > 0.0f) {
          tableSprite->print(String(station.freqMHz, 3));
        } else {
          tableSprite->print("-");
        }

        if (medium) tableSprite->setTextFont(1);

        yPos += medium ? 20 : 17;
      }
    }

    if (visibleCount == 0) {
      tableSprite->setTextColor(TFT_RED);
      tableSprite->setTextSize(2);
      tableSprite->setCursor(40, 120 - tableTop);
      tableSprite->print("WAITING FOR APRS...");
    }

    tableSprite->pushSprite(0, tableTop);
    return;
  }

  // Fallback bez sprite (np. gdy zabraknie RAM)
  tft.fillRect(0, tableTop, 320, tableHeight, tftUiPalette.bg);

  int yPos = 40;
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(aprsKmXEnlarged, yPos); tft.print("KM");
    tft.setCursor(aprsFreqXEnlarged, yPos); tft.print("FREQ");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("SYMBOL");
    tft.setCursor(aprsKmXNormal, yPos); tft.print("KM");
    tft.setCursor(aprsFreqXNormal, yPos); tft.print("FREQ");
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  int order[MAX_APRS_DISPLAY_LCD];
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int visibleCount = min(displayCount, maxRows);
  for (int i = 0; i < visibleCount; i++) {
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    const APRSStation &station = aprsStations[order[i]];
    if (enlarged) {
      if (i % 2 == 0) {
        tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);
      }

      tft.setTextSize(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(getAprsCallsignColorForEnlarged(station));
      tft.setCursor(74, yPos);
      String callText = station.callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(aprsKmXEnlarged, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print(roundAprsDistanceKm(station.distance));
      } else {
        tft.print("-");
      }

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(aprsFreqXEnlarged, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      yPos += 27;
    } else {
      if (i % 2 == 0) {
        tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);
      }

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);

      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(station.callsign).substring(0, 9) : String(station.callsign));

      String symbolShort = getAPRSSymbolShort(station);
      uint16_t symbolColor = TFT_GREEN;
      if (isTftStyleLight()) {
        symbolColor = tftUiPalette.fg;
      } else if (symbolShort == "HUMAN") {
        symbolColor = 0x3C1F;
      } else if (symbolShort == "HOUSE") {
        symbolColor = TFT_YELLOW;
      } else if (symbolShort == "CAR") {
        symbolColor = TFT_RED;
      } else if (symbolShort == "HAMCLOCK") {
        symbolColor = TFT_ORANGE;
      }
      tft.setTextColor(symbolColor);
      tft.setCursor(125, yPos);
      tft.print(symbolShort);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(aprsKmXNormal, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print(roundAprsDistanceKm(station.distance));
        tft.print(" km");
      } else {
        tft.print("-");
      }

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(aprsFreqXNormal, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
  }

  if (visibleCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR APRS...");
  }
}

void drawAprsIs() {
  screen6ViewMode = APRS_VIEW_LIST;
  tft.fillScreen(tftUiPalette.bg);
  const int aprsKmXEnlarged = 195;
  const int aprsFreqXEnlarged = 234;
  const int aprsKmXNormal = 197;
  const int aprsFreqXNormal = 255;

  // 1. NAGĹĂ“WEK: Belka z menu, nazwÄ… serwera i czasem UTC
  tft.fillRect(0, 0, 320, 32, tftHeaderBgColor());
  tft.setTextColor(tftHeaderTextColor());

  // IKONA MENU (3D)
  drawHamburgerMenuButton3D(5, 7);

  // Nazwa serwera APRS-IS - przesuniÄ™ta w prawo (x=35 zamiast 5), by zrobiÄ‡ miejsce na menu
  tft.setTextSize(2);
  tft.setCursor(35, 8);
  if (isUart2AprsTncKissModuleSelected()) {
    tft.print("APRS-TNC-KISS");
  } else if (isUart2AprsUartSourceEnabled()) {
    tft.print("APRS-TNC2-UART");
  } else {
    tft.print("APRS-IS");
  }
  
  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H:%M", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(320 - timeWidth - 4, 8);
    tft.print(timeBuffer);
  }

  // 2. NAGĹĂ“WKI TABELI
  int yPos = 40;
  const bool enlarged = isDxTableEnlarged();
  const bool medium = isDxTableMedium();
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(aprsKmXEnlarged, yPos); tft.print("KM");
    tft.setCursor(aprsFreqXEnlarged, yPos); tft.print("FREQ");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("SYMBOL");
    tft.setCursor(aprsKmXNormal, yPos); tft.print("KM");
    tft.setCursor(aprsFreqXNormal, yPos); tft.print("FREQ");
  }
  tft.drawFastHLine(0, yPos + 10, 320, tftUiPalette.muted);
  yPos += enlarged ? 20 : 18;

  // 3. LISTA STACJI APRS (10 z paskiem nawigacji, 11 bez)
  int order[MAX_APRS_DISPLAY_LCD];
  int maxRows = getAprsTableMaxRows();
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int visibleCount = min(displayCount, maxRows);
  for (int i = 0; i < visibleCount; i++) {
    const APRSStation &station = aprsStations[order[i]];
    if (enlarged) {
      if (i % 2 == 0) tft.fillRect(0, yPos - 5, 320, 24, tftUiPalette.rowAlt);

      tft.setTextSize(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(getAprsCallsignColorForEnlarged(station));
      tft.setCursor(74, yPos);
      String callText = station.callsign;
      if (callText.length() > 9) callText = callText.substring(0, 9);
      tft.print(callText);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(aprsKmXEnlarged, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print(roundAprsDistanceKm(station.distance));
      } else {
        tft.print("-");
      }

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(aprsFreqXEnlarged, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      yPos += 27;
    } else {
      if (i % 2 == 0) tft.fillRect(0, yPos - 2, 320, medium ? 20 : 16, tftUiPalette.rowAlt);

      tft.setTextSize(1);

      if (medium) tft.setTextFont(2);
      tft.setTextColor(tftUiPalette.muted);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(tftUiPalette.fg);
      tft.setCursor(50, yPos);
      tft.print(medium ? String(station.callsign).substring(0, 9) : String(station.callsign));

      String symbolShort = getAPRSSymbolShort(station);
      uint16_t symbolColor = TFT_GREEN;
      if (isTftStyleLight()) {
        symbolColor = tftUiPalette.fg;
      } else if (symbolShort == "HUMAN") {
        symbolColor = 0x3C1F;
      } else if (symbolShort == "HOUSE") {
        symbolColor = TFT_YELLOW;
      } else if (symbolShort == "CAR") {
        symbolColor = TFT_RED;
      } else if (symbolShort == "HAMCLOCK") {
        symbolColor = TFT_ORANGE;
      }
      tft.setTextColor(symbolColor);
      tft.setCursor(125, yPos);
      tft.print(symbolShort);

      tft.setTextColor(tftTableAccentTextColor());
      tft.setCursor(aprsKmXNormal, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print(roundAprsDistanceKm(station.distance));
        tft.print(" km");
      } else {
        tft.print("-");
      }

      tft.setTextColor(tftTableFreqColor());
      tft.setCursor(aprsFreqXNormal, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      if (medium) tft.setTextFont(1);

      yPos += medium ? 20 : 17;
    }
  }

  // 4. Pasek nawigacji widoczny tylko przez 5 sekund od wejścia na ekran
  if (isTableNavFooterVisible(SCREEN_APRS_IS)) {
    drawSwitchScreenFooter();
  }

  if (visibleCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR APRS...");
  }
}

void drawAprsRadar() {
  screen6ViewMode = APRS_VIEW_RADAR;
  const bool lightTheme = isTftStyleLight();
  const uint16_t radarBgColor = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t radarTextColor = lightTheme ? tftUiPalette.fg : TFT_WHITE;
  tft.fillScreen(radarBgColor);
  drawAprsRadarBody();

  drawHamburgerMenuButton3D(5, 7);

  tft.setTextColor(radarTextColor);
  tft.setTextSize(2);
  const char *aprsTitleTop = "APRS";
  const char *aprsTitleBottom = "RADAR";
  int aprsTopWidth = strlen(aprsTitleTop) * 12;
  int aprsBottomWidth = strlen(aprsTitleBottom) * 12;
  int aprsTopX = tft.width() - aprsTopWidth - 4;
  int aprsBottomX = tft.width() - aprsBottomWidth - 4;
  tft.setCursor(aprsTopX, tft.height() - 36);
  tft.print(aprsTitleTop);
  tft.setCursor(aprsBottomX, tft.height() - 20);
  tft.print(aprsTitleBottom);

  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H:%M", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setTextColor(radarTextColor);
    tft.setCursor(tft.width() - timeWidth - 4, 5);
    tft.print(timeBuffer);
  }

  drawScreen6RadarZoomHints();
}

static String extractXmlTagValue(const String &xml, const char* tag) {
  String openTag = "<" + String(tag) + ">";
  String closeTag = "</" + String(tag) + ">";
  int start = xml.indexOf(openTag);
  if (start < 0) return "";
  start += openTag.length();
  int end = xml.indexOf(closeTag, start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

static String extractBandCondition(const String &xml, const char* bandName, const char* timeName) {
  String token = "<band name=\"" + String(bandName) + "\" time=\"" + String(timeName) + "\">";
  int start = xml.indexOf(token);
  if (start < 0) return "";
  start += token.length();
  int end = xml.indexOf("</band>", start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

static void setPropagationBandDefaults(PropagationData &out) {
  out.hfBandLabel[0] = "80m-40m";
  out.hfBandFreq[0] = "3.5-7.3 MHz";
  out.hfBandLabel[1] = "30m-20m";
  out.hfBandFreq[1] = "10.1-14.35 MHz";
  out.hfBandLabel[2] = "17m-15m";
  out.hfBandFreq[2] = "18.068-21.45 MHz";
  out.hfBandLabel[3] = "12m-10m";
  out.hfBandFreq[3] = "24.89-29.7 MHz";

  for (int i = 0; i < 4; i++) {
    out.hfBandDay[i] = "--";
    out.hfBandNight[i] = "--";
  }
}

static bool parsePropagationXml(const String &xml, PropagationData &out) {
  setPropagationBandDefaults(out);

  String sfi = extractXmlTagValue(xml, "solarflux");
  String kindex = extractXmlTagValue(xml, "kindex");
  String aindex = extractXmlTagValue(xml, "aindex");
  String muf = extractXmlTagValue(xml, "muf");
  String updated = extractXmlTagValue(xml, "updated");

  sfi.trim();
  kindex.trim();
  aindex.trim();
  muf.trim();
  updated.trim();

  if (sfi.length() == 0 || kindex.length() == 0) {
    return false;
  }

  out.sfi = sfi;
  out.kindex = kindex;
  out.aindex = aindex.length() ? aindex : "--";
  out.muf = muf.length() ? muf : "--";
  out.updated = updated.length() ? updated : "--";
  out.valid = true;
  out.lastError = "";
  out.fetchedAtMs = millis();

  for (int i = 0; i < 4; i++) {
    String day = extractBandCondition(xml, out.hfBandLabel[i].c_str(), "day");
    String night = extractBandCondition(xml, out.hfBandLabel[i].c_str(), "night");
    day.trim();
    night.trim();
    if (day.length() > 0) out.hfBandDay[i] = day;
    if (night.length() > 0) out.hfBandNight[i] = night;
  }

  return true;
}

bool fetchPropagationData() {
  if (WiFi.status() != WL_CONNECTED) {
    propagationData.lastError = "WiFi offline";
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, PROPAGATION_URL)) {
    propagationData.lastError = "HTTP begin failed";
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    propagationData.lastError = "HTTP " + String(httpCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  if (!parsePropagationXml(payload, propagationData)) {
    propagationData.lastError = "Parse error";
    propagationData.valid = false;
    return false;
  }

  return true;
}

void updateScreen3Clock() {
  if (!tftInitialized || currentScreen != 3 || inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen3Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)propagationData.valid;
  hash *= fnvPrime;

  const String* fields[] = {
    &propagationData.sfi,
    &propagationData.kindex,
    &propagationData.aindex,
    &propagationData.muf,
    &propagationData.updated,
    &propagationData.lastError
  };
  for (const String* field : fields) {
    for (size_t i = 0; i < field->length(); i++) {
      hash ^= (uint8_t)(*field)[i];
      hash *= fnvPrime;
    }
  }

  return hash;
}

static uint16_t conditionColor(String cond) {
  String up = cond;
  up.toUpperCase();
  if (up.indexOf("GOOD") >= 0) return TFT_GREEN;
  if (up.indexOf("FAIR") >= 0) return TFT_YELLOW;
  if (up.indexOf("POOR") >= 0) return TFT_RED;
  return TFT_LIGHTGREY;
}

// Translates the HF band condition text (Good/Fair/Poor) for display only.
// The raw value is kept untouched for conditionColor() and the web API.
static String conditionLabel(const String &cond) {
  if (tftLanguage != TFT_LANG_PT) {
    return cond;
  }
  String up = cond;
  up.toUpperCase();
  up.trim();
  if (up == "GOOD") return (cond == up) ? "BOA" : "Boa";
  if (up == "FAIR") return (cond == up) ? "MEDIA" : "Media";
  if (up == "POOR") return (cond == up) ? "RUIM" : "Ruim";
  return cond;
}

void drawSunSpotsBody() {
  const int bodyTop = 32;
  const int bodyBottom = 240;
  tft.fillRect(0, bodyTop, 320, bodyBottom - bodyTop, tftUiPalette.bg);

  String sfiText = propagationData.valid && propagationData.sfi.length() ? propagationData.sfi : "--";
  String kText = propagationData.valid && propagationData.kindex.length() ? propagationData.kindex : "--";
  String aText = propagationData.valid && propagationData.aindex.length() ? propagationData.aindex : "--";
  String mufText = propagationData.valid && propagationData.muf.length() ? propagationData.muf : "--";
  String updatedText = propagationData.valid && propagationData.updated.length() ? propagationData.updated : "--";

  tft.setTextSize(2);
  tft.setTextColor(tftUiPalette.muted);
  tft.setCursor(10, 52);
  tft.print("SFI");

  tft.setTextSize(4);
  tft.setTextColor(isTftStyleLight() ? tftUiPalette.fg : TFT_GREEN);
  int sfiWidth = sfiText.length() * 24;
  tft.setCursor(310 - sfiWidth, 44);
  tft.print(sfiText);

  tft.setTextSize(2);
  tft.setTextColor(tftUiPalette.muted);
  tft.setCursor(10, 97);
  tft.print("K-INDEX");

  float kVal = kText.length() ? kText.toFloat() : -1.0f;
  uint16_t kColor = tftUiPalette.fg;
  if (kVal >= 0.0f) {
    if (!isTftStyleLight()) {
      kColor = (kVal < 4.0f) ? TFT_GREEN : (kVal < 6.0f) ? TFT_ORANGE : TFT_RED;
    }
  }
  tft.setTextSize(4);
  tft.setTextColor(kColor);
  int kWidth = kText.length() * 24;
  tft.setCursor(310 - kWidth, 89);
  tft.print(kText);

  tft.setTextSize(2);
  tft.setTextColor(tftUiPalette.muted);
  tft.setCursor(10, 142);
  tft.print("A-INDEX");

  tft.setTextSize(4);
  tft.setTextColor(isTftStyleLight() ? tftUiPalette.fg : TFT_CYAN);
  int aWidth = aText.length() * 24;
  tft.setCursor(310 - aWidth, 134);
  tft.print(aText);

  tft.setTextSize(2);
  tft.setTextColor(tftUiPalette.muted);
  tft.setCursor(10, 182);
  tft.print("MUF");

  tft.setTextSize(3);
  tft.setTextColor(isTftStyleLight() ? tftUiPalette.fg : TFT_YELLOW);
  int mufWidth = mufText.length() * 18;
  tft.setCursor(310 - mufWidth, 174);
  tft.print(mufText);

  tft.setTextSize(1);
  tft.setTextColor(tftUiPalette.muted);
  tft.setCursor(10, 210);
  tft.print(LANG3("UPDATED UTC", "UPDATED UTC", "ATUALIZADO UTC"));
  tft.setTextSize(2);
  tft.setTextColor(tftUiPalette.fg);
  tft.setCursor(10, 222);
  tft.print(updatedText);

  if (!propagationData.valid) {
    tft.setTextSize(1);
    tft.setTextColor(isTftStyleLight() ? tftUiPalette.fg : TFT_RED);
    tft.setCursor(10, 196);
    String err = propagationData.lastError.length() ? propagationData.lastError : tr(TR_NO_DATA);
    tft.print(tr(TR_ERROR_PREFIX));
    tft.print(err);
  }
}

void updateScreen3Data() {
  if (!tftInitialized || currentScreen != 3 || inMenu) {
    return;
  }

  updateScreen3Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen3Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  drawSunSpotsBody();
}

void drawSunSpots() {
  tft.fillScreen(tftUiPalette.bg);

  tft.fillRect(0, 0, 320, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  // brak hamburger menu na ekranie Propagation

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print(LANG3("PROPAGACJA", "PROPAGATION", "PROPAGACAO"));

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[10];
    strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  drawSunSpotsBody();
}

void updateScreen4Clock() {
  if (!tftInitialized || currentScreen != 4 || inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen4Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)propagationData.valid;
  hash *= fnvPrime;

  for (int i = 0; i < 4; i++) {
    const String* fields[] = {
      &propagationData.hfBandLabel[i],
      &propagationData.hfBandFreq[i],
      &propagationData.hfBandDay[i],
      &propagationData.hfBandNight[i]
    };
    for (const String* field : fields) {
      for (size_t j = 0; j < field->length(); j++) {
        hash ^= (uint8_t)(*field)[j];
        hash *= fnvPrime;
      }
    }
  }

  for (size_t j = 0; j < propagationData.updated.length(); j++) {
    hash ^= (uint8_t)propagationData.updated[j];
    hash *= fnvPrime;
  }
  for (size_t j = 0; j < propagationData.lastError.length(); j++) {
    hash ^= (uint8_t)propagationData.lastError[j];
    hash *= fnvPrime;
  }

  return hash;
}

void drawBandInfoBody() {
  const int bodyTop = 32;
  const int bodyBottom = 220;
  tft.fillRect(0, bodyTop, 320, bodyBottom - bodyTop, tftUiPalette.bg);

  int yPos = 40;
  tft.setTextColor(tftUiPalette.muted);
  tft.setTextSize(2);
  tft.setCursor(5, yPos);
  tft.print(LANG3("BAND", "BAND", "BANDA"));
  tft.setCursor(150, yPos);
  tft.print(LANG3("DAY", "DAY", "DIA"));
  tft.setCursor(240, yPos);
  tft.print(LANG3("NIGHT", "NIGHT", "NOITE"));
  tft.drawFastHLine(0, yPos + 30, 320, tftUiPalette.muted);
  yPos += 26; //byĹ‚o 16

  for (int i = 0; i < 4; i++) {
    if (i % 2 == 0) {
      tft.fillRect(0, yPos - 2, 320, 28, tftUiPalette.rowAlt);
    }

    tft.setTextSize(2);
    tft.setTextColor(tftUiPalette.fg);
    tft.setCursor(5, yPos);
    tft.print(propagationData.hfBandLabel[i]);

    //tft.setTextColor(TFT_DARKGREY);
    //tft.setCursor(5, yPos + 9);
    //tft.print(propagationData.hfBandFreq[i]);

    tft.setTextSize(2);
    uint16_t dayColor = conditionColor(propagationData.hfBandDay[i]);
    if (isTftStyleLight()) {
      dayColor = tftUiPalette.fg;
    }
    tft.setTextColor(dayColor);
    tft.setCursor(150, yPos - 2);
    tft.print(conditionLabel(propagationData.hfBandDay[i]));

    uint16_t nightColor = conditionColor(propagationData.hfBandNight[i]);
    if (isTftStyleLight()) {
      nightColor = tftUiPalette.fg;
    }
    tft.setTextColor(nightColor);
    tft.setCursor(240, yPos - 2);
    tft.print(conditionLabel(propagationData.hfBandNight[i]));

    yPos += 30;
  }

  tft.setTextSize(1);
  tft.setTextColor(tftUiPalette.muted);
  tft.setCursor(10, 190);
  tft.print(LANG3("UPDATED UTC", "UPDATED UTC", "ATUALIZADO UTC"));
  tft.setTextSize(2);
  tft.setTextColor(tftUiPalette.fg);
  tft.setCursor(10, 202);
  tft.print(propagationData.updated.length() ? propagationData.updated : "--");

  if (!propagationData.valid) {
    tft.setTextSize(1);
    tft.setTextColor(isTftStyleLight() ? tftUiPalette.fg : TFT_RED);
    tft.setCursor(10, 176);
    String err = propagationData.lastError.length() ? propagationData.lastError : tr(TR_NO_DATA);
    tft.print(tr(TR_ERROR_PREFIX));
    tft.print(err);
  }
}

void updateScreen4Data() {
  if (!tftInitialized || currentScreen != 4 || inMenu) {
    return;
  }

  updateScreen4Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen4Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  drawBandInfoBody();
}

void drawBandInfo() {
  tft.fillScreen(tftUiPalette.bg);

  tft.fillRect(0, 0, 320, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  int menuX = 8;
  int menuY = 10;
  //tft.fillRect(menuX, menuY, 18, 3, TFT_BLACK);
  //tft.fillRect(menuX, menuY + 6, 18, 3, TFT_BLACK);
  //tft.fillRect(menuX, menuY + 12, 18, 3, TFT_BLACK);

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print("HF BAND INFO");

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[10];
    strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  drawBandInfoBody();

  tft.fillTriangle(10, 230, 20, 222, 20, 238, TFT_RADIO_ORANGE);
  tft.fillTriangle(310, 230, 300, 222, 300, 238, TFT_RADIO_ORANGE);
  tft.setTextColor(isTftStyleLight() ? tftUiPalette.fg : 0x52AA);
  tft.setTextSize(1);
  tft.setCursor(125, 226);
  tft.print("SWITCH SCREEN");
}

bool fetchWeatherForecast(double lat, double lon);

bool fetchWeatherData() {
  auto markWeatherError = [&](const String &errorText) {
    lockWeatherData();
    weatherData.lastError = errorText;
    weatherData.valid = false;
    unlockWeatherData();
    return false;
  };

  if (WiFi.status() != WL_CONNECTED) {
    return markWeatherError("WiFi offline");
  }
  if (weatherApiKey.length() == 0) {
    return markWeatherError("No API key");
  }

  double lat = 0.0;
  double lon = 0.0;
  if (!getServiceCoordinatesForServices(lat, lon)) {
    return markWeatherError("No station coordinates");
  }

  // Reset prognoz przed nowym pobraniem
  lockWeatherData();
  weatherData.forecast3hValid = false;
  weatherData.forecastNextDayValid = false;
  weatherData.forecastError = "";
  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    weatherData.detailValid[i] = false;
  }
  unlockWeatherData();

  WiFiClientSecure client;
  client.setInsecure();

  String langParam = LANG3("pl", "en", "pt_br");
  String url = "https://api.openweathermap.org/data/2.5/weather?lat=" +
               String(lat, 4) + "&lon=" + String(lon, 4) +
               "&appid=" + weatherApiKey + "&units=metric&lang=" + langParam;

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, url)) {
    return markWeatherError("HTTP begin failed");
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return markWeatherError("HTTP " + String(httpCode));
  }

  WiFiClient *stream = http.getStreamPtr();

  DynamicJsonDocument filter(512);
  filter["cod"] = true;
  filter["message"] = true;
  filter["name"] = true;
  filter["weather"][0]["description"] = true;
  filter["weather"][0]["icon"] = true;
  filter["weather"][0]["id"] = true;
  filter["main"]["temp"] = true;
  filter["main"]["humidity"] = true;
  filter["main"]["pressure"] = true;
  filter["wind"]["speed"] = true;

  DynamicJsonDocument doc(1536);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    return markWeatherError("JSON error");
  }

  int cod = doc["cod"] | 0;
  if (cod != 200) {
    String msg = doc["message"] | "";
    return markWeatherError(msg.length() ? msg : "API error");
  }

  String desc = doc["weather"][0]["description"] | "";
  String cityName = doc["name"] | "";
  String icon = doc["weather"][0]["icon"] | ""; // 01d / 01n itp. do wykrycia pory dnia
  int weatherId = doc["weather"][0]["id"] | 800; // pobieranie kodu pogody
  float temp = doc["main"]["temp"] | 0.0f;
  int humidity = doc["main"]["humidity"] | 0;
  int pressure = doc["main"]["pressure"] | 0;
  float wind = doc["wind"]["speed"] | 0.0f;


  if (desc.equalsIgnoreCase("zachmurzenie umiarkowane")) {
    desc = "zachmurzenie";
  }

  lockWeatherData();
  weatherData.description = desc.length() ? desc : "--";
  weatherData.cityName = cityName;
  weatherData.iconCode = icon;
  weatherData.weatherId = weatherId; // kod pogody
  weatherData.tempC = temp;
  weatherData.humidity = humidity;
  weatherData.pressure = pressure;
  weatherData.windMs = wind;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuf[6];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M", &timeinfo);
    weatherData.updated = String(timeBuf);
  } else {
    weatherData.updated = "--:--";
  }

  weatherData.valid = true;
  weatherData.lastError = "";
  weatherData.fetchedAtMs = millis();
  unlockWeatherData();

  // Prognozy (3h i jutro)
  fetchWeatherForecast(lat, lon);

  // Pobierz dane o jakoĹ›ci powietrza (PM2.5 i PM10)
  fetchAirPollutionData(lat, lon);

  return true;
}

bool fetchWeatherForecast(double lat, double lon) {
  auto setForecastError = [&](const String &errorText) {
    lockWeatherData();
    weatherData.forecastError = errorText;
    unlockWeatherData();
    return false;
  };

  auto formatForecastHttpError = [&](int code) {
    String msg = "Forecast: HTTP " + String(code);
    if (code < 0) {
      msg += " (";
      msg += HTTPClient::errorToString(code);
      msg += ")";
    }
    return msg;
  };

  if (WiFi.status() != WL_CONNECTED) {
    return setForecastError("Forecast: WiFi offline");
  }
  if (weatherApiKey.length() == 0) {
    return setForecastError("Forecast: No API key");
  }

  String langParam = LANG3("pl", "en", "pt_br");
  String paramsWithLang = "lat=" + String(lat, 4) +
                          "&lon=" + String(lon, 4) +
                          "&appid=" + weatherApiKey +
                          "&units=metric&lang=" + langParam;
  String paramsNoLang = "lat=" + String(lat, 4) +
                        "&lon=" + String(lon, 4) +
                        "&appid=" + weatherApiKey +
                        "&units=metric";

  String urlPrimary = "https://api.openweathermap.org/data/2.5/forecast?" + paramsWithLang + "&cnt=24";
  String urlRetrySecure = "https://api.openweathermap.org/data/2.5/forecast?" + paramsNoLang + "&cnt=12";
  String urlRetryHttp = "http://api.openweathermap.org/data/2.5/forecast?" + paramsNoLang + "&cnt=12";

  HTTPClient http;
  int httpCode = 0;
  String lastTransportError = "Forecast: HTTP begin failed";
  bool requestReady = false;

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  secureClient.setTimeout(12000);
  WiFiClient plainClient;
  plainClient.setTimeout(12000);

  for (uint8_t attempt = 0; attempt < 3 && !requestReady; attempt++) {
    bool secureAttempt = (attempt < 2);
    String attemptUrl = (attempt == 0) ? urlPrimary : (attempt == 1 ? urlRetrySecure : urlRetryHttp);

    http.setReuse(false);
    http.useHTTP10(true);
    http.setTimeout(attempt == 0 ? 8000 : 12000);

    bool beginOk = secureAttempt ? http.begin(secureClient, attemptUrl)
                                 : http.begin(plainClient, attemptUrl);
    if (!beginOk) {
      lastTransportError = "Forecast: HTTP begin failed";
      continue;
    }

    httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      requestReady = true;
      break;
    }

    lastTransportError = formatForecastHttpError(httpCode);
    http.end();
  }

  if (!requestReady) {
    return setForecastError(lastTransportError);
  }

  WiFiClient *stream = http.getStreamPtr();

  DynamicJsonDocument doc(28672);
  DeserializationError err = deserializeJson(doc, *stream);
  http.end();
  if (err) {
    return setForecastError("Forecast: JSON " + String(err.c_str()));
  }

  String cod = doc["cod"] | "";
  if (cod.length() > 0 && cod != "200") {
    String msg = doc["message"] | "";
    if (msg.length() == 0) {
      msg = "API error";
    }
    return setForecastError("Forecast: " + msg);
  }

  JsonArray list = doc["list"].as<JsonArray>();
  if (list.isNull() || list.size() == 0) {
    int cntApi = doc["cnt"] | -1;
    if (cntApi >= 0) {
      return setForecastError("Forecast: No data in API (cnt=" + String(cntApi) + ")");
    }
    return setForecastError("Forecast: No data in API");
  }

  bool stormPossible = false;
  int nearestStormEtaMinutes = 0;
  bool windPossible = false;
  float maxWindMsNext6h = 0.0f;
  float maxWindGustMsNext6h = 0.0f;
  long nowUnixForAlerts = (long)time(nullptr);
  if (nowUnixForAlerts > 100000L) {
    for (JsonObject entry : list) {
      long dt = entry["dt"] | 0L;
      if (dt <= 0L) {
        continue;
      }

      long deltaSec = dt - nowUnixForAlerts;
      if (deltaSec < 0L || deltaSec > (6L * 3600L)) {
        continue;
      }

      float windMs = entry["wind"]["speed"] | 0.0f;
      float windGustMs = entry["wind"]["gust"] | 0.0f;
      if (windMs > maxWindMsNext6h) {
        maxWindMsNext6h = windMs;
      }
      if (windGustMs > maxWindGustMsNext6h) {
        maxWindGustMsNext6h = windGustMs;
      }

      int weatherId = entry["weather"][0]["id"] | 800;
      if (weatherId >= 200 && weatherId < 300) {
        int etaMinutes = (int)(deltaSec / 60L);
        if ((deltaSec % 60L) != 0L) {
          etaMinutes++;
        }
        if (etaMinutes < 1) {
          etaMinutes = 1;
        }
        if (!stormPossible || etaMinutes < nearestStormEtaMinutes) {
          stormPossible = true;
          nearestStormEtaMinutes = etaMinutes;
        }
      }
    }

    windPossible = (maxWindMsNext6h >= weatherAlertWindThresholdMs) ||
             (maxWindGustMsNext6h >= weatherAlertWindGustThresholdMs());
  }

  updateWeatherAlertForecastState(stormPossible,
                                  nearestStormEtaMinutes,
                                  windPossible,
                                  maxWindMsNext6h,
                                  maxWindGustMsNext6h);

  lockWeatherData();
  if (weatherData.cityName.length() == 0) {
    String cityFromForecast = doc["city"]["name"] | "";
    if (cityFromForecast.length() > 0) {
      weatherData.cityName = cityFromForecast;
    }
  }

  const long nowUnix = (long)time(nullptr);
  const long targetOffsetsSec[WeatherData::DETAIL_COLS] = {
    3L * 3600L,
    6L * 3600L,
    24L * 3600L,
    48L * 3600L,
    72L * 3600L
  };
  const int fallbackIdx[WeatherData::DETAIL_COLS] = {0, 1, 7, 15, 23};
  const long timezoneOffsetSec = (long)(timezoneHours * 3600.0f);
  const long nowLocalUnix = nowUnix + timezoneOffsetSec;
  const long baseLocalDayStart = (nowLocalUnix / 86400L) * 86400L;

  weatherData.nightTempValid[0] = false;
  weatherData.nightTempValid[1] = false;

  auto assignSlot = [&](uint8_t slot, JsonObject entry) {
    weatherData.detailTempC[slot] = entry["main"]["temp"] | 0.0f;
    weatherData.detailHumidity[slot] = entry["main"]["humidity"] | 0;
    weatherData.detailWindMs[slot] = entry["wind"]["speed"] | 0.0f;
    weatherData.detailWeatherId[slot] = entry["weather"][0]["id"] | 800;
    weatherData.detailIconCode[slot] = String((const char *)(entry["weather"][0]["icon"] | ""));
    weatherData.detailValid[slot] = true;
  };

  for (uint8_t slot = 0; slot < WeatherData::DETAIL_COLS; slot++) {
    int chosenIdx = -1;

    if (slot >= 2 && nowUnix > 100000L) {
      const long targetDayStartLocal = baseLocalDayStart + (long)(slot - 1) * 86400L;
      const long targetNoonLocal = targetDayStartLocal + 12L * 3600L;
      long bestScore = 0x7FFFFFFF;

      for (uint16_t i = 0; i < list.size(); i++) {
        JsonObject entry = list[i];
        long dt = entry["dt"] | 0L;
        if (dt <= 0L) {
          continue;
        }

        long dtLocal = dt + timezoneOffsetSec;
        long entryDayStartLocal = (dtLocal / 86400L) * 86400L;
        if (entryDayStartLocal != targetDayStartLocal) {
          continue;
        }

        int localHour = (int)((dtLocal % 86400L) / 3600L);
        bool isDayHour = (localHour >= 9 && localHour <= 18);
        String iconCode = String((const char *)(entry["weather"][0]["icon"] | ""));
        bool iconIsDay = iconCode.endsWith("d");
        bool isDayCandidate = isDayHour || iconIsDay;

        long score = labs(dtLocal - targetNoonLocal);
        if (!isDayCandidate) {
          score += 12L * 3600L;
        }

        if (score < bestScore) {
          bestScore = score;
          chosenIdx = (int)i;
        }
      }
    }

    if (chosenIdx < 0 && nowUnix > 100000L) {
      long target = nowUnix + targetOffsetsSec[slot];
      long bestDiff = 0x7FFFFFFF;

      for (uint16_t i = 0; i < list.size(); i++) {
        JsonObject entry = list[i];
        long dt = entry["dt"] | 0L;
        if (dt <= 0L) {
          continue;
        }
        long diff = labs(dt - target);
        if (diff < bestDiff) {
          bestDiff = diff;
          chosenIdx = (int)i;
        }
      }
    }

    if (chosenIdx < 0) {
      int idx = fallbackIdx[slot];
      if (idx >= (int)list.size()) {
        idx = (int)list.size() - 1;
      }
      chosenIdx = idx;
    }

    if (chosenIdx >= 0 && chosenIdx < (int)list.size()) {
      assignSlot(slot, list[chosenIdx]);
    }
  }

  for (uint8_t daySlot = 0; daySlot < 2; daySlot++) {
    const long targetDayStartLocal = baseLocalDayStart + (long)(daySlot + 1) * 86400L;
    int bestNightIdx = -1;
    long bestNightScore = 0x7FFFFFFF;
    int bestAnyIdx = -1;
    float bestAnyTemp = 1000.0f;

    for (uint16_t i = 0; i < list.size(); i++) {
      JsonObject entry = list[i];
      long dt = entry["dt"] | 0L;
      if (dt <= 0L) {
        continue;
      }

      long dtLocal = dt + timezoneOffsetSec;
      long entryDayStartLocal = (dtLocal / 86400L) * 86400L;
      if (entryDayStartLocal != targetDayStartLocal) {
        continue;
      }

      float temp = entry["main"]["temp"] | 0.0f;
      if (bestAnyIdx < 0 || temp < bestAnyTemp) {
        bestAnyTemp = temp;
        bestAnyIdx = (int)i;
      }

      int localHour = (int)((dtLocal % 86400L) / 3600L);
      bool isNightHour = (localHour <= 6 || localHour >= 21);
      if (!isNightHour) {
        continue;
      }

      int refHour = (localHour >= 21) ? 24 + localHour : localHour;
      int refTarget = 26; // 02:00 w oknie 21..30
      long score = labs((long)refHour - (long)refTarget);
      if (score < bestNightScore) {
        bestNightScore = score;
        bestNightIdx = (int)i;
      }
    }

    int chosenNightIdx = (bestNightIdx >= 0) ? bestNightIdx : bestAnyIdx;
    if (chosenNightIdx >= 0 && chosenNightIdx < (int)list.size()) {
      JsonObject entry = list[chosenNightIdx];
      weatherData.nightTempC[daySlot] = entry["main"]["temp"] | 0.0f;
      weatherData.nightTempValid[daySlot] = true;
    }
  }

  if (weatherData.detailValid[0]) {
    weatherData.forecast3hTempC = weatherData.detailTempC[0];
    weatherData.forecast3hWindMs = weatherData.detailWindMs[0];
    weatherData.forecast3hDesc = "--";
    weatherData.forecast3hValid = true;
  }

  if (weatherData.detailValid[2]) {
    weatherData.forecastNextDayTempC = weatherData.detailTempC[2];
    weatherData.forecastNextDayWindMs = weatherData.detailWindMs[2];
    weatherData.forecastNextDayDesc = "--";
    weatherData.forecastNextDayValid = true;
  }

  bool anyDetailValid = false;
  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    if (weatherData.detailValid[i]) {
      anyDetailValid = true;
      break;
    }
  }

  if (!anyDetailValid) {
    weatherData.forecastError = "Forecast: No usable entries";
  }

  if (anyDetailValid) {
    weatherData.forecastError = "";
  }

  unlockWeatherData();
  return anyDetailValid;
}

bool fetchAirPollutionData(double lat, double lon) {
  if (WiFi.status() != WL_CONNECTED || weatherApiKey.length() == 0) {
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  String url = "https://api.openweathermap.org/data/2.5/air_pollution?lat=" +
               String(lat, 4) + "&lon=" + String(lon, 4) +
               "&appid=" + weatherApiKey;

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, url)) {
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();

  DynamicJsonDocument filter(256);
  filter["list"][0]["components"]["pm2_5"] = true;
  filter["list"][0]["components"]["pm10"] = true;

  DynamicJsonDocument doc(768);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    return false;
  }

  // Pobierz wartoĹ›ci PM2.5 i PM10 z komponentĂłw
  JsonArray list = doc["list"].as<JsonArray>();
  if (list.isNull() || list.size() == 0) {
    return false;
  }

  JsonObject listItem = list[0];
  JsonObject components = listItem["components"].as<JsonObject>();
  if (components.isNull()) {
    return false;
  }

  lockWeatherData();
  weatherData.pm25 = components["pm2_5"] | 0.0f;
  weatherData.pm10 = components["pm10"] | 0.0f;
  unlockWeatherData();
  return true;
}

// Funkcja okreĹ›lajÄ…ca kolor na podstawie wartoĹ›ci PM2.5
uint16_t getPM25Color(float pm25) {
  if (pm25 <= 12) return TFT_GREEN;      // Dobra
  if (pm25 <= 35) return TFT_YELLOW;     // Umiarkowana
  if (pm25 <= 55) return TFT_ORANGE;     // Niezdrowa dla wraĹĽliwych
  if (pm25 <= 150) return TFT_RED;       // Niezdrowa
  return 0xF800; // Ciemny czerwony - Bardzo niezdrowa
}

// Funkcja okreĹ›lajÄ…ca kolor na podstawie wartoĹ›ci PM10
uint16_t getPM10Color(float pm10) {
  if (pm10 <= 20) return TFT_GREEN;      // Dobra
  if (pm10 <= 50) return TFT_YELLOW;     // Umiarkowana
  if (pm10 <= 100) return TFT_ORANGE;   // Niezdrowa dla wraĹĽliwych
  if (pm10 <= 200) return TFT_RED;      // Niezdrowa
  return 0xF800; // Ciemny czerwony - Bardzo niezdrowa
}

void updateScreen5Clock() {
  if (!tftInitialized ||
      (currentScreen != SCREEN_WEATHER_DSP && currentScreen != SCREEN_WEATHER_FORECAST) ||
      inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return;
  }

  char timeBuffer[6];
  strftime(timeBuffer, 6, "%H:%M", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen5Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)weatherData.valid;
  hash *= fnvPrime;
  bool footerVisible = (currentScreen == SCREEN_WEATHER_FORECAST)
                       ? isTableNavFooterVisible(SCREEN_WEATHER_FORECAST)
                       : isTableNavFooterVisible(SCREEN_WEATHER_DSP);
  hash ^= footerVisible ? 1u : 0u;
  hash *= fnvPrime;

  const String* fields[] = {
    &weatherData.cityName,
    &weatherData.description,
    &weatherData.updated,
    &weatherData.lastError,
    &weatherData.forecastError
  };
  for (const String* field : fields) {
    for (size_t j = 0; j < field->length(); j++) {
      hash ^= (uint8_t)(*field)[j];
      hash *= fnvPrime;
    }
  }
  hash ^= (uint32_t)(weatherData.tempC * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)weatherData.humidity;
  hash *= fnvPrime;
  hash ^= (uint32_t)weatherData.pressure;
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.windMs * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.pm25 * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.pm10 * 10);
  hash *= fnvPrime;

  // Prognozy
  hash ^= (uint32_t)weatherData.forecast3hValid;
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecast3hTempC * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecast3hWindMs * 10);
  hash *= fnvPrime;
  for (size_t j = 0; j < weatherData.forecast3hDesc.length(); j++) {
    hash ^= (uint8_t)weatherData.forecast3hDesc[j];
    hash *= fnvPrime;
  }

  hash ^= (uint32_t)weatherData.forecastNextDayValid;
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecastNextDayTempC * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecastNextDayWindMs * 10);
  hash *= fnvPrime;
  for (size_t j = 0; j < weatherData.forecastNextDayDesc.length(); j++) {
    hash ^= (uint8_t)weatherData.forecastNextDayDesc[j];
    hash *= fnvPrime;
  }

  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    hash ^= (uint32_t)weatherData.detailValid[i];
    hash *= fnvPrime;
    hash ^= (uint32_t)(weatherData.detailTempC[i] * 10);
    hash *= fnvPrime;
    hash ^= (uint32_t)weatherData.detailHumidity[i];
    hash *= fnvPrime;
    hash ^= (uint32_t)(weatherData.detailWindMs[i] * 10);
    hash *= fnvPrime;
    hash ^= (uint32_t)weatherData.detailWeatherId[i];
    hash *= fnvPrime;
    for (size_t j = 0; j < weatherData.detailIconCode[i].length(); j++) {
      hash ^= (uint8_t)weatherData.detailIconCode[i][j];
      hash *= fnvPrime;
    }
  }

  return hash;
}

// Funkcja pomocnicza: Kolor wiatru zaleĹĽny od prÄ™dkoĹ›ci (m/s)
uint16_t getWindColor(float speed) {
  if (speed < 5.0)  return TFT_GREEN;       // Bezpiecznie
  if (speed < 10.0) return TFT_YELLOW;      // Umiarkowanie
  if (speed < 15.0) return 0xFBE0;          // Silny (Orange)
  return TFT_RED;                           // Niebezpieczny dla anten
}

// Mapowanie kodu OWM na plik ikony (zgodnie z przygotowaną listą)
String weatherIconPathForId(int id, bool isNight) {
  const String base = "/icon50/";

  // 2xx burza
  if (id >= 200 && id <= 232) return base + "200.bmp";

  // 3xx mżawka
  if (id >= 300 && id <= 321) return base + "300.bmp";

  // 5xx deszcz
  if (id >= 500 && id <= 531) {
    if (id == 500 || id == 501) return base + "500.bmp";
    if (id == 502 || id == 503) return base + "502.bmp";
    if (id == 511 || id == 531) return base + "511.bmp";
    return base + "520.bmp";
  }

  // 6xx śnieg
  if (id >= 600 && id <= 622) {
    if (id == 600 || id == 601 || id == 602 || id == 620 || id == 621 || id == 622) return base + "600.bmp";
    // 611..616 (także 613/615/616) -> 611
    return base + "611.bmp";
  }

  // 7xx atmosfera
  if (id >= 700 && id <= 781) return base + "700.bmp";

  // 800 clear (dzień/noc)
  if (id == 800) return base + (isNight ? "800n.bmp" : "800.bmp");

  // 80x chmury
  if (id == 801 || id == 802) return base + (isNight ? "801n.bmp" : "801.bmp");
  if (id == 803 || id == 804) return base + "803.bmp";

  return base + "unknown.bmp";
}

static String weatherIconPathWithLightSuffix(const String &iconPath) {
  int dotIdx = iconPath.lastIndexOf('.');
  if (dotIdx <= 0) {
    return iconPath + "w";
  }
  return iconPath.substring(0, dotIdx) + "w" + iconPath.substring(dotIdx);
}

// Funkcja pomocnicza: Rysowanie ikon pogodowych (priorytet: id -> plik BMP, fallback: opis)
void drawWeatherIcon(int x, int y, int weatherId, String desc) {
  desc.toLowerCase();
  const bool lightTheme = isTftStyleLight();

  // Dzień/noc z ikony OWM (np. 10d/10n). Jeśli brak, fallback na czas lokalny.
  bool isNight = false;
  if (weatherData.iconCode.endsWith("n")) {
    isNight = true;
  } else if (weatherData.iconCode.endsWith("d")) {
    isNight = false;
  } else {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      isNight = (timeinfo.tm_hour < 6 || timeinfo.tm_hour >= 18);
    }
  }

  String iconFile = weatherIconPathForId(weatherId, isNight);

  if (lightTheme) {
    String iconFileLight = weatherIconPathWithLightSuffix(iconFile);
    if (drawBmpFromFS(iconFileLight, x - 25, y - 25)) {
      return;
    }
  }

  // Najpierw próbujemy gotową ikonę BMP
  if (drawBmpFromFS(iconFile, x - 25, y - 25)) {
    return;
  }

  // Fallback: stare ikonki rysowane na podstawie opisu (gdy brak pliku)
  if (desc.indexOf("slon") >= 0 || desc.indexOf("clear") >= 0 || desc.indexOf("pogod") >= 0) {
    tft.fillCircle(x, y, 18, TFT_YELLOW); // Słonce
    for(int i=0; i<360; i+=45) {
       float rad = i * 0.01745;
       tft.drawLine(x+cos(rad)*22, y+sin(rad)*22, x+cos(rad)*32, y+sin(rad)*32, TFT_YELLOW);
    }
  } 
  else if (desc.indexOf("burz") >= 0 || desc.indexOf("thunder") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_DARKGREY); // Chmura burzowa
    tft.fillTriangle(x+2, y+8, x+10, y+8, x+4, y+22, TFT_YELLOW); // Błysk
  }
  else if (desc.indexOf("mza") >= 0 || desc.indexOf("drizzle") >= 0) {
    tft.fillCircle(x-10, y+5, 12, TFT_LIGHTGREY); // Chmura + mżawka
    tft.fillCircle(x+10, y+5, 12, TFT_LIGHTGREY);
    tft.fillCircle(x, y-5, 15, TFT_LIGHTGREY);
    tft.drawLine(x-6, y+10, x-8, y+16, TFT_CYAN);
    tft.drawLine(x+2, y+10, x, y+16, TFT_CYAN);
  }
  else if (desc.indexOf("snieg") >= 0 || desc.indexOf("snow") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_LIGHTGREY); // Chmura + śnieg
    tft.drawLine(x-6, y+12, x-2, y+16, TFT_WHITE);
    tft.drawLine(x-2, y+12, x-6, y+16, TFT_WHITE);
    tft.drawLine(x+2, y+12, x+6, y+16, TFT_WHITE);
    tft.drawLine(x+6, y+12, x+2, y+16, TFT_WHITE);
  }
  else if (desc.indexOf("mg") >= 0 || desc.indexOf("mist") >= 0 || desc.indexOf("fog") >= 0 || desc.indexOf("haze") >= 0) {
    tft.drawLine(x-18, y-2, x+18, y-2, TFT_LIGHTGREY); // Mgła
    tft.drawLine(x-20, y+4, x+20, y+4, TFT_LIGHTGREY);
    tft.drawLine(x-16, y+10, x+16, y+10, TFT_LIGHTGREY);
  }
  else if (desc.indexOf("sleet") >= 0 || desc.indexOf("deszcz ze sniegiem") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_LIGHTGREY); // Deszcz ze śniegiem
    tft.drawLine(x-6, y+10, x-8, y+18, TFT_CYAN);
    tft.drawLine(x+2, y+10, x, y+18, TFT_CYAN);
    tft.drawLine(x+6, y+12, x+2, y+16, TFT_WHITE);
    tft.drawLine(x+2, y+12, x+6, y+16, TFT_WHITE);
  }
  else if (desc.indexOf("zachmur") >= 0 || desc.indexOf("cloud") >= 0 || desc.indexOf("pochmurnie") >= 0) {
    tft.fillCircle(x-10, y+5, 12, TFT_LIGHTGREY); // Chmura
    tft.fillCircle(x+10, y+5, 12, TFT_LIGHTGREY);
    tft.fillCircle(x, y-5, 15, TFT_LIGHTGREY);
  }
  else if (desc.indexOf("deszcz") >= 0 || desc.indexOf("rain") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_DARKGREY); // Chmura deszczowa
    tft.drawLine(x-5, y+10, x-8, y+20, TFT_CYAN);
    tft.drawLine(x+5, y+10, x+2, y+20, TFT_CYAN);
    tft.drawLine(x, y+12, x-3, y+22, TFT_CYAN);
  } else {
    tft.drawCircle(x, y, 15, TFT_WHITE); // Ikona domyślna (okrąg)
  }
}

// GĹ‚Ăłwna funkcja ciaĹ‚a ekranu pogodowego
void drawWeatherBody() {
  const int bodyTop = 32;
  const int bodyBottom = 220;
  const bool lightTheme = isTftStyleLight();
  const uint16_t bodyBg = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t bodyText = lightTheme ? TFT_BLACK : TFT_WHITE;
  const uint16_t bodyMuted = lightTheme ? TFT_BLACK : TFT_DARKGREY;
  const uint16_t bodySecondary = lightTheme ? TFT_BLACK : TFT_LIGHTGREY;
  
  // Czyszczenie obszaru roboczego
  tft.fillRect(0, bodyTop, 320, bodyBottom - bodyTop, bodyBg);

  if (!weatherData.valid) {
    tft.setTextColor(lightTheme ? TFT_BLACK : TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(50, 110);
    tft.print("WAITING FOR DATA...");
    return;
  }

  // 1. IKONA I OPIS (Prawa strona)
  drawWeatherIcon(240, 70, weatherData.weatherId, weatherData.description);

  // Priorytet: smooth font (VLW) dla polskich znaków, fallback ASCII gdy mało pamięci.
  int descY = 105; // shifted up by 10px
  int descX = 240; // shifted left by 10px
  bool smoothFontUsed = false;
  if (tryLoadSmoothFont(ROBOTO_FONT12_NAME, ROBOTO_FONT12_FILE)) {
    smoothFontUsed = true;
    tft.setTextColor(bodySecondary);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(weatherData.description, descX, descY);
    tft.setTextDatum(TL_DATUM);

    tft.setCursor(15, 120);
    tft.print(tr(TR_HUMIDITY));
    tft.setCursor(15, 144);
    tft.print(tr(TR_PRESSURE));
    tft.setCursor(15, 168);
    tft.print(tr(TR_WIND));
    tft.unloadFont();
  }

  if (!smoothFontUsed) {
    tft.setTextSize(1);
    tft.setTextColor(bodySecondary);
    String descAscii = sanitizePolishToAscii(weatherData.description);
    int fallbackX = descX - (descAscii.length() * 3);
    tft.setCursor(fallbackX, descY);
    tft.print(descAscii);

    tft.setTextSize(1);
    tft.setTextColor(bodyMuted);
    tft.setCursor(15, 120);
    tft.print(sanitizePolishToAscii(tr(TR_HUMIDITY)));
    tft.setCursor(15, 145);
    tft.print(sanitizePolishToAscii(tr(TR_PRESSURE)));
    tft.setCursor(15, 170);
    tft.print(sanitizePolishToAscii(tr(TR_WIND)));
  }

  // 2. TEMPERATURA (Lewa strona)
  tft.setTextSize(2);
  tft.setTextColor(bodyMuted);
  tft.setCursor(15, 45);
  tft.print(tr(TR_TEMPERATURE));

  tft.setTextSize(5);
  tft.setTextColor(lightTheme ? TFT_BLACK : TFT_CYAN);
  tft.setCursor(15, 70);
  tft.print(String(weatherData.tempC, 1));
  tft.setTextSize(2);
  tft.print(" C");

  // 3. PARAMETRY SZCZEGĂ“ĹOWE
  int startY = 120; // PrzesuniÄ™te w dĂłĹ‚ o 5px
  
  // WILGOTNOĹšÄ† (z paskiem postÄ™pu)
  tft.setTextSize(1);
  tft.setTextColor(bodyMuted);
  //tft.setCursor(15, startY);
  //tft.print("WILGOTNOSC");
  tft.setTextSize(2);
  tft.setTextColor(bodyText);
  tft.setCursor(110, startY - 5);
  tft.print(String(weatherData.humidity) + "%");
  
  // Pasek wilgotnoĹ›ci
  //tft.drawRect(200, startY - 3, 100, 12, TFT_DARKGREY);
  //int humBar = map(weatherData.humidity, 0, 100, 0, 100);
  //tft.fillRect(200, startY - 3, humBar, 12, TFT_BLUE);

  // CIĹšNIENIE
  tft.setTextSize(1);
  tft.setTextColor(bodyMuted);
  //tft.setCursor(15, startY + 25);
  //tft.print("CIŚNIENIE");
  tft.setTextSize(2);
  tft.setTextColor(bodyText);
  tft.setCursor(110, startY + 20);
  tft.print(String(weatherData.pressure) + " hPa");

  // WIATR (Dynamiczny kolor bez paska)
  tft.setTextSize(1);
  tft.setTextColor(bodyMuted);
  //tft.setCursor(15, startY + 50);
  //tft.print("WIATR");
  tft.setTextSize(2);
  tft.setTextColor(lightTheme ? TFT_BLACK : getWindColor(weatherData.windMs));
  tft.setCursor(110, startY + 45);
  tft.print(String(weatherData.windMs, 1) + " m/s");

  // PM2.5 i PM10 (w jednej linii, taka sama czcionka jak pomiar)
  tft.setTextSize(2);
  
  // PM2.5
  tft.setTextColor(bodyText);
  tft.setCursor(15, startY + 70);
  tft.print("PM2.5: ");
  uint16_t pm25Color = getPM25Color(weatherData.pm25);
  tft.setTextColor(lightTheme ? TFT_BLACK : pm25Color);
  tft.print(String(weatherData.pm25, 1));
  
  // Przerwa i PM10
  tft.setTextColor(bodyText);
  tft.print("  PM10: ");
  uint16_t pm10Color = getPM10Color(weatherData.pm10);
  tft.setTextColor(lightTheme ? TFT_BLACK : pm10Color);
  tft.print(String(weatherData.pm10, 1));

}

static void drawWeatherDetailIconCell(int x, int y, int weatherId, const String &iconCode) {
  const bool lightTheme = isTftStyleLight();
  bool isNight = false;
  if (iconCode.endsWith("n")) {
    isNight = true;
  } else if (iconCode.endsWith("d")) {
    isNight = false;
  } else {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      isNight = (timeinfo.tm_hour < 6 || timeinfo.tm_hour >= 18);
    }
  }

  String iconFile = weatherIconPathForId(weatherId, isNight);
  if (lightTheme) {
    String iconFileLight = weatherIconPathWithLightSuffix(iconFile);
    if (drawBmpFromFS(iconFileLight, x - 14, y - 14)) {
      return;
    }
  }
  if (!drawBmpFromFS(iconFile, x - 14, y - 14)) {
    tft.drawCircle(x, y, 10, lightTheme ? TFT_BLACK : TFT_LIGHTGREY);
  }
}

static void buildWeatherDetailHeaders(String headers[WeatherData::DETAIL_COLS]) {
  headers[0] = LANG3("+3godz", "+3h", "+3h");
  headers[1] = LANG3("+6godz", "+6h", "+6h");

  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    static const char *daysPl[7] = {"Nd", "Pn", "Wt", "Śr", "Czw", "Pt", "Sob"};
    static const char *daysEn[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    static const char *daysPt[7] = {"Dom", "Seg", "Ter", "Qua", "Qui", "Sex", "S\xC3\xA1" "b"};
    const char **days = LANG3(daysPl, daysEn, daysPt);

    for (uint8_t i = 0; i < 3; i++) {
      int dayIdx = (timeinfo.tm_wday + 1 + i) % 7;
      headers[i + 2] = String(days[dayIdx]);
    }
  } else {
    headers[2] = "D+1";
    headers[3] = "D+2";
    headers[4] = "D+3";
  }
}

void drawWeatherDetailPage() {
  const int bodyTop = 32;
  const int bodyBottom = 220;
  const int bodyLeft = 2;
  const int bodyRight = 318;
  const int colCount = WeatherData::DETAIL_COLS - 1;
  const int colW = (bodyRight - bodyLeft + 1) / colCount;
  const bool lightTheme = isTftStyleLight();
  const uint16_t bodyBg = lightTheme ? TFT_WHITE : TFT_BLACK;
  const uint16_t bodyText = lightTheme ? TFT_BLACK : TFT_WHITE;
  const uint16_t bodySecondary = lightTheme ? TFT_BLACK : TFT_LIGHTGREY;
  const uint16_t bodyMuted = lightTheme ? TFT_BLACK : TFT_DARKGREY;
  const uint16_t rowBg = lightTheme ? 0xDEFB : TFT_TABLE_ALT_ROW_COLOR;

  tft.fillRect(0, bodyTop, 320, bodyBottom - bodyTop, bodyBg);

  if (!weatherData.valid) {
    tft.setTextColor(lightTheme ? TFT_BLACK : TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(12, 96);
    tft.print(weatherData.lastError.length() ? weatherData.lastError : tr(TR_NO_DATA));
    return;
  }

  bool anyDetailValid = false;
  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    if (weatherData.detailValid[i]) {
      anyDetailValid = true;
      break;
    }
  }

  if (!anyDetailValid) {
    tft.setTextColor(lightTheme ? TFT_BLACK : TFT_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(12, 84);
    tft.print(LANG3("Brak prognozy", "No forecast", "Sem previsao"));

    String err = weatherData.forecastError;
    if (err.length() == 0) {
      err = LANG3("Prognoza: brak danych z API", "Forecast: no data from API", "Previsao: sem dados da API");
    }
    const int maxChars = 42;
    if ((int)err.length() > maxChars) {
      err = err.substring(0, maxChars - 3) + "...";
    }

    tft.setTextColor(bodySecondary);
    tft.setTextSize(1);
    tft.setCursor(12, 110);
    tft.print(err);
    return;
  }

  String headers[WeatherData::DETAIL_COLS];
  buildWeatherDetailHeaders(headers);

  const int yHeader = 38;
  const int yTemp = 78;
  const int yNight = 98;
  const int yHum = 121;
  const int yWind = 152;
  const int yIcon = 183;

  tft.fillRect(bodyLeft, bodyTop + 2, bodyRight - bodyLeft + 1, 31, rowBg);

  for (int i = 1; i < colCount; i++) {
    int vx = bodyLeft + i * colW;
    tft.drawFastVLine(vx, bodyTop + 2, bodyBottom - bodyTop - 4, bodyMuted);
  }

  for (int i = 0; i < colCount; i++) {
    int colX = bodyLeft + i * colW;
    int cx = colX + (colW / 2);

    String hdr = sanitizePolishToAscii(headers[i]);
    tft.setTextSize(2);
    int hdrW = hdr.length() * 12;
    tft.setTextColor(bodySecondary);
    tft.setCursor(cx - hdrW / 2, yHeader);
    tft.print(hdr);

    tft.setTextSize(1);

    if (!weatherData.detailValid[i]) {
      tft.setTextColor(bodyMuted);
      tft.setCursor(cx - 6, yTemp);
      tft.print("--");
      tft.setCursor(cx - 6, yNight);
      tft.print("--");
      tft.setCursor(cx - 6, yHum);
      tft.print("--");
      tft.setCursor(cx - 6, yWind);
      tft.print("--");
      continue;
    }

    String tempText = String(weatherData.detailTempC[i], 1);
    String humText = String(weatherData.detailHumidity[i]) + "%";
    String windText = String(weatherData.detailWindMs[i], 1) + "m/s";

    tft.setTextSize(2);
    tft.setTextColor(lightTheme ? TFT_BLACK : TFT_CYAN);
    int tempX = cx - ((int)tempText.length() * 6);
    tft.setCursor(tempX, yTemp);
    tft.print(tempText);
    tft.drawCircle(tempX + ((int)tempText.length() * 12) + 3, yTemp + 3, 2, lightTheme ? TFT_BLACK : TFT_CYAN);

    tft.setTextSize(1);
    tft.setTextColor(bodySecondary);
    String nightText = "--";
    if (i >= 2 && i <= 3 && weatherData.nightTempValid[i - 2]) {
      nightText = String(weatherData.nightTempC[i - 2], 1) + "C";
    }
    tft.setCursor(cx - ((int)nightText.length() * 3), yNight);
    tft.print(nightText);

    tft.setTextSize(2);
  tft.setTextColor(bodyText);
    tft.setCursor(cx - ((int)humText.length() * 6), yHum);
    tft.print(humText);

    tft.setTextSize(1);
  tft.setTextColor(lightTheme ? TFT_BLACK : getWindColor(weatherData.detailWindMs[i]));
    tft.setCursor(cx - ((int)windText.length() * 3), yWind);
    tft.print(windText);

    drawWeatherDetailIconCell(cx - 10, yIcon, weatherData.detailWeatherId[i], weatherData.detailIconCode[i]);
  }
}

static void drawWeatherFooterArea(ScreenType screenId) {
  const bool lightTheme = isTftStyleLight();
  tft.fillRect(0, 220, 320, 20, lightTheme ? TFT_WHITE : TFT_BLACK);

  if (isTableNavFooterVisible(screenId)) {
    drawSwitchScreenFooter();
    return;
  }

  String cityLabel = weatherData.cityName;
  cityLabel.trim();
  if (cityLabel.length() == 0) {
    cityLabel = "--";
  }
  cityLabel = sanitizePolishToAscii(cityLabel);

  const String prefix = LANG3("Pogoda dla lokalizacji: ",
                              "Weather for location: ",
                              "Clima do local: ");
  const int maxChars = 52; // 320px width at text size 1 (6px/char) with small margins
  int maxCityChars = maxChars - (int)prefix.length();
  if (maxCityChars < 3) {
    maxCityChars = 3;
  }
  if ((int)cityLabel.length() > maxCityChars) {
    cityLabel = cityLabel.substring(0, maxCityChars - 3) + "...";
  }
  String footerLabel = prefix + cityLabel;

  tft.setTextColor(lightTheme ? TFT_BLACK : 0x52AA);
  tft.setTextSize(1);
  int footerWidth = footerLabel.length() * 6;
  int footerX = (320 - footerWidth) / 2;
  if (footerX < 2) {
    footerX = 2;
  }
  tft.setCursor(footerX, 226);
  tft.print(footerLabel);
}

void updateScreen5Data() {
  if (!tftInitialized ||
      (currentScreen != SCREEN_WEATHER_DSP && currentScreen != SCREEN_WEATHER_FORECAST) ||
      inMenu) {
    return;
  }

  updateScreen5Clock();

  static uint32_t lastWeatherSig = 0;
  static uint32_t lastForecastSig = 0;

  lockWeatherData();
  uint32_t currentSig = computeScreen5Signature();

  if (currentScreen == SCREEN_WEATHER_FORECAST) {
    if (currentSig == lastForecastSig) {
      unlockWeatherData();
      return;
    }
    lastForecastSig = currentSig;
    drawWeatherDetailPage();
    drawWeatherFooterArea(SCREEN_WEATHER_FORECAST);
    unlockWeatherData();
    return;
  }

  if (currentSig == lastWeatherSig) {
    unlockWeatherData();
    return;
  }
  lastWeatherSig = currentSig;

  drawWeatherBody();
  drawWeatherFooterArea(SCREEN_WEATHER_DSP);
  unlockWeatherData();
}

void drawWeather() {
  tft.fillScreen(isTftStyleLight() ? TFT_WHITE : TFT_BLACK);

  tft.fillRect(0, 0, 320, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print(tr(TR_WEATHER));

  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H:%M", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  lockWeatherData();
  drawWeatherBody();
  drawWeatherFooterArea(SCREEN_WEATHER_DSP);
  unlockWeatherData();
}

void drawWeatherForecast() {
  tft.fillScreen(isTftStyleLight() ? TFT_WHITE : TFT_BLACK);

  tft.fillRect(0, 0, 320, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  const char *detailHeader = LANG3("Prognoza pogody", "Weather forecast", "Previsao do tempo");
  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print(detailHeader);

  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H:%M", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(315 - timeWidth, 8);
    tft.print(timeBuffer);
  }

  lockWeatherData();
  drawWeatherDetailPage();
  drawWeatherFooterArea(SCREEN_WEATHER_FORECAST);
  unlockWeatherData();
}

void setupMatrix() {
  for (int i = 0; i < numDrops; i++) {
    drops[i].x = i * MATRIX_COL_SPACING; // Rozstawienie kolumn
    resetMatrixDropRandom(i);
  }
  matrixInitialized = true;
}

void drawMatrixBackground(int bodyTop, int bodyBottom) {
  const int charStep = 8; // wysokość znaku przy setTextSize(1)
  const int charW = 6;    // szerokość znaku przy setTextSize(1)
  bool anyIntroActive = false;
  for (int i = 0; i < numDrops; i++) {
    int drawX = drops[i].x;
    int drawY = bodyTop + drops[i].y;
    bool dropIntroActive = (matrixIntroActive && drops[i].introActive);

    if (matrixIntroActive && !dropIntroActive && !drops[i].introParticipates) {
      continue;
    }

    if (dropIntroActive) {
      unsigned long introElapsed = millis() - matrixIntroStartMs;
      if (introElapsed >= (unsigned long)drops[i].introDelayMs) {
        unsigned long activeMs = introElapsed;
        float relY = (float)drops[i].introStartY +
                     ((float)((int)activeMs - drops[i].introDelayMs) * drops[i].introSpeedPxPerMs);
        drawY = bodyTop + (int)relY;
      } else {
        drawY = bodyTop + drops[i].introStartY;
      }

      int bodyRelY = drawY - bodyTop;
      if (bodyRelY > (bodyBottom - bodyTop)) {
        drops[i].introActive = false;
        dropIntroActive = false;
        resetMatrixDropRandom(i);
        drawY = bodyTop + drops[i].y;
      } else {
        anyIntroActive = true;
      }
    }

    if (drawX < 0 || drawX > (320 - 6)) {
      if (!dropIntroActive) {
        drops[i].y += drops[i].speed;
        if (drops[i].y > (bodyBottom - bodyTop)) {
          resetMatrixDropRandom(i);
        }
      }
      continue;
    }
    tft.setTextSize(1);
    for (int j = 0; j < drops[i].len; j++) {
      int charY = drawY + (j * charStep);
      if (charY >= bodyTop && charY < bodyBottom) {
        // Blokuj znak tła, jeśli jego prostokąt przecina obszar maski zegara.
        if (drawX < (clockMaskX + clockMaskW) && (drawX + charW) > clockMaskX &&
            charY < (clockMaskY + clockMaskH) && (charY + charStep) > clockMaskY) {
          continue;
        }
        int denom = (drops[i].len > 1) ? (drops[i].len - 1) : 1;
        uint8_t t = (uint8_t)((255 * j) / denom); // 0 (ciemno) -> 255 (jasno)
        // Najniższa litera (głowa) ma być wyraźnie jaśniejsza od drugiej.
        if (j == drops[i].len - 1) {
          t = 255;
        } else if (drops[i].len > 1 && j == drops[i].len - 2) {
          t = min<uint8_t>(t, 135);
        }
        uint16_t color = (j == drops[i].len - 1)
               ? TFT_WHITE
               : lerpColor565(TFT_BLACK, MATRIX_BRIGHTGREEN, t);
        tft.setTextColor(color);
        tft.setCursor(drawX, charY);
        char ch = (j == drops[i].len - 1) ? drops[i].headChar : (char)random(33, 126);
        tft.print(ch);
      }
    }

    if (!dropIntroActive) {
      drops[i].y += drops[i].speed;

      // Reset kropli, gdy wypadnie poza obszar
      if (drops[i].y > (bodyBottom - bodyTop)) {
        resetMatrixDropRandom(i);
      }
    }
  }

  matrixIntroActive = anyIntroActive;
}

void clearMatrixArea(int bodyTop, int bodyBottom) {
  int bodyH = bodyBottom - bodyTop;
  if (clockMaskW <= 0 || clockMaskH <= 0) {
    tft.fillRect(0, bodyTop, SCREEN10_WIDTH, bodyH, TFT_BLACK);
    return;
  }

  int leftW = clockMaskX;
  int rightX = clockMaskX + clockMaskW;
  int rightW = SCREEN10_WIDTH - rightX;
  int topH = clockMaskY - bodyTop;
  int bottomY = clockMaskY + clockMaskH;
  int bottomH = bodyBottom - bottomY;

  if (topH > 0) {
    tft.fillRect(0, bodyTop, SCREEN10_WIDTH, topH, TFT_BLACK);
  }
  if (bottomH > 0) {
    tft.fillRect(0, bottomY, SCREEN10_WIDTH, bottomH, TFT_BLACK);
  }
  if (leftW > 0) {
    tft.fillRect(0, clockMaskY, leftW, clockMaskH, TFT_BLACK);
  }
  if (rightW > 0) {
    tft.fillRect(rightX, clockMaskY, rightW, clockMaskH, TFT_BLACK);
  }
}

void drawMatrixStatic() {
  //tft.fillRect(0, 0, 320, SCREEN10_HEADER_H, TFT_RADIO_ORANGE);
  //tft.setTextColor(TFT_BLACK);
  //tft.setTextSize(2);
 // tft.setCursor(10, 8);
  //tft.print("ZEGAR");

  // StrzaĹ‚ki nawigacyjne na dole ekranu
  //tft.fillTriangle(10, 230, 20, 222, 20, 238, TFT_RADIO_ORANGE);
  //tft.fillTriangle(310, 230, 300, 222, 300, 238, TFT_RADIO_ORANGE);
  //tft.setTextColor(0x52AA); // Ciemny szary
  //tft.setTextSize(1);
  //tft.setCursor(125, 226);
  //tft.print("SWITCH SCREEN");
}

void drawMatrixFrame() {
  String timeLocal = getTimezoneTimeString("%H:%M:%S", 9);
  int timeWidth = timeLocal.length() * CLOCK_CHAR_W * CLOCK_TEXT_SIZE;
  int timeHeight = CLOCK_CHAR_H * CLOCK_TEXT_SIZE;
  const int maskPadX = 3;
  const int maskPadTop = 3;
  const int maskPadBottom = 0; // dolny margines zmniejszony o 3 px
  int timeX = (SCREEN10_WIDTH - timeWidth) / 2;
  int timeY = 20;
  clockMaskX = max(0, timeX - maskPadX);
  clockMaskY = max(0, timeY - maskPadTop);
  clockMaskW = min(SCREEN10_WIDTH - clockMaskX, timeWidth + (2 * maskPadX) + 2);
  clockMaskH = min(SCREEN10_BODY_BOTTOM - clockMaskY,
                   timeHeight + maskPadTop + maskPadBottom + 2);

  clearMatrixArea(SCREEN10_BODY_TOP, SCREEN10_BODY_BOTTOM);
  drawMatrixBackground(SCREEN10_BODY_TOP, SCREEN10_BODY_BOTTOM);

  if (clockNeedsRedraw || timeLocal != lastClockText) {
    tft.fillRect(clockMaskX, clockMaskY, clockMaskW, clockMaskH, TFT_BLACK);
    tft.setTextSize(CLOCK_TEXT_SIZE);

    // PoĹ›wiata
    tft.setTextColor(MATRIX_DARKGREEN);
    tft.setCursor(timeX + 2, timeY + 2);
    tft.print(timeLocal);

    // GĹ‚Ăłwny tekst
    tft.setTextColor(MATRIX_BRIGHTGREEN);
    tft.setCursor(timeX, timeY);
    tft.print(timeLocal);

    lastClockText = timeLocal;
    clockNeedsRedraw = false;
  }

  // Data pod zegarem
 

  // Znak wywoĹ‚awczy
  //tft.setTextSize(2);
  //tft.setTextColor(MATRIX_BRIGHTGREEN);
  //String call = "OP: " + String(userCallsign);
  //call.toUpperCase();
  //tft.setCursor(10, 190);
  //tft.print(call);

  // IP Status

}

// Ekran 10: Zegar z animowanym tĹ‚em
void drawMatrixClock() {
  if (!matrixInitialized) {
    setupMatrix();
  }
  drawMatrixStatic();
  screen10NeedsRedraw = false;
  lastScreen10UpdateMs = millis();
  lastMatrixUpdateMs = 0;
  clockNeedsRedraw = true;
  drawMatrixFrame();
}

void updateScreen10() {
  if (!tftInitialized || currentScreen != SCREEN_MATRIX_CLOCK || inMenu) {
    return;
  }

  unsigned long now = millis();
  if (now - lastScreen10UpdateMs < MATRIX_UPDATE_INTERVAL_MS) {
    return;
  }
  lastScreen10UpdateMs = now;

  if (screen10NeedsRedraw) {
    drawMatrixStatic();
    screen10NeedsRedraw = false;
    clockNeedsRedraw = true;
  }
  drawMatrixFrame();
}

// ========== Ekran 11: UnlisHunter ==========

bool unlisRunning = false;
bool unlisGameOver = false;
bool unlisIntroVisible = true;
bool unlisIntroNeedsRedraw = true;
bool unlisGameOverNeedsRedraw = false;
bool unlisRunningUiNeedsRedraw = false;
int unlisCaught = 0;
int unlisMissed = 0;
unsigned long unlisGameStartMs = 0;
unsigned long unlisLastFrameMs = 0;
unsigned long unlisElapsedSecFrozen = 0;
float unlisScanAngleDeg = 0.0f;
float unlisPrevScanAngleDeg = 0.0f;
bool unlisArrowPrevValid = false;

bool unlisTargetActive = false;
bool unlisTargetOuter = true;
float unlisTargetAngleDeg = 0.0f;
unsigned long unlisTargetShownMs = 0;
unsigned long unlisNextSpawnMs = 0;

bool unlisSecondTargetActive = false;
bool unlisSecondTargetOuter = true;
float unlisSecondTargetAngleDeg = 0.0f;
unsigned long unlisSecondTargetShownMs = 0;
unsigned long unlisSecondTargetNextSpawnMs = 0;

bool unlisGreenStationActive = false;
bool unlisGreenStationOuter = true;
float unlisGreenStationAngleDeg = 0.0f;
unsigned long unlisGreenStationShownMs = 0;
unsigned long unlisGreenStationNextSpawnMs = 0;

bool unlisOuterEdgeNeedsClean = false;
unsigned long unlisLastPttPressMs = 0;
bool unlisUiNeedsRefreshOnTargetChange = true;
unsigned long unlisLastTimerSecDrawn = 0;
bool unlisTimerDrawnValid = false;

static inline float normalizeDeg360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

static float unlisCurrentScanDegPerSec() {
  // Up to 60s: +5% per catch, after 60s: +3% per catch.
  unsigned long elapsedMs = 0;
  if (unlisRunning || unlisGameOver) {
    elapsedMs = millis() - unlisGameStartMs;
  }
  float accelPerCatch = (elapsedMs >= 60000UL) ? UNLIS_ACCEL_PER_CATCH_LATE : UNLIS_ACCEL_PER_CATCH_EARLY;
  return UNLIS_BASE_SCAN_DEG_PER_SEC * (1.0f + ((float)unlisCaught * accelPerCatch));
}

static unsigned long unlisCurrentTargetLifeMs() {
  float degPerSec = unlisCurrentScanDegPerSec();
  float fullRotationSec = 360.0f / degPerSec;
  return (unsigned long)(fullRotationSec * UNLIS_TARGET_LIFE_ROTATIONS * 1000.0f + 0.5f);
}

static unsigned long unlisCurrentPttCooldownMs() {
  float degPerSec = unlisCurrentScanDegPerSec();
  float fullRotationSec = 360.0f / degPerSec;
  return (unsigned long)(fullRotationSec * 0.25f * 1000.0f + 0.5f);
}

static void unlisResetPttCooldown(unsigned long nowMs) {
  unlisLastPttPressMs = nowMs - unlisCurrentPttCooldownMs();
}

static bool unlisPointInButton(int x, int y) {
  if (x >= UNLIS_START_X && x < (UNLIS_START_X + UNLIS_BTN_SIZE) &&
      y >= UNLIS_START_Y && y < (UNLIS_START_Y + UNLIS_BTN_SIZE)) {
    return true;
  }
  if (x >= UNLIS_PTT_X && x < (UNLIS_PTT_X + UNLIS_BTN_SIZE) &&
      y >= UNLIS_PTT_Y && y < (UNLIS_PTT_Y + UNLIS_BTN_SIZE)) {
    return true;
  }
  if (x >= UNLIS_EXIT_X && x < (UNLIS_EXIT_X + UNLIS_BTN_SIZE) &&
      y >= UNLIS_EXIT_Y && y < (UNLIS_EXIT_Y + UNLIS_BTN_SIZE)) {
    return true;
  }
  return false;
}

static void unlisGenerateRandomTarget(bool &targetOuter, float &targetAngleDeg) {
  bool foundVisibleSpot = false;
  bool outer = true;
  float angle = 0.0f;

  for (int i = 0; i < 24; i++) {
    bool candidateOuter = (random(0, 2) == 0);
    float candidateAngle = (float)random(0, 360);
    float a = candidateAngle * (float)M_PI / 180.0f;
    int r = candidateOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int tx = UNLIS_CENTER_X + (int)(sinf(a) * (float)r);
    int ty = UNLIS_CENTER_Y - (int)(cosf(a) * (float)r);
    if (!unlisPointInButton(tx, ty)) {
      outer = candidateOuter;
      angle = candidateAngle;
      foundVisibleSpot = true;
      break;
    }
  }

  if (!foundVisibleSpot) {
    outer = (random(0, 2) == 0);
    angle = (float)random(0, 360);
  }

  targetOuter = outer;
  targetAngleDeg = angle;
}

static bool unlisArrowHitsTarget(bool targetOuter, float targetAngleDeg, int hitPx) {
  float arrowDeg = targetOuter ? unlisScanAngleDeg : normalizeDeg360(unlisScanAngleDeg + 180.0f);
  int ringR = targetOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
  float arrowA = arrowDeg * (float)M_PI / 180.0f;
  float targetA = targetAngleDeg * (float)M_PI / 180.0f;
  int arrowX = UNLIS_CENTER_X + (int)(sinf(arrowA) * (float)ringR);
  int arrowY = UNLIS_CENTER_Y - (int)(cosf(arrowA) * (float)ringR);
  int targetX = UNLIS_CENTER_X + (int)(sinf(targetA) * (float)ringR);
  int targetY = UNLIS_CENTER_Y - (int)(cosf(targetA) * (float)ringR);
  int dx = arrowX - targetX;
  int dy = arrowY - targetY;
  return (dx * dx + dy * dy) <= (hitPx * hitPx);
}

static void drawUnlisButton(int x, int y, const char* label) {
  const int drawW = UNLIS_DRAW_BTN_SIZE;
  const int drawH = UNLIS_DRAW_BTN_SIZE;
  int drawX = x + 10;
  int drawY = y + 10;

  // Keep visual buttons closer to screen corners (~10 px margin).
  if (x == UNLIS_PTT_X) {
    drawX = x + (UNLIS_BTN_SIZE - drawW - 10);
  }
  if (y == UNLIS_START_Y) {
    drawY = y;
  }
  if (y == UNLIS_PTT_Y || y == UNLIS_EXIT_Y) {
    drawY = y + (UNLIS_BTN_SIZE - drawH);
  }
  const int r = 10;

  // Czarny przycisk z lekkim efektem wypuklosci.
  tft.fillRoundRect(drawX, drawY, drawW, drawH, r, TFT_BLACK);
  tft.drawRoundRect(drawX, drawY, drawW, drawH, r, TFT_LIGHTGREY);
  tft.drawFastHLine(drawX + 8, drawY + 4, drawW - 16, TFT_WHITE);
  tft.drawFastVLine(drawX + 4, drawY + 8, drawH - 16, TFT_WHITE);
  tft.drawFastHLine(drawX + 8, drawY + drawH - 5, drawW - 16, TFT_DARKGREY);
  tft.drawFastVLine(drawX + drawW - 5, drawY + 8, drawH - 16, TFT_DARKGREY);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int labelW = (int)strlen(label) * 12;
  tft.setCursor(drawX + (drawW - labelW) / 2, drawY + 18);
  tft.print(label);
}

static void drawUnlisButtons() {
  const char* startLabel = unlisRunning ? "STOP" : "START";
  drawUnlisButton(UNLIS_START_X, UNLIS_START_Y, startLabel);
  drawUnlisButton(UNLIS_PTT_X, UNLIS_PTT_Y, "PTT");
  drawUnlisButton(UNLIS_EXIT_X, UNLIS_EXIT_Y, "EXIT");
}

static void drawUnlisScoreAndMissMarkers() {
  tft.fillRect(2, 52, 84, 96, TFT_BLACK);

  String scoreText = String(unlisCaught);
  while (scoreText.length() < 3) {
    scoreText = "0" + scoreText;
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(4, 56);
  tft.print(scoreText);

  const int markerX = 12;
  const int markerMidY = 120;
  const int markerStepY = 18;
  const int markerR = 5;
  for (int i = 0; i < 3; i++) {
    int markerY = markerMidY + ((i - 1) * markerStepY);
    bool missed = (unlisMissed > i);
    if (missed) {
      tft.fillCircle(markerX, markerY, markerR, TFT_RED);
      tft.drawCircle(markerX, markerY, markerR, TFT_RED);
    } else {
      tft.fillCircle(markerX, markerY, markerR, TFT_BLACK);
      tft.drawCircle(markerX, markerY, markerR, TFT_DARKGREY);
    }
  }
}

static void drawUnlisBandLegend() {
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(286, 96);
  tft.print("BAND:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(292, 108);
  tft.print("2m");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.setCursor(286, 120);
  tft.print("70cm");
}

static void drawUnlisTimerValue(unsigned long elapsedSec) {
  String timerText = String(elapsedSec) + "s";
  int tw = timerText.length() * 12;
  tft.fillRect(236, 2, 82, 20, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(316 - tw, 6);
  tft.print(timerText);
}

static void drawUnlisHud() {
  drawUnlisButtons();
  
  unsigned long elapsedSec = 0;
  if (unlisRunning) {
    elapsedSec = (millis() - unlisGameStartMs) / 1000UL;
  } else if (unlisGameOver) {
    elapsedSec = unlisElapsedSecFrozen;
  }

  drawUnlisScoreAndMissMarkers();
  drawUnlisBandLegend();
  drawUnlisTimerValue(elapsedSec);
}

static void drawUnlisStatsOnly() {

  unsigned long elapsedSec = 0;
  if (unlisRunning) {
    elapsedSec = (millis() - unlisGameStartMs) / 1000UL;
  } else if (unlisGameOver) {
    elapsedSec = unlisElapsedSecFrozen;
  }

  drawUnlisScoreAndMissMarkers();
  drawUnlisTimerValue(elapsedSec);
}

static void drawUnlisIntroText() {
  tft.fillRect(40, 72, 240, 92, TFT_BLACK);
  tft.drawRect(40, 72, 240, 92, TFT_DARKGREY);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(52, 82);
  tft.print("UNLIS HUNTER GAME");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(52, 114);
  tft.print("Use a Yagi antenna to catch and");
  tft.setCursor(52, 130);
  tft.print("jam its signal by pressing PTT on it.");
  tft.setCursor(52, 146);
  tft.print("* Unlis = Unlicensed");
}

static void drawUnlisRadarBase() {
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_INNER_R, TFT_BLUE);
  tft.fillCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, 3, TFT_GREEN);
}

static void drawUnlisOuterRing() {
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R, TFT_YELLOW);
  // Black outer outline masks red artifacts outside the yellow ring.
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R + 1, TFT_BLACK);
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R + 2, TFT_BLACK);
}

static void drawUnlisTarget() {
  if (unlisTargetActive) {
    float a = unlisTargetAngleDeg * (float)M_PI / 180.0f;
    int r = unlisTargetOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int tx = UNLIS_CENTER_X + (int)(sinf(a) * (float)r);
    int ty = UNLIS_CENTER_Y - (int)(cosf(a) * (float)r);
    int targetRadius = unlisTargetOuter ? 6 : 4;
    tft.fillCircle(tx, ty, targetRadius, TFT_RED);
  }

  if (unlisSecondTargetActive) {
    float a2 = unlisSecondTargetAngleDeg * (float)M_PI / 180.0f;
    int r2 = unlisSecondTargetOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int tx2 = UNLIS_CENTER_X + (int)(sinf(a2) * (float)r2);
    int ty2 = UNLIS_CENTER_Y - (int)(cosf(a2) * (float)r2);
    int targetRadius2 = unlisSecondTargetOuter ? 6 : 4;
    tft.fillCircle(tx2, ty2, targetRadius2, TFT_RED);
  }

  if (unlisGreenStationActive) {
    float ag = unlisGreenStationAngleDeg * (float)M_PI / 180.0f;
    int rg = unlisGreenStationOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int txg = UNLIS_CENTER_X + (int)(sinf(ag) * (float)rg);
    int tyg = UNLIS_CENTER_Y - (int)(cosf(ag) * (float)rg);
    tft.fillCircle(txg, tyg, 4, TFT_GREEN);
  }
}

static void drawUnlisArrows(float angleDeg) {
  float a = angleDeg * (float)M_PI / 180.0f;
  int yx = UNLIS_CENTER_X + (int)(sinf(a) * (float)UNLIS_OUTER_R);
  int yy = UNLIS_CENTER_Y - (int)(cosf(a) * (float)UNLIS_OUTER_R);
  int bx = UNLIS_CENTER_X - (int)(sinf(a) * (float)UNLIS_INNER_R);
  int by = UNLIS_CENTER_Y + (int)(cosf(a) * (float)UNLIS_INNER_R);

  tft.drawLine(UNLIS_CENTER_X, UNLIS_CENTER_Y, yx, yy, TFT_YELLOW);
  tft.drawLine(UNLIS_CENTER_X, UNLIS_CENTER_Y, bx, by, TFT_BLUE);
}

static void drawUnlisGameOver() {
  tft.fillRect(60, 96, 200, 48, TFT_BLACK);
  tft.drawRect(60, 96, 200, 48, TFT_RED);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(2);
  const char* txt = "BAJO JAJO";
  int tw = (int)strlen(txt) * 12;
  tft.setCursor(60 + (200 - tw) / 2, 112);
  tft.print(txt);
}

void unlisStopGame() {
  unlisRunning = false;
  unlisGameOver = false;
  unlisIntroVisible = true;
  unlisIntroNeedsRedraw = true;
  unlisGameOverNeedsRedraw = false;
  unlisRunningUiNeedsRedraw = false;
  unlisTargetActive = false;
  unlisSecondTargetActive = false;
  unlisGreenStationActive = false;
  unlisArrowPrevValid = false;
  unlisUiNeedsRefreshOnTargetChange = true;
  unlisTimerDrawnValid = false;
}

void unlisStartResetGame() {
  unlisRunning = true;
  unlisGameOver = false;
  unlisIntroVisible = false;
  unlisIntroNeedsRedraw = false;
  unlisGameOverNeedsRedraw = false;
  unlisRunningUiNeedsRedraw = true;
  unlisCaught = 0;
  unlisMissed = 0;
  unlisElapsedSecFrozen = 0;
  unlisGameStartMs = millis();
  unlisLastFrameMs = millis();
  unlisScanAngleDeg = 0.0f;
  unlisPrevScanAngleDeg = 0.0f;
  unlisArrowPrevValid = false;
  unlisTargetActive = false;
  unlisSecondTargetActive = false;
  unlisGreenStationActive = false;
  unlisOuterEdgeNeedsClean = true;
  unlisResetPttCooldown(millis());
  unlisUiNeedsRefreshOnTargetChange = true;
  unlisTimerDrawnValid = false;
  unlisNextSpawnMs = millis() + random(200UL, 700UL);
  unlisSecondTargetNextSpawnMs = millis() + UNLIS_SECOND_TARGET_DELAY_MS + random(500UL, 3000UL);
  unlisGreenStationNextSpawnMs = millis() + random(UNLIS_GREEN_STATION_RESPAWN_MIN_MS, UNLIS_GREEN_STATION_RESPAWN_MAX_MS + 1UL);
}

static void unlisSpawnTarget(unsigned long nowMs) {
  bool targetOuter = true;
  float targetAngle = 0.0f;
  unlisGenerateRandomTarget(targetOuter, targetAngle);

  unlisTargetActive = true;
  unlisTargetOuter = targetOuter;
  unlisTargetAngleDeg = targetAngle;
  unlisTargetShownMs = nowMs;
  unlisOuterEdgeNeedsClean = true;
  unlisResetPttCooldown(nowMs);
  unlisUiNeedsRefreshOnTargetChange = true;
}

void unlisHandlePttPress(unsigned long nowMs) {
  if (!unlisRunning || unlisGameOver) return;

  unsigned long pttCooldownMs = unlisCurrentPttCooldownMs();
  if ((nowMs - unlisLastPttPressMs) < pttCooldownMs) return;
  unlisLastPttPressMs = nowMs;

  const int hitPx = 14;

  if (unlisGreenStationActive && unlisArrowHitsTarget(unlisGreenStationOuter, unlisGreenStationAngleDeg, hitPx)) {
    if (unlisCaught > 0) {
      unlisCaught--;
    }
    unlisGreenStationActive = false;
    unlisGreenStationNextSpawnMs = nowMs + random(UNLIS_GREEN_STATION_RESPAWN_MIN_MS, UNLIS_GREEN_STATION_RESPAWN_MAX_MS + 1UL);
    unlisUiNeedsRefreshOnTargetChange = true;
    return;
  }

  bool hitAnyUnlis = false;
  if (unlisTargetActive && unlisArrowHitsTarget(unlisTargetOuter, unlisTargetAngleDeg, hitPx)) {
    unlisCaught++;
    unlisTargetActive = false;
    hitAnyUnlis = true;
  }
  if (unlisSecondTargetActive && unlisArrowHitsTarget(unlisSecondTargetOuter, unlisSecondTargetAngleDeg, hitPx)) {
    unlisCaught++;
    unlisSecondTargetActive = false;
      unlisSecondTargetNextSpawnMs = nowMs + random(UNLIS_SECOND_TARGET_RESPAWN_MIN_MS, UNLIS_SECOND_TARGET_RESPAWN_MAX_MS + 1UL);
    hitAnyUnlis = true;
  }

  if (hitAnyUnlis) {
    unlisResetPttCooldown(nowMs);
    if (!unlisTargetActive && !unlisSecondTargetActive) {
      unlisNextSpawnMs = nowMs + random(UNLIS_TARGET_RESPAWN_MIN_MS, UNLIS_TARGET_RESPAWN_MAX_MS + 1UL);
    }
    unlisUiNeedsRefreshOnTargetChange = true;
  }
}

void drawUnlisHunter() {
  unlisIntroVisible = true;
  unlisIntroNeedsRedraw = true;
  unlisGameOverNeedsRedraw = false;
  unlisRunningUiNeedsRedraw = false;

  unlisRunning = false;
  unlisGameOver = false;
  unlisTargetActive = false;
  unlisSecondTargetActive = false;
  unlisGreenStationActive = false;
  unlisOuterEdgeNeedsClean = false;
  unlisArrowPrevValid = false;
  unlisElapsedSecFrozen = 0;
  unlisUiNeedsRefreshOnTargetChange = true;
  unlisTimerDrawnValid = false;
}

void updateUnlisHunter() {
  if (!tftInitialized || currentScreen != SCREEN_UNLIS_HUNTER || inMenu) {
    return;
  }

  if (!unlisRunning && !unlisGameOver) {
    if (unlisIntroVisible && unlisIntroNeedsRedraw) {
      tft.fillScreen(TFT_BLACK);
      drawUnlisHud();
      drawUnlisIntroText();
      unlisIntroNeedsRedraw = false;
    }
    return;
  }

  if (unlisGameOver) {
    if (unlisGameOverNeedsRedraw) {
      tft.fillScreen(TFT_BLACK);
      drawUnlisHud();
      drawUnlisOuterRing();
      drawUnlisRadarBase();
      drawUnlisArrows(unlisScanAngleDeg);
      drawUnlisGameOver();
      unlisGameOverNeedsRedraw = false;
    }
    return;
  }

  if (unlisRunningUiNeedsRedraw) {
    tft.fillScreen(TFT_BLACK);
    drawUnlisButtons();
    drawUnlisOuterRing();
    drawUnlisBandLegend();
    unlisUiNeedsRefreshOnTargetChange = true;
    unlisTimerDrawnValid = false;
    unlisRunningUiNeedsRedraw = false;
  }

  unsigned long nowMs = millis();
  if ((nowMs - unlisLastFrameMs) < UNLIS_FRAME_MS) {
    return;
  }

  float dt = (float)(nowMs - unlisLastFrameMs) / 1000.0f;
  unlisLastFrameMs = nowMs;

  if (unlisRunning && !unlisGameOver) {
    unlisScanAngleDeg = normalizeDeg360(unlisScanAngleDeg + (unlisCurrentScanDegPerSec() * dt));

    if (!unlisTargetActive && !unlisSecondTargetActive && nowMs >= unlisNextSpawnMs) {
      unlisSpawnTarget(nowMs);
    }

    unsigned long gameElapsedMs = nowMs - unlisGameStartMs;
    if (gameElapsedMs >= UNLIS_SECOND_TARGET_DELAY_MS &&
        !unlisSecondTargetActive &&
        nowMs >= unlisSecondTargetNextSpawnMs) {
      unlisGenerateRandomTarget(unlisSecondTargetOuter, unlisSecondTargetAngleDeg);
      unlisSecondTargetActive = true;
      unlisSecondTargetShownMs = nowMs;
      unlisOuterEdgeNeedsClean = true;
      unlisResetPttCooldown(nowMs);
      unlisUiNeedsRefreshOnTargetChange = true;
    }

    if (!unlisGreenStationActive && nowMs >= unlisGreenStationNextSpawnMs) {
      unlisGenerateRandomTarget(unlisGreenStationOuter, unlisGreenStationAngleDeg);
      unlisGreenStationActive = true;
      unlisGreenStationShownMs = nowMs;
      unlisGreenStationNextSpawnMs = nowMs + random(UNLIS_GREEN_STATION_RESPAWN_MIN_MS, UNLIS_GREEN_STATION_RESPAWN_MAX_MS + 1UL);
      unlisUiNeedsRefreshOnTargetChange = true;
    }

    if (unlisTargetActive && (nowMs - unlisTargetShownMs) >= unlisCurrentTargetLifeMs()) {
      unlisTargetActive = false;
      unlisResetPttCooldown(nowMs);
      unlisMissed++;
      if (unlisMissed >= 3) {
        unlisElapsedSecFrozen = (nowMs - unlisGameStartMs) / 1000UL;
        unlisGameOver = true;
        unlisRunning = false;
        unlisTargetActive = false;
        unlisSecondTargetActive = false;
        unlisGreenStationActive = false;
        unlisGameOverNeedsRedraw = true;
        unlisUiNeedsRefreshOnTargetChange = true;
      } else {
        if (!unlisSecondTargetActive) {
          unlisNextSpawnMs = nowMs + random(UNLIS_TARGET_RESPAWN_MIN_MS, UNLIS_TARGET_RESPAWN_MAX_MS + 1UL);
        }
        unlisUiNeedsRefreshOnTargetChange = true;
      }
    }

    if (unlisSecondTargetActive && (nowMs - unlisSecondTargetShownMs) >= unlisCurrentTargetLifeMs()) {
      unlisSecondTargetActive = false;
      unlisSecondTargetNextSpawnMs = nowMs + random(UNLIS_SECOND_TARGET_RESPAWN_MIN_MS, UNLIS_SECOND_TARGET_RESPAWN_MAX_MS + 1UL);
      unlisResetPttCooldown(nowMs);
      unlisMissed++;
      if (unlisMissed >= 3) {
        unlisElapsedSecFrozen = (nowMs - unlisGameStartMs) / 1000UL;
        unlisGameOver = true;
        unlisRunning = false;
        unlisTargetActive = false;
        unlisSecondTargetActive = false;
        unlisGreenStationActive = false;
        unlisGameOverNeedsRedraw = true;
      } else if (!unlisTargetActive) {
        unlisNextSpawnMs = nowMs + random(UNLIS_TARGET_RESPAWN_MIN_MS, UNLIS_TARGET_RESPAWN_MAX_MS + 1UL);
      }
      unlisUiNeedsRefreshOnTargetChange = true;
    }

    if (unlisGreenStationActive && (nowMs - unlisGreenStationShownMs) >= UNLIS_GREEN_STATION_LIFE_MS) {
      unlisGreenStationActive = false;
      unlisUiNeedsRefreshOnTargetChange = true;
    }
  }

  // Clear and redraw only radar zone; keep static buttons untouched.
  tft.fillCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R - 1, TFT_BLACK);

  bool redrawOuterRingNow = false;

  if (unlisOuterEdgeNeedsClean) {
    // Before drawing a new target, clean ring edge.
    for (int r = UNLIS_OUTER_R - 2; r <= UNLIS_OUTER_R + 6; r++) {
      tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, r, TFT_BLACK);
    }
    redrawOuterRingNow = true;
    unlisOuterEdgeNeedsClean = false;
  }

  bool scoreMarkersRedrawn = false;
  if (unlisUiNeedsRefreshOnTargetChange) {
    drawUnlisScoreAndMissMarkers();
    unlisUiNeedsRefreshOnTargetChange = false;
    scoreMarkersRedrawn = true;
  }

  // Keep ring above left score/marker area and redraw after those elements.
  if (redrawOuterRingNow || scoreMarkersRedrawn) {
    drawUnlisOuterRing();
  }

  unsigned long elapsedSec = (nowMs - unlisGameStartMs) / 1000UL;
  if (!unlisTimerDrawnValid || elapsedSec != unlisLastTimerSecDrawn) {
    drawUnlisTimerValue(elapsedSec);
    unlisLastTimerSecDrawn = elapsedSec;
    unlisTimerDrawnValid = true;
  }

  // Keep radar and moving gameplay elements on top of the screen.
  drawUnlisRadarBase();
  drawUnlisTarget();
  drawUnlisArrows(unlisScanAngleDeg);
  unlisPrevScanAngleDeg = unlisScanAngleDeg;
  unlisArrowPrevValid = true;
}

// Ekrany 6-10: Puste (tylko napis)
void drawScreenEmpty(int screenNum) {
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(3);
  
  String screenText = String(tr(TR_PAGE)) + " " + String(screenNum);
  int textWidth = screenText.length() * 18;
  int xPos = (320 - textWidth) / 2;
  int yPos = 110; // WyĹ›rodkowane w pionie
  
  tft.setCursor(xPos, yPos);
  tft.print(screenText);

    // 4. STRZAĹKI ZMIANY EKRANU (Nawigacja)
  // Lewa strzaĹ‚ka (TrĂłjkÄ…t: x1, y1, x2, y2, x3, y3, kolor)
  tft.fillTriangle(10, 230, 20, 222, 20, 238, TFT_RADIO_ORANGE);
  
  // Prawa strzaĹ‚ka
  tft.fillTriangle(310, 230, 300, 222, 300, 238, TFT_RADIO_ORANGE);
  
  // Opcjonalny opis miÄ™dzy strzaĹ‚kami (maĹ‚Ä… czcionkÄ…)
  tft.setTextColor(0x52AA); // Ciemny szary
  tft.setTextSize(1);
  tft.setCursor(125, 226);
  tft.print("SWITCH SCREEN");
}

// Rysuj strzałki nawigacyjne < >
void drawNavigationArrows() {
  // Intentionally empty: nawigacja przez dotyk bez elementĂłw UI
}

static volatile bool uiPendingScreen1Redraw = false;
static volatile bool uiPendingScreen2Redraw = false;
static volatile bool uiPendingScreen6Redraw = false;
static volatile bool uiPendingScreen7Redraw = false;
static volatile bool uiPendingAnyScreenRedraw = false;
static volatile uint8_t uiPendingAnyScreenId = SCREEN_HAM_CLOCK;
static portMUX_TYPE uiPendingRedrawMux = portMUX_INITIALIZER_UNLOCKED;

static void requestUiScreenRedraw(uint8_t pendingScreenId) {
  portENTER_CRITICAL(&uiPendingRedrawMux);
  uiPendingAnyScreenId = pendingScreenId;
  uiPendingAnyScreenRedraw = true;
  switch (pendingScreenId) {
    case SCREEN_HAM_CLOCK:
      uiPendingScreen1Redraw = true;
      break;
    case SCREEN_DX_CLUSTER:
      uiPendingScreen2Redraw = true;
      break;
    case SCREEN_APRS_IS:
    case SCREEN_APRS_RADAR:
      uiPendingScreen6Redraw = true;
      break;
    case SCREEN_POTA_CLUSTER:
      uiPendingScreen7Redraw = true;
      break;
    default:
      break;
  }
  portEXIT_CRITICAL(&uiPendingRedrawMux);
}

// Aktualizuj ekran 2 (DX Cluster) - wywoływane gdy przyjdą nowe spoty
void updateScreen2() {
  if (currentScreen == SCREEN_DX_CLUSTER && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(SCREEN_DX_CLUSTER);
  }
}

void updateScreen7() {
  if (currentScreen == SCREEN_POTA_CLUSTER && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(SCREEN_POTA_CLUSTER);
  }
}

void updateScreen6() {
  if ((currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(currentScreen);
  }
}

// Aktualizuj ekran 1 (Info) - wywoływane gdy zmieni się IP
void updateScreen1() {
  if (bootSequenceActive) {
    return;
  }
  if (currentScreen == SCREEN_HAM_CLOCK && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(SCREEN_HAM_CLOCK);
  }
}
#endif

void bootLogLine(const String &line) {
  Serial.println(line);
#ifdef ENABLE_TFT_DISPLAY
  if (tftInitialized) {
    tftBootPrintLine(line);
  }
#endif
  if (bootSequenceActive) {
    delay(500);
  }
}

// ========== FUNKCJE POMOCNICZE ==========
static uint16_t readBmp16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

static uint32_t readBmp32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read();
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read();
  return result;
}

static bool drawBmpFromFS(const String &filename, int16_t x, int16_t y) {
  bool oldSwap = false;
  bool swapChanged = false;
  try {
  if (!littleFsReady) {
    return false;
  }
  if ((x >= tft.width()) || (y >= tft.height())) {
    return false;
  }
  // Nie sprawdzaj exists() osobno: to robi dodatkowe fopen() i bywa niestabilne
  // po update frameworku; bezpieczniej jest od razu próbować open().
  fs::File bmpFS = LittleFS.open(filename.c_str(), "r");
  if (!bmpFS) {
    return false;
  }

  uint32_t seekOffset;
  int32_t w, h;
  if (readBmp16(bmpFS) != 0x4D42) {
    bmpFS.close();
    return false;
  }

  readBmp32(bmpFS);
  readBmp32(bmpFS);
  seekOffset = readBmp32(bmpFS);
  readBmp32(bmpFS);
  w = readBmp32(bmpFS);
  h = readBmp32(bmpFS);

  uint16_t planes = readBmp16(bmpFS);
  uint16_t bpp = readBmp16(bmpFS);
  uint32_t compression = readBmp32(bmpFS);

  // Support classic 24-bit BMP (weather icons) and 32-bit BMP (e.g. startup logo).
  bool bmp24 = (bpp == 24 && compression == 0);
  bool bmp32 = (bpp == 32 && (compression == 0 || compression == 3));
  if (planes != 1 || (!bmp24 && !bmp32) || w <= 0 || h == 0) {
    bmpFS.close();
    return false;
  }

  int32_t absH = (h > 0) ? h : -h;
  // Twarde limity dla ikon/tła na tym TFT, by nie ryzykować uszkodzenia stosu.
  if (w > 320 || absH > 320) {
    bmpFS.close();
    return false;
  }

  bool bottomUp = (h > 0);
  if (bottomUp) {
    y += (int16_t)(absH - 1);
  }

  oldSwap = tft.getSwapBytes();
  tft.setSwapBytes(true);
  swapChanged = true;
  bmpFS.seek(seekOffset);

  const uint8_t bytesPerPixel = bmp32 ? 4 : 3;
  uint16_t padding = (4 - (((uint32_t)w * bytesPerPixel) & 3)) & 3;
  uint32_t rowSize = (uint32_t)w * bytesPerPixel + padding;
  if (rowSize == 0 || rowSize > 2048) {
    tft.setSwapBytes(oldSwap);
    bmpFS.close();
    return false;
  }
  static uint8_t lineBuffer[2048];

  for (int32_t row = 0; row < absH; row++) {
    if (bmpFS.read(lineBuffer, rowSize) != (int)rowSize) {
      tft.setSwapBytes(oldSwap);
      bmpFS.close();
      return false;
    }
    uint8_t *bptr = lineBuffer;
    uint16_t *tptr = (uint16_t *)lineBuffer;
    for (int32_t col = 0; col < w; col++) {
      uint8_t b = *bptr++;
      uint8_t g = *bptr++;
      uint8_t r = *bptr++;
      if (bmp32) {
        bptr++; // Skip alpha/extra byte.
      }
      *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
    if (bottomUp) {
      tft.pushImage(x, y--, (uint16_t)w, 1, (uint16_t *)lineBuffer);
    } else {
      tft.pushImage(x, y + (int16_t)row, (uint16_t)w, 1, (uint16_t *)lineBuffer);
    }
  }

  tft.setSwapBytes(oldSwap);
  swapChanged = false;
  bmpFS.close();
  return true;
  } catch (...) {
    if (swapChanged) {
      tft.setSwapBytes(oldSwap);
    }
    Serial.println("[BMP] Wyjatek podczas odczytu ikony BMP - pomijam rysowanie");
    return false;
  }
}

// ZwrĂłÄ‡ surowy symbol APRS jako 2 znaki (table + code)
String getAPRSSymbolRaw(const APRSStation &station) {
  if (station.symbolTable.length() > 0 && station.symbol.length() > 0) {
    return station.symbolTable + station.symbol;
  }
  if (station.symbol.length() > 0) {
    return station.symbol;
  }
  return "-";
}

// ZwrĂłÄ‡ krĂłtki opis symbolu (<=10 znakĂłw) zgodny z tabelÄ… APRS
String getAPRSSymbolShort(const APRSStation &station) {
  String raw = getAPRSSymbolRaw(station);
  if (raw == "/-") return "HOUSE";
  if (raw == "/>") return "CAR";
  if (raw == "/#") return "DIGI";
  if (raw == "/r") return "REPEATER";
  if (raw == "/[") return "HUMAN";
  if (raw == "/l") return "LAPTOP";
  if (raw == "/L") return "LOGIN";
  if (raw == "/_") return "WX";
  if (raw == "/s") return "SHIP";
  if (raw == "/a") return "AMBUL";
  if (raw == "/b") return "BIKE";
  if (raw == "/c") return "ICP";
  if (raw == "/d") return "FIRE";
  if (raw == "/f") return "FIRETRK";
  if (raw == "/h") return "HOSP";
  if (raw == "/y") return "YAGI";
  if (raw == "/g") return "GLIDER";
  if (raw == "/k") return "TRUCK";
  if (raw == "/u") return "TRUCK";
  if (raw == "/v") return "VAN";
  if (raw == "/V") return "HAMCLOCK";
  if (raw == "/p") return "ROVER";
  if (raw == "/o") return "EOC";
  if (raw == "/t") return "TRKSTOP";
  if (raw == "/m") return "MICERPT";
  if (raw == "/n") return "NODE";
  if (raw == "/q") return "GRID";
  if (raw == "La" || raw == "L_" ) return "LORA";
  if (raw == "L#" || raw == "L&") return "LORA";
  return raw;
}

static String getAprsAlertSymbolDescription(const APRSStation &station) {
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "HOUSE") return "Home station";
  if (symbolShort == "CAR") return "CAR";
  if (symbolShort == "DIGI") return "Digipeater";
  if (symbolShort == "REPEATER") return "Repeater";
  if (symbolShort == "HUMAN") return "Pedestrian";
  if (symbolShort == "LAPTOP") return "Mobile station";
  if (symbolShort == "LOGIN") return "Login point";
  if (symbolShort == "WX") return "Weather station";
  if (symbolShort == "SHIP") return "Ship";
  if (symbolShort == "AMBUL") return "Ambulans";
  if (symbolShort == "BIKE") return "Bicycle";
  if (symbolShort == "FIRE") return "Fire";
  if (symbolShort == "FIRETRK") return "Fire truck";
  if (symbolShort == "HOSP") return "Hospital";
  if (symbolShort == "GLIDER") return "Glider";
  if (symbolShort == "TRUCK") return "Truck";
  if (symbolShort == "VAN") return "Van";
  if (symbolShort == "ROVER") return "Rover";
  if (symbolShort == "NODE") return "Node";
  if (symbolShort == "LORA") return "LoRa";
  return String("Symbol: ") + symbolShort;
}

static bool isAprsWeatherStationSymbol(const APRSStation &station) {
  if (station.symbol == "_") {
    return true;
  }
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "WX") {
    return true;
  }
  String rawSymbol = getAPRSSymbolRaw(station);
  return rawSymbol.length() >= 2 && rawSymbol.charAt(rawSymbol.length() - 1) == '_';
}

struct AprsWxDecoded {
  bool hasAny = false;
  bool hasWindDir = false;
  int windDirDeg = 0;
  bool hasWindSpeed = false;
  float windSpeedKmh = 0.0f;
  bool hasTempC = false;
  float tempC = 0.0f;
  bool hasHumidity = false;
  int humidityPct = 0;
  bool hasPressure = false;
  float pressureHpa = 0.0f;
};

static bool aprsWxReadDigits(const String &text, int start, int len, int &valueOut) {
  if (start < 0 || len <= 0 || (start + len) > text.length()) {
    return false;
  }
  int value = 0;
  for (int i = 0; i < len; i++) {
    char c = text.charAt(start + i);
    if (!isDigit(c)) {
      return false;
    }
    value = (value * 10) + (c - '0');
  }
  valueOut = value;
  return true;
}

static bool decodeAprsWxFromComment(const String &commentRaw, AprsWxDecoded &wx) {
  wx = AprsWxDecoded();

  String text = commentRaw;
  text.trim();
  if (text.length() == 0) {
    return false;
  }

  int dirDeg = 0;
  int speedMph = 0;
  if (text.length() >= 7 &&
      aprsWxReadDigits(text, 0, 3, dirDeg) &&
      text.charAt(3) == '/' &&
      aprsWxReadDigits(text, 4, 3, speedMph)) {
    wx.hasWindDir = true;
    wx.windDirDeg = dirDeg % 360;
    wx.hasWindSpeed = true;
    wx.windSpeedKmh = speedMph * 1.60934f;
  }

  for (int i = 0; i < text.length(); i++) {
    char tag = text.charAt(i);
    int digits = 0;
    switch (tag) {
      case 'c':
      case 's':
      case 'g':
      case 't':
      case 'r':
      case 'p':
      case 'P':
        digits = 3;
        break;
      case 'h':
        digits = 2;
        break;
      case 'b':
        digits = 5;
        break;
      default:
        break;
    }

    if (digits == 0) {
      continue;
    }

    int value = 0;
    if (!aprsWxReadDigits(text, i + 1, digits, value)) {
      continue;
    }

    if (tag == 'c') {
      wx.hasWindDir = true;
      wx.windDirDeg = value % 360;
    } else if (tag == 's' || (tag == 'g' && !wx.hasWindSpeed)) {
      wx.hasWindSpeed = true;
      wx.windSpeedKmh = value * 1.60934f;
    } else if (tag == 't') {
      wx.hasTempC = true;
      wx.tempC = (value - 32.0f) * (5.0f / 9.0f);
    } else if (tag == 'h') {
      wx.hasHumidity = true;
      wx.humidityPct = (value == 0) ? 100 : value;
      if (wx.humidityPct > 100) {
        wx.humidityPct = 100;
      }
    } else if (tag == 'b') {
      wx.hasPressure = true;
      wx.pressureHpa = value / 10.0f;
    }
  }

  wx.hasAny = wx.hasWindDir || wx.hasWindSpeed || wx.hasTempC || wx.hasHumidity || wx.hasPressure;
  return wx.hasAny;
}

static bool isAprsWxPayloadValidForAlert(const APRSStation &station) {
  if (!isAprsWeatherStationSymbol(station)) {
    return false;
  }

  AprsWxDecoded wx;
  if (!decodeAprsWxFromComment(station.comment, wx)) {
    return false;
  }

  // Minimalny warunek poprawnej ramki WX do alertu: temperatura + wilgotnosc.
  return wx.hasTempC && wx.hasHumidity;
}

static void drawAprsAlertWrappedComment(const String &comment, int x, int y, int widthPx, int maxLines, int textSize) {
  String text = comment;
  text.trim();
  if (text.length() == 0) {
    text = "-";
  }

  const int safeTextSize = (textSize < 1) ? 1 : textSize;
  const int charWidth = 6 * safeTextSize;
  const int lineHeight = (8 * safeTextSize) + 2;
  const int maxCharsPerLine = (charWidth > 0) ? (widthPx / charWidth) : widthPx;
  int cursor = 0;
  int line = 0;

  while (cursor < text.length() && line < maxLines) {
    int end = cursor + maxCharsPerLine;
    if (end >= text.length()) {
      end = text.length();
    } else {
      int split = end;
      while (split > cursor && text.charAt(split) != ' ') {
        split--;
      }
      if (split > cursor) {
        end = split;
      }
    }

    String lineText = text.substring(cursor, end);
    lineText.trim();
    if (lineText.length() > 0) {
      tft.setCursor(x, y + line * lineHeight);
      tft.print(lineText);
      line++;
    }

    cursor = end;
    while (cursor < text.length() && text.charAt(cursor) == ' ') {
      cursor++;
    }
  }
}

static void drawAprsAlertCarIcon(int centerX, int topY, uint16_t color) {
  const int bodyW = 60;
  const int bodyH = 14;
  const int roofW = 33;
  const int roofH = 9;

  const int bodyX = centerX - (bodyW / 2);
  const int roofX = centerX - (roofW / 2);
  const int roofY = topY;
  const int bodyY = topY + 9;

  tft.fillRoundRect(bodyX, bodyY, bodyW, bodyH, 2, color);
  tft.fillRoundRect(roofX, roofY, roofW, roofH, 2, color);

  tft.fillCircle(bodyX + 13, bodyY + bodyH, 6, TFT_DARKGREY);
  tft.fillCircle(bodyX + bodyW - 13, bodyY + bodyH, 6, TFT_DARKGREY);
  tft.fillCircle(bodyX + 13, bodyY + bodyH, 3, TFT_LIGHTGREY);
  tft.fillCircle(bodyX + bodyW - 13, bodyY + bodyH, 3, TFT_LIGHTGREY);
  tft.fillCircle(bodyX + 3, bodyY + 3, 2, TFT_YELLOW);
  tft.fillCircle(bodyX + 3, bodyY + 8, 2, TFT_YELLOW);
}

static void drawAprsAlertHouseIcon(int centerX, int topY) {
  const int bodyW = 39;
  const int bodyH = 24;
  const int bodyX = centerX - (bodyW / 2);
  const int bodyY = topY + 10;

  // Ściany domu (żółte)
  tft.fillRect(bodyX, bodyY, bodyW, bodyH, TFT_YELLOW);

  // Dach dwuspadowy (brązowy)
  const uint16_t roofColor = TFT_GREEN;
  int roofTopY = topY;
  tft.fillTriangle(centerX, roofTopY, bodyX - 3, bodyY + 1, bodyX + bodyW + 3, bodyY + 1, roofColor);

  // Drzwi i okna dla czytelności
  tft.fillRect(centerX - 4, bodyY + 12, 8, 12, TFT_DARKGREY);
  tft.fillRect(bodyX + 4, bodyY + 6, 6, 6, TFT_LIGHTGREY);
  tft.fillRect(bodyX + bodyW - 10, bodyY + 6, 6, 6, TFT_LIGHTGREY);
}

static void drawAprsAlertHumanIcon(int centerX, int topY) {
  const uint16_t bodyColor = TFT_RED;
  const uint16_t headColor = TFT_BLUE;
  int headX = centerX;
  int headY = topY + 5;

  // Głowa
  tft.fillCircle(headX, headY, 4, headColor);

  // Tułów (lekko pochylony)
  tft.drawLine(headX, headY + 4, headX + 2, headY + 18, bodyColor);
  tft.drawLine(headX + 1, headY + 4, headX + 3, headY + 18, bodyColor);
  tft.drawLine(headX + 2, headY + 4, headX + 4, headY + 18, bodyColor);
  tft.drawLine(headX + 3, headY + 4, headX + 5, headY + 18, bodyColor);

  // Ręce (ruch spaceru)
  tft.drawLine(headX + 1, headY + 8, headX - 6, headY + 12, bodyColor);
  tft.drawLine(headX + 3, headY + 8, headX + 10, headY + 11, bodyColor);
  tft.drawLine(headX + 2, headY + 9, headX - 5, headY + 13, bodyColor);
  tft.drawLine(headX + 4, headY + 9, headX + 11, headY + 12, bodyColor);
  tft.drawLine(headX + 3, headY + 10, headX - 4, headY + 14, bodyColor);
  tft.drawLine(headX + 5, headY + 10, headX + 12, headY + 13, bodyColor);

  // Nogi (jedna do przodu, druga do tyłu)
  tft.drawLine(headX + 3, headY + 18, headX - 4, headY + 28, bodyColor);
  tft.drawLine(headX + 3, headY + 18, headX + 12, headY + 27, bodyColor);
  tft.drawLine(headX + 4, headY + 18, headX - 3, headY + 28, bodyColor);
  tft.drawLine(headX + 4, headY + 18, headX + 13, headY + 27, bodyColor);
  tft.drawLine(headX + 5, headY + 18, headX - 2, headY + 28, bodyColor);
  tft.drawLine(headX + 5, headY + 18, headX + 14, headY + 27, bodyColor);
}

static void drawAprsAlertCompass(int centerX, int centerY, int radius, bool hasBearing, float bearingDeg) {
  tft.drawCircle(centerX, centerY, radius, TFT_DARKGREY);
  tft.drawCircle(centerX, centerY, radius - 1, TFT_DARKGREY);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(centerX - 3, centerY - radius - 9);
  tft.print("N");

  tft.drawFastVLine(centerX, centerY - radius + 2, 4, TFT_DARKGREY);
  tft.drawFastVLine(centerX, centerY + radius - 5, 4, TFT_DARKGREY);
  tft.drawFastHLine(centerX - radius + 2, centerY, 4, TFT_DARKGREY);
  tft.drawFastHLine(centerX + radius - 5, centerY, 4, TFT_DARKGREY);

  if (!hasBearing) {
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(centerX - 6, centerY + radius + 2);
    tft.print("---");
    return;
  }

  float angleRad = bearingDeg * M_PI / 180.0f;
  int tipX = centerX + (int)(sin(angleRad) * (radius - 3));
  int tipY = centerY - (int)(cos(angleRad) * (radius - 3));

  int perpX = (int)round(cos(angleRad));
  int perpY = (int)round(sin(angleRad));
  tft.drawLine(centerX, centerY, tipX, tipY, TFT_CYAN);
  tft.drawLine(centerX + perpX, centerY + perpY, tipX + perpX, tipY + perpY, TFT_CYAN);
  tft.fillCircle(tipX, tipY, 2, TFT_CYAN);
  tft.fillCircle(centerX, centerY, 1, TFT_CYAN);

  String bearingTxt = String((int)(bearingDeg + 0.5f)) + "d";
  int txtX = centerX - (bearingTxt.length() * 3);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(txtX, centerY + radius + 2);
  tft.print(bearingTxt);
}

static void drawAprsAlertFrame(bool pulseOn) {
  const int frameX = 10;
  const int frameY = 50;
  const int frameW = 300;
  const int frameH = 185;
  const int frameR = 8;
  uint16_t frameColor = pulseOn ? TFT_WHITE : TFT_LIGHTGREY;

  tft.drawRoundRect(frameX - 3, frameY - 3, frameW + 6, frameH + 6, frameR + 3, TFT_BLACK);
  tft.drawRoundRect(frameX - 1, frameY - 1, frameW + 2, frameH + 2, frameR + 1, TFT_BLACK);
  tft.drawRoundRect(frameX - 2, frameY - 2, frameW + 4, frameH + 4, frameR + 2, TFT_BLACK);
  tft.drawRoundRect(frameX, frameY, frameW, frameH, frameR, TFT_BLACK);
  tft.drawRoundRect(frameX, frameY, frameW, frameH, frameR, frameColor);
  if (pulseOn) {
    tft.drawRoundRect(frameX - 3, frameY - 3, frameW + 6, frameH + 6, frameR + 3, frameColor);
    tft.drawRoundRect(frameX - 1, frameY - 1, frameW + 2, frameH + 2, frameR + 1, frameColor);
    tft.drawRoundRect(frameX - 2, frameY - 2, frameW + 4, frameH + 4, frameR + 2, frameColor);
  }
}

static void drawAprsAlertCloseButton() {
  tft.fillRoundRect(APRS_ALERT_CLOSE_BTN_X, APRS_ALERT_CLOSE_BTN_Y, APRS_ALERT_CLOSE_BTN_W, APRS_ALERT_CLOSE_BTN_H, 5, TFT_RADIO_ORANGE);
  tft.drawRoundRect(APRS_ALERT_CLOSE_BTN_X, APRS_ALERT_CLOSE_BTN_Y, APRS_ALERT_CLOSE_BTN_W, APRS_ALERT_CLOSE_BTN_H, 5, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + 8, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 7, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + 8,
               APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 7, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 7, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + 8, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + 8,
               APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 7, TFT_BLACK);
}

static bool isAprsAlertCloseButtonHit(uint16_t x, uint16_t y) {
  int hitX = APRS_ALERT_CLOSE_BTN_X + ((APRS_ALERT_CLOSE_BTN_W - APRS_ALERT_CLOSE_HIT_W) / 2);
  int hitY = APRS_ALERT_CLOSE_BTN_Y + ((APRS_ALERT_CLOSE_BTN_H - APRS_ALERT_CLOSE_HIT_H) / 2);
  if (hitX < 0) hitX = 0;
  if (hitY < 0) hitY = 0;

  int hitRight = hitX + APRS_ALERT_CLOSE_HIT_W;
  int hitBottom = hitY + APRS_ALERT_CLOSE_HIT_H;
  if (hitRight > 320) hitRight = 320;
  if (hitBottom > 240) hitBottom = 240;

  return x >= hitX && x < hitRight && y >= hitY && y < hitBottom;
}

static void dismissAprsAlertScreen() {
  if (!aprsAlertScreenActive) {
    return;
  }
  aprsAlertScreenActive = false;
  activeAlertScreenKind = ALERT_SCREEN_NONE;
  drawScreen(currentScreen);
  if (currentScreen == SCREEN_HAM_CLOCK) {
    updateScreen1();
  }
}

static void setRgbLedChannel(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

static void applyRgbLedState(bool redOn, bool greenOn, bool blueOn) {
  setRgbLedChannel(RGB_LED_RED_PIN, redOn);
  setRgbLedChannel(RGB_LED_GREEN_PIN, greenOn);
  setRgbLedChannel(RGB_LED_BLUE_PIN, blueOn);
}

static void initStatusRgbLed() {
  pinMode(RGB_LED_RED_PIN, OUTPUT);
  pinMode(RGB_LED_GREEN_PIN, OUTPUT);
  pinMode(RGB_LED_BLUE_PIN, OUTPUT);

  rgbLedPrevWifiConnected = false;
  rgbRedBlinkLastToggleMs = millis();
  rgbRedBlinkStateOn = false;
  rgbBlueAprsAlertActive = false;
  rgbBlueAprsAlertUntilMs = 0;
  rgbBlueAprsLastToggleMs = 0;
  rgbBlueAprsStateOn = false;

  applyRgbLedState(false, false, false);
}


static int normalizeLedAlertDurationMs(int durationMs) {
  if (durationMs < 100) return 100;
  if (durationMs > 60000) return 60000;
  return durationMs;
}

static void applyLedAlertDurationMs(int durationMs) {
  ledAlertDurationMs = normalizeLedAlertDurationMs(durationMs);
}

static int normalizeLedAlertBlinkMs(int blinkMs) {
  if (blinkMs < 50) return 50;
  if (blinkMs > 5000) return 5000;
  return blinkMs;
}

static void applyLedAlertBlinkMs(int blinkMs) {
  ledAlertBlinkMs = normalizeLedAlertBlinkMs(blinkMs);
}

static void triggerAprsRgbLedAlert() {
  if (!enableLedAlert) {
    return;
  }

  unsigned long nowMs = millis();
  rgbBlueAprsAlertActive = true;
  rgbBlueAprsAlertUntilMs = nowMs + (unsigned long)ledAlertDurationMs;
  rgbBlueAprsLastToggleMs = nowMs;
  rgbBlueAprsStateOn = true;
  applyRgbLedState(false, false, true);
}

static void updateStatusRgbLed() {
  unsigned long nowMs = millis();

  if (rgbBlueAprsAlertActive) {
    if ((long)(nowMs - rgbBlueAprsAlertUntilMs) >= 0) {
      rgbBlueAprsAlertActive = false;
      rgbBlueAprsStateOn = false;
    } else if (nowMs - rgbBlueAprsLastToggleMs >= (unsigned long)ledAlertBlinkMs) {
      rgbBlueAprsLastToggleMs = nowMs;
      rgbBlueAprsStateOn = !rgbBlueAprsStateOn;
    }

    if (rgbBlueAprsAlertActive) {
      applyRgbLedState(false, false, rgbBlueAprsStateOn);
      return;
    }
  }

  applyRgbLedState(false, false, false);
}

void ALERT_Screen(const APRSStation &station) {
  if (!tftInitialized) {
    return;
  }

  aprsAlertScreenStation = station;
  aprsAlertScreenActive = true;
  activeAlertScreenKind = ALERT_SCREEN_APRS;
  aprsAlertScreenUntilMs = millis() + ((unsigned long)aprsAlertScreenSeconds * 1000UL);
  aprsAlertFrameLastToggleMs = millis();
  aprsAlertFramePulseOn = true;

  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 40, TFT_RADIO_ORANGE);
  tft.drawFastHLine(0, 39, 320, 0x8410);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  const String headerText = "APRS ALERT";
  const int headerX = (320 - ((int)headerText.length() * 12)) / 2;
  tft.setCursor(headerX, 12);
  tft.print(headerText);
  tft.setCursor(headerX + 1, 12);
  tft.print(headerText);
  drawAprsAlertCloseButton();

  drawAprsAlertFrame(aprsAlertFramePulseOn);

  String callsign = station.callsign;
  callsign.toUpperCase();
  if (callsign.length() == 0) {
    callsign = "-";
  }

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(4);
  int callX = (320 - (callsign.length() * 24)) / 2;
  if (callX < 15) {
    callX = 15;
  }
  callX += 20;
  tft.setCursor(callX, 65);
  tft.print(callsign);

  String freqText = "-";
  if (station.freqMHz > 0.0f) {
    freqText = String(station.freqMHz, 3) + " MHz";
  }
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(2);
  int freqX = (320 - (freqText.length() * 12)) / 2;
  freqX += 20;
  if (freqX < 12) {
    freqX = 12;
  }
  tft.setCursor(freqX, 106);
  tft.print(freqText);

  String distanceText = "- km";
  if (station.hasLatLon && station.distance > 0.0f) {
    distanceText = String(station.distance, 1) + " km";
  }
  const int compassX = 46;
  const int compassY = 91;
  const int compassR = 24;
  bool hasBearing = false;
  float bearingDeg = 0.0f;
  double userLatForBearing = 0.0;
  double userLonForBearing = 0.0;
  if (station.hasLatLon && getServiceCoordinatesForServices(userLatForBearing, userLonForBearing)) {
    hasBearing = true;
    bearingDeg = calculateBearing(userLatForBearing, userLonForBearing, (double)station.lat, (double)station.lon);
  }
  drawAprsAlertCompass(compassX, compassY, compassR, hasBearing, bearingDeg);

  tft.setTextColor(TFT_RADIO_ORANGE);
  tft.setTextSize(2);
  int distX = compassX - ((int)distanceText.length() * 12) / 2;
  distX += 22;
  if (distX < 12) {
    distX = 12;
  }
  tft.setCursor(distX, compassY + compassR + 18);
  tft.print(distanceText);

  bool weatherAlert = isAprsWeatherStationSymbol(station);
  String iconDescription = getAprsAlertSymbolDescription(station);
  String iconShort = getAPRSSymbolShort(station);
  uint16_t iconColor = TFT_WHITE;
  if (iconShort == "CAR") {
    iconColor = TFT_RED;
  } else if (iconShort == "HOUSE") {
    iconColor = TFT_YELLOW;
  } else if (iconShort == "HUMAN") {
    iconColor = TFT_BLUE;
  }
  if (weatherAlert) {
    const String weatherTitle = LANG3("Pogoda:", "Weather:", "Clima:");
    tft.setTextColor(TFT_CYAN);
    tft.setTextSize(2);
    int weatherTitleX = (320 - ((int)weatherTitle.length() * 12)) / 2;
    if (weatherTitleX < 15) {
      weatherTitleX = 15;
    }
    weatherTitleX += 50;
    if (weatherTitleX > 300) {
      weatherTitleX = 300;
    }
    tft.setCursor(weatherTitleX, 133);
    tft.print(weatherTitle);

    AprsWxDecoded wx;
    decodeAprsWxFromComment(station.comment, wx);

    const String labelTemp = "TEMP";
    const String labelHumidity = LANG3("WILGOTNOŚĆ", "HUMIDITY", "UMIDADE");
    const String labelPressure = LANG3("CISNIENIE", "PRESSURE", "PRESSAO");
    const String labelWind = (tftLanguage == TFT_LANG_PT) ? "VENTO" : "WIATR";
    const String labelTempWithColon = labelTemp + ":";
    const String labelHumidityWithColon = labelHumidity + ":";
    const String labelPressureWithColon = labelPressure + ":";
    const String labelWindWithColon = labelWind + ":";

    String tempValue = wx.hasTempC ? (String(wx.tempC, 1) + " C") : "--";
    String humidityValue = wx.hasHumidity ? (String(wx.humidityPct) + "%") : "--";
    String pressureValue = wx.hasPressure ? (String(wx.pressureHpa, 1) + " hPa") : "--";
    String windValue = "--";
    if (wx.hasWindSpeed && wx.hasWindDir) {
      windValue = String(wx.windDirDeg) + " deg " + String(wx.windSpeedKmh, 1) + " km/h";
    } else if (wx.hasWindSpeed) {
      windValue = String(wx.windSpeedKmh, 1) + " km/h";
    } else if (wx.hasWindDir) {
      windValue = String(wx.windDirDeg) + " deg";
    }

    const int leftColX = 20;
    const int rightColX = 170;
    const int topRowY = 184;
    const int bottomRowY = 212;
    const int weatherTopRowY = topRowY - 20;
    const int weatherLabelRowY = weatherTopRowY + 5;
    const int humidityLabelX = rightColX - 20;
    const int tempValueShiftX = 20;
    const int humidityValueShiftX = 20;

    int tempLabelW = (int)labelTempWithColon.length() * 6;
    int humidityLabelW = (int)labelHumidityWithColon.length() * 6;
    if (tryLoadSmoothFont(ROBOTO_FONT12_NAME, ROBOTO_FONT12_FILE)) {
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(TFT_WHITE);
      tft.drawString(labelTempWithColon, leftColX, weatherLabelRowY);
      tempLabelW = tft.textWidth(labelTempWithColon);
      tft.drawString(labelHumidityWithColon, humidityLabelX, weatherLabelRowY);
      humidityLabelW = tft.textWidth(labelHumidityWithColon);
      tft.unloadFont();

      // Wartosc wieksza (+1) obok etykiety.
      tft.setTextSize(2);
      tft.setTextColor(TFT_BLUE);
      tft.setCursor(leftColX + tempLabelW + 2 + tempValueShiftX, weatherTopRowY + 2);
      tft.print(tempValue);

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(humidityLabelX + humidityLabelW + 2 + humidityValueShiftX, weatherTopRowY + 2);
      tft.print(humidityValue);
    } else {
      // Fallback gdy SmoothFont nie jest dostepny.
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.setCursor(leftColX, weatherLabelRowY);
      tft.print(labelTempWithColon);
      tft.setCursor(humidityLabelX, weatherLabelRowY);
      tft.print(labelHumidityWithColon);

      tft.setTextSize(2);
      tft.setTextColor(TFT_BLUE);
      tft.setCursor(leftColX + tempLabelW + 2 + tempValueShiftX, weatherTopRowY + 2);
      tft.print(tempValue);

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(humidityLabelX + humidityLabelW + 2 + humidityValueShiftX, weatherTopRowY + 2);
      tft.print(humidityValue);
    }

    tft.setTextSize(1);

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(leftColX, bottomRowY);
    tft.print(labelPressureWithColon);
    tft.print(" ");
    tft.setTextColor(TFT_WHITE);
    tft.print(pressureValue);

    tft.setTextColor(TFT_WHITE);
    tft.setCursor(rightColX, bottomRowY);
    tft.print(labelWindWithColon);
    tft.print(" ");
    tft.setTextColor(TFT_GREEN);
    tft.print(windValue);
  } else {
    tft.setTextColor(iconColor);
    tft.setTextSize(iconDescription.length() > 26 ? 1 : 2);
    int iconCharW = (iconDescription.length() > 26) ? 6 : 12;
    int iconX = (320 - (iconDescription.length() * iconCharW)) / 2;
    if (iconX < 15) {
      iconX = 15;
    }
    if (iconShort == "CAR") {
      drawAprsAlertCarIcon(160, 140, iconColor);
    } else if (iconShort == "HOUSE") {
      drawAprsAlertHouseIcon(160, 136);
    } else if (iconShort == "HUMAN") {
      drawAprsAlertHumanIcon(160, 136);
    } else {
      tft.setCursor(iconX, 153);
      tft.print(iconDescription);
    }

    tft.setTextColor(TFT_CYAN);
    tft.setTextSize(1);
    tft.setCursor(20, 172);
    tft.print("Comment:");
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setTextSize(2);
    drawAprsAlertWrappedComment(station.comment, 20, 184, 280, 2, 2);
  }
}

void APRS_BEACONING_Screen(const String &txCallsign, const String &positionInfo, const String &pathInfo, const String &frameInfo) {
  if (!tftInitialized || brightnessMenuActive) {
    return;
  }

  aprsAlertScreenActive = true;
  activeAlertScreenKind = ALERT_SCREEN_BEACONING;
  aprsAlertScreenUntilMs = millis() + APRS_BEACONING_SCREEN_MS;
  aprsAlertFrameLastToggleMs = millis();
  aprsAlertFramePulseOn = true;

  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 40, TFT_RADIO_ORANGE);
  tft.drawFastHLine(0, 39, 320, 0x8410);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  const String headerText = "APRS BECONING...";
  int headerX = (320 - ((int)headerText.length() * 12)) / 2;
  if (headerX < 4) {
    headerX = 4;
  }
  tft.setCursor(headerX, 12);
  tft.print(headerText);
  tft.setCursor(headerX + 1, 12);
  tft.print(headerText);
  drawAprsAlertCloseButton();

  drawAprsAlertFrame(aprsAlertFramePulseOn);

  String callsign = txCallsign;
  callsign.toUpperCase();
  if (callsign.length() == 0) {
    callsign = "-";
  }

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  int callX = (320 - ((int)callsign.length() * 18)) / 2;
  if (callX < 14) {
    callX = 14;
  }
  tft.setCursor(callX, 62);
  tft.print(callsign);

  tft.setTextColor(TFT_CYAN);
  tft.setTextSize(2);
  int posX = (320 - ((int)positionInfo.length() * 12)) / 2;
  if (posX < 12) {
    posX = 12;
  }
  tft.setCursor(posX, 96);
  tft.print(positionInfo);

  String pathLine = "PATH: " + pathInfo;
  tft.setTextColor(TFT_RADIO_ORANGE);
  tft.setTextSize(2);
  int pathX = (320 - ((int)pathLine.length() * 12)) / 2;
  if (pathX < 12) {
    pathX = 12;
  }
  tft.setCursor(pathX, 122);
  tft.print(pathLine);

  tft.setTextColor(TFT_CYAN);
  tft.setTextSize(1);
  tft.setCursor(20, 148);
  tft.print("Frame:");
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  drawAprsAlertWrappedComment(frameInfo, 20, 160, 280, 6, 1);
}

void WEATHER_ALERT_Screen(const WeatherAlertInfo &info) {
  if (!tftInitialized) {
    return;
  }

  weatherAlertScreenInfo = info;
  aprsAlertScreenActive = true;
  activeAlertScreenKind = ALERT_SCREEN_WEATHER;
  aprsAlertScreenUntilMs = millis() + ((unsigned long)aprsAlertScreenSeconds * 1000UL);
  aprsAlertFrameLastToggleMs = millis();
  aprsAlertFramePulseOn = true;

  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 40, TFT_RADIO_ORANGE);
  tft.drawFastHLine(0, 39, 320, 0x8410);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  String headerText = LANG3("ALERT POGODOWY", "WEATHER ALERT", "ALERTA DE CLIMA");
  int headerX = (320 - ((int)headerText.length() * 12)) / 2;
  if (headerX < 6) {
    headerX = 6;
  }
  tft.setCursor(headerX, 12);
  tft.print(headerText);
  tft.setCursor(headerX + 1, 12);
  tft.print(headerText);
  drawAprsAlertCloseButton();

  drawAprsAlertFrame(aprsAlertFramePulseOn);

  const String iconPath = info.stormPossible ? "/icon50/200.bmp" : "/icon50/wind.bmp";
  drawBmpFromFS(iconPath, 18, 78);

  int line2Y = 104;
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  if (info.stormPossible) {
    String stormLine1 = LANG3("MOZLIWA", "POSSIBLE", "POSSIVEL");
    String stormLine2 = LANG3("BURZA", "STORM", "TEMPESTADE");

    int stormX1 = (320 - ((int)stormLine1.length() * 18)) / 2;
    int stormX2 = (320 - ((int)stormLine2.length() * 18)) / 2;
    if (stormX1 < 72) {
      stormX1 = 72;
    }
    if (stormX2 < 72) {
      stormX2 = 72;
    }
    tft.setCursor(stormX1, 62);
    tft.print(stormLine1);
    tft.setCursor(stormX2, 90);
    tft.print(stormLine2);
    line2Y = 120;
  } else {
    String mainLine = LANG3("SILNY WIATR", "STRONG WIND", "VENTO FORTE");
    int mainX = (320 - ((int)mainLine.length() * 18)) / 2;
    if (mainX < 72) {
      mainX = 72;
    }
    tft.setCursor(mainX, 68);
    tft.print(mainLine);
  }

  String line2;
  if (info.stormPossible) {
    if (info.stormNow) {
      line2 = LANG3("BURZA TERAZ", "STORM NOW", "TEMPESTADE AGORA");
    } else {
      line2 = (tftLanguage == TFT_LANG_PL)
                ? ("ETA<= " + String(info.stormEtaHours) + "godz")
                : ("ETA<=" + String(info.stormEtaHours) + " h");
    }
  } else {
    line2 = LANG3("Prognoza <= 6h", "Forecast <= 6h", "Previsao <= 6h");
  }
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(2);
  int line2X = (320 - ((int)line2.length() * 12)) / 2;
  if (line2X < 72) {
    line2X = 72;
  }
  tft.setCursor(line2X, line2Y);
  tft.print(line2);

  int detailY = info.stormPossible ? 156 : 142;
  tft.setTextSize(2);
  if (info.stormPossible) {
    uint8_t repeatMin = info.stormEtaHours;
    if (repeatMin < 1) repeatMin = 1;
    if (repeatMin > 6) repeatMin = 6;
    String repeatLine = String(LANG3("Powtorz: ", "Repeat: ", "Repetir: ")) + String(repeatMin) + " min";
    tft.setTextColor(TFT_CYAN);
    tft.setCursor(74, detailY);
    tft.print(repeatLine);
    detailY += 24;
  }

  if (info.windPossible) {
    String windLabel = LANG3("Wiatr: ", "Wind: ", "Vento: ");
    String gustLabel = LANG3("MAX: ", "Gust: ", "Rajada: ");
    String windLine = windLabel + String(info.maxWindMs, 1) + " m/s";
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(74, detailY);
    tft.print(windLine);
    detailY += 22;

    if (info.maxWindGustMs > 0.0f) {
      String gustLine = gustLabel + String(info.maxWindGustMs, 1) + " m/s";
      tft.setCursor(74, detailY);
      tft.print(gustLine);
      detailY += 22;
    }
  }

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  String footer = LANG3("Na podstawie prognozy OpenWeather",
                        "Based on OpenWeather forecast",
                        "Baseado na previsao do OpenWeather");
  tft.setCursor(20, 220);
  tft.print(footer);
}

void updateAlertScreenTimeout() {
  if (!aprsAlertScreenActive) {
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - aprsAlertFrameLastToggleMs >= APRS_ALERT_FRAME_PULSE_MS) {
    aprsAlertFrameLastToggleMs = nowMs;
    aprsAlertFramePulseOn = !aprsAlertFramePulseOn;
    drawAprsAlertFrame(aprsAlertFramePulseOn);
  }

  if ((long)(millis() - aprsAlertScreenUntilMs) < 0) {
    return;
  }

  dismissAprsAlertScreen();
}

// WyciÄ…gnij czÄ™stotliwoĹ›Ä‡ z komentarza APRS (MHz)
bool extractAPRSFrequencyMHz(const String &text, float &outMHz) {
  const int len = text.length();
  for (int i = 0; i < len; i++) {
    char c = text.charAt(i);
    if (isDigit(c) && (i == 0 || !isDigit(text.charAt(i - 1)))) {
      int j = i;
      bool hasDot = false;
      while (j < len) {
        char cj = text.charAt(j);
        if (cj == '.') {
          if (hasDot) break;
          hasDot = true;
          j++;
          continue;
        }
        if (!isDigit(cj)) break;
        j++;
      }
      if (j <= i) continue;

      String numStr = text.substring(i, j);
      float value = numStr.toFloat();
      if (value <= 0.0f) continue;

      int k = j;
      while (k < len && (text.charAt(k) == ' ' || text.charAt(k) == '\t')) {
        k++;
      }
      if (k + 2 < len) {
        char m1 = text.charAt(k);
        char m2 = text.charAt(k + 1);
        char m3 = text.charAt(k + 2);
        if ((m1 == 'M' || m1 == 'm') && (m2 == 'H' || m2 == 'h') && (m3 == 'Z' || m3 == 'z')) {
          outMHz = value;
          return true;
        }
      }

      // Bez jednostki: akceptuj tylko liczby dziesiÄ™tne w paĹ›mie VHF/UHF
      if (hasDot && value >= 30.0f && value <= 1000.0f) {
        outMHz = value;
        return true;
      }
    }
  }
  return false;
}

// Konwersja Maidenhead Locator do wspĂłĹ‚rzÄ™dnych geograficznych
void locatorToLatLon(String locator, double &lat, double &lon) {
  if (locator.length() < 4) {
    lat = 0;
    lon = 0;
    return;
  }
  
  // Pierwsze 2 znaki - kwadrat (field)
  char c1 = toupper(locator.charAt(0));
  char c2 = toupper(locator.charAt(1));
  lon = (c1 - 'A') * 20 - 180;
  lat = (c2 - 'A') * 10 - 90;
  
  // NastÄ™pne 2 znaki - kwadrat (square)
  if (locator.length() >= 4) {
    int n1 = locator.charAt(2) - '0';
    int n2 = locator.charAt(3) - '0';
    lon += n1 * 2;
    lat += n2 * 1;
  }
  
  // Ĺšrodek kwadratu
  lon += 1;
  lat += 0.5;
}

void updateUserLatLonFromLocator() {
  if (userLocator.length() >= 4) {
    locatorToLatLon(userLocator, userLat, userLon);
    userLatLonValid = true;
  } else {
    userLat = 0.0;
    userLon = 0.0;
    userLatLonValid = false;
  }
}

// SprawdĹş czy tekst wspĂłĹ‚rzÄ™dnych ma tylko cyfry i kropkÄ™ (nie obsĹ‚ugujemy formatĂłw skompresowanych)
static bool isAprsNumeric(const String &raw_aprs) {
    if (raw_aprs.length() == 0) {
        return false;
    }
    int dotCount = 0;
    for (size_t i = 0; i < raw_aprs.length(); i++) {
        char c = raw_aprs.charAt(i);
        if (c == '.') {
            dotCount++;
            if (dotCount > 1) {
                return false;
            }
        } else if (!isDigit(c)) {
            return false;
        }
    }
    return true;
}

// Konwersja surowej pozycji APRS na stopnie dziesiÄ™tne
// Konwersja formatu APRS (DDMM.mmN lub DDDMM.mmE) na stopnie dziesiÄ™tne
// Format APRS: DDMM.mm dla szerokoĹ›ci, DDDMM.mm dla dĹ‚ugoĹ›ci
float convertToDecimal(String raw_aprs, char direction) {
    if (raw_aprs.length() < 4) {
        LOGV_PRINTF("[APRS] convertToDecimal: za krĂłtki string: %s\n", raw_aprs.c_str());
        return NAN;
    }
    if (!isAprsNumeric(raw_aprs)) {
        LOGV_PRINTF("[APRS] convertToDecimal: nie-numeryczny format: %s\n", raw_aprs.c_str());
        return NAN;
    }

    double degrees = 0.0;
    double minutes = 0.0;

    if (direction == 'N' || direction == 'S') {
        // SzerokoĹ›Ä‡ (Latitude): pierwsze 2 znaki to stopnie (DDMM.mm)
        // PrzykĹ‚ad: "5202.40" -> degrees=52, minutes=02.40
        if (raw_aprs.length() >= 2) {
            String degStr = raw_aprs.substring(0, 2);
            degrees = degStr.toDouble();
        }
        if (raw_aprs.length() > 2) {
            String minStr = raw_aprs.substring(2);
            minutes = minStr.toDouble();
        }
    } else if (direction == 'E' || direction == 'W') {
        // DĹ‚ugoĹ›Ä‡ (Longitude): pierwsze 3 znaki to stopnie (DDDMM.mm)
        // PrzykĹ‚ad: "01655.12" -> degrees=016, minutes=55.12
        if (raw_aprs.length() >= 3) {
            String degStr = raw_aprs.substring(0, 3);
            degrees = degStr.toDouble();
        }
        if (raw_aprs.length() > 3) {
            String minStr = raw_aprs.substring(3);
            minutes = minStr.toDouble();
        }
    } else {
        LOGV_PRINTF("[APRS] convertToDecimal: nieznany kierunek: %c\n", direction);
        return NAN;
    }

    // Walidacja zakresĂłw
    if (direction == 'N' || direction == 'S') {
        if (degrees < 0 || degrees > 90 || minutes < 0 || minutes >= 60) {
            LOGV_PRINTF("[APRS] convertToDecimal: nieprawidĹ‚owa szerokoĹ›Ä‡: %s%c (deg=%.1f, min=%.2f)\n", 
                       raw_aprs.c_str(), direction, degrees, minutes);
            return NAN;
        }
    } else {
        if (degrees < 0 || degrees > 180 || minutes < 0 || minutes >= 60) {
            LOGV_PRINTF("[APRS] convertToDecimal: nieprawidĹ‚owa dĹ‚ugoĹ›Ä‡: %s%c (deg=%.1f, min=%.2f)\n", 
                       raw_aprs.c_str(), direction, degrees, minutes);
            return NAN;
        }
    }

    double decimal = degrees + (minutes / 60.0);

    // JeĹ›li PoĹ‚udnie (S) lub ZachĂłd (W), wynik musi byÄ‡ ujemny
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    LOGV_PRINTF("[APRS] convertToDecimal: %s%c -> %.6f (deg=%.0f, min=%.2f)\n", 
               raw_aprs.c_str(), direction, decimal, degrees, minutes);

    return (float)decimal;
}

// ===== APRS Mic-E / Compressed / Uncompressed decoding (ported from ESP32APRS_Audio) =====
static bool validSymTableCompressed(char c) {
    return (c == '/' || c == '\\' || (c >= 0x41 && c <= 0x5A) || (c >= 0x61 && c <= 0x6A)); // [\/\\A-Za-j]
}

static bool validSymTableUncompressed(char c) {
    return (c == '/' || c == '\\' || (c >= 0x41 && c <= 0x5A) || (c >= 0x30 && c <= 0x39)); // [\/\\A-Z0-9]
}

static bool parseFixedUnsigned(const char *src, int offset, int digits, unsigned int &out) {
  out = 0;
  for (int i = 0; i < digits; i++) {
    char c = src[offset + i];
    if (c < '0' || c > '9') {
      return false;
    }
    out = out * 10U + (unsigned int)(c - '0');
  }
  return true;
}

static int aprsMicECommentBodyOffset(const unsigned char *body, size_t len) {
  const size_t kMicEFixedLen = 8; // lon(3) + spd/crs(3) + sym(2)
  if (!body || len <= kMicEFixedLen) {
    return (int)kMicEFixedLen;
  }

  size_t idx = kMicEFixedLen;
  unsigned char marker = body[idx];

  // Optional Mic-E telemetry may start with '`', '\'', 0x1C or 0x1D.
  if (marker == 0x60 || marker == 0x27 || marker == 0x1C || marker == 0x1D) {
    size_t probe = idx + 1;
    size_t telemetryLen = 0;
    while (probe < len && telemetryLen < 10) {
      unsigned char c = body[probe];
      if (c < 0x21 || c > 0x7B) {
        break;
      }
      telemetryLen++;
      probe++;
    }

    // Skip telemetry only when it looks like a valid telemetry block.
    if (telemetryLen >= 2) {
      idx = probe;
    }
  }

  return (int)idx;
}

  static bool aprsLatLonInRange(float lat, float lon) {
    if (isnan(lat) || isnan(lon)) return false;
    if (lat < -90.0f || lat > 90.0f) return false;
    if (lon < -180.0f || lon > 180.0f) return false;
    // APRS parser conventions: 0,0 is treated as invalid/no position.
    if (fabsf(lat) < 0.0001f && fabsf(lon) < 0.0001f) return false;
    return true;
  }

  static String normalizeAprsParsedComment(const String &rawComment) {
    String out;
    out.reserve(rawComment.length());
    for (size_t i = 0; i < rawComment.length(); i++) {
      char c = rawComment.charAt(i);
      if (c == '\r' || c == '\n') {
        out += ' ';
        continue;
      }
      if ((unsigned char)c >= 32 || c == '\t') {
        out += c;
      }
    }
    out.trim();
    return out;
  }

static bool parseAprsUncompressed(const char *body, size_t len, float &lat, float &lon, char &symTable, char &symCode) {
    if (len < 19) {
        return false;
    }
    char posbuf[20];
    memcpy(posbuf, body, 19);
    posbuf[19] = 0;

    // Position ambiguity handling
    if (posbuf[2] == ' ') posbuf[2] = '3';
    if (posbuf[3] == ' ') posbuf[3] = '5';
    if (posbuf[5] == ' ') posbuf[5] = '5';
    if (posbuf[6] == ' ') posbuf[6] = '5';
    if (posbuf[12] == ' ') posbuf[12] = '3';
    if (posbuf[13] == ' ') posbuf[13] = '5';
    if (posbuf[15] == ' ') posbuf[15] = '5';
    if (posbuf[16] == ' ') posbuf[16] = '5';

    if (posbuf[4] != '.' || posbuf[14] != '.') {
      return false;
    }

    unsigned int latDeg = 0, latMin = 0, latMinFrag = 0;
    unsigned int lonDeg = 0, lonMin = 0, lonMinFrag = 0;
    if (!parseFixedUnsigned(posbuf, 0, 2, latDeg) ||
      !parseFixedUnsigned(posbuf, 2, 2, latMin) ||
      !parseFixedUnsigned(posbuf, 5, 2, latMinFrag) ||
      !parseFixedUnsigned(posbuf, 9, 3, lonDeg) ||
      !parseFixedUnsigned(posbuf, 12, 2, lonMin) ||
      !parseFixedUnsigned(posbuf, 15, 2, lonMinFrag)) {
        return false;
    }

    char latHemi = posbuf[7];
    symTable = posbuf[8];
    char lonHemi = posbuf[17];
    symCode = posbuf[18];

    if (!validSymTableUncompressed(symTable)) {
        symTable = 0;
    }

    bool isSouth = (latHemi == 'S' || latHemi == 's');
    bool isWest = (lonHemi == 'W' || lonHemi == 'w');
    if (!isSouth && latHemi != 'N' && latHemi != 'n') return false;
    if (!isWest && lonHemi != 'E' && lonHemi != 'e') return false;
    if (latDeg > 90 || lonDeg > 180) return false;
    if (latMin >= 60 || lonMin >= 60) return false;

    lat = (float)latDeg + (float)latMin / 60.0f + (float)latMinFrag / 6000.0f;
    lon = (float)lonDeg + (float)lonMin / 60.0f + (float)lonMinFrag / 6000.0f;
    if (isSouth) lat = -lat;
    if (isWest) lon = -lon;
    return aprsLatLonInRange(lat, lon);
}

static bool parseAprsCompressed(const char *body, size_t len, float &lat, float &lon, char &symTable, char &symCode) {
    if (len < 13) {
        return false;
    }
    symTable = body[0];
    symCode = body[9];
    if (!validSymTableCompressed(symTable)) return false;
    for (int i = 1; i <= 8; i++) {
        if (body[i] < 0x21 || body[i] > 0x7b) return false;
    }

    int lat1 = (body[1] - 33);
    int lat2 = (body[2] - 33);
    int lat3 = (body[3] - 33);
    int lat4 = (body[4] - 33);
    lat1 = ((((lat1 * 91) + lat2) * 91) + lat3) * 91 + lat4;

    int lon1 = (body[5] - 33);
    int lon2 = (body[6] - 33);
    int lon3 = (body[7] - 33);
    int lon4 = (body[8] - 33);
    lon1 = ((((lon1 * 91) + lon2) * 91) + lon3) * 91 + lon4;

    lat = 90.0f - ((float)(lat1) / 380926.0f);
    lon = -180.0f + ((float)(lon1) / 190463.0f);

    if (symTable >= 'a' && symTable <= 'j') {
        symTable -= 81; // a-j -> 0-9
    }
    return aprsLatLonInRange(lat, lon);
}

static bool parseAprsMicE(const char *dstcall, const unsigned char *body, size_t len,
                          float &lat, float &lon, char &symTable, char &symCode) {
    if (!dstcall || strlen(dstcall) != 6) return false;
    if (len < 8) return false;

    // Validate destination callsign format
    for (int i = 0; i < 3; i++) {
        if (!((dstcall[i] >= '0' && dstcall[i] <= '9') ||
              (dstcall[i] >= 'A' && dstcall[i] <= 'L') ||
              (dstcall[i] >= 'P' && dstcall[i] <= 'Z'))) {
            return false;
        }
    }
    for (int i = 3; i < 6; i++) {
        if (!((dstcall[i] >= '0' && dstcall[i] <= '9') ||
              (dstcall[i] == 'L') ||
              (dstcall[i] >= 'P' && dstcall[i] <= 'Z'))) {
            return false;
        }
    }

    // Validate info field
    if (body[0] < 0x26 || body[0] > 0x7f) return false;
    if (body[1] < 0x26 || body[1] > 0x61) return false;
    if (body[2] < 0x1c || body[2] > 0x7f) return false;
    if (body[3] < 0x1c || body[3] > 0x7f) return false;
    if (body[4] < 0x1c || body[4] > 0x7d) return false;
    if (body[5] < 0x1c || body[5] > 0x7f) return false;
    if ((body[6] < 0x21 || body[6] > 0x7b) && body[6] != 0x7d) return false;
    if (body[7] != '/' && body[7] != '\\') return false;

    char dst[7];
    strncpy(dst, dstcall, 6);
    dst[6] = 0;

    for (char *p = dst; *p; p++) {
        if (*p >= 'A' && *p <= 'J') *p -= 'A' - '0';
        else if (*p >= 'P' && *p <= 'Y') *p -= 'P' - '0';
        else if (*p == 'K' || *p == 'L' || *p == 'Z') *p = '_';
    }

    int posAmbiguity = 0;
    if (dst[5] == '_') { dst[5] = '5'; posAmbiguity = 1; }
    if (dst[4] == '_') { dst[4] = '5'; posAmbiguity = 2; }
    if (dst[3] == '_') { dst[3] = '5'; posAmbiguity = 3; }
    if (dst[2] == '_') { dst[2] = '3'; posAmbiguity = 4; }
    if (dst[1] == '_' || dst[0] == '_') return false;

    unsigned int latDeg = 0, latMin = 0, latMinFrag = 0;
    if (!parseFixedUnsigned(dst, 0, 2, latDeg) ||
      !parseFixedUnsigned(dst, 2, 2, latMin) ||
      !parseFixedUnsigned(dst, 4, 2, latMinFrag)) {
      return false;
    }
    lat = (float)latDeg + (float)latMin / 60.0f + (float)latMinFrag / 6000.0f;
    if (dstcall[3] <= 0x4c) lat = -lat;

    unsigned int lonDeg = body[0] - 28;
    if (dstcall[4] >= 0x50) lonDeg += 100;
    if (lonDeg >= 180 && lonDeg <= 189) lonDeg -= 80;
    else if (lonDeg >= 190 && lonDeg <= 199) lonDeg -= 190;

    unsigned int lonMin = body[1] - 28;
    if (lonMin >= 60) lonMin -= 60;
    unsigned int lonMinFrag = body[2] - 28;

    switch (posAmbiguity) {
        case 0:
            lon = (float)lonDeg + (float)lonMin / 60.0f + (float)lonMinFrag / 6000.0f;
            break;
        case 1:
            lon = (float)lonDeg + (float)lonMin / 60.0f + (float)(lonMinFrag - lonMinFrag % 10 + 5) / 6000.0f;
            break;
        case 2:
            lon = (float)lonDeg + ((float)lonMin + 0.5f) / 60.0f;
            break;
        case 3:
            lon = (float)lonDeg + (float)(lonMin - lonMin % 10 + 5) / 60.0f;
            break;
        case 4:
            lon = (float)lonDeg + 0.5f;
            break;
        default:
            return false;
    }

    if (dstcall[5] >= 0x50) lon = -lon;

    symCode = body[6];
    symTable = body[7];
    return aprsLatLonInRange(lat, lon);
}

static bool parseAprsAdvancedPosition(const String &line, APRSStation &station) {
    int gtPos = line.indexOf('>');
    if (gtPos < 0) return false;
    int colonPos = line.indexOf(':', gtPos + 1);
    if (colonPos < 0) return false;

    String dstPath = line.substring(gtPos + 1, colonPos);
    int commaPos = dstPath.indexOf(',');
    String dstcall = (commaPos >= 0) ? dstPath.substring(0, commaPos) : dstPath;
    dstcall.trim();
    if (dstcall.length() == 0) return false;

    String payload = line.substring(colonPos + 1);
    if (payload.length() == 0) return false;

    char packettype = payload.charAt(0);
    const char *body = payload.c_str() + 1;
    size_t bodyLen = payload.length() > 0 ? (size_t)(payload.length() - 1) : 0;

    float lat = 0.0f, lon = 0.0f;
    char symTable = 0, symCode = 0;
    bool ok = false;
    int commentOffset = -1; // offset in payload
    String objectOrItemName = "";

    if (packettype == '`' || packettype == '\'') {
        ok = parseAprsMicE(dstcall.c_str(), (const unsigned char *)body, bodyLen, lat, lon, symTable, symCode);
      if (ok) commentOffset = 1 + aprsMicECommentBodyOffset((const unsigned char *)body, bodyLen);
    } else if (packettype == '!' || packettype == '=' || packettype == '/' || packettype == '@') {
        size_t offset = 1;
        if ((packettype == '/' || packettype == '@') && bodyLen >= 7) {
            body += 7;
            bodyLen -= 7;
            offset += 7;
        }
        if (bodyLen > 0) {
            char poschar = body[0];
            if (validSymTableCompressed(poschar)) {
                ok = parseAprsCompressed(body, bodyLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = (int)(offset + 13);
            } else if (poschar >= '0' && poschar <= '9') {
                ok = parseAprsUncompressed(body, bodyLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = (int)(offset + 19);
            }
        }
    } else if (packettype == ';') {
      // Object: name + (* or _) + timestamp(7) + position
      int nameEnd = -1;
      for (int i = 0; i < (int)bodyLen && i < 15; i++) {
        if (body[i] == '*' || body[i] == '_') {
          nameEnd = i;
          break;
        }
      }
      if (nameEnd >= 0) {
        objectOrItemName = payload.substring(1, 1 + nameEnd);
        objectOrItemName.trim();
      }
      if (nameEnd >= 0 && (nameEnd + 8) < (int)bodyLen) {
        const char *posBody = body + nameEnd + 8;
        size_t posLen = bodyLen - (nameEnd + 8);
            char poschar = posBody[0];
            if (validSymTableCompressed(poschar)) {
                ok = parseAprsCompressed(posBody, posLen, lat, lon, symTable, symCode);
          if (ok) commentOffset = 1 + nameEnd + 8 + 13;
            } else if (poschar >= '0' && poschar <= '9') {
                ok = parseAprsUncompressed(posBody, posLen, lat, lon, symTable, symCode);
          if (ok) commentOffset = 1 + nameEnd + 8 + 19;
            }
        }
    } else if (packettype == ')') {
        // Item: name ends with ! or _
        int nameEnd = -1;
        for (int i = 0; i < 9 && i < (int)bodyLen; i++) {
            if (body[i] == '!' || body[i] == '_') {
                nameEnd = i;
                break;
            }
        }
        if (nameEnd >= 0 && (nameEnd + 1) < (int)bodyLen) {
        objectOrItemName = payload.substring(1, 1 + nameEnd);
        objectOrItemName.trim();
            const char *posBody = body + nameEnd + 1;
            size_t posLen = bodyLen - (nameEnd + 1);
            char poschar = posBody[0];
            if (validSymTableCompressed(poschar)) {
                ok = parseAprsCompressed(posBody, posLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = 1 + nameEnd + 1 + 13;
            } else if (poschar >= '0' && poschar <= '9') {
                ok = parseAprsUncompressed(posBody, posLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = 1 + nameEnd + 1 + 19;
            }
        }
    }

    if (!ok) {
        return false;
    }

    station.lat = lat;
    station.lon = lon;
    station.hasLatLon = aprsLatLonInRange(lat, lon);
    if (!station.hasLatLon) {
      return false;
    }
    if (symCode != 0) {
        station.symbol = String(symCode);
    }
    if (symTable != 0) {
        station.symbolTable = String(symTable);
    }

    if (objectOrItemName.length() > 0) {
      // Dla raportów Object/Item użyj nazwy obiektu jako identyfikatora stacji.
      station.callsign = objectOrItemName;
    }

    if (commentOffset > 0 && commentOffset < payload.length()) {
      station.comment = normalizeAprsParsedComment(payload.substring(commentOffset));
    }
    return true;
}

// Obliczanie odlegĹ‚oĹ›ci metodÄ… Haversine (km)
// UĹĽywa double dla wiÄ™kszej precyzji obliczeĹ„
// UĹĽywa promienia Ziemi 6366.71 km (jak w ESP32APRS_Audio) dla zgodnoĹ›ci z aprs.fi
float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double r = 6366.71; // PromieĹ„ Ziemi w km (uĹĽywany przez ESP32APRS_Audio i aprs.fi)
    const double dLat = (lat2 - lat1) * M_PI / 180.0;
    const double dLon = (lon2 - lon1) * M_PI / 180.0;

    const double a = sin(dLat/2.0) * sin(dLat/2.0) +
                     cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
                     sin(dLon/2.0) * sin(dLon/2.0);
    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return (float)(r * c); // Wynik w km
}

// Oblicz azymut początkowy (0..360), gdzie 0 = Północ
float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    const double phi1 = lat1 * M_PI / 180.0;
    const double phi2 = lat2 * M_PI / 180.0;
    const double dLon = (lon2 - lon1) * M_PI / 180.0;

    const double y = sin(dLon) * cos(phi2);
    const double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);
    double bearingDeg = atan2(y, x) * 180.0 / M_PI;
    if (bearingDeg < 0.0) {
      bearingDeg += 360.0;
    }
    return (float)bearingDeg;
}

String formatDistanceOrCountry(const DXSpot &spot, size_t maxLen) {
  if (spot.country.length() > 0) {
    String c = spot.country;
    c.trim();
    if (c.length() > maxLen) {
      c = c.substring(0, maxLen);
    }
    return c;
  }
  return "-";
}

// OkreĹ›lanie pasma na podstawie czÄ™stotliwoĹ›ci (kHz) wg bandplanu
String getBand(float freq) {
  // Normalize to kHz so both kHz (DX cluster) and MHz (POTA API) inputs map correctly
  float freqKHz = freq;
  if (freqKHz > 0.0f && freqKHz < 1000.0f) {
    freqKHz *= 1000.0f;
  }

  if (freqKHz >= 1810 && freqKHz <= 2000) return "160m";
  if (freqKHz >= 3500 && freqKHz <= 3800) return "80m";
  if (freqKHz >= 7000 && freqKHz <= 7200) return "40m";
  if (freqKHz >= 10100 && freqKHz <= 10150) return "30m";
  if (freqKHz >= 14000 && freqKHz <= 14350) return "20m";
  if (freqKHz >= 18068 && freqKHz <= 18168) return "17m";
  if (freqKHz >= 21000 && freqKHz <= 21450) return "15m";
  if (freqKHz >= 24890 && freqKHz <= 24990) return "12m";
  if (freqKHz >= 28000 && freqKHz <= 29700) return "10m";
  return "Other";
}

// OkreĹ›lanie modulacji na podstawie komentarza
String getMode(String comment) {
  comment.toUpperCase();
  if (comment.indexOf("CW") >= 0) return "CW";
  if (comment.indexOf("FT4") >= 0) return "FT4";
  if (comment.indexOf("FT8") >= 0) return "FT8";
  if (comment.indexOf("SSB") >= 0 || comment.indexOf("USB") >= 0 || comment.indexOf("LSB") >= 0) return "SSB";
  return "SSB"; // DomyĹ›lnie SSB
}

String extractXmlTagValue(const String &xml, const String &tag) {
  String openTag = "<" + tag + ">";
  String closeTag = "</" + tag + ">";
  int start = xml.indexOf(openTag);
  if (start < 0) return "";
  start += openTag.length();
  int end = xml.indexOf(closeTag, start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

bool ensureQrzSession(String &sessionKey) {
  static String cachedKey = "";
  static unsigned long cachedAt = 0;
  const unsigned long SESSION_TTL_MS = 30UL * 60UL * 1000UL; // 30 minut

  if (qrzUsername.length() == 0 || qrzPassword.length() == 0) {
    qrzStatus = "QRZ: no credentials";
    return false;
  }

  unsigned long now = millis();
  if (cachedKey.length() > 0 && (now - cachedAt) < SESSION_TTL_MS) {
    sessionKey = cachedKey;
    qrzStatus = "QRZ: session ok";
    return true;
  }

  String url = "https://xmldata.qrz.com/xml/current/?username=" + qrzUsername +
               ";password=" + qrzPassword;
  HTTPClient http;
  http.setTimeout(3000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    qrzStatus = "QRZ: login http " + String(code);
    return false;
  }
  String body = http.getString();
  http.end();

  String err = extractXmlTagValue(body, "Error");
  if (err.length() == 0) {
    err = extractXmlTagValue(body, "error");
  }

  String key = extractXmlTagValue(body, "Key");
  if (key.length() == 0) {
    key = extractXmlTagValue(body, "key");
  }
  if (key.length() == 0) {
    if (err.length() > 0) {
      qrzStatus = "QRZ: login " + err;
      Serial.print("[QRZ] Login error: ");
      Serial.println(err);
    } else {
      qrzStatus = "QRZ: login failed";
    }
    return false;
  }

  cachedKey = key;
  cachedAt = now;
  sessionKey = key;
  qrzStatus = "QRZ: login ok";
  Serial.println("[QRZ] Session OK");
  return true;
}

bool fetchQrzCallsignInfo(const String &callsign, String &outGrid, String &outCountry,
                          double &outLat, double &outLon, bool &outHasLatLon) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    qrzStatus = "QRZ: no wifi";
    return false;
  }

  outGrid = "";
  outCountry = "";
  outLat = 0.0;
  outLon = 0.0;
  outHasLatLon = false;

  String call = callsign;
  call.toUpperCase();
  unsigned long now = millis();
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].callsign == call && (now - qrzCache[i].fetchedAtMs) < QRZ_CACHE_TTL_MS) {
      outGrid = qrzCache[i].grid;
      outCountry = qrzCache[i].country;
      outLat = qrzCache[i].lat;
      outLon = qrzCache[i].lon;
      outHasLatLon = qrzCache[i].hasLatLon;
      return (outGrid.length() > 0 || outCountry.length() > 0);
    }
  }

  String sessionKey;
  if (!ensureQrzSession(sessionKey)) {
    return false;
  }

  String url = "https://xmldata.qrz.com/xml/current/?s=" + sessionKey +
               ";callsign=" + callsign;
  HTTPClient http;
  http.setTimeout(3000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    qrzStatus = "QRZ: lookup http " + String(code);
    return false;
  }
  String body = http.getString();
  http.end();

  // ObsĹ‚uga bĹ‚Ä™du sesji - sprĂłbuj raz odĹ›wieĹĽyÄ‡
  if (body.indexOf("Session Timeout") >= 0 || body.indexOf("Invalid session") >= 0) {
    String dummy;
    ensureQrzSession(dummy);
    qrzStatus = "QRZ: session expired";
    return false;
  }

  String grid = extractXmlTagValue(body, "grid");
  String country = extractXmlTagValue(body, "country");
  String latStr = extractXmlTagValue(body, "lat");
  String lonStr = extractXmlTagValue(body, "lon");
  grid.trim();
  country.trim();
  if (country.length() == 0) {
    country = extractXmlTagValue(body, "dxcc");
    country.trim();
  }

  outGrid = grid;
  outCountry = country;
  latStr.trim();
  lonStr.trim();
  outHasLatLon = (latStr.length() > 0 && lonStr.length() > 0);
  if (outHasLatLon) {
    outLat = latStr.toDouble();
    outLon = lonStr.toDouble();
  } else {
    outLat = 0.0;
    outLon = 0.0;
  }
  qrzStatus = "QRZ: lookup ok";

  // Zapisz do cache (nadpisz najstarszy)
  int oldestIdx = 0;
  unsigned long oldestMs = qrzCache[0].fetchedAtMs;
  for (int i = 1; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].fetchedAtMs < oldestMs) {
      oldestMs = qrzCache[i].fetchedAtMs;
      oldestIdx = i;
    }
  }
  qrzCache[oldestIdx].callsign = call;
  qrzCache[oldestIdx].grid = grid;
  qrzCache[oldestIdx].country = country;
  qrzCache[oldestIdx].lat = (float)outLat;
  qrzCache[oldestIdx].lon = (float)outLon;
  qrzCache[oldestIdx].hasLatLon = outHasLatLon;
  qrzCache[oldestIdx].fetchedAtMs = now;

  return (grid.length() > 0 || country.length() > 0);
}

bool getQrzCacheFresh(const String &callsign, String &outGrid, String &outCountry,
                     double &outLat, double &outLon, bool &outHasLatLon,
                     unsigned long ttlMs) {
  outGrid = "";
  outCountry = "";
  outLat = 0.0;
  outLon = 0.0;
  outHasLatLon = false;

  String call = callsign;
  call.toUpperCase();
  unsigned long now = millis();
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].callsign == call && (now - qrzCache[i].fetchedAtMs) < ttlMs) {
      outGrid = qrzCache[i].grid;
      outCountry = qrzCache[i].country;
      outLat = qrzCache[i].lat;
      outLon = qrzCache[i].lon;
      outHasLatLon = qrzCache[i].hasLatLon;
      return (outGrid.length() > 0 || outCountry.length() > 0);
    }
  }
  return false;
}

void applyQrzCacheToSpot(DXSpot &spot, unsigned long ttlMs) {
  if (spot.callsign.length() == 0) {
    return;
  }

  String grid;
  String country;
  double lat = 0.0;
  double lon = 0.0;
  bool hasLatLon = false;
  if (!getQrzCacheFresh(spot.callsign, grid, country, lat, lon, hasLatLon, ttlMs)) {
    return;
  }

  if (spot.country.length() == 0 && country.length() > 0) {
    spot.country = country;
  }
  if (spot.locator.length() < 4 && grid.length() >= 4) {
    spot.locator = grid;
  }
  if (!spot.hasLatLon && hasLatLon) {
    spot.lat = (float)lat;
    spot.lon = (float)lon;
    spot.hasLatLon = true;
  }
}

void logQrzAllFields(const String &callsign, const String &xml) {
  int csStart = xml.indexOf("<Callsign>");
  int csEnd = xml.indexOf("</Callsign>");
  if (csStart < 0 || csEnd < 0 || csEnd <= csStart) {
    Serial.print("[QRZ][DATA] ");
    Serial.print(callsign);
    Serial.println(" - brak CallSign w XML");
    return;
  }
  String section = xml.substring(csStart + 10, csEnd);

  Serial.print("[QRZ][DATA] ");
  Serial.println(callsign);

  int pos = 0;
  while (pos < section.length()) {
    int openTagStart = section.indexOf('<', pos);
    if (openTagStart < 0) break;
    int openTagEnd = section.indexOf('>', openTagStart + 1);
    if (openTagEnd < 0) break;

    String tag = section.substring(openTagStart + 1, openTagEnd);
    tag.trim();
    if (tag.length() == 0 || tag.indexOf('/') == 0) {
      pos = openTagEnd + 1;
      continue;
    }
    int closeTagStart = section.indexOf("</" + tag + ">", openTagEnd + 1);
    if (closeTagStart < 0) {
      pos = openTagEnd + 1;
      continue;
    }
    String value = section.substring(openTagEnd + 1, closeTagStart);
    value.trim();
    if (value.length() > 0) {
      Serial.print("  ");
      Serial.print(tag);
      Serial.print(": ");
      Serial.println(value);
    }
    pos = closeTagStart + tag.length() + 3;
  }
}

void logSpotList() {
  Serial.println("[SPOTS] Lista spotow (max 8)");
  int count = min(spotCount, 8);
  for (int i = 0; i < count; i++) {
    String call = spots[i].callsign.length() ? spots[i].callsign : "-";
    String timeStr = spots[i].time.length() ? spots[i].time : "----Z";
    float mhz = spots[i].frequency / 1000.0f;
    Serial.print(" ");
    Serial.print(i + 1);
    Serial.print(") ");
    Serial.print(call);
    Serial.print(" ");
    Serial.print(timeStr);
    Serial.print(" ");
    Serial.print(mhz, 3);
    Serial.println(" MHz");

    double lat = 0.0;
    double lon = 0.0;
    bool hasLatLon = spots[i].hasLatLon;
    if (hasLatLon) {
      lat = spots[i].lat;
      lon = spots[i].lon;
    } else if (spots[i].locator.length() >= 4) {
      locatorToLatLon(spots[i].locator, lat, lon);
      hasLatLon = true;
    }

    if (hasLatLon) {
      Serial.print("    lat=");
      Serial.print(lat, 4);
      Serial.print(" lon=");
      Serial.println(lon, 4);
    } else {
      Serial.println("    lat=-- lon=--");
    }

    if (spots[i].distance > 0) {
      Serial.print("    dist=");
      Serial.print(spots[i].distance, 0);
      Serial.println(" km");
    } else {
      Serial.println("    dist=-- km");
    }
  }
}

bool isQrzQueued(const String &callsign) {
  String normalized = callsign;
  normalized.trim();
  normalized.toUpperCase();
  for (int i = 0; i < qrzQueueLen; i++) {
    String queuedCall = qrzQueue[i].callsign;
    queuedCall.toUpperCase();
    if (queuedCall == normalized) {
      return true;
    }
  }
  return false;
}

void enqueueQrzLookup(const String &callsign) {
  String normalized = callsign;
  normalized.trim();
  normalized.toUpperCase();
  if (normalized.length() == 0) {
    return;
  }
  if (qrzQueueLen >= QRZ_QUEUE_SIZE) {
    return;
  }
  if (isQrzQueued(normalized)) {
    return;
  }

  // Nie dodawaj do kolejki, jeżeli mamy już świeży wpis w cache.
  String grid;
  String country;
  double lat = 0.0;
  double lon = 0.0;
  bool hasLatLon = false;
  if (getQrzCacheFresh(normalized, grid, country, lat, lon, hasLatLon, QRZ_CACHE_TTL_MS)) {
    return;
  }

  PendingQrzLookup item;
  item.callsign = normalized;
  item.nextTryMs = millis();
  item.attempts = 0;
  qrzQueue[qrzQueueLen++] = item;
}

// Pobierz spoty POTA z publicznego API i uzupełnij bufor + kolejkę QRZ
bool fetchPotaApi() {
  if (!wifiConnected) return false;
  auto decodeToSpots = [&](JsonDocument &doc) -> bool {
    JsonArray arr;
    if (doc.is<JsonArray>()) {
      arr = doc.as<JsonArray>();
    } else if (doc["spots"].is<JsonArray>()) {
      arr = doc["spots"].as<JsonArray>();
    }
    if (arr.isNull() || arr.size() == 0) {
      return false;
    }

    lockPotaSpots();
    potaSpotCount = 0;
    for (JsonObject spot : arr) {
      if (potaSpotCount >= MAX_POTA_SPOTS) break;
      DXSpot s;
      s.time = spot["activatorLastSpotTime"] | spot["spotTime"] | "";
      s.callsign = spot["activator"] | spot["call"] | spot["callsign"] | "";
      float freq = 0.0f;
      if (spot["frequency"].is<float>() || spot["frequency"].is<double>()) {
        freq = spot["frequency"].as<float>();
      } else if (spot["frequency"].is<int>() || spot["frequency"].is<long>()) {
        freq = (float)spot["frequency"].as<long>();
      } else if (spot["frequency"].is<const char*>() || spot["frequency"].is<String>()) {
        String fstr = spot["frequency"].as<String>();
        freq = fstr.toFloat();
      }
      if (freq > 0) {
        s.frequency = freq / 1000.0f;
      } else {
        s.frequency = 0.0f;
      }
      s.band = getBand(s.frequency);
      s.mode = spot["mode"] | "";
      s.mode.toUpperCase();
      if (s.mode.length() == 0) {
        s.mode = getMode(spot["comments"] | "");
      }
      s.country = spot["country"] | "";
      s.spotter = spot["spotter"] | "";
      s.comment = spot["comments"] | "";
      s.distance = 0;
      s.hasLatLon = false;
      applyQrzCacheToSpot(s, QRZ_CACHE_TTL_MS);
      compactDxSpotStrings(s);
      potaSpots[potaSpotCount++] = s;
      if (s.country.length() == 0 && s.callsign.length() > 0) {
        enqueueQrzLookup(s.callsign);
      }
    }
    bool hasPotaSpots = potaSpotCount > 0;
    unlockPotaSpots();
    return hasPotaSpots;
  };

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(potaApiUrl)) {
    Serial.println("[POTA] http.begin failed");
    return false;
  }
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.print("[POTA] HTTP GET failed: ");
    Serial.println(code);
    http.end();
    return false;
  }
  int contentLength = http.getSize();
  Serial.print("[POTA] HTTP ");
  Serial.print(code);
  Serial.print(", content-length=");
  Serial.print(contentLength);
  Serial.println(" (may be -1 if chunked)");

  WiFiClient *stream = http.getStreamPtr();
  DynamicJsonDocument filter(512);
  JsonObject filterRootArray = filter[0].to<JsonObject>();
  filterRootArray["activatorLastSpotTime"] = true;
  filterRootArray["spotTime"] = true;
  filterRootArray["activator"] = true;
  filterRootArray["call"] = true;
  filterRootArray["callsign"] = true;
  filterRootArray["frequency"] = true;
  filterRootArray["mode"] = true;
  filterRootArray["country"] = true;
  filterRootArray["spotter"] = true;
  filterRootArray["comments"] = true;

  DynamicJsonDocument doc(60000);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  bool parsedOk = (!err && decodeToSpots(doc));
  if (!parsedOk) {
    if (err) {
      Serial.print("[POTA] Filtered JSON parse error: ");
      Serial.println(err.c_str());
    }
    Serial.println("[POTA] Filtered parse empty - retrying without filter");

    HTTPClient retry;
    retry.setTimeout(8000);
    if (!retry.begin(potaApiUrl)) {
      return false;
    }
    int retryCode = retry.GET();
    if (retryCode != HTTP_CODE_OK) {
      retry.end();
      return false;
    }
    WiFiClient *retryStream = retry.getStreamPtr();
    DynamicJsonDocument fullDoc(120000);
    DeserializationError retryErr = deserializeJson(fullDoc, *retryStream);
    retry.end();
    if (retryErr) {
      Serial.print("[POTA] Fallback JSON parse error: ");
      Serial.println(retryErr.c_str());
      return false;
    }
    if (!decodeToSpots(fullDoc)) {
      Serial.println("[POTA] Fallback parse also returned 0 spots");
      return false;
    }
  }

  // Posortuj malejąco po czasie (ISO string porównuje się leksykograficznie poprawnie)
  lockPotaSpots();
  for (int i = 0; i < potaSpotCount - 1; i++) {
    for (int j = i + 1; j < potaSpotCount; j++) {
      if (potaSpots[j].time > potaSpots[i].time) {
        DXSpot tmp = potaSpots[i];
        potaSpots[i] = potaSpots[j];
        potaSpots[j] = tmp;
      }
    }
  }
  bool hasPotaSpots = potaSpotCount > 0;
  unlockPotaSpots();

  return hasPotaSpots;
}

void removeQrzQueueAt(int idx) {
  if (idx < 0 || idx >= qrzQueueLen) {
    return;
  }
  for (int i = idx; i < qrzQueueLen - 1; i++) {
    qrzQueue[i] = qrzQueue[i + 1];
  }
  qrzQueueLen--;
}

void updateSpotsWithQrz(const String &callsign, const String &grid,
                        const String &country, double lat, double lon, bool hasLatLon) {
  bool updated = false;
  bool updatedPota = false;
  if (!userLatLonValid && userLocator.length() >= 4) {
    double tmpLat = 0.0;
    double tmpLon = 0.0;
    locatorToLatLon(userLocator, tmpLat, tmpLon);
    userLat = tmpLat;
    userLon = tmpLon;
    userLatLonValid = true;
  }
  lockDxSpots();
  for (int i = 0; i < spotCount; i++) {
    if (!spots[i].callsign.equalsIgnoreCase(callsign)) {
      continue;
    }

    if (grid.length() >= 4) {
      spots[i].locator = grid;
    }
    if (country.length() > 0) {
      spots[i].country = country;
    }

    double spotLat = 0.0;
    double spotLon = 0.0;
    bool spotHasLatLon = false;
    if (hasLatLon) {
      spotLat = lat;
      spotLon = lon;
      spotHasLatLon = true;
      spots[i].lat = (float)lat;
      spots[i].lon = (float)lon;
      spots[i].hasLatLon = true;
    } else if (spots[i].locator.length() >= 4) {
      locatorToLatLon(spots[i].locator, spotLat, spotLon);
      spotHasLatLon = true;
      spots[i].lat = (float)spotLat;
      spots[i].lon = (float)spotLon;
      spots[i].hasLatLon = true;
    }

    double userLatLocal = 0.0;
    double userLonLocal = 0.0;
    bool userHasLatLon = getServiceCoordinatesForServices(userLatLocal, userLonLocal);

    if (userHasLatLon && spotHasLatLon) {
      spots[i].distance = calculateDistance(userLatLocal, userLonLocal, spotLat, spotLon);
      updated = true;
    }
  }
  unlockDxSpots();

  lockPotaSpots();
  for (int i = 0; i < potaSpotCount; i++) {
    if (!potaSpots[i].callsign.equalsIgnoreCase(callsign)) {
      continue;
    }

    if (grid.length() >= 4) {
      potaSpots[i].locator = grid;
    }
    if (country.length() > 0) {
      potaSpots[i].country = country;
    }

    double spotLat = 0.0;
    double spotLon = 0.0;
    bool spotHasLatLon = false;
    if (hasLatLon) {
      spotLat = lat;
      spotLon = lon;
      spotHasLatLon = true;
      potaSpots[i].lat = (float)lat;
      potaSpots[i].lon = (float)lon;
      potaSpots[i].hasLatLon = true;
    } else if (potaSpots[i].locator.length() >= 4) {
      locatorToLatLon(potaSpots[i].locator, spotLat, spotLon);
      spotHasLatLon = true;
      potaSpots[i].lat = (float)spotLat;
      potaSpots[i].lon = (float)spotLon;
      potaSpots[i].hasLatLon = true;
    }

    double userLatLocal = 0.0;
    double userLonLocal = 0.0;
    bool userHasLatLon = getServiceCoordinatesForServices(userLatLocal, userLonLocal);

    if (userHasLatLon && spotHasLatLon) {
      potaSpots[i].distance = calculateDistance(userLatLocal, userLonLocal, spotLat, spotLon);
      updatedPota = true;
    }
  }
  unlockPotaSpots();

  if (updated) {
#ifdef ENABLE_TFT_DISPLAY
    updateScreen2();
#endif
  }
  if (updatedPota) {
#ifdef ENABLE_TFT_DISPLAY
    updateScreen7();
#endif
  }
}

bool fetchQrzRawXml(const String &callsign, String &body) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    qrzStatus = "QRZ: no wifi";
    return false;
  }
  String sessionKey;
  if (!ensureQrzSession(sessionKey)) {
    return false;
  }

  String url = "https://xmldata.qrz.com/xml/current/?s=" + sessionKey +
               ";callsign=" + callsign;
  HTTPClient http;
  http.setTimeout(3000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    qrzStatus = "QRZ: lookup http " + String(code);
    return false;
  }
  body = http.getString();
  http.end();

  if (body.indexOf("Session Timeout") >= 0 || body.indexOf("Invalid session") >= 0) {
    String dummy;
    ensureQrzSession(dummy);
    qrzStatus = "QRZ: session expired";
    return false;
  }
  return true;
}

void runQrzSingleTest(const String &callsign) {
  (void)callsign;
}

// Parsowanie spotu DX z formatu "DX de [Spotter]: [Freq] [Call] [Comments] [Time]"
bool parseDXSpot(String line, DXSpot &spot) {
  LOGV_PRINT("[PARSE] parseDXSpot START, len=");
  LOGV_PRINTLN(line.length());
  
  // Format: DX de SP5XYZ: 14025.0 SP9ABC Test comment 1234Z
  if (!line.startsWith("DX de")) {
    LOGV_PRINTLN("[PARSE] Nie zaczyna siÄ™ od 'DX de' - wyjĹ›cie");
    return false;
  }
  
  int dePos = line.indexOf("DX de");
  if (dePos < 0) return false;
  
  int colonPos = line.indexOf(":", dePos);
  if (colonPos < 0) return false;
  
  // WyciÄ…gnij spottera
  String spotterPart = line.substring(dePos + 5, colonPos);
  spotterPart.trim();
  spot.spotter = spotterPart;
  
  // Reszta po dwukropku
  String rest = line.substring(colonPos + 1);
  rest.trim();
  
  // Parsuj czÄ™stotliwoĹ›Ä‡ (pierwsza liczba)
  int spacePos = rest.indexOf(" ");
  if (spacePos < 0) return false;
  
  String freqStr = rest.substring(0, spacePos);
  spot.frequency = freqStr.toFloat();
  rest = rest.substring(spacePos + 1);
  rest.trim();
  
  // Parsuj znak wywoĹ‚awczy (nastÄ™pne sĹ‚owo)
  spacePos = rest.indexOf(" ");
  if (spacePos < 0) {
    spot.callsign = rest;
    spot.comment = "";
  } else {
    spot.callsign = rest.substring(0, spacePos);
    rest = rest.substring(spacePos + 1);
    rest.trim();
    
    // Reszta to komentarz (moĹĽe zawieraÄ‡ czas na koĹ„cu)
    // Czas jest zwykle na koĹ„cu w formacie HHMMZ
    int timePos = rest.lastIndexOf("Z");
    if (timePos > 0 && rest.length() >= timePos + 1) {
      String timeStr = rest.substring(timePos - 4, timePos + 1);
      if (timeStr.length() == 5) {
        spot.time = timeStr;
        spot.comment = rest.substring(0, timePos - 4);
        spot.comment.trim();
      } else {
        spot.comment = rest;
        // Pobierz czas z NTP
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeBuffer[6];
          strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
          spot.time = String(timeBuffer);
        } else {
          spot.time = "----Z";
        }
      }
    } else {
      spot.comment = rest;
      // Pobierz czas z NTP (z timeout - nie blokuj jeĹ›li NTP nie dziaĹ‚a)
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 1)) { // Timeout 1 sekunda
        char timeBuffer[6];
        strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
        spot.time = String(timeBuffer);
      } else {
        spot.time = "----Z";
      }
    }
  }
  
  // OkreĹ›l pasmo i modulacjÄ™
  spot.band = getBand(spot.frequency);
  spot.mode = getMode(spot.comment);
  
  // SprĂłbuj wyciÄ…gnÄ…Ä‡ locator z komentarza (format JO82LK)
  spot.locator = "";
  int commentLen = spot.comment.length();
  if (commentLen >= 6) { // Locator ma minimum 6 znakĂłw
    for (int i = 0; i <= commentLen - 6; i++) {
      String sub = spot.comment.substring(i, i + 6);
      if (sub.length() == 6 && 
          isAlpha(sub.charAt(0)) && isAlpha(sub.charAt(1)) &&
          isDigit(sub.charAt(2)) && isDigit(sub.charAt(3)) &&
          isAlpha(sub.charAt(4)) && isAlpha(sub.charAt(5))) {
        spot.locator = sub;
        break;
      }
    }
  }
  
  LOGV_PRINTLN("[PARSE] Obliczanie odlegĹ‚oĹ›ci...");
  
  spot.distance = 0;
  spot.country = "";
  spot.lat = 0.0f;
  spot.lon = 0.0f;
  spot.hasLatLon = false;
  bool qrzConfigured = (qrzUsername.length() > 0 && qrzPassword.length() > 0);
  // Najpierw licz z lokatora jeĹ›li mamy lokalizacjÄ™ i lokator w spocie
  double userLatLocal = 0.0;
  double userLonLocal = 0.0;
  if (getServiceCoordinatesForServices(userLatLocal, userLonLocal) && spot.locator.length() >= 4) {
    double spotLat, spotLon;
    locatorToLatLon(spot.locator, spotLat, spotLon);
    spot.distance = calculateDistance(userLatLocal, userLonLocal, spotLat, spotLon);
    spot.lat = (float)spotLat;
    spot.lon = (float)spotLon;
    spot.hasLatLon = true;
  }

  if (qrzConfigured) {
    applyQrzCacheToSpot(spot, QRZ_CACHE_TTL_MS);
  }

  if (qrzConfigured && spot.callsign.length() > 0 && (spot.country.length() == 0 || !spot.hasLatLon)) {
    String call = spot.callsign;
    call.toUpperCase();
    enqueueQrzLookup(call);
  }

  compactDxSpotStrings(spot);
  
  LOGV_PRINTLN("[PARSE] parseDXSpot END - OK");
  return true;
}

// Prostsze parsowanie spotu dla POTA (bez QRZ i bez obliczeĹ„ dystansu)
bool parsePotaSpot(String line, DXSpot &spot) {
  if (!line.startsWith("DX de")) {
    return false;
  }

  int dePos = line.indexOf("DX de");
  if (dePos < 0) return false;

  int colonPos = line.indexOf(":", dePos);
  if (colonPos < 0) return false;

  String spotterPart = line.substring(dePos + 5, colonPos);
  spotterPart.trim();
  spot.spotter = spotterPart;

  String rest = line.substring(colonPos + 1);
  rest.trim();

  int spacePos = rest.indexOf(" ");
  if (spacePos < 0) return false;

  String freqStr = rest.substring(0, spacePos);
  spot.frequency = freqStr.toFloat();
  rest = rest.substring(spacePos + 1);
  rest.trim();

  spacePos = rest.indexOf(" ");
  if (spacePos < 0) {
    spot.callsign = rest;
    spot.comment = "";
  } else {
    spot.callsign = rest.substring(0, spacePos);
    rest = rest.substring(spacePos + 1);
    rest.trim();

    int timePos = rest.lastIndexOf("Z");
    if (timePos > 0 && rest.length() >= timePos + 1) {
      String timeStr = rest.substring(timePos - 4, timePos + 1);
      if (timeStr.length() == 5) {
        spot.time = timeStr;
        spot.comment = rest.substring(0, timePos - 4);
        spot.comment.trim();
      } else {
        spot.comment = rest;
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeBuffer[6];
          strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
          spot.time = String(timeBuffer);
        } else {
          spot.time = "----Z";
        }
      }
    } else {
      spot.comment = rest;
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 1)) {
        char timeBuffer[6];
        strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
        spot.time = String(timeBuffer);
      } else {
        spot.time = "----Z";
      }
    }
  }

  spot.band = getBand(spot.frequency);
  spot.mode = getMode(spot.comment);
  spot.locator = "";
  spot.distance = 0;
  spot.country = "";
  spot.lat = 0.0f;
  spot.lon = 0.0f;
  spot.hasLatLon = false;
  compactDxSpotStrings(spot);

  return true;
}

// Dodaj nowy spot do tablicy
void addSpot(DXSpot spot) {
  compactDxSpotStrings(spot);
  lockDxSpots();
  LOGV_PRINT("[SPOT] addSpot: ");
  LOGV_PRINT(spot.callsign);
  LOGV_PRINT(" @ ");
  LOGV_PRINT(spot.frequency);
  LOGV_PRINT(" kHz, spotCount=");
  LOGV_PRINTLN(spotCount);
  
  // PrzesuĹ„ istniejÄ…ce spoty
  if (spotCount < MAX_SPOTS) {
    spotCount++;
  }
  
  // PrzesuĹ„ wszystkie spoty o jeden w dĂłĹ‚
  for (int i = MAX_SPOTS - 1; i > 0; i--) {
    spots[i] = spots[i - 1];
  }
  
  // Dodaj nowy spot na poczÄ…tku
  spots[0] = spot;
  
  LOGV_PRINT("[SPOT] addSpot END, nowy spotCount=");
  LOGV_PRINTLN(spotCount);
  unlockDxSpots();
  // logSpotList(); // WyĹ‚Ä…czone - nie wypisuj listy spotĂłw do Serial
}

void addPotaSpot(DXSpot spot) {
  compactDxSpotStrings(spot);
  lockPotaSpots();
  if (potaSpotCount < MAX_POTA_SPOTS) {
    potaSpotCount++;
  }

  for (int i = MAX_POTA_SPOTS - 1; i > 0; i--) {
    potaSpots[i] = potaSpots[i - 1];
  }

  potaSpots[0] = spot;
  unlockPotaSpots();
}

void addHamalertSpot(DXSpot spot) {
  compactDxSpotStrings(spot);
  lockHamalertSpots();
  if (hamalertSpotCount < MAX_POTA_SPOTS) {
    hamalertSpotCount++;
  }

  for (int i = MAX_POTA_SPOTS - 1; i > 0; i--) {
    hamalertSpots[i] = hamalertSpots[i - 1];
  }

  hamalertSpots[0] = spot;
  unlockHamalertSpots();
}

bool parseHamalertJsonSpot(const String &line, DXSpot &spot) {
  DynamicJsonDocument doc(3072);
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    return false;
  }

  JsonVariant root = doc.as<JsonVariant>();
  String callsign = root["dx"] | root["call"] | root["callsign"] | "";
  if (callsign.length() == 0) {
    return false;
  }

  spot.callsign = callsign;
  spot.spotter = root["de"] | root["spotter"] | root["source"] | "";
  spot.comment = root["comment"] | root["info"] | root["message"] | "";
  spot.mode = root["mode"] | "";
  if (spot.mode.length() == 0) {
    spot.mode = getMode(spot.comment);
  }
  spot.mode.toUpperCase();
  spot.country = root["country"] | root["entity"] | "";

  float freq = 0.0f;
  if (root["frequency"].is<float>() || root["frequency"].is<double>()) {
    freq = (float)root["frequency"].as<double>();
  } else if (root["frequency"].is<int>() || root["frequency"].is<long>()) {
    freq = (float)root["frequency"].as<long>();
  } else if (root["freq"].is<float>() || root["freq"].is<double>()) {
    freq = (float)root["freq"].as<double>();
  } else if (root["freq"].is<int>() || root["freq"].is<long>()) {
    freq = (float)root["freq"].as<long>();
  } else if (root["frequency"].is<const char*>()) {
    freq = String(root["frequency"].as<const char*>()).toFloat();
  } else if (root["freq"].is<const char*>()) {
    freq = String(root["freq"].as<const char*>()).toFloat();
  }
  if (freq <= 0.0f) {
    return false;
  }
  // HAMALERT i DX cluster zwykle podają kHz, ale obsłuż także MHz.
  spot.frequency = (freq > 1000.0f) ? (freq / 1000.0f) : freq;
  spot.band = getBand(spot.frequency * 1000.0f);

  String timeStr = root["time"] | root["spotTime"] | root["timestamp"] | "";
  if (timeStr.length() == 0) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      char timeBuffer[6];
      strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
      timeStr = String(timeBuffer);
    } else {
      timeStr = "----Z";
    }
  }
  spot.time = timeStr;
  spot.distance = 0;
  spot.locator = "";
  spot.lat = 0.0f;
  spot.lon = 0.0f;
  spot.hasLatLon = false;
  compactDxSpotStrings(spot);
  return true;
}

bool parseHamalertSpotLine(const String &lineIn, DXSpot &spot) {
  String line = lineIn;
  line.trim();
  if (line.length() < 6) {
    return false;
  }
  if (line == ">" || line == "#" || line.startsWith("Welcome") || line.startsWith("Password")) {
    return false;
  }
  if (line.charAt(0) == '{' && parseHamalertJsonSpot(line, spot)) {
    return true;
  }

  DXSpot parsed;
  if (!parsePotaSpot(line, parsed)) {
    return false;
  }
  parsed.frequency = (parsed.frequency > 1000.0f) ? (parsed.frequency / 1000.0f) : parsed.frequency;
  parsed.band = getBand(parsed.frequency * 1000.0f);
  parsed.mode.toUpperCase();
  compactDxSpotStrings(parsed);
  spot = parsed;
  return true;
}

bool fetchHamalertTelnet() {
  const size_t HAMALERT_MAX_LINE_LEN = 1400;
  Serial.println("[HAMALERT] fetch start");
  if (!wifiConnected) {
    Serial.println("[HAMALERT] skip: WiFi offline");
    return false;
  }
  if (hamalertHost.length() == 0 || hamalertPort <= 0 || hamalertLogin.length() == 0 || hamalertPassword.length() == 0) {
    Serial.println("[HAMALERT] skip: missing host/port/login/password");
    return false;
  }

  String login = hamalertLogin;
  login.trim();
  if (login.length() == 0) {
    Serial.println("[HAMALERT] skip: hamalert_login is empty");
    return false;
  }

  Serial.print("[HAMALERT] connect ");
  Serial.print(hamalertHost);
  Serial.print(":");
  Serial.println(hamalertPort);

  WiFiClient client;
  client.setTimeout(1200);
  if (!client.connect(hamalertHost.c_str(), hamalertPort)) {
    Serial.println("[HAMALERT] connect failed");
    return false;
  }
  Serial.println("[HAMALERT] connected");

  lockHamalertSpots();
  hamalertSpotCount = 0;
  unlockHamalertSpots();
  String line;
  line.reserve(HAMALERT_MAX_LINE_LEN + 16);

  // Odczytaj ewentualny banner/prompt przed logowaniem
  unsigned long preLoginUntil = millis() + 1200;
  while (millis() < preLoginUntil && client.connected()) {
    while (client.available()) {
      char c = (char)client.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (line.length() > 0) {
          Serial.print("[HAMALERT] < ");
          Serial.println(line);
        }
        line = "";
      } else if (line.length() < HAMALERT_MAX_LINE_LEN) {
        line += c;
      }
    }
    delay(2);
    yield();
  }
  if (line.length() > 0) {
    Serial.print("[HAMALERT] < ");
    Serial.println(line);
    line = "";
  }

  // HamAlert telnet wymaga username + password
  Serial.print("[HAMALERT] > username: ");
  Serial.println(login);
  client.print(login);
  client.print("\n");

  delay(180);
  Serial.print("[HAMALERT] > password: (");
  Serial.print(hamalertPassword.length());
  Serial.println(" chars)");
  client.print(hamalertPassword);
  client.print("\n");

  delay(220);
  Serial.println("[HAMALERT] > set/json");
  client.print("set/json\n");

  delay(160);
  Serial.println("[HAMALERT] > sh/dx 30");
  client.print("sh/dx 30\n");

  const unsigned long started = millis();
  unsigned long lastRx = millis();
  const unsigned long totalTimeoutMs = 9000;
  const unsigned long idleTimeoutMs = 1800;
  int parsedOkCount = 0;
  int parsedFailCount = 0;
  int diagLineCount = 0;

  while (client.connected() && (millis() - started) < totalTimeoutMs) {
    while (client.available()) {
      char c = (char)client.read();
      lastRx = millis();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        if (line.length() > 0) {
          DXSpot s;
          if (parseHamalertSpotLine(line, s)) {
            addHamalertSpot(s);
            parsedOkCount++;
            if (hamalertSpotCount >= MAX_POTA_SPOTS) {
              break;
            }
          } else {
            parsedFailCount++;
            String lowerLine = line;
            lowerLine.toLowerCase();
            if (lowerLine.indexOf("invalid") >= 0 || lowerLine.indexOf("error") >= 0 || lowerLine.indexOf("denied") >= 0 ||
                lowerLine.indexOf("login") >= 0 || lowerLine.indexOf("password") >= 0 || lowerLine.indexOf("failed") >= 0) {
              Serial.print("[HAMALERT] server: ");
              Serial.println(line);
            } else if (diagLineCount < 8) {
              Serial.print("[HAMALERT] line(unparsed): ");
              Serial.println(line);
              diagLineCount++;
            }
          }
        }
        line = "";
      } else if (line.length() < HAMALERT_MAX_LINE_LEN) {
        line += c;
      }
    }
    if ((millis() - lastRx) > idleTimeoutMs && hamalertSpotCount > 0) {
      break;
    }
    if (hamalertSpotCount >= MAX_POTA_SPOTS) {
      break;
    }
    yield();
    delay(2);
  }

  if (line.length() > 0 && hamalertSpotCount < MAX_POTA_SPOTS) {
    DXSpot s;
    if (parseHamalertSpotLine(line, s)) {
      addHamalertSpot(s);
      parsedOkCount++;
    } else {
      parsedFailCount++;
    }
  }

  client.stop();
  Serial.print("[HAMALERT] done: spots=");
  Serial.print(hamalertSpotCount);
  Serial.print(", parsed_ok=");
  Serial.print(parsedOkCount);
  Serial.print(", parsed_fail=");
  Serial.println(parsedFailCount);
  return hamalertSpotCount > 0;
}

// ========== APRS-IS FUNKCJE ==========

// Połącz z serwerem APRS-IS
void connectToAPRS() {
  Serial.println("[APRS] connectToAPRS() START");
  
  if (!wifiConnected) {
    Serial.println("[APRS] WiFi nie połączony - wyjście");
    return;
  }
  
  if (aprsClient.connected()) {
    // Jeśli konfiguracja się zmieniła, rozłącz i połącz ponownie
    Serial.println("[APRS] Już połączony - sprawdzam czy potrzeba reconnect...");
    // Można dodać sprawdzenie czy konfiguracja się zmieniła, ale na razie zostawiamy jak jest
    // W razie potrzeby reconnect nastąpi automatycznie przez watchdog
    return;
  }
  
  unsigned long now = millis();
  if (now - lastAPRSAttempt < 5000) {
    return; // Cichy return - nie spamuj logu
  }
  
  lastAPRSAttempt = now;
  
  Serial.print("[APRS] Łączenie z APRS-IS: ");
  Serial.print(aprsIsHost);
  Serial.print(":");
  Serial.println(aprsIsPort);
  
  // Diagnostyka DNS
  IPAddress resolvedIp;
  if (WiFi.hostByName(aprsIsHost.c_str(), resolvedIp)) {
    Serial.print("DNS OK: ");
    Serial.print(aprsIsHost);
    Serial.print(" -> ");
    Serial.println(resolvedIp);
  } else {
    Serial.print("DNS FAIL dla hosta: ");
    Serial.println(aprsIsHost);
  }
  
  Serial.println("[APRS] Wywołanie aprsClient.connect()...");
  unsigned long connectStart = millis();
  
  // Ustaw timeout połączenia (300 sekund jak w wymaganiach)
  aprsClient.setTimeout(300);
  
  if (aprsClient.connect(aprsIsHost.c_str(), aprsIsPort)) {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[APRS] Połączono z APRS-IS! (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    aprsConnected = true;
    aprsBuffer = "";
    aprsLoginSent = false;
    lastAPRSRxMs = millis();
    
    // Zaplanuj wysłanie loginu
    delay(500); // Krótkie opóźnienie przed loginem
    sendAPRSLogin();
  } else {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[APRS] Błąd połączenia z APRS-IS (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    aprsConnected = false;
  }
  Serial.println("[APRS] connectToAPRS() END");
}

// Wyślij login do APRS-IS
void sendAPRSLogin() {
  if (!aprsConnected || !aprsClient.connected()) {
    Serial.println("[APRS] Nie mozna wysłać loginu - brak połączenia");
    return;
  }
  
  // Format loginu: user Callsign pass 23123 vers ESP32-HAM-CLOCK 1.2
  String login = "user ";
  login += getAprsTxCallsignWithSsid();
  login += " pass ";
  login += String(aprsPasscode);
  login += " vers ESP32-HAM-CLOCK 1.2";
  
  Serial.print("[APRS] Wysyłanie loginu: ");
  Serial.println(login);
  
  aprsClient.println(login);
  aprsLoginSent = true;
  lastAPRSRxMs = millis();
  
  // Po zalogowaniu wyślij filtr
  delay(500);
  sendAPRSFilter();
}

// Wyślij komendę filtra do APRS-IS
void sendAPRSFilter() {
  if (!aprsConnected || !aprsClient.connected()) {
    Serial.println("[APRS] Nie można wysłać filtra - brak połączenia");
    return;
  }
  
  // Format filtra: #filter r/52.40/16.92/50
  // Używamy współrzędnych z sekcji "Moja Stacja"
  double filterLat = 52.40;  // Fallback jeśli nie ustawione
  double filterLon = 16.92;
  getServiceCoordinatesForServices(filterLat, filterLon);
  unsigned long nowMs = millis();
  bool gpsBased = useGpsIfActive && gpsLocationValid && isGpsFixActive(nowMs);
  String filter = "#filter r/";
  filter += String(filterLat, 2);
  filter += "/";
  filter += String(filterLon, 2);
  filter += "/";
  filter += String(aprsFilterRadius);
  
  Serial.print("[APRS] Wysyłanie filtra: ");
  Serial.println(filter);
  
  aprsClient.println(filter);
  lastAPRSRxMs = millis();
  aprsFilterLastSentMs = nowMs;
  aprsFilterLastLat = filterLat;
  aprsFilterLastLon = filterLon;
  aprsFilterLastValid = true;
  aprsFilterLastGpsBased = gpsBased;
}

static void refreshAprsFilterIfNeeded(unsigned long nowMs) {
  if (!aprsConnected || !aprsClient.connected() || !aprsLoginSent) {
    return;
  }

  if (aprsFilterLastSentMs != 0 && (nowMs - aprsFilterLastSentMs) < APRS_FILTER_REFRESH_MIN_INTERVAL_MS) {
    return;
  }

  double currentLat = 0.0;
  double currentLon = 0.0;
  if (!getServiceCoordinatesForServices(currentLat, currentLon)) {
    return;
  }

  bool gpsBasedNow = useGpsIfActive && gpsLocationValid && isGpsFixActive(nowMs);
  if (!aprsFilterLastValid || gpsBasedNow != aprsFilterLastGpsBased) {
    sendAPRSFilter();
    return;
  }

  float movedKm = calculateDistance(aprsFilterLastLat, aprsFilterLastLon, currentLat, currentLon);
  if (movedKm >= APRS_FILTER_REFRESH_MOVE_KM) {
    sendAPRSFilter();
  }
}

String formatAprsCoordinate(double value, bool isLat) {
  double absVal = fabs(value);
  int degrees = (int)absVal;
  double minutes = (absVal - degrees) * 60.0;
  String degText;
  if (isLat) {
    degText = (degrees < 10 ? "0" : "") + String(degrees);
  } else {
    if (degrees < 10) degText = "00" + String(degrees);
    else if (degrees < 100) degText = "0" + String(degrees);
    else degText = String(degrees);
  }

  String minText = String(minutes, 2);
  if (minutes < 10.0) {
    minText = "0" + minText;
  }

  char hemi = isLat ? ((value >= 0.0) ? 'N' : 'S') : ((value >= 0.0) ? 'E' : 'W');
  return degText + minText + String(hemi);
}

static bool isValidAprsSymbolChar(char c) {
  return (c >= 33 && c <= 126); // widoczne ASCII bez spacji/kontrolnych
}

static String sanitizeAprsSymbol(const String &sym) {
  String s = sym;
  s.trim();
  if (s.length() < 2) {
    return String(DEFAULT_APRS_SYMBOL_TABLE) + String(DEFAULT_APRS_SYMBOL_CODE);
  }
  s = s.substring(0, 2);
  if (!isValidAprsSymbolChar(s.charAt(0)) || !isValidAprsSymbolChar(s.charAt(1))) {
    return String(DEFAULT_APRS_SYMBOL_TABLE) + String(DEFAULT_APRS_SYMBOL_CODE);
  }
  return s;
}

static void applyAprsSymbol(const String &sym) {
  String s = sanitizeAprsSymbol(sym);
  aprsSymbolTwoChar = s;
  aprsSymbolTable = s.charAt(0);
  aprsSymbolCode = s.charAt(1);
}

static int normalizeAprsSsid(int ssid) {
  if (ssid < 0) return 0;
  if (ssid > 15) return 15;
  return ssid;
}

static void applyAprsSsid(int ssid) {
  aprsSsid = normalizeAprsSsid(ssid);
}

static String getAprsBaseCallsign() {
  String call = aprsCallsign.length() ? aprsCallsign : userCallsign;
  call.trim();
  call.toUpperCase();
  int dash = call.indexOf('-');
  if (dash > 0) {
    call = call.substring(0, dash);
  }
  return call;
}

static String getAprsTxCallsignWithSsid() {
  String baseCall = getAprsBaseCallsign();
  if (aprsSsid <= 0) {
    return baseCall;
  }
  return baseCall + "-" + String(aprsSsid);
}

static String sanitizeAprsComment(const String &cmt) {
  String s = cmt;
  s.trim();
  String out = "";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c >= 32 && c <= 126) { // drukowalne ASCII
      out += c;
    }
    if (out.length() >= 43) { // rekomendowany limit komentarza APRS
      break;
    }
  }
  return out;
}

static String sanitizeAprsAlertWatchToken(const String &raw) {
  String token = raw;
  token.trim();
  token.toUpperCase();

  bool wildcardAnySsid = false;
  if (token.endsWith("*")) {
    wildcardAnySsid = true;
    token.remove(token.length() - 1);
  }

  String cleaned = "";
  for (size_t i = 0; i < token.length(); i++) {
    char ch = token.charAt(i);
    bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '-' || ch == '/';
    if (ok) cleaned += ch;
  }

  if (cleaned.length() == 0) {
    return "";
  }
  if (wildcardAnySsid) {
    cleaned += "*";
  }
  return cleaned;
}

static String sanitizeAprsAlertList(const String &raw) {
  String src = raw;
  src.trim();
  if (src.length() == 0) return "";

  String out = "";
  int count = 0;
  int start = 0;

  while (start <= src.length() && count < 20) {
    int comma = src.indexOf(',', start);
    int end = (comma >= 0) ? comma : src.length();
    String token = src.substring(start, end);
    String cleaned = sanitizeAprsAlertWatchToken(token);

    if (cleaned.length() > 0) {
      if (count > 0) out += ",";
      out += cleaned;
      count++;
    }

    if (comma < 0) break;
    start = comma + 1;
  }

  return out;
}

static int normalizeAprsAlertMinSeconds(int seconds) {
  if (seconds < 0) return 0;
  if (seconds > 86400) return 86400;
  return seconds;
}

static void applyAprsAlertMinSeconds(int seconds) {
  aprsAlertMinSeconds = normalizeAprsAlertMinSeconds(seconds);
}

static float normalizeAprsAlertDistanceKm(float km) {
  if (km < 0.1f) return 0.1f;
  if (km > 500.0f) return 500.0f;
  return km;
}

static void applyAprsAlertDistanceKm(float km) {
  aprsAlertDistanceKm = normalizeAprsAlertDistanceKm(km);
}

static int normalizeAprsAlertScreenSeconds(int seconds) {
  if (seconds < 1) return 1;
  if (seconds > 60) return 60;
  return seconds;
}

static void applyAprsAlertScreenSeconds(int seconds) {
  aprsAlertScreenSeconds = normalizeAprsAlertScreenSeconds(seconds);
}

static String sanitizeAprsCallsignToken(const String &raw) {
  String token = raw;
  token.trim();
  token.toUpperCase();

  String cleaned = "";
  for (size_t i = 0; i < token.length(); i++) {
    char ch = token.charAt(i);
    bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '-' || ch == '/';
    if (ok) cleaned += ch;
  }
  return cleaned;
}

static String getAprsCallsignBase(const String &callsign) {
  String normalized = sanitizeAprsCallsignToken(callsign);
  int dash = normalized.indexOf('-');
  if (dash > 0) {
    normalized = normalized.substring(0, dash);
  }
  return normalized;
}

static bool isAprsAlertCallMatch(const String &watchedToken, const String &incomingCallsign) {
  if (watchedToken.length() == 0 || incomingCallsign.length() == 0) {
    return false;
  }

  String watched = sanitizeAprsAlertWatchToken(watchedToken);
  String incoming = sanitizeAprsCallsignToken(incomingCallsign);
  if (watched.length() == 0 || incoming.length() == 0) {
    return false;
  }

  if (watched.endsWith("*")) {
    String base = watched.substring(0, watched.length() - 1);
    if (base.length() == 0) {
      return false;
    }
    if (incoming.equals(base)) {
      return true;
    }
    String withSsidPrefix = base + "-";
    return incoming.startsWith(withSsidPrefix);
  }

  return watched.equals(incoming);
}

static bool isAprsCallsignOnAlertList(const String &incomingCallsign) {
  String src = aprsAlertCsv;
  src.trim();
  if (src.length() == 0) return false;

  int start = 0;
  while (start <= src.length()) {
    int comma = src.indexOf(',', start);
    int end = (comma >= 0) ? comma : src.length();
    String token = src.substring(start, end);
    token.trim();

    if (isAprsAlertCallMatch(token, incomingCallsign)) {
      return true;
    }

    if (comma < 0) break;
    start = comma + 1;
  }

  return false;
}

static bool isAprsMobileSsid7or9(const String &incomingCallsign) {
  String normalized = sanitizeAprsCallsignToken(incomingCallsign);
  if (normalized.length() == 0) {
    return false;
  }

  int dash = normalized.lastIndexOf('-');
  if (dash <= 0 || dash >= (normalized.length() - 1)) {
    return false;
  }

  String ssid = normalized.substring(dash + 1);
  return ssid.equals("7") || ssid.equals("9");
}

static void resetAprsAlertCooldownState() {
  for (int i = 0; i < MAX_APRS_STATIONS; i++) {
    aprsAlertCooldown[i].callsign = "";
    aprsAlertCooldown[i].lastAlertMs = 0;
  }
  aprsAlertCooldownReplaceIdx = 0;
}

static bool shouldTriggerAprsAlert(const APRSStation &station) {
  bool watchListMatched = isAprsCallsignOnAlertList(station.callsign);

  bool nearbyMobileMatched = false;
  if (aprsAlertNearbyEnabled && station.hasLatLon && station.distance >= 0.0f && station.distance <= aprsAlertDistanceKm) {
    nearbyMobileMatched = isAprsMobileSsid7or9(station.callsign);
  }

  bool weatherStationMatched = aprsAlertWxEnabled && isAprsWxPayloadValidForAlert(station);

  if (!watchListMatched && !nearbyMobileMatched && !weatherStationMatched) {
    return false;
  }

  String normalized = sanitizeAprsCallsignToken(station.callsign);
  if (normalized.length() == 0) {
    return false;
  }

  const unsigned long minGapMs = (unsigned long)aprsAlertMinSeconds * 1000UL;
  const unsigned long nowMs = millis();

  for (int i = 0; i < MAX_APRS_STATIONS; i++) {
    if (aprsAlertCooldown[i].callsign.equals(normalized)) {
      if (minGapMs > 0 && (nowMs - aprsAlertCooldown[i].lastAlertMs) < minGapMs) {
        return false;
      }
      aprsAlertCooldown[i].lastAlertMs = nowMs;
      return true;
    }
  }

  for (int i = 0; i < MAX_APRS_STATIONS; i++) {
    if (aprsAlertCooldown[i].callsign.length() == 0) {
      aprsAlertCooldown[i].callsign = normalized;
      aprsAlertCooldown[i].lastAlertMs = nowMs;
      return true;
    }
  }

  int replaceIdx = aprsAlertCooldownReplaceIdx;
  if (replaceIdx < 0 || replaceIdx >= MAX_APRS_STATIONS) {
    replaceIdx = 0;
  }
  aprsAlertCooldown[replaceIdx].callsign = normalized;
  aprsAlertCooldown[replaceIdx].lastAlertMs = nowMs;
  aprsAlertCooldownReplaceIdx = (replaceIdx + 1) % MAX_APRS_STATIONS;
  return true;
}

static uint8_t weatherStormRepeatMinutesFromEta(int etaMinutes) {
  if (etaMinutes <= 0) {
    return 1;
  }
  int hours = etaMinutes / 60;
  if ((etaMinutes % 60) != 0) {
    hours++;
  }
  if (hours < 1) {
    hours = 1;
  }
  if (hours > 6) {
    hours = 6;
  }
  return (uint8_t)hours;
}

static void queueWeatherAlertDraw(const WeatherAlertInfo &info) {
  portENTER_CRITICAL(&weatherAlertPendingMux);
  weatherAlertPendingInfo = info;
  weatherAlertDrawPending = true;
  portEXIT_CRITICAL(&weatherAlertPendingMux);
}

static void updateWeatherAlertForecastState(bool stormPossible,
                                            int stormEtaMinutes,
                                            bool windPossible,
                                            float maxWindMs,
                                            float maxWindGustMs) {
  bool stormChanged = (weatherStormForecastActive != stormPossible);
  bool windChanged = (weatherWindForecastActive != windPossible);

  weatherStormForecastActive = stormPossible;
  weatherWindForecastActive = windPossible;
  weatherAlertForecastUpdatedMs = millis();

  if (stormPossible) {
    if (stormEtaMinutes < 1) {
      stormEtaMinutes = 1;
    }
    if (stormEtaMinutes > 360) {
      stormEtaMinutes = 360;
    }
    weatherStormEtaMinutes = stormEtaMinutes;
    if (stormChanged) {
      nextWeatherStormAlertMs = 0;
    }
  } else {
    weatherStormEtaMinutes = 0;
    nextWeatherStormAlertMs = 0;
  }

  weatherWindForecastMaxMs = maxWindMs;
  weatherWindForecastGustMs = maxWindGustMs;
  if (!windPossible) {
    nextWeatherWindAlertMs = 0;
  } else if (windChanged) {
    nextWeatherWindAlertMs = 0;
  }
}

static void processWeatherAlertSchedule() {
  const bool stormTestMode = WEATHER_ALERT_STORM_TEST_MODE;
  const bool windForceMode = WEATHER_ALERT_WIND_FORCE_ENABLED;

  if (!(weatherAlertStormEnabled || stormTestMode) && !(weatherAlertWindEnabled || windForceMode)) {
    return;
  }
  if (!tftInitialized || brightnessMenuActive || currentScreen == SCREEN_UNLIS_HUNTER || aprsAlertScreenActive) {
    return;
  }

  unsigned long nowMs = millis();
  WeatherAlertInfo info;
  bool shouldNotify = false;

  bool stormNowDetected = false;
  if (weatherAlertStormEnabled && !stormTestMode) {
    int currentWeatherId = 800;
    bool currentWeatherValid = false;
    unsigned long currentWeatherFetchedAtMs = 0;

    lockWeatherData();
    currentWeatherId = weatherData.weatherId;
    currentWeatherValid = weatherData.valid;
    currentWeatherFetchedAtMs = weatherData.fetchedAtMs;
    unlockWeatherData();

    if (currentWeatherValid && currentWeatherId >= 200 && currentWeatherId < 300) {
      if (currentWeatherFetchedAtMs > 0 && (nowMs - currentWeatherFetchedAtMs) <= WEATHER_ALERT_CURRENT_DATA_MAX_AGE_MS) {
        stormNowDetected = true;
      }
    }
  }

  if ((weatherAlertStormEnabled && (weatherStormForecastActive || stormNowDetected)) || stormTestMode) {
    int elapsedMinutes = 0;
    if (weatherAlertForecastUpdatedMs > 0 && nowMs > weatherAlertForecastUpdatedMs) {
      elapsedMinutes = (int)((nowMs - weatherAlertForecastUpdatedMs) / 60000UL);
    }
    int etaMinutesNow = weatherStormEtaMinutes - elapsedMinutes;
    if (etaMinutesNow < 1) {
      etaMinutesNow = 1;
    }

    uint8_t repeatMinutes = (stormTestMode || stormNowDetected) ? 1 : weatherStormRepeatMinutesFromEta(etaMinutesNow);
    if (nextWeatherStormAlertMs == 0 || (long)(nowMs - nextWeatherStormAlertMs) >= 0) {
      info.stormPossible = true;
      info.stormNow = stormNowDetected;
      info.stormEtaHours = repeatMinutes;
      shouldNotify = true;
      nextWeatherStormAlertMs = nowMs + ((unsigned long)repeatMinutes * 60UL * 1000UL);
    }
  }

  if ((weatherAlertWindEnabled || windForceMode) && (weatherWindForecastActive || windForceMode)) {
    if (nextWeatherWindAlertMs == 0 || (long)(nowMs - nextWeatherWindAlertMs) >= 0) {
      float currentWindMs = 0.0f;
      lockWeatherData();
      currentWindMs = weatherData.windMs;
      unlockWeatherData();

      info.windPossible = true;
      info.maxWindMs = weatherWindForecastMaxMs;
      info.maxWindGustMs = weatherWindForecastGustMs;
      if (currentWindMs > info.maxWindMs) {
        info.maxWindMs = currentWindMs;
      }
      if (windForceMode && info.maxWindMs <= 0.0f) {
        info.maxWindMs = weatherAlertWindThresholdMs;
      }
      shouldNotify = true;
      nextWeatherWindAlertMs = nowMs + WEATHER_ALERT_WIND_REPEAT_MS;
    }
  }

  if (shouldNotify) {
    queueWeatherAlertDraw(info);
  }
}

static void applyAprsIntervalMinutes(int minutes) {
  int m = minutes;
  if (m < 1) m = 1;
  if (m > 180) m = 180; // sanity bound 1..180 min
  aprsIntervalMinutes = m;
  aprsPositionIntervalMs = (unsigned long)m * 60UL * 1000UL;
  nextAPRSPositionDueMs = 0; // przelicz harmonogram od nowa
}

bool isAprsTxCallValid(const String &call) {
  String c = call;
  c.trim();
  c.toUpperCase();
  if (c.length() == 0) return false;
  int dash = c.indexOf('-');
  if (dash > 0) {
    c = c.substring(0, dash);
  }
  return c.length() > 0 && c != "NOCALL";
}

void sendAprsPosition() {
  if (!aprsBeaconEnabled || !aprsConnected || !aprsClient.connected() || !aprsLoginSent) {
    return;
  }
  double txLat = 0.0;
  double txLon = 0.0;
  if (!getServiceCoordinatesForServices(txLat, txLon)) {
    Serial.println("[APRS] Pomijam TX pozycji - brak poprawnych współrzędnych stacji");
    return;
  }

  String txCallsign = getAprsTxCallsignWithSsid();
  txCallsign.trim();
  if (!isAprsTxCallValid(txCallsign)) {
    Serial.println("[APRS] Pomijam TX pozycji - ustaw znak w konfiguracji APRS");
    return;
  }
  txCallsign.toUpperCase();

  String latStr = formatAprsCoordinate(txLat, true);
  String lonStr = formatAprsCoordinate(txLon, false);

  String frame = txCallsign;
  frame += ">APRS,TCPIP*:";
  frame += "!";
  frame += latStr;
  frame += aprsSymbolTable;
  frame += lonStr;
  frame += aprsSymbolCode;
  frame += " ";
  bool useProjectComment = (((aprsBeaconTxCount + 1) % 5) == 0); // co piąty beacon
  String comment = useProjectComment ? String(APRS_POSITION_COMMENT)
                                     : sanitizeAprsComment(aprsUserComment);
  frame += comment;

  aprsClient.println(frame);
  lastAPRSPositionTxMs = millis();
  aprsBeaconTxCount++;

  Serial.print("[APRS] Wysłano pozycję: ");
  Serial.println(frame);
}

static bool parseAx25CallsignToken(const String &raw, String &baseOut, uint8_t &ssidOut) {
  String token = raw;
  token.trim();
  token.toUpperCase();

  if (token.endsWith("*")) {
    token.remove(token.length() - 1);
  }

  int dashPos = token.indexOf('-');
  String base = (dashPos >= 0) ? token.substring(0, dashPos) : token;
  String ssidText = (dashPos >= 0) ? token.substring(dashPos + 1) : "";

  String cleanedBase = "";
  cleanedBase.reserve(base.length());
  for (size_t i = 0; i < base.length(); i++) {
    char c = base.charAt(i);
    if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
      cleanedBase += c;
    }
  }

  if (cleanedBase.length() == 0) {
    return false;
  }
  if (cleanedBase.length() > 6) {
    cleanedBase = cleanedBase.substring(0, 6);
  }

  int ssid = 0;
  if (ssidText.length() > 0) {
    bool ssidValid = true;
    for (size_t i = 0; i < ssidText.length(); i++) {
      if (!isdigit((unsigned char)ssidText.charAt(i))) {
        ssidValid = false;
        break;
      }
    }
    if (!ssidValid) {
      return false;
    }
    ssid = ssidText.toInt();
    if (ssid < 0 || ssid > 15) {
      return false;
    }
  }

  baseOut = cleanedBase;
  ssidOut = (uint8_t)ssid;
  return true;
}

static bool appendAx25AddressField(uint8_t *buffer, size_t bufferSize, size_t &offset, const String &callsignToken, bool isLastField, bool markRepeated) {
  String baseCall = "";
  uint8_t ssid = 0;
  if (!parseAx25CallsignToken(callsignToken, baseCall, ssid)) {
    return false;
  }

  if ((offset + 7) > bufferSize) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    char c = (i < (int)baseCall.length()) ? baseCall.charAt(i) : ' ';
    buffer[offset + i] = (uint8_t)(((uint8_t)c) << 1);
  }

  uint8_t ssidByte = (uint8_t)(0x60 | ((ssid & 0x0F) << 1));
  if (markRepeated) {
    ssidByte |= 0x80;
  }
  if (isLastField) {
    ssidByte |= 0x01;
  }
  buffer[offset + 6] = ssidByte;
  offset += 7;
  return true;
}

static void writeKissEscapedByte(uint8_t b) {
  if (b == 0xC0) {
    gpsSerial.write((uint8_t)0xDB);
    gpsSerial.write((uint8_t)0xDC);
    return;
  }
  if (b == 0xDB) {
    gpsSerial.write((uint8_t)0xDB);
    gpsSerial.write((uint8_t)0xDD);
    return;
  }
  gpsSerial.write(b);
}

static bool sendAprsPositionToKiss() {
  if (!aprsBeaconEnabled || !isUart2AprsTncKissModuleSelected()) {
    return false;
  }

  double txLat = 0.0;
  double txLon = 0.0;
  if (!getServiceCoordinatesForServices(txLat, txLon)) {
    Serial.println("[KISS] Pomijam TX pozycji - brak poprawnych współrzędnych stacji");
    return false;
  }

  String txCallsign = getAprsTxCallsignWithSsid();
  txCallsign.trim();
  if (!isAprsTxCallValid(txCallsign)) {
    Serial.println("[KISS] Pomijam TX pozycji - ustaw znak w konfiguracji APRS");
    return false;
  }
  txCallsign.toUpperCase();

  String latStr = formatAprsCoordinate(txLat, true);
  String lonStr = formatAprsCoordinate(txLon, false);
  bool useProjectComment = (((aprsBeaconTxCount + 1) % 5) == 0); // co piąty beacon
  String comment = useProjectComment ? String(APRS_POSITION_COMMENT)
                                     : sanitizeAprsComment(aprsUserComment);
  String info = "!" + latStr + String(aprsSymbolTable) + lonStr + String(aprsSymbolCode) + " " + comment;

  uint8_t ax25Payload[360];
  size_t payloadLen = 0;

  if (!appendAx25AddressField(ax25Payload, sizeof(ax25Payload), payloadLen, "APRS", false, false)) {
    Serial.println("[KISS] Błąd budowy AX.25 (adres docelowy)");
    return false;
  }
  if (!appendAx25AddressField(ax25Payload, sizeof(ax25Payload), payloadLen, txCallsign, false, false)) {
    Serial.println("[KISS] Błąd budowy AX.25 (adres źródłowy)");
    return false;
  }
  // RF New-N: domyslna sciezka dla mobilnej stacji APRS.
  if (!appendAx25AddressField(ax25Payload, sizeof(ax25Payload), payloadLen, APRS_KISS_RF_PATH1, false, false)) {
    Serial.println("[KISS] Błąd budowy AX.25 (ścieżka 1)");
    return false;
  }
  if (!appendAx25AddressField(ax25Payload, sizeof(ax25Payload), payloadLen, APRS_KISS_RF_PATH2, true, false)) {
    Serial.println("[KISS] Błąd budowy AX.25 (ścieżka 2)");
    return false;
  }

  if ((payloadLen + 2) >= sizeof(ax25Payload)) {
    Serial.println("[KISS] Błąd budowy AX.25 (za mały bufor nagłówka)");
    return false;
  }
  ax25Payload[payloadLen++] = 0x03; // UI frame
  ax25Payload[payloadLen++] = 0xF0; // no layer 3

  for (size_t i = 0; i < info.length(); i++) {
    if (payloadLen >= sizeof(ax25Payload)) {
      Serial.println("[KISS] Błąd budowy AX.25 (za długi payload)");
      return false;
    }
    ax25Payload[payloadLen++] = (uint8_t)info.charAt(i);
  }

  gpsSerial.write((uint8_t)0xC0);
  gpsSerial.write((uint8_t)0x00); // KISS port 0, command Data
  for (size_t i = 0; i < payloadLen; i++) {
    writeKissEscapedByte(ax25Payload[i]);
  }
  gpsSerial.write((uint8_t)0xC0);

  lastAPRSPositionTxMs = millis();
  aprsBeaconTxCount++;

  String pathInfo = String(APRS_KISS_RF_PATH1) + "," + String(APRS_KISS_RF_PATH2);
  String positionInfo = "!" + latStr + String(aprsSymbolTable) + lonStr + String(aprsSymbolCode);
  String tnc2Preview = txCallsign + ">APRS," + pathInfo + ":" + info;
  Serial.print("[KISS] Wysłano beacon: ");
  Serial.println(tnc2Preview);

  APRS_BEACONING_Screen(txCallsign, positionInfo, pathInfo, tnc2Preview);
  return true;
}

static void serviceAprsBeaconTx(unsigned long nowAprsMs) {
  bool kissTxMode = isUart2AprsTncKissModuleSelected();
  bool aprsTxCoordsReady = false;
  double aprsTxLat = 0.0;
  double aprsTxLon = 0.0;
  aprsTxCoordsReady = getServiceCoordinatesForServices(aprsTxLat, aprsTxLon);

  bool txChannelReady = false;
  if (kissTxMode) {
    txChannelReady = true;
  } else {
    txChannelReady = isAprsNetworkFeedEnabled() && aprsConnected && aprsClient.connected() && aprsLoginSent;
  }

  if (aprsBeaconEnabled && aprsTxCoordsReady && txChannelReady) {
    if (nextAPRSPositionDueMs == 0) {
      nextAPRSPositionDueMs = nowAprsMs + APRS_POSITION_FIRST_DELAY_MS;
    } else if ((long)(nowAprsMs - nextAPRSPositionDueMs) >= 0) {
      if (kissTxMode) {
        sendAprsPositionToKiss();
      } else {
        sendAprsPosition();
      }
      nextAPRSPositionDueMs = nowAprsMs + aprsPositionIntervalMs;
    }
  } else {
    nextAPRSPositionDueMs = 0; // reset harmonogramu gdy beacon wyłączony lub brak warunków TX
    aprsBeaconTxCount = 0;
  }
}

// Parsuj ramkÄ™ APRS
// Format przykĹ‚adowy: SP3KON-1>APRS,TCPIP*,qAC,T2POLAND:!5202.40N/01655.12E#PHG5130/Poznan
bool parseAPRSFrame(String line, APRSStation &station) {
  LOGV_PRINTF("[APRS] Parsing frame: %s\n", line.c_str());
  
  // WyczyĹ›Ä‡ strukturÄ™
  station.time = "";
  station.callsign = "";
  station.symbol = "";
  station.symbolTable = "";
  station.lat = 0.0f;
  station.lon = 0.0f;
  station.comment = "";
  station.freqMHz = 0.0f;
  station.distance = 0.0f;
  station.hasLatLon = false;
  
  // Pobierz czas UTC
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
    station.time = String(timeBuffer);
  } else {
    station.time = "----Z";
  }
  
  // Parsuj callsign (przed >)
  int gtPos = line.indexOf('>');
  if (gtPos < 0) {
    LOGV_PRINTLN("[APRS] Brak znaku '>' w ramce - nieprawidĹ‚owy format");
    return false;
  }
  station.callsign = line.substring(0, gtPos);
  station.callsign.trim();
  LOGV_PRINTF("[APRS] Callsign: %s\n", station.callsign.c_str());

  // Najpierw sprĂłbuj dekodowaÄ‡ Mic-E / skompresowane / obiekty z gotowego parsera
  bool advancedParsed = parseAprsAdvancedPosition(line, station);
  if (advancedParsed) {
    LOGV_PRINTF("[APRS] Advanced decode lat/lon: %.6f, %.6f\n", station.lat, station.lon);
  }
  
  // Parsuj pozycjÄ™ GPS (prosty parser) tylko jeĹ›li advanced nie zadziaĹ‚aĹ‚
  // Szukaj formatu: !5202.40N/01655.12E, @5202.40N/01655.12E, =5202.40N/01655.12E, lub ;...*HHMMzDDMM.mmN/DDDMM.mmE
  int posStart = -1;
  char tableSymbol = 0;
  int commentBeforePos = -1; // Pozycja poczÄ…tku komentarza przed timestampem (dla formatu z ;)
  
  if (!advancedParsed) {
  // Szukaj standardowych formatĂłw pozycji (!, @, =)
  posStart = line.indexOf('!');
  if (posStart >= 0) {
    tableSymbol = '!';
  } else {
    posStart = line.indexOf('@');
    if (posStart >= 0) {
      tableSymbol = '@';
    } else {
      posStart = line.indexOf('=');
      if (posStart >= 0) {
        tableSymbol = '=';
      }
    }
  }
  
  // JeĹ›li nie znaleziono standardowych formatĂłw, szukaj w komentarzu (format z ;)
  if (posStart < 0) {
    int colonPos = line.indexOf(':');
    if (colonPos >= 0) {
      String commentPart = line.substring(colonPos + 1);
      // Szukaj formatu z ; i timestampem: ;...*HHMMzDDMM.mmN/DDDMM.mmE
      int semicolonPos = commentPart.indexOf(';');
      if (semicolonPos >= 0) {
        // Zapisz pozycjÄ™ poczÄ…tku komentarza przed timestampem
        commentBeforePos = colonPos + 1 + semicolonPos + 1; // +1 bo colonPos, +1 bo po ';'
        // Szukaj timestampu *HHMMz
        int timestampPos = commentPart.indexOf('*', semicolonPos);
        if (timestampPos >= 0 && timestampPos + 7 < commentPart.length()) {
          // SprawdĹş czy po timestampie jest pozycja (format: *HHMMzDDMM.mmN)
          char zChar = commentPart.charAt(timestampPos + 6);
          if (zChar == 'z' || zChar == 'Z') {
            // Pozycja zaczyna siÄ™ po 'z'
            posStart = colonPos + 1 + timestampPos + 7; // +1 bo colonPos, +timestampPos, +7 bo "*HHMMz"
            tableSymbol = ';';
          }
        }
      }
    }
  }
  
  if (posStart < 0) {
    // Brak pozycji - sprĂłbuj wyciÄ…gnÄ…Ä‡ tylko callsign i komentarz
    LOGV_PRINTLN("[APRS] Brak pozycji GPS w ramce");
    int colonPos = line.indexOf(':');
    if (colonPos >= 0) {
      station.comment = line.substring(colonPos + 1);
      station.comment.trim();
    }
    return true; // ZwrĂłÄ‡ true nawet bez pozycji
  }
  
  LOGV_PRINTF("[APRS] Znaleziono pozycjÄ™ na indeksie %d, symbol tabeli: %c\n", posStart, tableSymbol);
  
  // Symbol table ustawimy po wyciÄ…gniÄ™ciu pozycji
  
  // Wyznacz poczÄ…tek pozycji (lat) zaleĹĽnie od formatu
  bool posStartIsSymbol = (tableSymbol == '!' || tableSymbol == '@' || tableSymbol == '=' || tableSymbol == '/');
  int latStart = posStartIsSymbol ? (posStart + 1) : posStart; // dla ';' posStart wskazuje juĹĽ na lat

  // ObsĹ‚uga timestampu dla @ lub /: @DDHHMMh... lub /DDHHMMh...
  if (tableSymbol == '@' || tableSymbol == '/') {
    if (posStart + 7 < line.length()) {
      bool tsDigits = true;
      for (int i = 1; i <= 6; i++) {
        char c = line.charAt(posStart + i);
        if (!isDigit(c)) {
          tsDigits = false;
          break;
        }
      }
      char tsChar = line.charAt(posStart + 7);
      if (tsDigits && (tsChar == 'h' || tsChar == 'z' || tsChar == '/')) {
        latStart = posStart + 8;
      }
    }
  }

  // SprawdĹş czy jest format z "/" czy bez "/"
  int slashPos = line.indexOf('/', latStart);
  bool hasSlash = (slashPos >= 0);
  
  bool latOk = false;
  bool lonOk = false;

  if (hasSlash) {
    // Format z "/": !DDMM.mmN/DDDMM.mmEsymbol
    // Parsuj szerokoĹ›Ä‡ geograficznÄ…: 5202.40N (format: DDMM.mmN)
    if (latStart + 7 < line.length()) {
      String latStr = line.substring(latStart, latStart + 7);
      if (latStr.length() == 7) {
        char dir = line.charAt(latStart + 7);
        // UĹĽyj funkcji convertToDecimal do parsowania
        float parsedLat = convertToDecimal(latStr, dir);
        if (!isnan(parsedLat)) {
          station.lat = parsedLat;
          latOk = true;
        }
        
        LOGV_PRINTF("[APRS] Parsed lat: %s%c -> %.6f\n", 
                    latStr.c_str(), dir, station.lat);
      }
    }
    
    // Parsuj dĹ‚ugoĹ›Ä‡ geograficznÄ…: 01655.12E (format: DDDMM.mmE)
    if (slashPos >= 0 && slashPos + 9 < line.length()) {
      String lonStr = line.substring(slashPos + 1, slashPos + 9);
      if (lonStr.length() == 8) {
        char dir = line.charAt(slashPos + 9);
        // UĹĽyj funkcji convertToDecimal do parsowania
        float parsedLon = convertToDecimal(lonStr, dir);
        if (!isnan(parsedLon)) {
          station.lon = parsedLon;
          lonOk = true;
        }
        
        LOGV_PRINTF("[APRS] Parsed lon: %s%c -> %.6f\n", 
                    lonStr.c_str(), dir, station.lon);
      }
    }
    
    // Symbol table (znak przed /)
    if (slashPos >= 0) {
      station.symbolTable = line.substring(slashPos, slashPos + 1);
    }
    // WyciÄ…gnij symbol code (znak po dĹ‚ugoĹ›ci geograficznej, po "/")
    // Format: !DDMM.mmN/DDDMM.mmEsymbol
    if (slashPos >= 0 && slashPos + 10 < line.length()) {
      station.symbol = line.substring(slashPos + 10, slashPos + 11);
    }
  } else {
    // Format bez "/": DDMM.mmNsymbolDDDMM.mmE (symbol miÄ™dzy lat a lon)
    // PrzykĹ‚ad: 5223.73NW01655.41E lub 5225.05NL01651.66E
    if (latStart + 7 < line.length()) {
      String latStr = line.substring(latStart, latStart + 7);
      if (latStr.length() == 7) {
        char dir = line.charAt(latStart + 7);
        // UĹĽyj funkcji convertToDecimal do parsowania
        float parsedLat = convertToDecimal(latStr, dir);
        if (!isnan(parsedLat)) {
          station.lat = parsedLat;
          latOk = true;
        }
        
        LOGV_PRINTF("[APRS] Parsed lat: %s%c -> %.6f\n", 
                    latStr.c_str(), dir, station.lat);
        
        // Symbol table jest zaraz po kierunku lat
        if (latStart + 8 < line.length()) {
          station.symbolTable = line.substring(latStart + 8, latStart + 9);
        }
        
        // DĹ‚ugoĹ›Ä‡ geograficzna zaczyna siÄ™ po symbolu table
        int lonStart = latStart + 9;
        if (lonStart + 7 < line.length()) {
          String lonStr = line.substring(lonStart, lonStart + 8);
          if (lonStr.length() == 8) {
            char dir = line.charAt(lonStart + 8);
            // UĹĽyj funkcji convertToDecimal do parsowania
            float parsedLon = convertToDecimal(lonStr, dir);
            if (!isnan(parsedLon)) {
              station.lon = parsedLon;
              lonOk = true;
            }
            
            LOGV_PRINTF("[APRS] Parsed lon: %s%c -> %.6f\n", 
                        lonStr.c_str(), dir, station.lon);

            // Symbol code jest po kierunku lon (jeĹ›li wystÄ™puje)
            if (lonStart + 9 < line.length()) {
              station.symbol = line.substring(lonStart + 9, lonStart + 10);
            }
          }
        }
      }
    }
  }
  
  // Ustaw flagÄ™ poprawnoĹ›ci pozycji tylko jeĹ›li mamy lat i lon
  station.hasLatLon = (latOk && lonOk);

  // WyciÄ…gnij komentarz (po symbolu i pozycji)
  // Format z "/": !5202.40N/01655.12E#symbol/komentarz lub !5202.40N/01655.12E#symbol komentarz
  // Format bez "/": DDMM.mmNsymbolDDDMM.mmE&komentarz
  // Format z ";": ;komentarz_przed*HHMMzDDMM.mmNsymbolDDDMM.mmE&komentarz_po
  int colonPos = line.indexOf(':');
  if (colonPos >= 0 && colonPos + 1 < line.length()) {
    String fullComment = "";
    
    if (tableSymbol == ';' && commentBeforePos >= 0) {
      // Format z ";": komentarz skĹ‚ada siÄ™ z czÄ™Ĺ›ci przed timestampem i po pozycji
      // CzÄ™Ĺ›Ä‡ przed timestampem
      int timestampStart = line.indexOf('*', commentBeforePos);
      if (timestampStart > commentBeforePos) {
        fullComment = line.substring(commentBeforePos, timestampStart);
      }
      
      // CzÄ™Ĺ›Ä‡ po pozycji
      int commentAfterPos = -1;
      if (hasSlash) {
        int afterSymbol = slashPos + 11;
        if (afterSymbol < line.length()) {
          char c = line.charAt(afterSymbol);
          if (c == '&' || c == '#' || c == '/') {
            commentAfterPos = afterSymbol + 1;
          } else {
            commentAfterPos = afterSymbol;
          }
        }
      } else {
        // Format bez "/": komentarz zaczyna siÄ™ po lon
        int afterLon = latStart + 18;
        if (afterLon < line.length()) {
          char c = line.charAt(afterLon);
          if (c == '&' || c == '#' || c == '/') {
            commentAfterPos = afterLon + 1;
          } else {
            commentAfterPos = afterLon;
          }
        }
      }
      
      if (commentAfterPos >= 0 && commentAfterPos < line.length()) {
        if (fullComment.length() > 0) {
          fullComment += " ";
        }
        fullComment += line.substring(commentAfterPos);
      }
      
      station.comment = fullComment;
      station.comment.trim();
    } else {
      // Standardowe formaty (!, @, =)
      int commentStart = colonPos + 1;
      
      if (hasSlash && slashPos >= 0) {
        // Format z "/": komentarz zaczyna siÄ™ po symbolu
        int afterSymbol = slashPos + 11;
        if (afterSymbol < line.length()) {
          char c = line.charAt(afterSymbol);
          if (c == '#' || c == '/') {
            commentStart = afterSymbol + 1;
          } else {
            // Komentarz zaczyna siÄ™ zaraz po symbolu
            commentStart = afterSymbol;
          }
        }
      } else {
        // Format bez "/": komentarz zaczyna siÄ™ po lon (po kierunku E/W)
        int afterLon = latStart + 18;
        if (afterLon < line.length()) {
          char c = line.charAt(afterLon);
          if (c == '&' || c == '#' || c == '/') {
            commentStart = afterLon + 1;
          } else {
            // Komentarz zaczyna siÄ™ zaraz po lon
            commentStart = afterLon;
          }
        }
      }
      
      // JeĹ›li commentStart jest przed koĹ„cem linii, wyciÄ…gnij komentarz
      if (commentStart < line.length() && commentStart >= colonPos + 1) {
        station.comment = line.substring(commentStart);
        station.comment.trim();
      } else if (commentStart < colonPos + 1) {
        // JeĹ›li nie znaleziono komentarza po pozycji, uĹĽyj caĹ‚ej czÄ™Ĺ›ci po ":"
        station.comment = line.substring(colonPos + 1);
        station.comment.trim();
      }
    }
  }
  } // !advancedParsed
  
  // SprĂłbuj wyciÄ…gnÄ…Ä‡ czÄ™stotliwoĹ›Ä‡ z komentarza
  if (station.comment.length() > 0) {
    float freq = 0.0f;
    if (extractAPRSFrequencyMHz(station.comment, freq)) {
      station.freqMHz = freq;
    }
  }
  
  // Oblicz odlegĹ‚oĹ›Ä‡ jeĹ›li mamy pozycjÄ™
  // UĹĽywamy wspĂłĹ‚rzÄ™dnych z sekcji "Moja Stacja"
  if (station.hasLatLon) {
    // UĹĽyj double dla wiÄ™kszej precyzji obliczeĹ„
    double userLatForDistance = 52.40;  // Fallback jeśli nie ustawione
    double userLonForDistance = 16.92;
    getServiceCoordinatesForServices(userLatForDistance, userLonForDistance);
    double stationLat = (double)station.lat;
    double stationLon = (double)station.lon;
    
    station.distance = calculateDistance(userLatForDistance, userLonForDistance, 
                                         stationLat, stationLon);
    LOGV_PRINTF("[APRS] Distance: user(%.6f,%.6f) -> station(%.6f,%.6f) = %.1f km\n",
                userLatForDistance, userLonForDistance, stationLat, stationLon, station.distance);
  }
  
  return true;
}

// Dodaj nowÄ… stacjÄ™ APRS do tablicy (bez duplikatĂłw - aktualizuje istniejÄ…ce)
void addAPRSStation(APRSStation station) {
  compactAprsStationStrings(station);
  LOGV_PRINT("[APRS] addAPRSStation: ");
  LOGV_PRINT(station.callsign);
  if (station.hasLatLon) {
    LOGV_PRINT(" @ ");
    if (LOG_VERBOSE) Serial.print(station.lat, 4);
    LOGV_PRINT(",");
    if (LOG_VERBOSE) Serial.print(station.lon, 4);
    LOGV_PRINT(" (");
    if (LOG_VERBOSE) Serial.print(station.distance, 1);
    LOGV_PRINT(" km)");
  }
  LOGV_PRINTLN();
  
  // SprawdĹş czy stacja juĹĽ istnieje (po callsign)
  int existingIndex = -1;
  for (int i = 0; i < aprsStationCount; i++) {
    if (aprsStations[i].callsign.equalsIgnoreCase(station.callsign)) {
      existingIndex = i;
      break;
    }
  }
  
  if (existingIndex >= 0) {
    // Stacja juĹĽ istnieje - usuĹ„ jÄ… z obecnej pozycji
    for (int i = existingIndex; i > 0; i--) {
      aprsStations[i] = aprsStations[i - 1];
    }
    // Zaktualizuj dane stacji (nowy czas, pozycja, itp.)
    aprsStations[0] = station;
    LOGV_PRINT("[APRS] Stacja juĹĽ istnieje - zaktualizowano na pozycji 0");
  } else {
    // Nowa stacja - dodaj na poczÄ…tku
    if (aprsStationCount < MAX_APRS_STATIONS) {
      aprsStationCount++;
    }
    
    // PrzesuĹ„ wszystkie stacje o jeden w dĂłĹ‚
    for (int i = MAX_APRS_STATIONS - 1; i > 0; i--) {
      aprsStations[i] = aprsStations[i - 1];
    }
    
    // Dodaj nowÄ… stacjÄ™ na poczÄ…tku
    aprsStations[0] = station;
    LOGV_PRINT("[APRS] Dodano nowÄ… stacjÄ™");
  }
  
  LOGV_PRINT("[APRS] addAPRSStation END, nowy aprsStationCount=");
  LOGV_PRINTLN(aprsStationCount);
}

static void processAprsFrameFromLine(const String &line, const char *sourceTag) {
  APRSStation station;
  if (!parseAPRSFrame(line, station)) {
    return;
  }

  addAPRSStation(station);

  // While Unlis Hunter or TFT settings screen is active,
  // suppress APRS alert popups and LED/buzzer alerts.
  if (aprsAlertEnabled && currentScreen != SCREEN_UNLIS_HUNTER && !brightnessMenuActive && shouldTriggerAprsAlert(station)) {
    Serial.print("[APRS ALERT] stacja wykryta (");
    Serial.print(sourceTag);
    Serial.print("): ");
    Serial.println(line);
    bool isWxStation = aprsAlertWxEnabled && isAprsWxPayloadValidForAlert(station);
    if (!isWxStation) {
      triggerAprsRgbLedAlert();
    }
    #ifdef ENABLE_TFT_DISPLAY
    portENTER_CRITICAL(&aprsAlertPendingMux);
    aprsAlertPendingStation = station;
    aprsAlertDrawPending = true;
    portEXIT_CRITICAL(&aprsAlertPendingMux);
    #endif
  }
}

// ObsĹ‚uga danych z APRS-IS
void handleAPRSData() {
  if (!aprsConnected || !aprsClient.connected()) {
    if (aprsConnected) {
      LOGV_PRINTLN("[APRS] PoĹ‚Ä…czenie zerwane - reset flagĂłw");
    }
    aprsConnected = false;
    aprsLoginSent = false;
    return;
  }
  
  // Odczytaj dostÄ™pne dane
  while (aprsClient.available()) {
    unsigned char c = (unsigned char)aprsClient.read();
    lastAPRSRxMs = millis();
    
    if (c == '\n' || c == '\r') {
      if (aprsBuffer.length() > 0) {
        String line = aprsBuffer;
        aprsBuffer = "";
        
        // Ignoruj linie zaczynajÄ…ce siÄ™ od # (komentarze serwera)
        if (line.startsWith("#")) {
          Serial.print("[APRS] Server: ");
          Serial.println(line);
          continue;
        }
        
        processAprsFrameFromLine(line, "APRS-IS");
      }
    } else if (c != 0) {
      // Keep control bytes (e.g. 0x1C/0x1D) for Mic-E decode compatibility.
      aprsBuffer += (char)c;
      // Ograniczenie dĹ‚ugoĹ›ci bufora (zapobieganie przepeĹ‚nieniu)
      if (aprsBuffer.length() > 512) {
        aprsBuffer = aprsBuffer.substring(aprsBuffer.length() - 256);
      }
    }
  }
}

// ========== WIFI MANAGER ==========

void startAPMode() {
  // Stabilny portal konfiguracyjny: sam AP (bez STA w tle),
  // ĹĽeby nie gubiÄ‡ poĹ‚Ä…czenia na telefonie/PC przy zmianie kanaĹ‚u przez STA.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  // Odmierzaj ponowną próbę STA od momentu wejścia do AP.
  lastWiFiReconnectAttempt = millis();
  Serial.println("AP Mode uruchomiony");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  
  // Aktualizuj wyĹ›wietlacz TFT z IP AP (jeĹ›li jesteĹ›my na ekranie 1)
#ifdef ENABLE_TFT_DISPLAY
  updateScreen1();
#endif
}

void requestRestart(unsigned long delayMs = 1500) {
  restartRequested = true;
  restartAtMs = millis() + delayMs;
}

bool connectToWiFi() {
  return connectToWiFiWithAttempts(40, true);
}

bool connectToWiFiWithAttempts(uint8_t maxAttemptsPerSsid, bool runDiagnostics) {
  if (wifiSSID.length() == 0 && wifiSSID2.length() == 0) {
    return false;
  }

  struct WiFiCred {
    String ssid;
    String pass;
  };

  WiFiCred candidates[2] = {{wifiSSID, wifiPassword}, {wifiSSID2, wifiPassword2}};

  auto attemptConnect = [maxAttemptsPerSsid, runDiagnostics](const WiFiCred &cred) -> bool {
    Serial.println("=== WiFi connect ===");
    Serial.print("SSID: '");
    Serial.print(cred.ssid);
    Serial.print("' (len=");
    Serial.print(cred.ssid.length());
    Serial.println(")");
    Serial.print("PASS len=");
    Serial.println(cred.pass.length());

    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(250);

    WiFi.mode(WIFI_STA);
    delay(50);
    WiFi.begin(cred.ssid.c_str(), cred.pass.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttemptsPerSsid) {
      delay(500);
      yield();
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      return true;
    }

    Serial.println("");
    Serial.println("BĹ‚Ä…d poĹ‚Ä…czenia WiFi");
    Serial.print("WiFi.status(): ");
    Serial.println((int)WiFi.status());

    if (runDiagnostics) {
      Serial.println("--- WiFi diag ---");
      WiFi.printDiag(Serial);
      Serial.println("--- Scan networks ---");
      int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
      Serial.print("Znaleziono sieci: ");
      Serial.println(n);
      bool found = false;
      for (int i = 0; i < n; i++) {
        String s = WiFi.SSID(i);
        int32_t rssi = WiFi.RSSI(i);
        wifi_auth_mode_t enc = WiFi.encryptionType(i);
        if (s == cred.ssid) {
          found = true;
          Serial.print("MATCH SSID: ");
          Serial.print(s);
          Serial.print(" RSSI=");
          Serial.print(rssi);
          Serial.print(" enc=");
          Serial.println((int)enc);
        }
      }
      if (!found) {
        Serial.println("UWAGA: Nie widzÄ™ Twojego SSID w skanie (moĹĽe 5GHz / inny SSID / poza zasiÄ™giem / ukryte SSID).");
      }
    }

    return false;
  };

  for (int i = 0; i < 2; i++) {
    const WiFiCred &cred = candidates[i];
    if (cred.ssid.length() == 0) {
      continue;
    }

    if (i == 1 && cred.ssid == candidates[0].ssid) {
      continue; // avoid duplicate attempt
    }

    bool ok = attemptConnect(cred);
    if (ok) {
      wifiConnected = true;
      Serial.print("PoĹ‚Ä…czono z SSID: ");
      Serial.println(cred.ssid);
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());

      // Aktualizuj wyĹ›wietlacz TFT z IP STA (jeĹ›li jesteĹ›my na ekranie 1)
#ifdef ENABLE_TFT_DISPLAY
      updateScreen1();
#endif

      return true;
    } else {
      Serial.println("PrĂłba poĹ‚Ä…czenia nieudana dla SSID: " + cred.ssid);
    }
  }

  wifiConnected = false;
  return false;
}

void retryWiFiFromAPIfDue(unsigned long now) {
  if (wifiConnected) {
    return;
  }

  if (wifiSSID.length() == 0 && wifiSSID2.length() == 0) {
    return;
  }

  // Retry uruchamiaj tylko wtedy, gdy urządzenie realnie działa jako AP.
  if ((WiFi.getMode() & WIFI_AP) == 0) {
    return;
  }

  if (lastWiFiReconnectAttempt != 0 && (now - lastWiFiReconnectAttempt) < WIFI_AP_RETRY_INTERVAL_MS) {
    return;
  }

  lastWiFiReconnectAttempt = now;
  Serial.println("[WIFI] AP retry timer elapsed -> trying STA (SSID1/SSID2)");

  if (connectToWiFi()) {
    Serial.println("[WIFI] STA reconnect successful");
    return;
  }

  Serial.println("[WIFI] STA reconnect failed, returning to AP mode");
  startAPMode();
}

void retryWiFiFromSTAIfDue(unsigned long now) {
  if (wifiConnected) {
    wifiStaRetryWindowStartMs = 0;
    return;
  }

  if (wifiSSID.length() == 0 && wifiSSID2.length() == 0) {
    return;
  }

  // W AP retry obsługuje osobna funkcja.
  if (WiFi.getMode() & WIFI_AP) {
    return;
  }

  if (wifiStaRetryWindowStartMs == 0) {
    wifiStaRetryWindowStartMs = now;
  }

  if ((now - wifiStaRetryWindowStartMs) >= WIFI_STA_RETRY_WINDOW_MS) {
    Serial.println("[WIFI] STA retry window (10 min) elapsed -> switching to AP mode");
    wifiStaRetryWindowStartMs = 0;
    startAPMode();
    return;
  }

  if (lastWiFiStaReconnectAttempt != 0 && (now - lastWiFiStaReconnectAttempt) < WIFI_STA_RETRY_INTERVAL_MS) {
    return;
  }

  lastWiFiStaReconnectAttempt = now;
  Serial.println("[WIFI] STA offline -> retry connect (SSID1/SSID2)");
  if (connectToWiFiWithAttempts(WIFI_STA_RECONNECT_ATTEMPTS, false)) {
    Serial.println("[WIFI] STA reconnect successful");
    wifiStaRetryWindowStartMs = 0;
    updateNTPTime();
  }
}

// ========== TELNET CLUSTER ==========

void connectToCluster() {
  Serial.println("[CLUSTER] connectToCluster() START");
  
  if (!wifiConnected) {
    Serial.println("[CLUSTER] WiFi nie połączony - wyjście");
    return;
  }
  
  if (telnetClient.connected()) {
    Serial.println("[CLUSTER] Już połączony - wyjście");
    return;
  }
  
  unsigned long now = millis();
  if (now - lastTelnetAttempt < 5000) {
    return; // Cichy return - nie spamuj logu (to byĹ‚o gĹ‚Ăłwne ĹşrĂłdĹ‚o spamu!)
  }
  
  lastTelnetAttempt = now;
  
  Serial.print("[CLUSTER] Łączenie z DX Cluster: ");
  Serial.print(clusterHost);
  Serial.print(":");
  Serial.println(clusterPort);

  // Diagnostyka DNS (czÄ™sty problem gdy host nie rozwiÄ…zuje siÄ™ na ESP32)
  IPAddress resolvedIp;
  if (WiFi.hostByName(clusterHost.c_str(), resolvedIp)) {
    Serial.print("DNS OK: ");
    Serial.print(clusterHost);
    Serial.print(" -> ");
    Serial.println(resolvedIp);
  } else {
    Serial.print("DNS FAIL dla hosta: ");
    Serial.println(clusterHost);
  }
  
  Serial.println("[CLUSTER] Wywołanie telnetClient.connect()...");
  unsigned long connectStart = millis();
  if (telnetClient.connect(clusterHost.c_str(), clusterPort)) {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[CLUSTER] Połączono z DX Cluster! (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    telnetConnected = true;
    telnetBuffer = "";
    clusterLoginSent = false;
    clusterLoginScheduled = false;
    lastClusterKeepAliveMs = millis();
    lastTelnetRxMs = millis();

    // Zawsze wysyĹ‚amy login po poĹ‚Ä…czeniu (tak jak w projekcie referencyjnym).
    // JeĹ›li uĹĽytkownik nie ustawiĹ‚ znaku, uĹĽywamy domyĹ›lnego (DEFAULT_CALLSIGN).
    clusterLoginScheduled = true;
    clusterSendLoginAtMs = millis() + 1000;
  } else {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[CLUSTER] Błąd połączenia z DX Cluster (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    telnetConnected = false;
  }
  Serial.println("[CLUSTER] connectToCluster() END");
}

void connectToPotaCluster() {
  if (!wifiConnected) {
    return;
  }
  if (potaClusterHost.length() == 0 || potaClusterPort <= 0) {
    return;
  }
  if (potaTelnetClient.connected()) {
    return;
  }

  unsigned long now = millis();
  if (now - lastPotaAttempt < 5000) {
    return;
  }
  lastPotaAttempt = now;

  Serial.print("[POTA] Łączenie z POTA Cluster: ");
  Serial.print(potaClusterHost);
  Serial.print(":");
  Serial.println(potaClusterPort);

  IPAddress resolvedIp;
  if (WiFi.hostByName(potaClusterHost.c_str(), resolvedIp)) {
    Serial.print("[POTA] DNS OK: ");
    Serial.print(potaClusterHost);
    Serial.print(" -> ");
    Serial.println(resolvedIp);
  } else {
    Serial.print("[POTA] DNS FAIL: ");
    Serial.println(potaClusterHost);
  }

  if (potaTelnetClient.connect(potaClusterHost.c_str(), potaClusterPort)) {
    Serial.println("[POTA] Połączono z POTA Cluster!");
    potaTelnetConnected = true;
    potaTelnetBuffer = "";
    pendingPotaLine = "";
    potaLoginSent = false;
    potaLoginScheduled = false;
    lastPotaKeepAliveMs = millis();
    lastPotaRxMs = millis();

    String login = userCallsign;
    login.trim();
    if (login.length() == 0) {
      login = DEFAULT_CALLSIGN;
    }
    if (userLocator.length() >= 4) {
      login += "/";
      login += userLocator;
    }
    potaTelnetClient.print(login);
    potaTelnetClient.print("\r\n");
    potaLoginSent = true;
    lastPotaKeepAliveMs = millis();

    if (potaFilterCommand.length() > 0) {
      potaTelnetClient.print(potaFilterCommand);
      potaTelnetClient.print("\r\n");
    }
  } else {
    Serial.println("[POTA] Błąd połączenia z POTA Cluster");
    potaTelnetConnected = false;
  }
}

// WyĹ›lij komendÄ™ konfiguracyjnÄ… do DX Cluster (CC-Cluster)
// UWAGA: UĹĽywane TYLKO do konfiguracji odbioru (set/noann, set/nowwv, set/filter, etc.)
// NIE wysyĹ‚a ĹĽadnych spotĂłw - urzÄ…dzenie dziaĹ‚a tylko w trybie odbioru
void sendClusterCommand(String command) {
  if (!telnetConnected || !telnetClient.connected()) {
    Serial.print("[CLUSTER] Nie mo\u017Cna wys\u0142a\u0107 komendy '");
    Serial.print(command);
    Serial.println("' - brak połączenia");
    return;
  }
  
  Serial.print("[CLUSTER] Wysy\u0142anie komendy: ");
  Serial.println(command);
  telnetClient.print(command);
  telnetClient.print("\r\n");
  delay(50); // KrĂłtkie opĂłĹşnienie miÄ™dzy komendami
}

// WyĹ›lij komendy konfiguracyjne CC-Cluster po zalogowaniu
void sendClusterConfigCommands() {
  if (!telnetConnected || !telnetClient.connected()) {
    return;
  }
  
  Serial.println("[CLUSTER] Wysy\u0142anie komend konfiguracyjnych CC-Cluster...");
  
  // WyĹ‚Ä…cz ogĹ‚oszenia (domyĹ›lnie wĹ‚Ä…czone)
  if (clusterNoAnnouncements) {
    sendClusterCommand("set/noann");
  }
  
  // WyĹ‚Ä…cz WWV (domyĹ›lnie wĹ‚Ä…czone)
  if (clusterNoWWV) {
    sendClusterCommand("set/nowwv");
  }
  
  // WyĹ‚Ä…cz WCY (domyĹ›lnie wĹ‚Ä…czone)
  if (clusterNoWCY) {
    sendClusterCommand("set/nowcy");
  }
  
  // Ustaw filtry (jeĹ›li wĹ‚Ä…czone)
  if (clusterUseFilters && clusterFilterCommands.length() > 0) {
    // Parsuj i wyĹ›lij komendy filtrĂłw (moĹĽe byÄ‡ kilka linii oddzielonych \n)
    String filters = clusterFilterCommands;
    int pos = 0;
    while (pos < filters.length()) {
      int nextPos = filters.indexOf('\n', pos);
      if (nextPos < 0) {
        nextPos = filters.length();
      }
      String cmd = filters.substring(pos, nextPos);
      cmd.trim();
      if (cmd.length() > 0) {
        sendClusterCommand(cmd);
      }
      pos = nextPos + 1;
    }
  } else {
    // JeĹ›li filtry wyĹ‚Ä…czone, wyczyĹ›Ä‡ wszystkie filtry
    sendClusterCommand("set/nofilter");
  }
  
  Serial.println("[CLUSTER] Komendy konfiguracyjne wys\u0142ane");
}

void handleTelnetData() {
  static unsigned long lastTelnetPrint = 0;
  static int telnetCallCount = 0;
  telnetCallCount++;
  
  if (!telnetConnected || !telnetClient.connected()) {
    if (telnetConnected) {
      LOGV_PRINTLN("[TELNET] PoĹ‚Ä…czenie zerwane - reset flagĂłw");
    }
    telnetConnected = false;
    clusterLoginSent = false;
    clusterLoginScheduled = false;
    return;
  }
  
  // Print co 1000 wywoĹ‚aĹ„ (dla debugowania)
  unsigned long now = millis();
  if (now - lastTelnetPrint > 30000) { // Co 30 sekund
    LOGV_PRINT("[TELNET] handleTelnetData wywoĹ‚ane ");
    LOGV_PRINT(telnetCallCount);
    LOGV_PRINT(" razy, available=");
    LOGV_PRINTLN(telnetClient.available());
    lastTelnetPrint = now;
    telnetCallCount = 0;
  }
  
  // Nie blokuj pÄ™tli gĹ‚Ăłwnej: w jednej iteracji czytaj maksymalnie N bajtĂłw
  // i dawaj yield, ĹĽeby nie wpaĹ›Ä‡ w WDT przy duĹĽym strumieniu.
  int processed = 0;
  const int maxPerLoop = 256;
  int availableBefore = telnetClient.available();
  
  while (telnetClient.available() && processed < maxPerLoop) {
    char c = telnetClient.read();
    processed++;
    lastTelnetRxMs = millis();
    if ((processed % 64) == 0) {
      yield();
      // obsĹ‚uĹĽ WWW nawet jeĹ›li telnet zalewa danymi
      if (server != nullptr) {
        server->handleClient();
      }
    }
    
    if (c == '\n' || c == '\r') {
      if (telnetBuffer.length() > 0) {
        // Nie parsuj tu (to bywa kosztowne). PrzekaĹĽ liniÄ™ do przetworzenia w loop().
        if (pendingTelnetLine.length() == 0) {
          pendingTelnetLine = telnetBuffer;
        } else {
          pendingTelnetDropped++;
        }
        telnetBuffer = "";
      }
    } else if (c >= 32 && c < 127) {
      telnetBuffer += c;
      if (telnetBuffer.length() > 512) {
        telnetBuffer = ""; // Ochrona przed przepeĹ‚nieniem
      }
    }
  }
  
  if (processed > 0) {
    LOGV_PRINT("[TELNET] Przetworzono ");
    LOGV_PRINT(processed);
    LOGV_PRINT(" bajtĂłw (byĹ‚o ");
    LOGV_PRINT(availableBefore);
    LOGV_PRINT(", zostaĹ‚o ");
    LOGV_PRINT(telnetClient.available());
    LOGV_PRINTLN(")");
  }
  
  if (telnetClient.available() && processed >= maxPerLoop) {
    LOGV_PRINTLN("[TELNET] WARNING: ZostaĹ‚o wiÄ™cej danych - nastÄ™pna iteracja");
    // Zostaw resztÄ™ na nastÄ™pnÄ… iteracjÄ™ loop()
    yield();
  }
  
  // SprawdĹş czy poĹ‚Ä…czenie nadal dziaĹ‚a
  if (!telnetClient.connected()) {
    LOGV_PRINTLN("[TELNET] RozĹ‚Ä…czono z DX Cluster");
    telnetConnected = false;
    clusterLoginSent = false;
    clusterLoginScheduled = false;
  }
}

void handlePotaTelnetData() {
  if (!potaTelnetConnected || !potaTelnetClient.connected()) {
    if (potaTelnetConnected) {
      LOGV_PRINTLN("[POTA] Połączenie zerwane - reset flag");
    }
    potaTelnetConnected = false;
    potaLoginSent = false;
    potaLoginScheduled = false;
    return;
  }

  int processed = 0;
  const int maxPerLoop = 256;
  while (potaTelnetClient.available() && processed < maxPerLoop) {
    char c = potaTelnetClient.read();
    processed++;
    lastPotaRxMs = millis();
    if ((processed % 64) == 0) {
      yield();
      if (server != nullptr) {
        server->handleClient();
      }
    }

    if (c == '\n' || c == '\r') {
      if (potaTelnetBuffer.length() > 0) {
        if (pendingPotaLine.length() == 0) {
          pendingPotaLine = potaTelnetBuffer;
        } else {
          pendingPotaDropped++;
        }
        potaTelnetBuffer = "";
      }
    } else if (c >= 32 && c < 127) {
      potaTelnetBuffer += c;
      if (potaTelnetBuffer.length() > 512) {
        potaTelnetBuffer = "";
      }
    }
  }

  if (!potaTelnetClient.connected()) {
    LOGV_PRINTLN("[POTA] Rozłączono z POTA Cluster");
    potaTelnetConnected = false;
    potaLoginSent = false;
    potaLoginScheduled = false;
  }
}

// ========== NTP TIME ==========

void updateNTPTime() {
  unsigned long now = millis();
  if (isGpsTimeSourceActive(now)) {
    return;
  }

  if (lastNTPUpdate != 0 && (now - lastNTPUpdate < 3600000)) { // Aktualizuj co godzinÄ™
    return;
  }
  
  if (!wifiConnected) {
    return;
  }
  
  Serial.println("[NTP] Aktualizacja czasu NTP...");
  configTime(GMT_OFFSET_SEC, 0, NTP_SERVER);
  lastNTPUpdate = now;
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("[NTP] Czas NTP zsynchronizowany");
  } else {
    Serial.println("[NTP] BĹÄ„D: Nie udaĹ‚o siÄ™ pobraÄ‡ czasu NTP");
  }
}

// ========== PREFERENCES ==========

void loadPreferences() {
  if (preferences == nullptr) {
    preferences = new Preferences();
  }
  
  preferences->begin("dxcluster", false);
  yield();

  // Kolejność ekranów
  loadDefaultScreenOrder();
  size_t bytesRead = preferences->getBytes("screen_order", screenOrder, sizeof(screenOrder));
  if (bytesRead != sizeof(screenOrder)) {
    loadDefaultScreenOrder();
  }
  ensureScreenOrderValid();
  currentScreen = firstActiveScreen();
  tftAutoSwitchEnabled = preferences->getBool("tft_autosw", false);
  applyTftAutoSwitchTimeSec(preferences->getInt("tft_sw_sec", DEFAULT_TFT_SWITCH_TIME_SEC));
  tftAutoSwitchLastMs = millis();
  tftAutoSwitchLastScreen = currentScreen;
  
  wifiSSID = preferences->getString("wifi_ssid", "");
  yield();
  wifiPassword = preferences->getString("wifi_pass", "");
  yield();
  wifiSSID2 = preferences->getString("wifi_ssid2", "");
  yield();
  wifiPassword2 = preferences->getString("wifi_pass2", "");
  yield();
  
  clusterHost = preferences->getString("cluster_host", DEFAULT_CLUSTER_HOST);
  yield();
  clusterPort = preferences->getInt("cluster_port", DEFAULT_CLUSTER_PORT);
  yield();
  potaClusterHost = preferences->getString("pota_host", DEFAULT_POTA_CLUSTER_HOST);
  yield();
  potaClusterPort = preferences->getInt("pota_port", DEFAULT_POTA_CLUSTER_PORT);
  yield();
  potaFilterCommand = preferences->getString("pota_filter", DEFAULT_POTA_FILTER_COMMAND);
  yield();
  potaApiUrl = preferences->getString("pota_api_url", DEFAULT_POTA_API_URL);
  yield();
  hamalertHost = preferences->getString("hama_host", DEFAULT_HAMALERT_HOST);
  yield();
  hamalertPort = preferences->getInt("hama_port", DEFAULT_HAMALERT_PORT);
  if (hamalertPort <= 0 || hamalertPort > 65535) hamalertPort = DEFAULT_HAMALERT_PORT;
  yield();
  hamalertLogin = preferences->getString("hama_user", "");
  yield();
  hamalertPassword = preferences->getString("hama_pass", "");
  yield();
  userCallsign = preferences->getString("user_callsign", "");
  yield();
  userLocator = preferences->getString("user_locator", "");
  yield();
  timezoneHours = preferences->getFloat("timezone", DEFAULT_TIMEZONE_HOURS);
  if (timezoneHours < -12.0f) timezoneHours = -12.0f;
  if (timezoneHours > 14.0f) timezoneHours = 14.0f;
  yield();
  userLat = preferences->getFloat("user_lat", 0.0f);
  yield();
  userLon = preferences->getFloat("user_lon", 0.0f);
  yield();
  userLatLonValid = preferences->getBool("user_ll_ok", false);
  useGpsIfActive = preferences->getBool(NVS_KEY_USE_GPS_IF_ACTIVE, false);
  uart2ConnectedModule = sanitizeUart2ConnectedModule(
    preferences->getString(NVS_KEY_UART2_CONNECTED_MODULE, "none")
  );
  uart2Mode = uart2ModeFromCode(preferences->getString(NVS_KEY_UART2_MODE, "gps"));
  applyUart2BaudRate(preferences->getInt(NVS_KEY_UART2_BAUD, (int)DEFAULT_UART2_BAUD));
  applyUart2DataBits(preferences->getInt(NVS_KEY_UART2_DATABITS, DEFAULT_UART2_DATA_BITS));
  uart2Parity = uart2ParityFromCode(preferences->getString(NVS_KEY_UART2_PARITY, "none"));
  applyUart2StopBits(preferences->getInt(NVS_KEY_UART2_STOP_BITS, DEFAULT_UART2_STOP_BITS));
  bridgeBtPairedAddress = normalizeBridgeBtAddress(preferences->getString(NVS_KEY_BRIDGE_PAIR_ADDR, ""));
  bridgeBtPairedName = sanitizeBridgeTextField(preferences->getString(NVS_KEY_BRIDGE_PAIR_NAME, ""), 40);
  bridgeBtAutoConnectPending = (bridgeBtPairedAddress.length() == 12);
  yield();
  qrzUsername = preferences->getString("qrz_user", "");
  yield();
  qrzPassword = preferences->getString("qrz_pass", "");
  yield();
  weatherApiKey = preferences->getString("weather_key", "");
  yield();
  openWebRxUrl = preferences->getString("openwebrx_url", DEFAULT_OPENWEBRX_URL);
  yield();
  backlightPercent = preferences->getInt("tft_backlight", TFT_BACKLIGHT);
  if (backlightPercent < MIN_BACKLIGHT_PERCENT) backlightPercent = MIN_BACKLIGHT_PERCENT;
  if (backlightPercent > 100) backlightPercent = 100;
  tftNightDimmingPercent = preferences->getInt(NVS_KEY_TFT_NIGHT_DIMMING, 0);
  if (tftNightDimmingPercent < 0) tftNightDimmingPercent = 0;
  if (tftNightDimmingPercent > 100) tftNightDimmingPercent = 100;
  tftInvertColors = preferences->getBool("tft_inv", tftInvertColors);
  tftLanguage = preferences->getUChar("tft_lang", TFT_LANG_PL);
  if (tftLanguage > TFT_LANG_PT) tftLanguage = TFT_LANG_PL;
  tftStyleMode = preferences->getUChar("tft_style", TFT_STYLE_DARK);
  if (tftStyleMode > TFT_STYLE_LIGHT) tftStyleMode = TFT_STYLE_DARK;
  dxTableSizeMode = preferences->getUChar("tft_tbl_size", DX_TABLE_SIZE_NORMAL);
  if (dxTableSizeMode > DX_TABLE_SIZE_MEDIUM) dxTableSizeMode = DX_TABLE_SIZE_NORMAL;
  int savedMenuHue = preferences->getInt("menu_hue", DEFAULT_MENU_THEME_HUE);
  if (savedMenuHue < 0) savedMenuHue = 0;
  if (savedMenuHue > 255) savedMenuHue = 255;
  menuThemeHue = (uint8_t)savedMenuHue;
  applyMenuThemeFromHue();
  touchXMin = preferences->getInt("touch_xmin", TOUCH_X_MIN);
  touchXMax = preferences->getInt("touch_xmax", TOUCH_X_MAX);
  touchYMin = preferences->getInt("touch_ymin", TOUCH_Y_MIN);
  touchYMax = preferences->getInt("touch_ymax", TOUCH_Y_MAX);
  touchSwapXY = preferences->getBool("touch_swap", TOUCH_SWAP_XY);
  touchInvertX = preferences->getBool("touch_invx", TOUCH_INVERT_X);
  touchInvertY = preferences->getBool("touch_invy", TOUCH_INVERT_Y);
  touchRotation = preferences->getUChar("touch_rot", 1);
  if (touchRotation > 3) touchRotation = 1;
  tftRotation = preferences->getUChar("tft_rot", 1);
  if (tftRotation > 3) tftRotation = 1;
  if (touchXMin < 0 || touchXMax > 4095 || touchXMin >= touchXMax) {
    touchXMin = TOUCH_X_MIN;
    touchXMax = TOUCH_X_MAX;
  }
  if (touchYMin < 0 || touchYMax > 4095 || touchYMin >= touchYMax) {
    touchYMin = TOUCH_Y_MIN;
    touchYMax = TOUCH_Y_MAX;
  }
  screen1TimeMode = (uint8_t)preferences->getInt("screen1_time", SCREEN1_TIME_UTC);
  if (screen1TimeMode > SCREEN1_TIME_LOCAL) screen1TimeMode = SCREEN1_TIME_UTC;

  screen2FilterModeMask = preferences->getUChar("s2_mode", 0);
  screen2FilterModeMask &= (FILTER_MODE_BIT_CW | FILTER_MODE_BIT_SSB | FILTER_MODE_BIT_DIGI);
  uint16_t screen2BandAllowedMask = 0;
  for (int bandIdx = 1; bandIdx < SCREEN8_FILTER_BANDS_COUNT; bandIdx++) {
    screen2BandAllowedMask |= getBandFilterBitForIndex(bandIdx);
  }
  screen2FilterBandMask = (uint16_t)preferences->getUInt("s2_band", 0);
  screen2FilterBandMask &= screen2BandAllowedMask;

  uint8_t loadedScreen6Sort = preferences->getUChar("s6_sort", APRS_SORT_TIME);
  if (loadedScreen6Sort > APRS_SORT_DISTANCE) {
    loadedScreen6Sort = APRS_SORT_TIME;
  }
  screen6SortMode = (Screen6SortMode)loadedScreen6Sort;

  screen7FilterModeMask = preferences->getUChar("s7_mode", 0);
  screen7FilterModeMask &= (FILTER_MODE_BIT_CW | FILTER_MODE_BIT_SSB | FILTER_MODE_BIT_DIGI);
  uint16_t screen7BandAllowedMask = 0;
  for (int bandIdx = 1; bandIdx < SCREEN2_FILTER_BANDS_COUNT; bandIdx++) {
    screen7BandAllowedMask |= getBandFilterBitForIndex(bandIdx);
  }
  screen7FilterBandMask = (uint16_t)preferences->getUInt("s7_band", 0);
  screen7FilterBandMask &= screen7BandAllowedMask;

  screen8FilterModeMask = preferences->getUChar("s8_mode", 0);
  screen8FilterModeMask &= (FILTER_MODE_BIT_CW | FILTER_MODE_BIT_SSB | FILTER_MODE_BIT_DIGI);
  uint16_t screen8BandAllowedMask = 0;
  for (int bandIdx = 1; bandIdx < SCREEN8_FILTER_BANDS_COUNT; bandIdx++) {
    screen8BandAllowedMask |= getBandFilterBitForIndex(bandIdx);
  }
  screen8FilterBandMask = (uint16_t)preferences->getUInt("s8_band", 0);
  screen8FilterBandMask &= screen8BandAllowedMask;
  
  // Konfiguracja filtrĂłw CC-Cluster
  clusterNoAnnouncements = preferences->getBool("cluster_noann", true);
  yield();
  clusterNoWWV = preferences->getBool("cluster_nowwv", true);
  yield();
  clusterNoWCY = preferences->getBool("cluster_nowcy", true);
  yield();
  clusterUseFilters = preferences->getBool(NVS_KEY_CLUSTER_USEFILTERS, true);
  yield();
  clusterFilterCommands = preferences->getString("cluster_filters", "set/ft8");
  yield();
  
  // Konfiguracja APRS-IS
  aprsIsHost = preferences->getString("aprs_host", DEFAULT_APRS_IS_HOST);
  yield();
  aprsIsPort = preferences->getInt("aprs_port", DEFAULT_APRS_IS_PORT);
  yield();
  aprsCallsign = preferences->getString("aprs_callsign", DEFAULT_APRS_CALLSIGN);
  yield();
  aprsPasscode = preferences->getInt("aprs_passcode", DEFAULT_APRS_PASSCODE);
  applyAprsSsid(preferences->getInt("aprs_ssid", DEFAULT_APRS_SSID));
  yield();
  aprsFilterRadius = preferences->getInt("aprs_radius", DEFAULT_APRS_FILTER_RADIUS);
  yield();
  aprsBeaconEnabled = preferences->getBool("aprs_beacon", true);
  String aprsSymPref = preferences->getString("aprs_symbol", aprsSymbolTwoChar);
  applyAprsSymbol(aprsSymPref);
  aprsUserComment = preferences->getString("aprs_comment", "");
  aprsAlertCsv = sanitizeAprsAlertList(preferences->getString("aprs_alert", ""));
  aprsAlertEnabled = preferences->getBool("aprs_alert_en", true);
  applyAprsAlertMinSeconds(preferences->getInt(NVS_KEY_APRS_ALERT_MIN_SEC, DEFAULT_APRS_ALERT_MIN_SEC));
  applyAprsAlertScreenSeconds(preferences->getInt(NVS_KEY_APRS_ALERT_SCREEN_SEC, DEFAULT_APRS_ALERT_SCREEN_SEC));
  aprsAlertNearbyEnabled = preferences->getBool("aprs_alert_near_en", true);
  aprsAlertWxEnabled = preferences->getBool(NVS_KEY_APRS_ALERT_WX_ENABLED, false);
  applyAprsAlertDistanceKm(preferences->getFloat(NVS_KEY_APRS_ALERT_DISTANCE_KM, DEFAULT_APRS_ALERT_DISTANCE_KM));
  weatherAlertStormEnabled = preferences->getBool(NVS_KEY_WEATHER_ALERT_STORM_ENABLED, DEFAULT_WEATHER_ALERT_STORM_ENABLED);
  weatherAlertWindEnabled = preferences->getBool(NVS_KEY_WEATHER_ALERT_WIND_ENABLED, DEFAULT_WEATHER_ALERT_WIND_ENABLED);
  applyWeatherAlertWindThresholdMs(preferences->getFloat(NVS_KEY_WEATHER_ALERT_WIND_THRESHOLD_MS, DEFAULT_WEATHER_ALERT_WIND_THRESHOLD_MS));
  enableLedAlert = preferences->getBool(NVS_KEY_ENABLE_LED_ALERT, DEFAULT_ENABLE_LED_ALERT);
  applyLedAlertDurationMs(preferences->getInt(NVS_KEY_LED_ALERT_DURATION_MS, DEFAULT_LED_ALERT_DURATION_MS));
  applyLedAlertBlinkMs(preferences->getInt(NVS_KEY_LED_ALERT_BLINK_MS, DEFAULT_LED_ALERT_BLINK_MS));
  resetAprsAlertCooldownState();
  int storedIntervalMin = preferences->getInt(NVS_KEY_APRS_INTERVAL_MIN, DEFAULT_APRS_INTERVAL_MIN);
  applyAprsIntervalMinutes(storedIntervalMin);
  yield();
  // Ograniczenie promienia do 1-50 km
  if (aprsFilterRadius < 1) aprsFilterRadius = 1;
  if (aprsFilterRadius > 50) aprsFilterRadius = 50;
  // Uwaga: APRS uĹĽywa wspĂłĹ‚rzÄ™dnych z sekcji "Moja Stacja" (userLat, userLon) - nie ma osobnych pĂłl

  if (!userLatLonValid && userLocator.length() >= 4) {
    updateUserLatLonFromLocator();
    preferences->putFloat("user_lat", (float)userLat);
    preferences->putFloat("user_lon", (float)userLon);
    preferences->putBool("user_ll_ok", userLatLonValid);
  }
  
  preferences->end();

  // Zastosuj inwersję kolorów po wczytaniu ustawień
  if (tftInitialized) {
    applyTftInversion();
  }
  
  Serial.println("Konfiguracja wczytana");
}

void savePreferences() {
  if (preferences == nullptr) {
    preferences = new Preferences();
  }
  
  preferences->begin("dxcluster", false);
  preferences->putBytes("screen_order", screenOrder, sizeof(screenOrder));
  
  preferences->putString("wifi_ssid", wifiSSID);
  preferences->putString("wifi_pass", wifiPassword);
  preferences->putString("wifi_ssid2", wifiSSID2);
  preferences->putString("wifi_pass2", wifiPassword2);
  preferences->putString("cluster_host", clusterHost);
  preferences->putInt("cluster_port", clusterPort);
  preferences->putString("pota_host", potaClusterHost);
  preferences->putInt("pota_port", potaClusterPort);
  preferences->putString("pota_filter", potaFilterCommand);
  preferences->putString("pota_api_url", potaApiUrl);
  preferences->putString("hama_host", hamalertHost);
  preferences->putInt("hama_port", hamalertPort);
  preferences->putString("hama_user", hamalertLogin);
  preferences->putString("hama_pass", hamalertPassword);
  preferences->putString("user_callsign", userCallsign);
  preferences->putString("user_locator", userLocator);
  preferences->putFloat("timezone", timezoneHours);
  preferences->putFloat("user_lat", (float)userLat);
  preferences->putFloat("user_lon", (float)userLon);
  preferences->putBool("user_ll_ok", userLatLonValid);
  preferences->putBool(NVS_KEY_USE_GPS_IF_ACTIVE, useGpsIfActive);
  preferences->putString(NVS_KEY_UART2_CONNECTED_MODULE, uart2ConnectedModule);
  preferences->putString(NVS_KEY_UART2_MODE, uart2ModeToCode(uart2Mode));
  preferences->putInt(NVS_KEY_UART2_BAUD, (int)uart2BaudRate);
  preferences->putInt(NVS_KEY_UART2_DATABITS, (int)uart2DataBits);
  preferences->putString(NVS_KEY_UART2_PARITY, uart2ParityToCode(uart2Parity));
  preferences->putInt(NVS_KEY_UART2_STOP_BITS, (int)uart2StopBits);
  preferences->putString(NVS_KEY_BRIDGE_PAIR_ADDR, bridgeBtPairedAddress);
  preferences->putString(NVS_KEY_BRIDGE_PAIR_NAME, bridgeBtPairedName);
  preferences->putString("qrz_user", qrzUsername);
  preferences->putString("qrz_pass", qrzPassword);
  preferences->putString("weather_key", weatherApiKey);
  preferences->putString("openwebrx_url", openWebRxUrl);
  preferences->putInt("tft_backlight", backlightPercent);
  preferences->putInt(NVS_KEY_TFT_NIGHT_DIMMING, tftNightDimmingPercent);
  preferences->putBool("tft_inv", tftInvertColors);
  preferences->putUChar("tft_lang", tftLanguage);
  preferences->putUChar("tft_style", tftStyleMode);
  preferences->putUChar("tft_tbl_size", dxTableSizeMode);
  preferences->putInt("menu_hue", menuThemeHue);
  preferences->putInt("screen1_time", screen1TimeMode);
  preferences->putUChar("s2_mode", screen2FilterModeMask);
  preferences->putUInt("s2_band", (uint32_t)screen2FilterBandMask);
  preferences->putUChar("s6_sort", (uint8_t)screen6SortMode);
  preferences->putUChar("s7_mode", screen7FilterModeMask);
  preferences->putUInt("s7_band", (uint32_t)screen7FilterBandMask);
  preferences->putUChar("s8_mode", screen8FilterModeMask);
  preferences->putUInt("s8_band", (uint32_t)screen8FilterBandMask);
  preferences->putBool("tft_autosw", tftAutoSwitchEnabled);
  preferences->putInt("tft_sw_sec", tftAutoSwitchTimeSec);
  preferences->putInt("touch_xmin", touchXMin);
  preferences->putInt("touch_xmax", touchXMax);
  preferences->putInt("touch_ymin", touchYMin);
  preferences->putInt("touch_ymax", touchYMax);
  preferences->putBool("touch_swap", touchSwapXY);
  preferences->putBool("touch_invx", touchInvertX);
  preferences->putBool("touch_invy", touchInvertY);
  preferences->putUChar("touch_rot", touchRotation);
  preferences->putUChar("tft_rot", tftRotation);
  
  // Konfiguracja APRS-IS
  preferences->putString("aprs_host", aprsIsHost);
  preferences->putInt("aprs_port", aprsIsPort);
  preferences->putString("aprs_callsign", aprsCallsign);
  preferences->putInt("aprs_passcode", aprsPasscode);
  preferences->putInt("aprs_ssid", aprsSsid);
  preferences->putBool("aprs_beacon", aprsBeaconEnabled);
  preferences->putString("aprs_symbol", aprsSymbolTwoChar);
  preferences->putString("aprs_comment", aprsUserComment);
  preferences->putString("aprs_alert", aprsAlertCsv);
  preferences->putBool("aprs_alert_en", aprsAlertEnabled);
  preferences->putInt(NVS_KEY_APRS_ALERT_MIN_SEC, aprsAlertMinSeconds);
  preferences->putInt(NVS_KEY_APRS_ALERT_SCREEN_SEC, aprsAlertScreenSeconds);
  preferences->putBool("aprs_alert_near_en", aprsAlertNearbyEnabled);
  preferences->putBool(NVS_KEY_APRS_ALERT_WX_ENABLED, aprsAlertWxEnabled);
  preferences->putFloat(NVS_KEY_APRS_ALERT_DISTANCE_KM, aprsAlertDistanceKm);
  preferences->putBool(NVS_KEY_WEATHER_ALERT_STORM_ENABLED, weatherAlertStormEnabled);
  preferences->putBool(NVS_KEY_WEATHER_ALERT_WIND_ENABLED, weatherAlertWindEnabled);
  preferences->putFloat(NVS_KEY_WEATHER_ALERT_WIND_THRESHOLD_MS, weatherAlertWindThresholdMs);
  preferences->putBool(NVS_KEY_ENABLE_LED_ALERT, enableLedAlert);
  preferences->putInt(NVS_KEY_LED_ALERT_DURATION_MS, ledAlertDurationMs);
  preferences->putInt(NVS_KEY_LED_ALERT_BLINK_MS, ledAlertBlinkMs);
  preferences->putInt(NVS_KEY_APRS_INTERVAL_MIN, aprsIntervalMinutes);
  // Ograniczenie promienia do 1-50 km przed zapisem
  int radiusToSave = aprsFilterRadius;
  if (radiusToSave < 1) radiusToSave = 1;
  if (radiusToSave > 50) radiusToSave = 50;
  preferences->putInt("aprs_radius", radiusToSave);
  // Uwaga: APRS używa współrzędnych z sekcji "Moja Stacja" (userLat, userLon) - nie zapisujemy osobnych pól
  
  // Konfiguracja filtrów CC-Cluster
  preferences->putBool("cluster_noann", clusterNoAnnouncements);
  preferences->putBool("cluster_nowwv", clusterNoWWV);
  preferences->putBool("cluster_nowcy", clusterNoWCY);
  preferences->putBool(NVS_KEY_CLUSTER_USEFILTERS, clusterUseFilters);
  preferences->putString("cluster_filters", clusterFilterCommands);
  
  preferences->end();
  
  Serial.println("Konfiguracja zapisana");
}

// ========== WEB SERVER ==========

String getMainHTML();
String getConfigHTML();

static bool streamLittleFsFile(const char *path, const char *contentType) {
  if (!littleFsReady) {
    return false;
  }
  File f = LittleFS.open(path, "r");
  if (!f) {
    return false;
  }
  server->streamFile(f, contentType);
  f.close();
  return true;
}

void setupWebServer() {
  // UtwĂłrz serwer jeĹ›li jeszcze nie istnieje
  if (server == nullptr) {
    server = new WebServer(80);
  }
  
  // Strona gĹ‚Ăłwna
  server->on("/", HTTP_GET, []() {
    if (!streamLittleFsFile("/index.html", "text/html; charset=utf-8")) {
      server->send(200, "text/html; charset=utf-8", getMainHTML());
    }
  });

  // Strona gĹ‚Ăłwna (alias /index.html)
  server->on("/index.html", HTTP_GET, []() {
    if (!streamLittleFsFile("/index.html", "text/html; charset=utf-8")) {
      server->send(200, "text/html; charset=utf-8", getMainHTML());
    }
  });

  // Strona gĹ‚Ăłwna (EN)
  server->on("/indexEN.html", HTTP_GET, []() {
    if (!streamLittleFsFile("/indexEN.html", "text/html; charset=utf-8") &&
        !streamLittleFsFile("/index.html", "text/html; charset=utf-8")) {
      server->send(404, "text/plain", "indexEN.html missing in LittleFS");
    }
  });

  // Main page (PT-BR)
  server->on("/indexPT.html", HTTP_GET, []() {
    if (!streamLittleFsFile("/indexPT.html", "text/html; charset=utf-8") &&
        !streamLittleFsFile("/indexEN.html", "text/html; charset=utf-8") &&
        !streamLittleFsFile("/index.html", "text/html; charset=utf-8")) {
      server->send(404, "text/plain", "indexPT.html missing in LittleFS");
    }
  });

  // Manual (PT-BR)
  server->on("/manual_pt.txt", HTTP_GET, []() {
    if (!streamLittleFsFile("/manual_pt.txt", "text/plain; charset=utf-8")) {
      server->send(404, "text/plain", "manual_pt.txt missing in LittleFS");
    }
  });

  // Instrukcja (PL)
  server->on("/instrukcja.txt", HTTP_GET, []() {
    if (!streamLittleFsFile("/instrukcja.txt", "text/plain; charset=utf-8")) {
      server->send(404, "text/plain", "instrukcja.txt missing in LittleFS");
    }
  });

  // Manual (EN)
  server->on("/manual.txt", HTTP_GET, []() {
    if (!streamLittleFsFile("/manual.txt", "text/plain; charset=utf-8")) {
      server->send(404, "text/plain", "manual.txt missing in LittleFS");
    }
  });

  // Arkusze stylów CSS (wspólne + desktop/mobile)
  server->on("/style.css", HTTP_GET, []() {
    if (!streamLittleFsFile("/style.css", "text/css; charset=utf-8")) {
      server->send(404, "text/plain", "style.css missing in LittleFS");
    }
  });

  server->on("/style.base.css", HTTP_GET, []() {
    if (!streamLittleFsFile("/style.base.css", "text/css; charset=utf-8")) {
      server->send(404, "text/plain", "style.base.css missing in LittleFS");
    }
  });

  server->on("/style.desktop.css", HTTP_GET, []() {
    if (!streamLittleFsFile("/style.desktop.css", "text/css; charset=utf-8")) {
      server->send(404, "text/plain", "style.desktop.css missing in LittleFS");
    }
  });

  server->on("/style.mobile.css", HTTP_GET, []() {
    if (!streamLittleFsFile("/style.mobile.css", "text/css; charset=utf-8")) {
      server->send(404, "text/plain", "style.mobile.css missing in LittleFS");
    }
  });
  
  // Strona konfiguracji
  server->on("/config", HTTP_GET, []() {
    if (!streamLittleFsFile("/index.html", "text/html; charset=utf-8")) {
      server->send(200, "text/html; charset=utf-8", getConfigHTML());
    }
  });
  
  // API - pobierz wszystkie spoty (maksymalnie 50)
  server->on("/api/spots", HTTP_GET, []() {
    // ZwiÄ™kszony bufor JSON dla 50 spotĂłw (kaĹĽdy spot ~150-200 bajtĂłw)
    StaticJsonDocument<12000> doc;  // 50 spotĂłw * ~200 bajtĂłw = ~10KB + margines
    JsonArray spotsArray = doc.createNestedArray("spots");
    
    // ZwrĂłÄ‡ wszystkie spoty (maksymalnie 50)
    for (int i = 0; i < spotCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["time"] = spots[i].time;
      spotObj["callsign"] = spots[i].callsign;
      spotObj["frequency"] = spots[i].frequency;
      spotObj["distance"] = spots[i].distance;
      spotObj["country"] = spots[i].country;
      spotObj["band"] = spots[i].band;
      spotObj["mode"] = spots[i].mode;
      spotObj["spotter"] = spots[i].spotter;
      spotObj["comment"] = spots[i].comment;
    }
    
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });
  
  // API - pobierz stacje APRS
  server->on("/api/aprs", HTTP_GET, []() {
    // Bufor JSON dla 20 stacji APRS (kaĹĽda stacja ~200-250 bajtĂłw)
    StaticJsonDocument<6000> doc;  // 20 stacji * ~250 bajtĂłw = ~5KB + margines
    JsonArray aprsArray = doc.createNestedArray("stations");
    
    // ZwrĂłÄ‡ wszystkie stacje APRS (maksymalnie 20)
    for (int i = 0; i < aprsStationCount; i++) {
      JsonObject stationObj = aprsArray.createNestedObject();
      stationObj["time"] = aprsStations[i].time;
      stationObj["callsign"] = aprsStations[i].callsign;
      stationObj["symbol"] = getAPRSSymbolShort(aprsStations[i]);
      stationObj["lat"] = aprsStations[i].lat;
      stationObj["lon"] = aprsStations[i].lon;
      stationObj["comment"] = aprsStations[i].comment;
      stationObj["distance"] = aprsStations[i].distance;
      stationObj["hasLatLon"] = aprsStations[i].hasLatLon;
      stationObj["freq_mhz"] = aprsStations[i].freqMHz;
    }
    
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - POTA (bufor z backendu, z krajami z QRZ jeśli dostępne)
  server->on("/api/pota", HTTP_GET, []() {
    StaticJsonDocument<12000> doc; // do 30 spotów * ~300-350 bajtów
    JsonArray spotsArray = doc.createNestedArray("spots");
    const int apiSpotLimit = 30;
    const int potaApiCount = (potaSpotCount < apiSpotLimit) ? potaSpotCount : apiSpotLimit;
    for (int i = 0; i < potaApiCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["time"] = potaSpots[i].time;
      spotObj["spotTime"] = potaSpots[i].time; // zgodność z POTA API
      spotObj["callsign"] = potaSpots[i].callsign;
      spotObj["frequency"] = potaSpots[i].frequency;
      spotObj["mode"] = potaSpots[i].mode;
      spotObj["country"] = potaSpots[i].country;
      spotObj["spotter"] = potaSpots[i].spotter;
      spotObj["comment"] = potaSpots[i].comment;
      spotObj["band"] = potaSpots[i].band;
    }
    doc["count"] = potaApiCount;
    doc["total"] = potaSpotCount;
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - HAMALERT (ostatnie spoty z telnetu)
  server->on("/api/hamalert", HTTP_GET, []() {
    StaticJsonDocument<12000> doc; // do 30 spotów
    JsonArray spotsArray = doc.createNestedArray("spots");
    const int apiSpotLimit = 30;
    const int hamalertApiCount = (hamalertSpotCount < apiSpotLimit) ? hamalertSpotCount : apiSpotLimit;
    for (int i = 0; i < hamalertApiCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["time"] = hamalertSpots[i].time;
      spotObj["spotTime"] = hamalertSpots[i].time;
      spotObj["callsign"] = hamalertSpots[i].callsign;
      spotObj["frequency"] = hamalertSpots[i].frequency;
      spotObj["mode"] = hamalertSpots[i].mode;
      spotObj["country"] = hamalertSpots[i].country;
      spotObj["spotter"] = hamalertSpots[i].spotter;
      spotObj["comment"] = hamalertSpots[i].comment;
      spotObj["band"] = hamalertSpots[i].band;
    }
    doc["count"] = hamalertApiCount;
    doc["total"] = hamalertSpotCount;
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - Propagation + HF Band Info
  server->on("/api/propagation", HTTP_GET, []() {
    StaticJsonDocument<2048> doc;
    doc["valid"] = propagationData.valid;
    doc["sfi"] = propagationData.sfi;
    doc["kindex"] = propagationData.kindex;
    doc["aindex"] = propagationData.aindex;
    doc["muf"] = propagationData.muf;
    doc["updated"] = propagationData.updated;
    doc["lastError"] = propagationData.lastError;

    JsonArray bands = doc.createNestedArray("hfBands");
    for (int i = 0; i < 4; i++) {
      JsonObject band = bands.createNestedObject();
      band["label"] = propagationData.hfBandLabel[i];
      band["freq"] = propagationData.hfBandFreq[i];
      band["day"] = propagationData.hfBandDay[i];
      band["night"] = propagationData.hfBandNight[i];
    }

    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });
  
  // API - zapisz konfiguracjÄ™
  server->on("/api/save", HTTP_POST, []() {
    if (server->hasArg("plain")) {
      String body = server->arg("plain");
      
        StaticJsonDocument<8192> doc;  // Bufor dla pełnego payloadu konfiguracji z WWW
      DeserializationError err = deserializeJson(doc, body);
      if (err) {
        server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"json_parse\"}");
        return;
      }
      
      wifiSSID = doc["wifi_ssid"].as<String>();
      wifiPassword = doc["wifi_pass"].as<String>();
      if (doc["wifi_ssid2"].is<String>()) {
        wifiSSID2 = doc["wifi_ssid2"].as<String>();
      }
      if (doc["wifi_pass2"].is<String>()) {
        wifiPassword2 = doc["wifi_pass2"].as<String>();
      }
      if (doc["uart2_mode"].is<String>()) {
        uart2Mode = uart2ModeFromCode(doc["uart2_mode"].as<String>());
      }
      if (doc.containsKey("uart2_baud")) {
        int uart2BaudCandidate = (int)uart2BaudRate;
        if (doc["uart2_baud"].is<int>()) {
          uart2BaudCandidate = doc["uart2_baud"].as<int>();
        } else if (doc["uart2_baud"].is<long>()) {
          uart2BaudCandidate = (int)doc["uart2_baud"].as<long>();
        } else if (doc["uart2_baud"].is<float>() || doc["uart2_baud"].is<double>()) {
          uart2BaudCandidate = (int)doc["uart2_baud"].as<double>();
        } else if (doc["uart2_baud"].is<String>()) {
          String uart2BaudStr = doc["uart2_baud"].as<String>();
          uart2BaudStr.trim();
          uart2BaudCandidate = uart2BaudStr.toInt();
        }
        applyUart2BaudRate(uart2BaudCandidate);
      }
      if (doc.containsKey("uart2_data_bits")) {
        int uart2DataBitsCandidate = (int)uart2DataBits;
        if (doc["uart2_data_bits"].is<int>()) {
          uart2DataBitsCandidate = doc["uart2_data_bits"].as<int>();
        } else if (doc["uart2_data_bits"].is<long>()) {
          uart2DataBitsCandidate = (int)doc["uart2_data_bits"].as<long>();
        } else if (doc["uart2_data_bits"].is<float>() || doc["uart2_data_bits"].is<double>()) {
          uart2DataBitsCandidate = (int)doc["uart2_data_bits"].as<double>();
        } else if (doc["uart2_data_bits"].is<String>()) {
          String uart2DataBitsStr = doc["uart2_data_bits"].as<String>();
          uart2DataBitsStr.trim();
          uart2DataBitsCandidate = uart2DataBitsStr.toInt();
        }
        applyUart2DataBits(uart2DataBitsCandidate);
      }
      if (doc["uart2_parity"].is<String>()) {
        uart2Parity = uart2ParityFromCode(doc["uart2_parity"].as<String>());
      }
      if (doc.containsKey("uart2_stop_bits")) {
        int uart2StopBitsCandidate = (int)uart2StopBits;
        if (doc["uart2_stop_bits"].is<int>()) {
          uart2StopBitsCandidate = doc["uart2_stop_bits"].as<int>();
        } else if (doc["uart2_stop_bits"].is<long>()) {
          uart2StopBitsCandidate = (int)doc["uart2_stop_bits"].as<long>();
        } else if (doc["uart2_stop_bits"].is<float>() || doc["uart2_stop_bits"].is<double>()) {
          uart2StopBitsCandidate = (int)doc["uart2_stop_bits"].as<double>();
        } else if (doc["uart2_stop_bits"].is<String>()) {
          String uart2StopBitsStr = doc["uart2_stop_bits"].as<String>();
          uart2StopBitsStr.trim();
          uart2StopBitsCandidate = uart2StopBitsStr.toInt();
        }
        applyUart2StopBits(uart2StopBitsCandidate);
      }
      clusterHost = doc["cluster_host"].as<String>();
      if (doc["pota_host"].is<String>()) {
        potaClusterHost = doc["pota_host"].as<String>();
      }
      if (doc["pota_filter"].is<String>()) {
        potaFilterCommand = doc["pota_filter"].as<String>();
      }
      if (doc["pota_api_url"].is<String>()) {
        potaApiUrl = doc["pota_api_url"].as<String>();
      }
      if (doc["hamalert_host"].is<String>()) {
        hamalertHost = doc["hamalert_host"].as<String>();
      }
      if (doc["hamalert_port"].is<int>()) {
        hamalertPort = doc["hamalert_port"].as<int>();
      }
      if (doc["hamalert_login"].is<String>()) {
        hamalertLogin = doc["hamalert_login"].as<String>();
      }
      if (doc["hamalert_password"].is<String>()) {
        hamalertPassword = doc["hamalert_password"].as<String>();
      }
      userCallsign = doc["user_callsign"].as<String>();
      userLocator = doc["user_locator"].as<String>();
      if (doc["timezone"].is<float>() || doc["timezone"].is<double>() || doc["timezone"].is<int>()) {
        timezoneHours = doc["timezone"].as<float>();
        if (timezoneHours < -12.0f) timezoneHours = -12.0f;
        if (timezoneHours > 14.0f) timezoneHours = 14.0f;
      }
      
      // WspĂłĹ‚rzÄ™dne geograficzne (LAT/LON)
      bool latProvided = false;
      bool lonProvided = false;
      if (doc["user_lat"].is<float>() || doc["user_lat"].is<double>()) {
        userLat = doc["user_lat"].as<double>();
        latProvided = true;
      }
      if (doc["user_lon"].is<float>() || doc["user_lon"].is<double>()) {
        userLon = doc["user_lon"].as<double>();
        lonProvided = true;
      }
      // JeĹ›li podano obie wspĂłĹ‚rzÄ™dne, ustaw flagÄ™ jako waĹĽne
      // Uwaga: sprawdzamy czy wartoĹ›ci sÄ… w prawidĹ‚owym zakresie (nie tylko != 0)
      if (latProvided && lonProvided && 
          userLat >= -90.0 && userLat <= 90.0 && 
          userLon >= -180.0 && userLon <= 180.0) {
        userLatLonValid = true;
      } else if (latProvided || lonProvided) {
        // JeĹ›li podano tylko jednÄ… wspĂłĹ‚rzÄ™dnÄ…, uznaj za nieprawidĹ‚owe
        userLatLonValid = false;
      }
      if (doc["use_gps_if_active"].is<bool>()) {
        useGpsIfActive = doc["use_gps_if_active"].as<bool>();
      }
      if (doc.containsKey("uart2_connected_module")) {
        uart2ConnectedModule = sanitizeUart2ConnectedModule(doc["uart2_connected_module"].as<String>());
      }
      
      if (doc["qrz_user"].is<String>()) {
        qrzUsername = doc["qrz_user"].as<String>();
      }
      if (doc["qrz_pass"].is<String>()) {
        qrzPassword = doc["qrz_pass"].as<String>();
      }
      if (doc["weather_key"].is<String>()) {
        weatherApiKey = doc["weather_key"].as<String>();
      }
      if (doc["openwebrx_url"].is<String>()) {
        openWebRxUrl = doc["openwebrx_url"].as<String>();
      }
      if (doc["tft_backlight"].is<int>()) {
        backlightPercent = doc["tft_backlight"].as<int>();
        if (backlightPercent < MIN_BACKLIGHT_PERCENT) backlightPercent = MIN_BACKLIGHT_PERCENT;
        if (backlightPercent > 100) backlightPercent = 100;
      }
      if (doc["tft_night_dimming"].is<int>()) {
        tftNightDimmingPercent = doc["tft_night_dimming"].as<int>();
        if (tftNightDimmingPercent < 0) tftNightDimmingPercent = 0;
        if (tftNightDimmingPercent > 100) tftNightDimmingPercent = 100;
      }
      if (doc["tft_invert"].is<bool>()) {
        tftInvertColors = doc["tft_invert"].as<bool>();
      }
      if (doc["tft_lang"].is<String>()) {
        tftLanguage = tftLangFromCode(doc["tft_lang"].as<String>());
      } else if (doc["tft_lang"].is<int>()) {
        int lang = doc["tft_lang"].as<int>();
        tftLanguage = (lang == TFT_LANG_EN) ? TFT_LANG_EN : ((lang == TFT_LANG_PT) ? TFT_LANG_PT : TFT_LANG_PL);
      }
      if (doc["tft_style"].is<String>()) {
        tftStyleMode = tftStyleFromCode(doc["tft_style"].as<String>());
        applyTftUiPalette();
      } else if (doc["tft_style"].is<int>()) {
        int styleMode = doc["tft_style"].as<int>();
        tftStyleMode = (styleMode == TFT_STYLE_LIGHT) ? TFT_STYLE_LIGHT : TFT_STYLE_DARK;
        applyTftUiPalette();
      }
      if (doc["table_size"].is<String>()) {
        dxTableSizeMode = dxTableSizeFromCode(doc["table_size"].as<String>());
      } else if (doc["table_size"].is<int>()) {
        int sizeMode = doc["table_size"].as<int>();
        dxTableSizeMode = (sizeMode == DX_TABLE_SIZE_ENLARGED) ? DX_TABLE_SIZE_ENLARGED
                         : ((sizeMode == DX_TABLE_SIZE_MEDIUM) ? DX_TABLE_SIZE_MEDIUM : DX_TABLE_SIZE_NORMAL);
      }
      if (doc["menu_hue"].is<int>()) {
        int hue = doc["menu_hue"].as<int>();
        if (hue < 0) hue = 0;
        if (hue > 255) hue = 255;
        menuThemeHue = (uint8_t)hue;
        applyMenuThemeFromHue();
      }
      
      // Konfiguracja filtrĂłw CC-Cluster
      if (doc["cluster_noann"].is<bool>()) {
        clusterNoAnnouncements = doc["cluster_noann"].as<bool>();
      }
      if (doc["cluster_nowwv"].is<bool>()) {
        clusterNoWWV = doc["cluster_nowwv"].as<bool>();
      }
      if (doc["cluster_nowcy"].is<bool>()) {
        clusterNoWCY = doc["cluster_nowcy"].as<bool>();
      }
      if (doc["cluster_usefilters"].is<bool>()) {
        clusterUseFilters = doc["cluster_usefilters"].as<bool>();
      }
      if (doc["cluster_filters"].is<String>()) {
        clusterFilterCommands = doc["cluster_filters"].as<String>();
      }
      
      // Konfiguracja APRS-IS
      if (doc["aprs_host"].is<String>()) {
        aprsIsHost = doc["aprs_host"].as<String>();
      }
      if (doc["aprs_port"].is<int>()) {
        aprsIsPort = doc["aprs_port"].as<int>();
      }
      if (doc["aprs_callsign"].is<String>()) {
        aprsCallsign = doc["aprs_callsign"].as<String>();
      }
      if (doc["aprs_passcode"].is<int>()) {
        aprsPasscode = doc["aprs_passcode"].as<int>();
      }
      if (doc["aprs_ssid"].is<int>()) {
        applyAprsSsid(doc["aprs_ssid"].as<int>());
      }
      if (doc["aprs_beacon"].is<bool>()) {
        aprsBeaconEnabled = doc["aprs_beacon"].as<bool>();
      }
      if (doc["aprs_symbol"].is<String>()) {
        applyAprsSymbol(doc["aprs_symbol"].as<String>());
      }
      if (doc["aprs_comment"].is<String>()) {
        aprsUserComment = sanitizeAprsComment(doc["aprs_comment"].as<String>());
      }
      bool aprsAlertConfigChanged = false;
      if (doc["aprs_alert"].is<String>()) {
        aprsAlertCsv = sanitizeAprsAlertList(doc["aprs_alert"].as<String>());
        aprsAlertConfigChanged = true;
      }
      if (doc["aprs_alert_enabled"].is<bool>()) {
        aprsAlertEnabled = doc["aprs_alert_enabled"].as<bool>();
      }
      if (doc["aprs_alert_nearby_enabled"].is<bool>()) {
        aprsAlertNearbyEnabled = doc["aprs_alert_nearby_enabled"].as<bool>();
      }
      if (doc["aprs_alert_wx_enabled"].is<bool>()) {
        aprsAlertWxEnabled = doc["aprs_alert_wx_enabled"].as<bool>();
        aprsAlertConfigChanged = true;
      }
      if (doc.containsKey("aprs_alert_min_sec")) {
        int alertMinSecCandidate = aprsAlertMinSeconds;
        if (doc["aprs_alert_min_sec"].is<int>()) {
          alertMinSecCandidate = doc["aprs_alert_min_sec"].as<int>();
        } else if (doc["aprs_alert_min_sec"].is<long>()) {
          alertMinSecCandidate = (int)doc["aprs_alert_min_sec"].as<long>();
        } else if (doc["aprs_alert_min_sec"].is<float>() || doc["aprs_alert_min_sec"].is<double>()) {
          alertMinSecCandidate = (int)doc["aprs_alert_min_sec"].as<double>();
        } else if (doc["aprs_alert_min_sec"].is<String>()) {
          String alertMinStr = doc["aprs_alert_min_sec"].as<String>();
          alertMinStr.trim();
          alertMinSecCandidate = alertMinStr.toInt();
        }
        applyAprsAlertMinSeconds(alertMinSecCandidate);
        aprsAlertConfigChanged = true;
      }
      if (doc.containsKey("aprs_alert_screen_sec")) {
        int alertScreenSecCandidate = aprsAlertScreenSeconds;
        if (doc["aprs_alert_screen_sec"].is<int>()) {
          alertScreenSecCandidate = doc["aprs_alert_screen_sec"].as<int>();
        } else if (doc["aprs_alert_screen_sec"].is<long>()) {
          alertScreenSecCandidate = (int)doc["aprs_alert_screen_sec"].as<long>();
        } else if (doc["aprs_alert_screen_sec"].is<float>() || doc["aprs_alert_screen_sec"].is<double>()) {
          alertScreenSecCandidate = (int)doc["aprs_alert_screen_sec"].as<double>();
        } else if (doc["aprs_alert_screen_sec"].is<String>()) {
          String alertScreenSecStr = doc["aprs_alert_screen_sec"].as<String>();
          alertScreenSecStr.trim();
          alertScreenSecCandidate = alertScreenSecStr.toInt();
        }
        applyAprsAlertScreenSeconds(alertScreenSecCandidate);
      }
      if (doc.containsKey("aprs_alert_distance_km")) {
        float alertDistanceCandidate = aprsAlertDistanceKm;
        if (doc["aprs_alert_distance_km"].is<float>() || doc["aprs_alert_distance_km"].is<double>()) {
          alertDistanceCandidate = (float)doc["aprs_alert_distance_km"].as<double>();
        } else if (doc["aprs_alert_distance_km"].is<int>()) {
          alertDistanceCandidate = (float)doc["aprs_alert_distance_km"].as<int>();
        } else if (doc["aprs_alert_distance_km"].is<long>()) {
          alertDistanceCandidate = (float)doc["aprs_alert_distance_km"].as<long>();
        } else if (doc["aprs_alert_distance_km"].is<String>()) {
          String alertDistanceStr = doc["aprs_alert_distance_km"].as<String>();
          alertDistanceStr.trim();
          alertDistanceStr.replace(',', '.');
          alertDistanceCandidate = alertDistanceStr.toFloat();
        }
        applyAprsAlertDistanceKm(alertDistanceCandidate);
      }
      if (doc["weather_alert_storm_enabled"].is<bool>()) {
        weatherAlertStormEnabled = doc["weather_alert_storm_enabled"].as<bool>();
        if (!weatherAlertStormEnabled) {
          nextWeatherStormAlertMs = 0;
        }
      }
      if (doc["weather_alert_wind_enabled"].is<bool>()) {
        weatherAlertWindEnabled = doc["weather_alert_wind_enabled"].as<bool>();
        if (!weatherAlertWindEnabled) {
          nextWeatherWindAlertMs = 0;
        }
      }
      if (doc.containsKey("weather_alert_wind_threshold_ms")) {
        float windThresholdCandidate = weatherAlertWindThresholdMs;
        if (doc["weather_alert_wind_threshold_ms"].is<float>() || doc["weather_alert_wind_threshold_ms"].is<double>()) {
          windThresholdCandidate = (float)doc["weather_alert_wind_threshold_ms"].as<double>();
        } else if (doc["weather_alert_wind_threshold_ms"].is<int>()) {
          windThresholdCandidate = (float)doc["weather_alert_wind_threshold_ms"].as<int>();
        } else if (doc["weather_alert_wind_threshold_ms"].is<long>()) {
          windThresholdCandidate = (float)doc["weather_alert_wind_threshold_ms"].as<long>();
        } else if (doc["weather_alert_wind_threshold_ms"].is<String>()) {
          String windThresholdStr = doc["weather_alert_wind_threshold_ms"].as<String>();
          windThresholdStr.trim();
          windThresholdStr.replace(',', '.');
          windThresholdCandidate = windThresholdStr.toFloat();
        }
        applyWeatherAlertWindThresholdMs(windThresholdCandidate);
      }
      if (doc["enable_led_alert"].is<bool>()) {
        enableLedAlert = doc["enable_led_alert"].as<bool>();
      }
      if (doc.containsKey("led_alert_duration_ms")) {
        int ledDurationCandidate = ledAlertDurationMs;
        if (doc["led_alert_duration_ms"].is<int>()) {
          ledDurationCandidate = doc["led_alert_duration_ms"].as<int>();
        } else if (doc["led_alert_duration_ms"].is<long>()) {
          ledDurationCandidate = (int)doc["led_alert_duration_ms"].as<long>();
        } else if (doc["led_alert_duration_ms"].is<float>() || doc["led_alert_duration_ms"].is<double>()) {
          ledDurationCandidate = (int)doc["led_alert_duration_ms"].as<double>();
        } else if (doc["led_alert_duration_ms"].is<String>()) {
          String ledDurationStr = doc["led_alert_duration_ms"].as<String>();
          ledDurationStr.trim();
          ledDurationCandidate = ledDurationStr.toInt();
        }
        applyLedAlertDurationMs(ledDurationCandidate);
      }
      if (doc.containsKey("led_alert_blink_ms")) {
        int ledBlinkCandidate = ledAlertBlinkMs;
        if (doc["led_alert_blink_ms"].is<int>()) {
          ledBlinkCandidate = doc["led_alert_blink_ms"].as<int>();
        } else if (doc["led_alert_blink_ms"].is<long>()) {
          ledBlinkCandidate = (int)doc["led_alert_blink_ms"].as<long>();
        } else if (doc["led_alert_blink_ms"].is<float>() || doc["led_alert_blink_ms"].is<double>()) {
          ledBlinkCandidate = (int)doc["led_alert_blink_ms"].as<double>();
        } else if (doc["led_alert_blink_ms"].is<String>()) {
          String ledBlinkStr = doc["led_alert_blink_ms"].as<String>();
          ledBlinkStr.trim();
          ledBlinkCandidate = ledBlinkStr.toInt();
        }
        applyLedAlertBlinkMs(ledBlinkCandidate);
      }
      if (doc.containsKey("aprs_interval_min")) {
        int intervalCandidate = aprsIntervalMinutes;
        if (doc["aprs_interval_min"].is<int>()) {
          intervalCandidate = doc["aprs_interval_min"].as<int>();
        } else if (doc["aprs_interval_min"].is<long>()) {
          intervalCandidate = (int)doc["aprs_interval_min"].as<long>();
        } else if (doc["aprs_interval_min"].is<float>() || doc["aprs_interval_min"].is<double>()) {
          intervalCandidate = (int)doc["aprs_interval_min"].as<double>();
        } else if (doc["aprs_interval_min"].is<String>()) {
          String intervalStr = doc["aprs_interval_min"].as<String>();
          intervalStr.trim();
          intervalCandidate = intervalStr.toInt();
        }
        applyAprsIntervalMinutes(intervalCandidate);
      }
      if (aprsAlertConfigChanged) {
        resetAprsAlertCooldownState();
      }
      if (doc["aprs_radius"].is<int>()) {
        aprsFilterRadius = doc["aprs_radius"].as<int>();
        // Ograniczenie promienia do 1-50 km
        if (aprsFilterRadius < 1) aprsFilterRadius = 1;
        if (aprsFilterRadius > 50) aprsFilterRadius = 50;
      }
      // Uwaga: APRS uĹĽywa wspĂłĹ‚rzÄ™dnych z sekcji "Moja Stacja" (userLat, userLon) - nie ma osobnych pĂłl
      
      // Konfiguracja trybu kalibracji dotyku (ręczne nadpisanie)
      if (doc["touch_swap_mode"].is<String>()) {
        String mode = doc["touch_swap_mode"].as<String>();
        // Resetuj wszystkie flagi
        touchSwapXY = false;
        touchInvertX = false;
        touchInvertY = false;
        // Ustaw odpowiednią flagę lub kombinację
        if (mode == "xy") {
          touchSwapXY = true;
        } else if (mode == "x") {
          touchInvertX = true;
        } else if (mode == "y") {
          touchInvertY = true;
        } else if (mode == "both") {
          touchInvertX = true;
          touchInvertY = true;
        } else if (mode == "rot90cw") {
          // Obrót 90° w prawo (clockwise)
          touchSwapXY = true;
          touchInvertX = true;
        } else if (mode == "rot90ccw") {
          // Obrót 90° w lewo (counter-clockwise)
          touchSwapXY = true;
          touchInvertY = true;
        } else if (mode == "xy_both") {
          touchSwapXY = true;
          touchInvertX = true;
          touchInvertY = true;
        }
        // "none" pozostawia wszystkie false
        Serial.print("Touch swap mode set to: ");
        Serial.println(mode);
      }
      
      // Touch rotation (0-3)
      if (doc["touch_rotation"].is<int>()) {
        int rot = doc["touch_rotation"].as<int>();
        if (rot >= 0 && rot <= 3) {
          touchRotation = (uint8_t)rot;
          Serial.print("Touch rotation set to: ");
          Serial.println(touchRotation);
        }
      }

      // TFT rotation (0-3)
      if (doc["tft_rotation"].is<int>()) {
        int rot = doc["tft_rotation"].as<int>();
        if (rot >= 0 && rot <= 3) {
          tftRotation = (uint8_t)rot;
          Serial.print("TFT rotation set to: ");
          Serial.println(tftRotation);
        }
      }

      // Sanitizacja (najczÄ™stsza przyczyna: spacje na koĹ„cu SSID/hasĹ‚a)
      wifiSSID.trim();
      wifiPassword.trim();
      wifiSSID2.trim();
      wifiPassword2.trim();
      clusterHost.trim();
      potaClusterHost.trim();
      potaFilterCommand.trim();
      potaApiUrl.trim();
      hamalertHost.trim();
      hamalertLogin.trim();
      hamalertPassword.trim();
      userCallsign.trim();
      userLocator.trim();
      clusterFilterCommands.trim();
      weatherApiKey.trim();
      openWebRxUrl.trim();
      aprsIsHost.trim();
      aprsCallsign.trim();
      // KiwiSDR URL - może być puste (użytkownik nie musi ustawiać)

      if (potaApiUrl.length() == 0) {
        potaApiUrl = DEFAULT_POTA_API_URL;
      }
      bool hamalertCredentialsProvided = (hamalertLogin.length() > 0 && hamalertPassword.length() > 0);
      if (hamalertCredentialsProvided) {
        if (hamalertHost.length() == 0) {
          hamalertHost = DEFAULT_HAMALERT_HOST;
        }
        if (hamalertPort <= 0 || hamalertPort > 65535) {
          hamalertPort = DEFAULT_HAMALERT_PORT;
        }
      } else {
        // Empty login/password means HAMALERT is intentionally disabled.
        hamalertHost = "";
        hamalertPort = 0;
      }

      // Kolejność ekranów konfigurowana z WWW
      if (doc["screen_order"].is<JsonArray>()) {
        JsonArray arr = doc["screen_order"].as<JsonArray>();
        int idx = 0;
        for (JsonVariant v : arr) {
          if (idx >= SCREEN_ORDER_COUNT) break;
          if (v.is<const char*>()) {
            String code = v.as<const char*>();
            ScreenType parsed = screenCodeToType(code);
            // Unknown code should not silently clear slot to OFF.
            if (parsed == SCREEN_OFF) {
              String tmp = code;
              tmp.toLowerCase();
              tmp.trim();
              if (tmp != "off") {
                parsed = DEFAULT_SCREEN_ORDER[idx];
              }
            }
            screenOrder[idx] = parsed;
          } else if (v.is<int>()) {
            int raw = v.as<int>();
            ScreenType parsed = normalizeScreenType(raw);
            if (parsed == SCREEN_OFF && raw != 0) {
              parsed = DEFAULT_SCREEN_ORDER[idx];
            }
            screenOrder[idx] = parsed;
          }
          idx++;
        }
        // Jeśli JSON był krótszy, pozostałe uzupełnij wartościami domyślnymi,
        // aby nie gubić ekranów (np. APRS RADAR) przy starszym/okrojonym payloadzie.
        for (; idx < SCREEN_ORDER_COUNT; idx++) {
          screenOrder[idx] = DEFAULT_SCREEN_ORDER[idx];
        }
      }
      ensureScreenOrderValid();

      if (doc["tft_auto_switch"].is<bool>()) {
        tftAutoSwitchEnabled = doc["tft_auto_switch"].as<bool>();
      }
      if (doc.containsKey("tft_switch_time_sec")) {
        int switchTimeCandidate = tftAutoSwitchTimeSec;
        if (doc["tft_switch_time_sec"].is<int>()) {
          switchTimeCandidate = doc["tft_switch_time_sec"].as<int>();
        } else if (doc["tft_switch_time_sec"].is<long>()) {
          switchTimeCandidate = (int)doc["tft_switch_time_sec"].as<long>();
        } else if (doc["tft_switch_time_sec"].is<float>() || doc["tft_switch_time_sec"].is<double>()) {
          switchTimeCandidate = (int)doc["tft_switch_time_sec"].as<double>();
        } else if (doc["tft_switch_time_sec"].is<String>()) {
          String switchTimeStr = doc["tft_switch_time_sec"].as<String>();
          switchTimeStr.trim();
          switchTimeCandidate = switchTimeStr.toInt();
        }
        applyTftAutoSwitchTimeSec(switchTimeCandidate);
      }
      resetTftAutoSwitchTimer();

      // Port bywa null, jeĹ›li w JS wyszedĹ‚ NaN
      if (doc["cluster_port"].is<int>()) {
        clusterPort = doc["cluster_port"].as<int>();
      } else {
        clusterPort = DEFAULT_CLUSTER_PORT;
      }
      if (doc["pota_port"].is<int>()) {
        potaClusterPort = doc["pota_port"].as<int>();
      } else {
        potaClusterPort = DEFAULT_POTA_CLUSTER_PORT;
      }

      if (wifiSSID.length() == 0 && wifiSSID2.length() == 0) {
        server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"empty_ssid\"}");
        return;
      }

      // Aktualizuj współrzędne z locatora TYLKO jeśli nie podano bezpośrednio współrzędnych
      // Priorytet: jeĹ›li userLatLonValid == true (podano bezpoĹ›rednio LAT/LON), nie nadpisuj z locatora
      if (!userLatLonValid && userLocator.length() >= 4) {
        updateUserLatLonFromLocator();
      }
      savePreferences();
      setBacklightPercent(backlightPercent);
      if (tftInitialized) {
        applyTftInversion();
      }
      
      server->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"restart\"}");
      
      // W trybie AP (wifiConnected=false) bez restartu nigdy nie przejdziemy na STA.
      // Restart jest najpewniejszy i upraszcza flow.
      Serial.println("Zapisano konfiguracjÄ™. Restart za chwilÄ™...");
      requestRestart(1500);
    } else {
      server->send(400, "application/json", "{\"status\":\"error\"}");
    }
  });
  
  // API - resetuj kalibrację dotyku do wartości domyślnych
  server->on("/api/reset_touch", HTTP_POST, []() {
    touchXMin = TOUCH_X_MIN;
    touchXMax = TOUCH_X_MAX;
    touchYMin = TOUCH_Y_MIN;
    touchYMax = TOUCH_Y_MAX;
    touchSwapXY = TOUCH_SWAP_XY;
    touchInvertX = TOUCH_INVERT_X;
    touchInvertY = TOUCH_INVERT_Y;
    touchRotation = 1;
    applyTouchRotation();
    
    savePreferences();
    
    Serial.println("Touch calibration reset to defaults");
    server->send(200, "application/json", "{\"status\":\"ok\"}");
  });

  // API - status runtime UART2 (GPS/BT)
  server->on("/api/uart_status", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    unsigned long nowMs = millis();
    bool gpsMode = isUart2GpsParsingEnabled();
    bool btMode = (uart2Mode == UART2_MODE_BT);

    doc["uart2_mode"] = uart2ModeToCode(uart2Mode);
    doc["uart2_connected_module"] = uart2ConnectedModule;
    doc["gps_active"] = gpsMode && isGpsUartActive(nowMs);
    doc["gps_fix"] = gpsMode && gpsLocationValid && isGpsFixActive(nowMs);
    doc["bt_active"] = btMode;
    doc["aprs_uart_active"] = isUart2AprsUartSourceEnabled();

    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - status skanera/parowania BT SPP dla ESP32-BRIDGE
  server->on("/api/bridge_bt_status", HTTP_GET, []() {
    StaticJsonDocument<4096> doc;
    doc["enabled"] = isUart2BridgeModuleSelected();
    doc["scan_in_progress"] = bridgeBtScanInProgress;
    doc["list_collecting"] = bridgeBtListCollecting;
    doc["list_expected"] = bridgeBtListExpected;
    doc["last_error"] = bridgeBtLastError;
    doc["last_ack"] = bridgeBtLastAck;
    doc["disc_status"] = bridgeBtLastDiscStatus;
    doc["last_scan_ms"] = bridgeBtLastScanMs;
    doc["pair_pending"] = bridgeBtPairPending;
    doc["pair_result"] = (bridgeBtPairLastResult > 0) ? "ok" : ((bridgeBtPairLastResult < 0) ? "err" : "none");
    doc["pair_result_ms"] = bridgeBtPairLastResultMs;
    doc["paired_name"] = bridgeBtPairedName;
    doc["paired_addr"] = formatBridgeBtAddressForDisplay(bridgeBtPairedAddress);

    JsonArray devices = doc.createNestedArray("devices");
    for (int i = 0; i < bridgeBtDeviceCount; i++) {
      JsonObject d = devices.createNestedObject();
      d["name"] = bridgeBtDevices[i].name;
      d["addr"] = formatBridgeBtAddressForDisplay(bridgeBtDevices[i].addressNoColon);
      d["addr_raw"] = bridgeBtDevices[i].addressNoColon;
    }

    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - uruchom skan BT SPP dla ESP32-BRIDGE
  server->on("/api/bridge_bt_scan", HTTP_POST, []() {
    if (!isUart2BridgeModuleSelected()) {
      server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"bridge_mode_required\"}");
      return;
    }

    startBridgeScanSequence();
    server->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"scan_started\"}");
  });

  // API - sparuj/połącz z urządzeniem BT SPP
  server->on("/api/bridge_bt_pair", HTTP_POST, []() {
    if (!isUart2BridgeModuleSelected()) {
      server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"bridge_mode_required\"}");
      return;
    }

    if (!server->hasArg("plain")) {
      server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"missing_payload\"}");
      return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, server->arg("plain"));
    if (err) {
      server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"json_parse\"}");
      return;
    }

    String name = doc["name"].is<String>() ? doc["name"].as<String>() : "";
    String addr = doc["address"].is<String>() ? doc["address"].as<String>() : "";
    String password = doc["password"].is<String>() ? doc["password"].as<String>() : "";
    if (password.length() > 16) {
      server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"password_too_long\"}");
      return;
    }

    if (!requestBridgePairConnect(name, addr, password, true)) {
      String msg = bridgeBtLastError.length() ? bridgeBtLastError : String("pair_start_failed");
      msg.replace("\"", "'");
      server->send(400, "application/json", String("{\"status\":\"error\",\"message\":\"") + msg + "\"}");
      return;
    }

    server->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"pair_started\"}");
  });

  // API - resetuj rotację TFT do wartości domyślnych
  server->on("/api/reset_tft_rotation", HTTP_POST, []() {
    tftRotation = 1;
    applyTftRotation();
    savePreferences();

    Serial.println("TFT rotation reset to default");
    server->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"restart\"}");
    requestRestart(1500);
  });
  
  // API - pobierz konfigurację
  server->on("/api/config", HTTP_GET, []() {
    StaticJsonDocument<4096> doc;
    doc["wifi_ssid"] = wifiSSID;
    doc["wifi_pass"] = wifiPassword;
    doc["wifi_ssid2"] = wifiSSID2;
    doc["wifi_pass2"] = wifiPassword2;
    doc["uart2_mode"] = uart2ModeToCode(uart2Mode);
    doc["uart2_baud"] = uart2BaudRate;
    doc["uart2_data_bits"] = uart2DataBits;
    doc["uart2_parity"] = uart2ParityToCode(uart2Parity);
    doc["uart2_stop_bits"] = uart2StopBits;
    doc["uart2_rx_pin"] = GPS_UART_RX_PIN;
    doc["uart2_tx_pin"] = GPS_UART_TX_PIN;
    doc["cluster_host"] = clusterHost;
    doc["cluster_port"] = clusterPort;
    doc["pota_host"] = potaClusterHost;
    doc["pota_port"] = potaClusterPort;
    doc["pota_filter"] = potaFilterCommand;
    doc["pota_api_url"] = potaApiUrl;
    doc["hamalert_host"] = hamalertHost;
    doc["hamalert_port"] = hamalertPort;
    doc["hamalert_login"] = hamalertLogin;
    doc["hamalert_password"] = hamalertPassword;
    doc["user_callsign"] = userCallsign;
    doc["user_locator"] = userLocator;
    doc["timezone"] = timezoneHours;
    doc["user_lat"] = userLat;
    doc["user_lon"] = userLon;
    doc["user_lat_lon_valid"] = userLatLonValid;
    doc["use_gps_if_active"] = useGpsIfActive;
    doc["uart2_connected_module"] = uart2ConnectedModule;
    doc["bridge_paired_name"] = bridgeBtPairedName;
    doc["bridge_paired_addr"] = formatBridgeBtAddressForDisplay(bridgeBtPairedAddress);
    doc["qrz_user"] = qrzUsername;
    doc["qrz_pass"] = qrzPassword;
    doc["weather_key"] = weatherApiKey;
    doc["openwebrx_url"] = openWebRxUrl;
    doc["tft_backlight"] = backlightPercent;
    doc["tft_night_dimming"] = tftNightDimmingPercent;
    doc["tft_invert"] = tftInvertColors;
    doc["tft_rotation"] = tftRotation;
    doc["tft_lang"] = tftLangToCode(tftLanguage);
    doc["tft_style"] = tftStyleToCode(tftStyleMode);
    doc["table_size"] = dxTableSizeToCode(dxTableSizeMode);
    doc["tft_auto_switch"] = tftAutoSwitchEnabled;
    doc["tft_switch_time_sec"] = tftAutoSwitchTimeSec;
    doc["menu_hue"] = menuThemeHue;
    doc["qrz_status"] = qrzStatus;
    doc["cluster_noann"] = clusterNoAnnouncements;
    doc["cluster_nowwv"] = clusterNoWWV;
    doc["cluster_nowcy"] = clusterNoWCY;
    doc["cluster_usefilters"] = clusterUseFilters;
    doc["cluster_filters"] = clusterFilterCommands;
    doc["aprs_host"] = aprsIsHost;
    doc["aprs_port"] = aprsIsPort;
    doc["aprs_callsign"] = aprsCallsign;
    doc["aprs_passcode"] = aprsPasscode;
    doc["aprs_ssid"] = aprsSsid;
    doc["aprs_radius"] = aprsFilterRadius;
    doc["aprs_beacon"] = aprsBeaconEnabled;
    doc["aprs_symbol"] = aprsSymbolTwoChar;
    doc["aprs_comment"] = aprsUserComment;
    doc["aprs_alert"] = aprsAlertCsv;
    doc["aprs_alert_enabled"] = aprsAlertEnabled;
    doc["aprs_alert_nearby_enabled"] = aprsAlertNearbyEnabled;
    doc["aprs_alert_wx_enabled"] = aprsAlertWxEnabled;
    doc["weather_alert_storm_enabled"] = weatherAlertStormEnabled;
    doc["weather_alert_wind_enabled"] = weatherAlertWindEnabled;
    doc["weather_alert_wind_threshold_ms"] = weatherAlertWindThresholdMs;
    doc["aprs_alert_min_sec"] = aprsAlertMinSeconds;
    doc["aprs_alert_screen_sec"] = aprsAlertScreenSeconds;
    doc["aprs_alert_distance_km"] = aprsAlertDistanceKm;
    doc["enable_led_alert"] = enableLedAlert;
    doc["led_alert_duration_ms"] = ledAlertDurationMs;
    doc["led_alert_blink_ms"] = ledAlertBlinkMs;
    doc["aprs_interval_min"] = aprsIntervalMinutes;
    
    // Touch calibration mode - priorytet: pełna kombinacja > rotacje > swap > both > single invert
    String touchMode = "none";
    if (touchSwapXY && touchInvertX && touchInvertY) {
      touchMode = "xy_both";
    } else if (touchSwapXY && touchInvertX) {
      touchMode = "rot90cw";
    } else if (touchSwapXY && touchInvertY) {
      touchMode = "rot90ccw";
    } else if (touchSwapXY) {
      touchMode = "xy";
    } else if (touchInvertX && touchInvertY) {
      touchMode = "both";
    } else if (touchInvertX) {
      touchMode = "x";
    } else if (touchInvertY) {
      touchMode = "y";
    }
    doc["touch_swap_mode"] = touchMode;
    doc["touch_rotation"] = touchRotation;
    
    JsonArray orderArr = doc.createNestedArray("screen_order");
    for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
      orderArr.add(screenTypeToCodeStr(screenOrder[i]));
    }
    
    
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });
  
  server->begin();
  Serial.println("Serwer WWW uruchomiony");
}

// ========== HTML PAGES ==========

String getMainHTML() {
  return "<!DOCTYPE html><html><head><meta charset='UTF-8'></head>"
         "<body>index.html missing in LittleFS</body></html>";
}
String getConfigHTML() {
  return getMainHTML(); // UĹĽywa tego samego HTML z JavaScript
}

#ifdef ENABLE_TFT_DISPLAY
TaskHandle_t uiTaskHandle = nullptr;

void uiTaskLoop(void *parameter) {
  (void)parameter;
  for (;;) {
    unsigned long now = millis();

    if (tftInitialized) {
      bool pendingScreen1 = false;
      bool pendingScreen2 = false;
      bool pendingScreen6 = false;
      bool pendingScreen7 = false;
      bool pendingAnyScreen = false;
      uint8_t pendingAnyScreenId = SCREEN_HAM_CLOCK;
      portENTER_CRITICAL(&uiPendingRedrawMux);
      pendingScreen1 = uiPendingScreen1Redraw;
      pendingScreen2 = uiPendingScreen2Redraw;
      pendingScreen6 = uiPendingScreen6Redraw;
      pendingScreen7 = uiPendingScreen7Redraw;
      pendingAnyScreen = uiPendingAnyScreenRedraw;
      pendingAnyScreenId = uiPendingAnyScreenId;
      uiPendingScreen1Redraw = false;
      uiPendingScreen2Redraw = false;
      uiPendingScreen6Redraw = false;
      uiPendingScreen7Redraw = false;
      uiPendingAnyScreenRedraw = false;
      portEXIT_CRITICAL(&uiPendingRedrawMux);

      if (pendingAnyScreen && currentScreen == (ScreenType)pendingAnyScreenId && !inMenu && !aprsAlertScreenActive) {
        drawScreen((ScreenType)pendingAnyScreenId);
      }

      if (pendingScreen1 && currentScreen == SCREEN_HAM_CLOCK && !inMenu && !aprsAlertScreenActive) {
        drawScreen(SCREEN_HAM_CLOCK);
      }
      if (pendingScreen2 && currentScreen == SCREEN_DX_CLUSTER && !inMenu && !aprsAlertScreenActive) {
        drawScreen(SCREEN_DX_CLUSTER);
      }
      if (pendingScreen7 && currentScreen == SCREEN_POTA_CLUSTER && !inMenu && !aprsAlertScreenActive) {
        drawScreen(SCREEN_POTA_CLUSTER);
      }
      if (pendingScreen6 && (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) && !inMenu && !aprsAlertScreenActive) {
        drawScreen(currentScreen);
      }

      if (aprsAlertDrawPending && !brightnessMenuActive) {
        APRSStation pendingStation;
        portENTER_CRITICAL(&aprsAlertPendingMux);
        pendingStation = aprsAlertPendingStation;
        aprsAlertDrawPending = false;
        portEXIT_CRITICAL(&aprsAlertPendingMux);
        ALERT_Screen(pendingStation);
      }
      if (weatherAlertDrawPending && !brightnessMenuActive && !aprsAlertScreenActive) {
        WeatherAlertInfo pendingInfo;
        portENTER_CRITICAL(&weatherAlertPendingMux);
        pendingInfo = weatherAlertPendingInfo;
        weatherAlertDrawPending = false;
        portEXIT_CRITICAL(&weatherAlertPendingMux);
        WEATHER_ALERT_Screen(pendingInfo);
      }
      updateAlertScreenTimeout();
      handleTouchNavigation();
    }

    if (tftAutoSwitchEnabled && tftInitialized && !inMenu && !aprsAlertScreenActive && !touchCalActive && !brightnessMenuActive && currentScreen != SCREEN_UNLIS_HUNTER) {
      unsigned long nowSwitch = millis();
      if (autoswitchPausedUntilMs > 0 && nowSwitch < autoswitchPausedUntilMs) {
        // autoswitch zapauzowany po dotknięciu środka ekranu
      } else {
      if (tftAutoSwitchLastMs == 0 || tftAutoSwitchLastScreen != currentScreen) {
        tftAutoSwitchLastMs = nowSwitch;
        tftAutoSwitchLastScreen = currentScreen;
      } else {
        unsigned long intervalMs = (unsigned long)tftAutoSwitchTimeSec * 1000UL;
        if (intervalMs > 0 && (nowSwitch - tftAutoSwitchLastMs) >= intervalMs) {
          ScreenType nextScreen = getNextScreenId(currentScreen);
          if (nextScreen != SCREEN_OFF && nextScreen != currentScreen) {
            currentScreen = nextScreen;
            drawScreen(currentScreen);
          }
          tftAutoSwitchLastMs = nowSwitch;
          tftAutoSwitchLastScreen = currentScreen;
        }
      }
      } // koniec else (autoswitch nie zapauzowany)
    }

    if (!restartRequested) {
      if (tftInitialized && currentScreen == SCREEN_HAM_CLOCK && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen1UpdateMs > 1000) {
          updateScreen1Clock();
          lastScreen1UpdateMs = now;
        }
        updateScreen1Header();
        updateScreen1Date();
      }

      if (tftInitialized && currentScreen == SCREEN_DX_CLUSTER && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreenUpdate > 100) {
          updateScreen2Data();
          lastScreenUpdate = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_POTA_CLUSTER && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen7UpdateMs > 200) {
          updateScreen7Data();
          lastScreen7UpdateMs = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_HAMALERT_CLUSTER && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen8UpdateMs > 200) {
          updateScreen8Data();
          lastScreen8UpdateMs = now;
        }
      }

      if (tftInitialized && (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreenUpdate > 100) {
          updateScreen6Data();
          lastScreenUpdate = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_SUN_SPOTS && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen3UpdateMs > 1000) {
          updateScreen3Data();
          lastScreen3UpdateMs = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_BAND_INFO && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen4UpdateMs > 1000) {
          updateScreen4Data();
          lastScreen4UpdateMs = now;
        }
      }

      if (tftInitialized &&
          (currentScreen == SCREEN_WEATHER_DSP || currentScreen == SCREEN_WEATHER_FORECAST) &&
          !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen5UpdateMs > 1000) {
          updateScreen5Data();
          lastScreen5UpdateMs = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_LOKALIZACJA && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreenLocationUpdateMs > 1000) {
          updateLokalizacjaFields();
          lastScreenLocationUpdateMs = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_MATRIX_CLOCK && !inMenu && !aprsAlertScreenActive) {
        updateScreen10();
      }

      if (tftInitialized && currentScreen == SCREEN_UNLIS_HUNTER && !inMenu && !aprsAlertScreenActive) {
        updateUnlisHunter();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif

// ========== SETUP & LOOP ==========

void setup() {
  // Ten log idzie "kana" (zwykle tym samym co bootlog),
  // wiÄ™c pozwala sprawdziÄ‡ czy aplikacja w ogĂłle startuje.
  esp_rom_printf("\nAPP: setup() start\n");

  Serial.begin(115200);
  // Na ESP32-C3 (zwłaszcza z USB CDC) warto chwilę poczekać na monitor portu
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) {
    delay(10);
  }
  delay(200);
  esp_rom_printf("APP: Serial.begin done\n");

  gpsNmeaLine.reserve(180);
  gpsLastRxMs = 0;
  gpsRawBridgeStartMs = 0;
  gpsRawBridgeEndLogged = true;
  gpsRawLastStatsMs = 0;
  gpsRawBytesTotal = 0;
  gpsRawBytesSinceStats = 0;

  // Daj chwilÄ™ na ustabilizowanie WiFi/PHY po resecie (zwĹ‚aszcza na "Super Mini")
  delay(300);

  if (dxSpotsMutex == nullptr) {
    dxSpotsMutex = xSemaphoreCreateMutex();
  }
  if (potaSpotsMutex == nullptr) {
    potaSpotsMutex = xSemaphoreCreateMutex();
  }
  if (hamalertSpotsMutex == nullptr) {
    hamalertSpotsMutex = xSemaphoreCreateMutex();
  }
  if (weatherDataMutex == nullptr) {
    weatherDataMutex = xSemaphoreCreateMutex();
  }

  initStatusRgbLed();
  updateStatusRgbLed();
  
  // WyĹ›wietlacz TFT (ESP32-2432S028) â€“ inicjalizacja NA POCZÄ„TKU (jak w projekcie referencyjnym)
  // TFT powinien byÄ‡ inicjalizowany zaraz po Serial.begin(), przed innymi peryferiami
#ifdef ENABLE_TFT_DISPLAY
  initTFT();
  yield();
#endif

  littleFsReady = LittleFS.begin(true);
  logRamUsage(millis());
  if (littleFsReady) {
    bootLogLine("LittleFS OK");
    if (LittleFS.exists(ROBOTO_FONT20_FILE)) {
      File f = LittleFS.open(ROBOTO_FONT20_FILE, "r");
      if (f) {
        bootLogLine(String("Font OK: ") + ROBOTO_FONT20_FILE + " size=" + f.size());
        f.close();
      }
    } else {
      bootLogLine(String("Font missing: ") + ROBOTO_FONT20_FILE);
    }
  } else {
    bootLogLine("LittleFS FAIL");
  }

  bootLogLine("");
  bootLogLine("ESP32 DX Cluster Receiver");
  bootLogLine("==============================");
  
  // DIAGNOSTYKA: Sprawdź czy ENABLE_TFT_DISPLAY jest zdefiniowane
#ifdef ENABLE_TFT_DISPLAY
  bootLogLine("ENABLE_TFT_DISPLAY - init TFT");
#else
  bootLogLine("UWAGA: ENABLE_TFT_DISPLAY NIE jest zdefiniowane - TFT dont work!");
  bootLogLine("Upewnij sie, że kompilujesz dla środowiska 'esp32-2432s028'");
#endif
  yield();

  bootLogLine("Config load...");
  loadPreferences();
  telnetBuffer.reserve(384);
  pendingTelnetLine.reserve(384);
  potaTelnetBuffer.reserve(384);
  pendingPotaLine.reserve(384);
  aprsBuffer.reserve(384);
  qrzStatus.reserve(64);
  clusterFilterCommands.reserve(128);
  weatherData.description.reserve(96);
  weatherData.cityName.reserve(48);
  weatherData.forecast3hDesc.reserve(64);
  weatherData.forecastNextDayDesc.reserve(64);
  weatherData.forecastError.reserve(80);
  weatherData.lastError.reserve(64);
  weatherData.iconCode.reserve(8);
  propagationData.lastError.reserve(48);
  yield();
  configureUart2Port();
#ifdef ENABLE_TFT_DISPLAY
  applyTftRotation();
  applyTouchRotation();
  setBacklightPercent(backlightPercent);
#endif
  
  bootLogLine("WIFI starting...");
  if (wifiSSID.length() > 0) {
    bootLogLine("Connecting WiFi: " + wifiSSID);
    if (connectToWiFi()) {
      wifiConnectedAtMs = millis();
      bootLogLine("WiFi connected!");
      yield();
      updateNTPTime();
      yield();
      connectToCluster();
      connectToPotaCluster();
    } else {
      bootLogLine("WiFi error, starting AP mode...");
      startAPMode();
    }
  } else {
    bootLogLine("No WiFi configuration, starting AP mode...");
    startAPMode();
  }
  yield();
  
  bootLogLine("Starting web server...");
  setupWebServer();
  yield();
  
  bootLogLine("System ready!");
  String ipStr = wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  bootLogLine("IP: " + ipStr);
  
  // Aktualizuj wyĹ›wietlacz TFT (ekran startowy)
#ifdef ENABLE_TFT_DISPLAY
  drawWelcomeScreenYellow();
  delayWithRamLogs(4000);
  drawWelcomeScreenGreen();
  delayWithRamLogs(3000);

  if (littleFsReady) {
    warmupSmoothFonts();
  }
  logRamUsage(millis());

  bootSequenceActive = false;
  drawScreen(currentScreen);
  resetTftAutoSwitchTimer();
  if (currentScreen == SCREEN_HAM_CLOCK) {
    updateScreen1();
  }

  if (uiTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(
      uiTaskLoop,
      "UI_Task",
      8192,
      nullptr,
      2,
      &uiTaskHandle,
      1
    );
  }
#endif
}

void loop() {
  static unsigned long loopCounter = 0;
  static unsigned long lastLoopPrint = 0;
  loopCounter++;
  
  unsigned long now = millis();
  
  // Feed watchdog
  yield();

  logRamUsage(now);
  pollGpsSerial();
  serviceBridgeBtsppOnRetry(now);
  serviceBridgeBootForceCommands(now);
  serviceAprsTncBootRestart(now);
  logGpsStatusEvery5s(now);

  bool gpsTimeActiveNow = isGpsTimeSourceActive(now);
  if (gpsTimeActiveNow) {
    syncSystemTimeFromGps(now);
  }

  if (gpsTimeActiveNow != gpsTimeSourceWasActive) {
    if (gpsTimeActiveNow) {
      Serial.printf("[TIME] Zrodlo czasu: GPS FIX (%s)\n", formatGpsUtcForLog().c_str());
    } else {
      Serial.println("[TIME] Zrodlo czasu: fallback NTP/www (brak aktywnego GPS FIX)");
      // WymuĹ› odĹ›wieĹĽenie NTP zaraz po utracie GPS.
      lastNTPUpdate = 0;
    }
    gpsTimeSourceWasActive = gpsTimeActiveNow;
  }

  // Odroczony restart (ĹĽeby odpowiedĹş HTTP zdÄ…ĹĽyĹ‚a wyjĹ›Ä‡)
  if (restartRequested && (long)(millis() - restartAtMs) >= 0) {
    LOGV_PRINTLN("[LOOP] Restart requested - restarting...");
    delay(50);
    ESP.restart();
  }
  
  // ObsĹ‚uga serwera WWW
  if (server != nullptr) {
    server->handleClient();
  }
  
  // Aktualizuj czas NTP
  updateNTPTime();
  
  // ObsĹ‚uga WiFi
  if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    wifiConnectedAtMs = now;
    Serial.print("STA połączone…czone. IP: ");
    Serial.println(WiFi.localIP());
    // Jeżeli wcześniej był uruchomiony AP do konfiguracji, wyłącz go po udanym połączeniu STA,
    // żeby nie robić wrażenia "zwiechy" (klient AP traci link przy przełączeniu kanału).
    if (WiFi.getMode() & WIFI_AP) {
      Serial.println("Wyłączam AP (portal) po połączeniu STA. Użyj IP z sieci domowej.");
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
    }
    

    updateNTPTime();
    connectToCluster();
    connectToPotaCluster();
  } else if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    LOGV_PRINTLN("[LOOP] WiFi status zmieniony: połączony -> rozłączony");
    wifiConnected = false;
    telnetConnected = false;
    potaTelnetConnected = false;
    aprsConnected = false;
    if (aprsClient.connected()) {
      aprsClient.stop();
    }
    LOGV_PRINTLN("[LOOP] STA rozłączone (wracam do trybu offline/AP jeśli aktywne).");
  }

  updateStatusRgbLed();

  // W trybie STA okresowo ponawiaj aktywną próbę połączenia.
  retryWiFiFromSTAIfDue(now);

  // W trybie AP okresowo podejmuj bezpieczną próbę powrotu do STA.
  retryWiFiFromAPIfDue(now);

  // Jeśli jesteśmy offline, zostajemy w AP (portal) — stabilnie.

  const bool networkServicesReady = wifiConnected && ((now - wifiConnectedAtMs) >= NETWORK_SERVICES_START_DELAY_MS);
  
  // Obsługa Telnet
  if (wifiConnected) {
    if (!telnetConnected) {
      LOGV_PRINTLN("[LOOP] WiFi OK, próba połączenia z Cluster...");
      connectToCluster();
    } else {
      handleTelnetData();
    }
  }

  // Obsługa POTA Telnet
  if (networkServicesReady) {
    // Tryb HTTP API zamiast Telnetu (źródło: https://api.pota.app/v1/spots)
    if (lastPotaApiFetchMs == 0 || now - lastPotaApiFetchMs > POTA_API_FETCH_INTERVAL_MS) {
      if (fetchPotaApi()) {
        lastPotaApiFetchMs = now;
      } else {
        // nawet przy błędzie aktualizuj czas, by nie spamować API
        lastPotaApiFetchMs = now;
      }
    }

    // HAMALERT przez Telnet: set/json + sh/dx 30
    if (lastHamalertFetchMs == 0 || now - lastHamalertFetchMs > HAMALERT_FETCH_INTERVAL_MS) {
      fetchHamalertTelnet();
      lastHamalertFetchMs = now;
    }
  }

  // ObsĹ‚uga APRS-IS (wylaczona, gdy APRS idzie z UART2 TNC)
  if (networkServicesReady) {
    if (isAprsNetworkFeedEnabled()) {
      if (!aprsConnected) {
        LOGV_PRINTLN("[LOOP] WiFi OK, prĂłba poĹ‚Ä…czenia z APRS-IS...");
        connectToAPRS();
      } else {
        handleAPRSData();
        unsigned long nowAprs = millis();
        refreshAprsFilterIfNeeded(nowAprs);
        
        // Watchdog APRS: jeĹ›li brak danych przez dĹ‚uĹĽszy czas, zrĂłb reconnect
        unsigned long inactivityTime = nowAprs - lastAPRSRxMs;
        if (inactivityTime > APRS_INACTIVITY_RECONNECT_MS) {
          LOGV_PRINT("[LOOP] WARNING: Brak danych z APRS-IS przez ");
          LOGV_PRINT(inactivityTime / 1000);
          LOGV_PRINTLN(" sekund -> reconnect");
          aprsClient.stop();
          aprsConnected = false;
          aprsLoginSent = false;
          // connectToAPRS() odpali siÄ™ w kolejnych iteracjach
        }
      }
    } else {
      if (aprsConnected || aprsClient.connected()) {
        aprsClient.stop();
      }
      aprsConnected = false;
      aprsLoginSent = false;
      if (!isUart2AprsTncKissModuleSelected()) {
        nextAPRSPositionDueMs = 0;
        aprsBeaconTxCount = 0;
      }
    }
  }

  serviceAprsBeaconTx(now);

  // Aktualizuj dane propagacyjne (hamqsl solarxml)
  if (wifiConnected) {
    unsigned long now = millis();
    unsigned long interval = lastPropagationFetchOk ? PROPAGATION_FETCH_INTERVAL_MS
                                                   : PROPAGATION_FETCH_RETRY_MS;
    if (lastPropagationFetchMs == 0 || now - lastPropagationFetchMs > interval) {
      lastPropagationFetchOk = fetchPropagationData();
      lastPropagationFetchMs = now;

    }
  }

  // ObsĹ‚uga kolejki QRZ (asynchronicznie)
  if (wifiConnected && qrzQueueLen > 0 &&
      qrzUsername.length() > 0 && qrzPassword.length() > 0) {
    unsigned long now = millis();
    unsigned long qrzInterval = getQrzLookupIntervalMs();
    if (now - lastQrzLookupMs >= qrzInterval) {
      for (int i = 0; i < qrzQueueLen; i++) {
        if (now < qrzQueue[i].nextTryMs) {
          continue;
        }
        String grid;
        String country;
        double lat = 0.0;
        double lon = 0.0;
        bool hasLatLon = false;
        bool ok = fetchQrzCallsignInfo(qrzQueue[i].callsign, grid, country, lat, lon, hasLatLon);
        lastQrzLookupMs = now;
        if (ok) {
          updateSpotsWithQrz(qrzQueue[i].callsign, grid, country, lat, lon, hasLatLon);
          removeQrzQueueAt(i);
        } else {
          qrzQueue[i].attempts++;
          if (qrzQueue[i].attempts >= QRZ_RETRY_LIMIT) {
            removeQrzQueueAt(i);
          } else {
            qrzQueue[i].nextTryMs = now + QRZ_RETRY_DELAY_MS;
          }
        }
        break; // jedna prĂłba na iteracjÄ™
      }
    }
  }

  // Aktualizuj pogodÄ™ (OpenWeather)
  if (wifiConnected) {
    unsigned long now = millis();
    unsigned long interval = lastWeatherFetchOk ? WEATHER_FETCH_INTERVAL_MS
                                               : WEATHER_FETCH_RETRY_MS;
    if (lastWeatherFetchMs == 0 || now - lastWeatherFetchMs > interval) {
      lastWeatherFetchOk = fetchWeatherData();
      lastWeatherFetchMs = now;
      applyBacklightByDayNight(true);
    }
  }

  // Weryfikacja dzień/noc z OWM co 5 minut i dostosowanie podświetlenia TFT
  if (tftInitialized && !brightnessMenuActive) {
    if (lastBacklightDayNightCheckMs == 0 || (now - lastBacklightDayNightCheckMs) >= BACKLIGHT_DAY_NIGHT_CHECK_INTERVAL_MS) {
      applyBacklightByDayNight(false);
      lastBacklightDayNightCheckMs = now;
    }
  }

  processWeatherAlertSchedule();

  // Przetwarzaj maks. 1 liniÄ™ telnet na iteracjÄ™ (ĹĽeby nie zamroziÄ‡ WWW/UI)
  if (pendingTelnetLine.length() > 0) {
    LOGV_PRINT("[LOOP] Przetwarzanie linii telnet, len=");
    LOGV_PRINTLN(pendingTelnetLine.length());
    LOGV_PRINT("[LOOP] Linia: ");
    if (pendingTelnetLine.length() > 80) {
      LOGV_PRINTLN(pendingTelnetLine.substring(0, 80) + "...");
    } else {
      LOGV_PRINTLN(pendingTelnetLine);
    }
    
    String line = pendingTelnetLine;
    pendingTelnetLine = ""; // WyczyĹ›Ä‡ PRZED parsowaniem (ĹĽeby nie gromadziÄ‡)
    yield(); // Feed watchdog przed dĹ‚ugÄ… operacjÄ…
    
    DXSpot spot;
    unsigned long parseStart = millis();
    if (parseDXSpot(line, spot)) {
      unsigned long parseTime = millis() - parseStart;
      if (parseTime > 50) {
        LOGV_PRINT("[LOOP] WARNING: parseDXSpot zajÄ™Ĺ‚o ");
        LOGV_PRINT(parseTime);
        LOGV_PRINTLN("ms");
      }
      addSpot(spot);
      // Aktualizuj wyĹ›wietlacz TFT z nowymi spotami (tylko jeĹ›li jesteĹ›my na ekranie 2)

    }
  }

  if (pendingPotaLine.length() > 0) {
    String line = pendingPotaLine;
    pendingPotaLine = "";

    DXSpot spot;
    if (parsePotaSpot(line, spot) && spot.mode == "SSB") {
      applyQrzCacheToSpot(spot, QRZ_CACHE_TTL_MS);
      addPotaSpot(spot);
      bool qrzConfigured = (qrzUsername.length() > 0 && qrzPassword.length() > 0);
      // Dodaj lookup QRZ tylko jeśli country jest puste (nie dubluj dla DX Cluster)
      if (qrzConfigured && spot.country.length() == 0 && spot.callsign.length() > 0) {
        String call = spot.callsign;
        call.toUpperCase();
        enqueueQrzLookup(call);
      }

    }
  }

  // Watchdog telnet: jeĹ›li brak danych z clustra przez dĹ‚uĹĽszy czas, zrĂłb reconnect
  if (telnetConnected && telnetClient.connected()) {
    unsigned long now = millis();
    unsigned long inactivityTime = now - lastTelnetRxMs;
    if (inactivityTime > TELNET_INACTIVITY_RECONNECT_MS) {
      LOGV_PRINT("[LOOP] WARNING: Brak danych z DX Cluster przez ");
      LOGV_PRINT(inactivityTime / 1000);
      LOGV_PRINTLN(" sekund -> reconnect");
      telnetClient.stop();
      telnetConnected = false;
      clusterLoginSent = false;
      clusterLoginScheduled = false;
      // connectToCluster() odpali siÄ™ w kolejnych iteracjach
    } else if (inactivityTime > 240000) { // OstrzeĹĽenie po 4 minutach
      LOGV_PRINT("[LOOP] WARNING: Brak danych z Cluster przez ");
      LOGV_PRINT(inactivityTime / 1000);
      LOGV_PRINTLN(" sekund (blisko timeout)");
    }
  }

  if (potaTelnetConnected && potaTelnetClient.connected()) {
    unsigned long now = millis();
    unsigned long inactivityTime = now - lastPotaRxMs;
    if (inactivityTime > POTA_TELNET_INACTIVITY_RECONNECT_MS) {
      LOGV_PRINT("[LOOP] WARNING: Brak danych z POTA Cluster przez ");
      LOGV_PRINT(inactivityTime / 1000);
      LOGV_PRINTLN(" sekund -> reconnect");
      potaTelnetClient.stop();
      potaTelnetConnected = false;
      potaLoginSent = false;
      potaLoginScheduled = false;
    }
  }

  // WyĹ›lij znak (login) jeĹ›li zaplanowany
  if (telnetConnected && telnetClient.connected() && clusterLoginScheduled) {
    if ((long)(millis() - clusterSendLoginAtMs) >= 0) {
      LOGV_PRINTLN("[LOOP] WysyĹ‚anie loginu do Cluster...");
      clusterLoginScheduled = false;
      if (!clusterLoginSent) {
        String login = userCallsign;
        login.trim();
        if (login.length() == 0) {
          login = DEFAULT_CALLSIGN;
        }
        
        // JeĹ›li mamy lokator, dodaj go do loginu w formacie: callsign/locator
        // Format zgodny z CC-Cluster (dxspots.com) i wiÄ™kszoĹ›ciÄ… DX ClusterĂłw
        if (userLocator.length() >= 4) {
          login += "/";
          login += userLocator;
          Serial.print("[CLUSTER] Login -> ");
          Serial.print(login);
          Serial.print(" (callsign/locator)");
          if (userCallsign.length() == 0) {
            Serial.print(" - domyĹ›lny znak, ustaw swĂłj w Config");
          }
          Serial.println();
        } else {
          Serial.print("[CLUSTER] Login -> ");
          Serial.print(login);
          if (userCallsign.length() == 0) {
            Serial.print(" (domyĹ›lny, ustaw swĂłj znak w Config)");
          } else {
            Serial.print(" (bez lokatora - ustaw w Config dla lepszej funkcjonalnoĹ›ci)");
          }
          Serial.println();
        }
        
        telnetClient.print(login);
        telnetClient.print("\r\n");
        clusterLoginSent = true;
        lastClusterKeepAliveMs = millis();
        Serial.println("[CLUSTER] Login wys\u0142any");
        
        // Po zalogowaniu wyĹ›lij komendy konfiguracyjne CC-Cluster (z opĂłĹşnieniem)
        // Daj czas clusterowi na przetworzenie loginu
        delay(500);
        sendClusterConfigCommands();
      }
    }
  }

  if (potaTelnetConnected && potaTelnetClient.connected() && potaLoginScheduled) {
    if ((long)(millis() - potaSendLoginAtMs) >= 0) {
      potaLoginScheduled = false;
      if (!potaLoginSent) {
        String login = userCallsign;
        login.trim();
        if (login.length() == 0) {
          login = DEFAULT_CALLSIGN;
        }
        if (userLocator.length() >= 4) {
          login += "/";
          login += userLocator;
        }
        potaTelnetClient.print(login);
        potaTelnetClient.print("\r\n");
        potaLoginSent = true;
        lastPotaKeepAliveMs = millis();
      }
    }
  }

  // Keepalive: niektĂłre klastry zrywajÄ… idle telnet, wiÄ™c co ~30s wysyĹ‚amy CRLF
  if (telnetConnected && telnetClient.connected()) {
    unsigned long now = millis();
    if (now - lastClusterKeepAliveMs > 30000) {
      LOGV_PRINTLN("[LOOP] WysyĹ‚anie keepalive do Cluster");
      telnetClient.print("\r\n");
      lastClusterKeepAliveMs = now;
    }
  }

  if (potaTelnetConnected && potaTelnetClient.connected()) {
    unsigned long now = millis();
    if (now - lastPotaKeepAliveMs > 30000) {
      potaTelnetClient.print("\r\n");
      lastPotaKeepAliveMs = now;
    }
  }
  
  // Feed watchdog przed delay
  yield();
  delay(10); // Małe opóźnienie dla stabilności
  
  // marker usuniÄ™ty - bez spamowania logu
}

