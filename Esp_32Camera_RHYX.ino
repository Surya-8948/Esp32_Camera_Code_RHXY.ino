/*============================================================================
  ESP32-CAM · RHYX M21-45 · AP + STA · Web UI  (Production Firmware)
-----------------------------------------------------------------------------
  Project      : ESP32-CAM Control Center
  Author       : Surya Bajpai
  GitHub       : https://github.com/Surya-8948
  Camera       : RHYX M21-45 (GC2145 class, no hardware JPEG encoder)
  Boards       : AI-Thinker ESP32-CAM (default) / any ESP32 with the same map
  Version      : 1.0.2
-----------------------------------------------------------------------------
  WHY THIS SKETCH IS DIFFERENT
  ----------------------------
  The RHYX M21-45 sensor has no on-chip JPEG encoder. The stock
  CameraWebServer example fails on it with:

      E camera: JPEG format is not supported on this sensor
      Camera init failed with error 0x106

  This firmware *auto-detects* that situation: it first tries a hardware
  JPEG init, and if the sensor refuses (0x106 / ESP_ERR_NOT_SUPPORTED) it
  re-inits in RGB565 and converts every frame to JPEG in software
  (frame2jpg). It also powers the sensor down/up before the retry so an
  already-running cone never "sticks". OV2640 / OV3660 / OV5640 modules keep
  using the fast hardware JPEG path. Nothing to change in the code either way.

  1.0.1 FIX (important if you stream)
  -----------------------------------
  The MJPEG loop handed frame2jpg()'s output buffer to nobody: it set the
  pointer to NULL instead of free()ing it, so every streamed frame leaked
  ~150 KB of PSRAM. The stream ran for ~12 frames and then died with
      E (...) to_jpg: JPG buffer malloc failed
  / [stream] software JPEG conversion failed
  and PSRAM free fell from 3.7 MB to almost nothing. The frame is now encoded
  through a callback straight into ONE buffer that is allocated per viewer and
  reused for every frame - no allocation churn, nothing to leak, and the
  stream now survives hours instead of seconds. A viewer cap of 2 keeps PSRAM
  bounded; the health line and /status show psram= and viewers= so you can
  watch it stay flat.

  1.0.2 FIX (Wi-Fi robustness - the disconnect storm / watchdog reset)
  --------------------------------------------------------------------
  A router that refuses the join (wrong password, 5 GHz-only, WPA3-only, MAC
  filter) made the log unreadable: a line for every disconnection event, and
  TWO reconnect state machines fighting over the radio - the core's own
  auto-reconnect plus this sketch's tick, which called disconnect()+begin()
  every 30 s. That combination is what produced the endless
      [wifi] STA disconnected (AP stays up)
  lines, the TG1WDT_SYS_RESET and a LoadProhibited panic in the Wi-Fi stack.
  Now: auto-reconnect is OFF, one backoff timer owns the retries (5 -> 10 ->
  20 -> 40 -> 60 s, using non-destructive WiFi.reconnect()), the log is rate
  limited to one line per 30 s, and every failure prints the esp_wifi reason
  code in plain English (e.g. reason 15 = wrong password / WPA3-only router,
  reason 201 = SSID not found / 5 GHz-only network). The dashboard shows the
  same diagnosis on the Network tab.

  FEATURES
  --------
   * AP + STA simultaneously (AP always stays up -> you can never lock
     yourself out; AP clients get a captive-portal style redirect)
   * Custom vanilla HTML/CSS/JS dashboard (no CDN, no framework) that works
     offline, exactly as required inside AP mode
   * Live MJPEG stream on its own port (default 81) so the control API on
     port 80 never blocks while streaming
   * Single-frame JPEG endpoint, bright flash LED PWM control, auto-flash on
     snapshot, live FPS counter
   * Full sensor control (resolution, quality, brightness, contrast,
     saturation, WB, exposure, gain, mirror, flip, effects)
   * Wi-Fi credentials + all preferences stored in NVS (survive reboot and
     power loss) and editable from the UI
   * mDNS  ->  http://suryacam.local
   * Arduino OTA (password protected) + manual reboot + factory reset
   * Watchdog safe, heap/PSRAM guards, serial diagnostics
   * Optional HTTP Basic auth for the whole UI (see WEB_AUTH_ENABLED)

  ONE FILE, NOTHING ELSE TO DO
  ----------------------------
  This is a complete single-file sketch. Create a new sketch in the Arduino
  IDE, select everything in the editor, paste this whole file over it and hit
  Upload. There is no second tab, no header file, no library folder to
  assemble, and no dependency to install.

  The dashboard is stored inside this file in a raw string literal
  (R"SURYAUI( ... )SURYAUI"). Its JavaScript deliberately uses ARROW
  FUNCTIONS: the Arduino build system runs an automatic C++ prototype
  generator over every .ino, and classic "function streamURL(){" lines were
  being turned into C++ prototypes OUTSIDE the string literal -- that is what
  used to break the build when the HTML lived in the .ino. Arrow functions
  are invisible to that generator, so the whole page now stays safely inside
  the string where it belongs.

  QUICK START
  -----------
   1. Arduino IDE > File > New Sketch. Select all, delete, paste this file.
      (The IDE may ask you to save -- give it any name, e.g. SuryaCAM.)
   2. Boards Manager: install "esp32 by Espressif" (2.0.14+ or 3.x, both are
      compile-verified). Then select:
        Tools > Board > ESP32 Arduino > "AI Thinker ESP32-CAM"
        Tools > PSRAM: "Enabled"
        Tools > Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)" is fine
   3. Optional: edit WIFI_SSID / WIFI_PASSWORD below. If you leave them
      empty the board simply boots in AP mode and you enter them in the UI.
   4. Wire GPIO0 -> GND, press RST, upload, remove the jumper, press RST.
   5. Connect to Wi-Fi "SuryaCAM-ESP32" (password: surya12345) and open
      http://192.168.4.1   -- or open the STA IP printed on the serial port.

  SERIAL MONITOR 115200 baud shows a full boot report.

  (If a stale library shadowing is reported, e.g. "Multiple libraries were
   found for WiFi.h", you can safely delete the old WiFi / WiFiNINA folders
   from Documents\Arduino\libraries -- this sketch uses the core WiFi only.)
============================================================================*/

/*===========================================================================
  1. BOARD / CAMERA WIRING
===========================================================================*/

#define CAM_BOARD_AITHINKER 1     // classic ESP32-CAM (the board sold with the
                                  // RHYX M21-45 module) -- ACTIVE
#define CAM_BOARD_ESP32S3   2

#define CAMERA_BOARD CAM_BOARD_AITHINKER

#if CAMERA_BOARD == CAM_BOARD_AITHINKER
  #define CAM_BOARD_NAME "AI-Thinker ESP32-CAM"
  #define PWDN_GPIO_NUM   32
  #define RESET_GPIO_NUM  -1
  #define XCLK_GPIO_NUM    0
  #define SIOD_GPIO_NUM   26
  #define SIOC_GPIO_NUM   27
  #define Y9_GPIO_NUM     35
  #define Y8_GPIO_NUM     34
  #define Y7_GPIO_NUM     39
  #define Y6_GPIO_NUM     36
  #define Y5_GPIO_NUM     21
  #define Y4_GPIO_NUM     19
  #define Y3_GPIO_NUM     18
  #define Y2_GPIO_NUM      5
  #define VSYNC_GPIO_NUM  25
  #define HREF_GPIO_NUM   23
  #define PCLK_GPIO_NUM   22
  #define LED_GPIO_NUM     4      // onboard "flash" LED, PWM dimmable
  #define SD_ENABLED       0      // set 1 only if you actually use the microSD

/*----------------------------------------------------------------------------
  ESP32-S3 boards (e.g. the ESP32-S3-N16R8 camera board that also ships with a
  RHYX M21-45). Pin maps differ between vendors -- VERIFY against your board's
  schematic before switching, then set CAMERA_BOARD to CAM_BOARD_ESP32S3.
  The values below are the common ESP32-S3-CAM / Freenove-style map.
----------------------------------------------------------------------------*/
#elif CAMERA_BOARD == CAM_BOARD_ESP32S3
  #define CAM_BOARD_NAME "ESP32-S3-CAM"
  #define PWDN_GPIO_NUM   -1
  #define RESET_GPIO_NUM  -1
  #define XCLK_GPIO_NUM   15
  #define SIOD_GPIO_NUM    4
  #define SIOC_GPIO_NUM    5
  #define Y9_GPIO_NUM     16
  #define Y8_GPIO_NUM     17
  #define Y7_GPIO_NUM     18
  #define Y6_GPIO_NUM     12
  #define Y5_GPIO_NUM     10
  #define Y4_GPIO_NUM      8
  #define Y3_GPIO_NUM      9
  #define Y2_GPIO_NUM     11
  #define VSYNC_GPIO_NUM   6
  #define HREF_GPIO_NUM    7
  #define PCLK_GPIO_NUM   13
  #define LED_GPIO_NUM     2
  #define SD_ENABLED       0
#else
  #error "Unknown CAMERA_BOARD selection"
#endif

/*===========================================================================
  2. USER CONFIGURATION  (everything here can also be changed from the UI)
===========================================================================*/

#define FW_VERSION        "1.0.2"
#define OWNER_NAME        "Surya Bajpai"
#define OWNER_GITHUB      "https://github.com/Surya-8948"

/* --- Wi-Fi -----------------------------------------------------------------
   Leave WIFI_SSID empty ("") to boot straight into AP mode and configure the
   network later from the dashboard -> Network tab.                       */
#define WIFI_SSID         ""
#define WIFI_PASSWORD     ""

/* --- Access point (always on, AP + STA run together) --------------------- */
#define AP_SSID           "SuryaCAM-ESP32"
#define AP_PASSWORD       "surya12345"     // min 8 chars ("" = open network)
#define AP_CHANNEL        6
#define AP_MAX_CLIENTS    4

/* --- Identity ------------------------------------------------------------ */
#define HOSTNAME          "suryacam"       // http://suryacam.local
#define OTA_PASSWORD      "surya-ota-2026" // Arduino IDE > Network port

/* --- Ports --------------------------------------------------------------- */
#define WEB_PORT          80               // dashboard + control API
#define STREAM_PORT       81               // MJPEG stream (raw httpd)
#define STREAM_CTRL_PORT  32769            // must differ from the port-80 ctrl

/* --- Defaults ------------------------------------------------------------ */
/* XCLK. 20 MHz is correct for OV2640/OV3660 and most GC2145 batches. If you
   get noisy/garbled frames from a RHYX M21-45 try 10000000 (10 MHz). */
#define XCLK_FREQ_HZ      20000000

#define DEFAULT_QUALITY   12               // JPEG 4..63 (lower = better)
#define DEFAULT_FPS       10               // stream FPS cap 1..25
#define STA_BACKOFF_MIN    5000UL          // first STA retry after 5 s
#define STA_BACKOFF_MAX   60000UL          // ...5 -> 10 -> 20 -> 40 -> 60 s
#define STA_CONNECT_WAIT  10000UL          // courtesy wait on first boot only

/* --- Optional HTTP Basic auth for the whole dashboard -------------------- */
#define WEB_AUTH_ENABLED  0                // 1 = protect UI + API + stream
#define WEB_AUTH_USER     "surya"
#define WEB_AUTH_PASS     "suryacam"

/*===========================================================================
  3. INCLUDES
===========================================================================*/
#include "esp_camera.h"
#include "esp_timer.h"
#include <WiFi.h>
#include <WebServer.h>
/* WebServer's HTTP_Method.h and esp_http_server.h both define HTTP_ANY with
   different values; drop the first one so the build stays warning-free. */
#undef HTTP_ANY
#include "esp_http_server.h"     // raw httpd, used by the MJPEG stream server
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <esp_system.h>          // ESP.restart(), esp_reset_reason()
#include <esp_heap_caps.h>       // heap_caps_malloc() for the PSRAM JPEG buffer
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>   // ESP_ARDUINO_VERSION_MAJOR (2.x vs 3.x APIs)
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2     // very old cores behave like 2.x here
#endif
#include <math.h>
/* note: the ESP class is declared in the core's Esp.h, which Arduino.h
   already pulls in -- do NOT add #include <ESP.h>, it does not exist on
   ESP32 core 3.x and breaks the build on case-sensitive filesystems. */

/* img_converters.h ships with the ESP32 core (it is what the stock
   CameraWebServer example uses). It gives us frame2jpg(), the software JPEG
   encoder that makes the RHYX M21-45 / GC2145 work at all. */
#ifdef __has_include
  #if __has_include("img_converters.h")
    #include "img_converters.h"
    #define HAVE_IMG_CONVERTERS 1
  #else
    #define HAVE_IMG_CONVERTERS 0
  #endif
  #if __has_include("soc/rtc_cntl_reg.h")
    #include "soc/rtc_cntl_reg.h"
  #endif
  #if __has_include("soc/soc.h")
    #include "soc/soc.h"
  #endif
#else
  #include "img_converters.h"
  #define HAVE_IMG_CONVERTERS 1
  #include "soc/rtc_cntl_reg.h"
  #include "soc/soc.h"
#endif

/*===========================================================================
  4. GLOBALS
===========================================================================*/
WebServer server(WEB_PORT);
Preferences prefs;

/* network / identity */
static String cfgSsid, cfgPass, cfgHost = HOSTNAME, cfgApSsid = AP_SSID, cfgApPass = AP_PASSWORD;

/* camera state */
static bool     gSwJpeg      = false;      // true  -> RGB565 + software JPEG
static bool     gCameraOk    = false;
static int      gFrameSize   = -1;
static uint8_t  gQuality     = DEFAULT_QUALITY;
static uint8_t  gFpsCap      = DEFAULT_FPS;
static uint8_t  gMirror      = 0;
static uint8_t  gVflip       = 0;
static uint8_t  gAutoFlash   = 0;
static uint8_t  gLedLevel    = 0;
static uint16_t gSensorPid   = 0;
static String   gSensorName  = "unknown";

/* frame statistics - written by the stream/httpd tasks, read by loop().
   Relaxed atomics keep it warning-free under C++20 and tear-free. */
static uint32_t gFrameCount = 0;
static uint32_t gFpsCount = 0, gFpsMs = 0;
static float    gFps = 0.0f;

static inline void frameCountInc(){
  __atomic_add_fetch(&gFrameCount, 1u, __ATOMIC_RELAXED);
}
static inline uint32_t frameCountGet(){
  return __atomic_load_n(&gFrameCount, __ATOMIC_RELAXED);
}

/* MJPEG viewers: each one owns a reusable JPEG buffer, so capping the count
   also caps PSRAM usage. Declared here because /status and the health line
   report it before the stream section appears. */
#define MAX_STREAM_CLIENTS 2
static int gStreamClients = 0;

/* sensor capability map (filled after init; GC2145 lacks some ops) */
struct SensorCaps {
  bool framesize, jpeg, quality, brightness, contrast, saturation, special,
       wb, awbGain, aec, aec2, aecValue, aeLevel, agc, agcGain, gainceiling,
       bpc, wpc, lenc, rawGma, dcw, colorbar, hmirror, vflip;
};
static SensorCaps caps;

/* stream server */
static httpd_handle_t streamServer = NULL;

/*===========================================================================
  5. SMALL HELPERS
===========================================================================*/
static uint32_t uptimeSeconds(){
  return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

static String jsonEsc(const String& in){
  String out; out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++){
    char c = in[i];
    if (c == '"' || c == '\\'){ out += '\\'; out += c; }
    else if (c == '\n'){ out += "\\n"; }
    else if (c == '\r'){ out += "\\r"; }
    else if ((uint8_t)c < 0x20){ out += ' '; }
    else out += c;
  }
  return out;
}

static String bytesToHuman(size_t n){
  char b[32];
  if (n < 1024)             snprintf(b, sizeof(b), "%u B", (unsigned)n);
  else if (n < 1024UL*1024) snprintf(b, sizeof(b), "%.1f KB", n / 1024.0);
  else                      snprintf(b, sizeof(b), "%.2f MB", n / 1048576.0);
  return String(b);
}

/* ---- flash LED (PWM dimmable) ------------------------------------------- */
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_NEW_API 1
#else
  #define LEDC_NEW_API 0
#endif

#define LEDC_LED_CHANNEL 7        // camera XCLK uses channel 0 - keep clear
#define LEDC_LED_TIMER   1

static void ledWrite(uint8_t level);   // forward declaration

static void ledInit(){
#if LED_GPIO_NUM >= 0
  #if LEDC_NEW_API
    ledcAttach(LED_GPIO_NUM, 5000, 8);
  #else
    ledcSetup(LEDC_LED_CHANNEL, 5000, 8);
    ledcAttachPin(LED_GPIO_NUM, LEDC_LED_CHANNEL);
  #endif
  ledWrite(0);
#endif
}
static void ledWrite(uint8_t level){
#if LED_GPIO_NUM >= 0
  #if LEDC_NEW_API
    ledcWrite(LED_GPIO_NUM, level);
  #else
    ledcWrite(LEDC_LED_CHANNEL, level);
  #endif
#endif
}

/* ---- optional HTTP basic auth ------------------------------------------- */
static bool requireAuth(){
#if WEB_AUTH_ENABLED
  if (server.authenticate(WEB_AUTH_USER, WEB_AUTH_PASS)) return true;
  server.requestAuthentication(BASIC_AUTH, "ESP32-CAM", "Authentication required");
  return false;
#else
  return true;
#endif
}

/*===========================================================================
  6. PREFERENCES (NVS)
===========================================================================*/
static void prefsLoad(){
  if (!prefs.begin("suryacam", false)){ Serial.println("[nvs] open failed, using defaults"); return; }
  cfgSsid   = prefs.getString("ssid",   WIFI_SSID);
  cfgPass   = prefs.getString("pass",   WIFI_PASSWORD);
  cfgHost   = prefs.getString("host",   HOSTNAME);
  cfgApSsid = prefs.getString("apssid", AP_SSID);
  cfgApPass = prefs.getString("appass", AP_PASSWORD);
  gQuality    = prefs.getUChar("quality", DEFAULT_QUALITY);
  gFpsCap     = prefs.getUChar("fps",     DEFAULT_FPS);
  gMirror     = prefs.getUChar("mirror",  0);
  gVflip      = prefs.getUChar("vflip",   0);
  gAutoFlash  = prefs.getUChar("aflash",  0);
  gLedLevel   = prefs.getUChar("led",     0);
  prefs.end();
  if (cfgHost.length() == 0) cfgHost = HOSTNAME;
  if (cfgApSsid.length() == 0) cfgApSsid = AP_SSID;
  if (gQuality < 4 || gQuality > 63) gQuality = DEFAULT_QUALITY;
  if (gFpsCap < 1 || gFpsCap > 25)   gFpsCap  = DEFAULT_FPS;
}

static void prefsSaveKV(const char* key, const String& v){
  prefs.begin("suryacam", false); prefs.putString(key, v); prefs.end();
}
static void prefsSaveKV(const char* key, uint8_t v){
  prefs.begin("suryacam", false); prefs.putUChar(key, v); prefs.end();
}
static void prefsClearAll(){
  prefs.begin("suryacam", false); prefs.clear(); prefs.end();
}

/*===========================================================================
  7. CAMERA
===========================================================================*/
static framesize_t defaultFrameSize(bool hardwareJpeg){
  if (!psramFound()) return FRAMESIZE_QVGA;                 // tiny heap only
  return hardwareJpeg ? FRAMESIZE_VGA : FRAMESIZE_QVGA;     // QVGA = smooth
}

static camera_config_t buildCameraConfig(pixformat_t fmt, framesize_t fs){
  camera_config_t c;
  memset(&c, 0, sizeof(c));
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_pwdn     = PWDN_GPIO_NUM;
  c.pin_reset    = RESET_GPIO_NUM;
  c.pin_xclk     = XCLK_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_d0       = Y2_GPIO_NUM;
  c.pin_d1       = Y3_GPIO_NUM;
  c.pin_d2       = Y4_GPIO_NUM;
  c.pin_d3       = Y5_GPIO_NUM;
  c.pin_d4       = Y6_GPIO_NUM;
  c.pin_d5       = Y7_GPIO_NUM;
  c.pin_d6       = Y8_GPIO_NUM;
  c.pin_d7       = Y9_GPIO_NUM;
  c.pin_vsync    = VSYNC_GPIO_NUM;
  c.pin_href     = HREF_GPIO_NUM;
  c.pin_pclk     = PCLK_GPIO_NUM;
  c.xclk_freq_hz = XCLK_FREQ_HZ;
  c.pixel_format = fmt;
  c.frame_size   = fs;
  c.jpeg_quality = gQuality;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  if (psramFound()){
    c.fb_count = 2;                       // double buffer = no tearing
  } else {
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.frame_size  = fs > FRAMESIZE_QVGA ? FRAMESIZE_QVGA : fs;
  }
  return c;
}

/* Hard power-cycle of the sensor, used between init attempts. */
static void sensorPowerCycle(){
#if PWDN_GPIO_NUM >= 0
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, HIGH);      // power down
  delay(60);
  digitalWrite(PWDN_GPIO_NUM, LOW);       // power up
  delay(60);
#endif
}

static bool cameraDetectSensor(){
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;
  gSensorPid = s->id.PID;
  /* Some GC2145 modules (including the RHYX M21-45) answer with PID 0x2145,
     which the library's table does not know, so label it properly instead of
     showing a bare "PID 0x2145" in the dashboard. */
  if (gSensorPid == 0x2145) gSensorName = "GC2145 (RHYX M21-45)";

  switch (gSensorPid){
    case OV2640_PID:  gSensorName = "OV2640";             break;
    case OV3660_PID:  gSensorName = "OV3660";             break;
    case OV5640_PID:  gSensorName = "OV5640";             break;
#ifdef GC2145_PID
    case GC2145_PID:  gSensorName = "GC2145 (RHYX M21-45)"; break;
#endif
#ifdef GC032A_PID
    case GC032A_PID:  gSensorName = "GC032A";             break;
#endif
#ifdef SC030IOT_PID
    case SC030IOT_PID:gSensorName = "SC030IOT";           break;
#endif
    default:          gSensorName = "PID 0x" + String((int)gSensorPid, HEX); break;
  }
  return true;
}

static void detectCaps(){
  memset(&caps, 0, sizeof(caps));
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  caps.framesize  = (s->set_framesize        != NULL);
  caps.jpeg       = (s->pixformat == PIXFORMAT_JPEG);
  caps.quality    = (s->set_quality          != NULL) && caps.jpeg;
  caps.brightness = (s->set_brightness       != NULL);
  caps.contrast   = (s->set_contrast         != NULL);
  caps.saturation = (s->set_saturation       != NULL);
  caps.special    = (s->set_special_effect   != NULL);
  caps.wb         = (s->set_whitebal     != NULL);
  caps.awbGain    = (s->set_awb_gain         != NULL);
  caps.aec        = (s->set_exposure_ctrl    != NULL);
  caps.aec2       = (s->set_aec2             != NULL);
  caps.aecValue   = (s->set_aec_value        != NULL);
  caps.aeLevel    = (s->set_ae_level         != NULL);
  caps.agc        = (s->set_gain_ctrl        != NULL);
  caps.agcGain    = (s->set_agc_gain         != NULL);
  caps.gainceiling= (s->set_gainceiling      != NULL);
  caps.bpc        = (s->set_bpc              != NULL);
  caps.wpc        = (s->set_wpc              != NULL);
  caps.lenc       = (s->set_lenc             != NULL);
  caps.rawGma     = (s->set_raw_gma          != NULL);
  caps.dcw        = (s->set_dcw              != NULL);
  caps.colorbar   = (s->set_colorbar         != NULL);
  caps.hmirror    = (s->set_hmirror          != NULL);
  caps.vflip      = (s->set_vflip            != NULL);
}

/* Apply the user's stored image preferences (mirror / flip). */
static void applyStoredImagePrefs(){
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  if (caps.hmirror) s->set_hmirror(s, gMirror);
  if (caps.vflip)   s->set_vflip(s, gVflip);
  if (caps.jpeg)    s->set_quality(s, gQuality);
}

static void cameraResetDefaults(){
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  if (caps.quality)    s->set_quality(s, DEFAULT_QUALITY);
  if (caps.framesize)  s->set_framesize(s, defaultFrameSize(!gSwJpeg));
  if (caps.brightness) s->set_brightness(s, 0);
  if (caps.contrast)   s->set_contrast(s, 0);
  if (caps.saturation) s->set_saturation(s, 0);
  if (caps.special)    s->set_special_effect(s, 0);
  if (caps.wb)         s->set_whitebal(s, 1);
  if (caps.awbGain)    s->set_awb_gain(s, 1);
  if (caps.aec)        s->set_exposure_ctrl(s, 1);
  if (caps.aec2)       s->set_aec2(s, 1);
  if (caps.agc)        s->set_gain_ctrl(s, 1);
  if (caps.agcGain)    s->set_agc_gain(s, 0);
  if (caps.gainceiling)s->set_gainceiling(s, GAINCEILING_2X);
  if (caps.bpc)        s->set_bpc(s, 0);
  if (caps.wpc)        s->set_wpc(s, 1);
  if (caps.lenc)       s->set_lenc(s, 1);
  if (caps.rawGma)     s->set_raw_gma(s, 1);
  if (caps.dcw)        s->set_dcw(s, 1);
  if (caps.colorbar)   s->set_colorbar(s, 0);
  if (caps.hmirror)    s->set_hmirror(s, gMirror);
  if (caps.vflip)      s->set_vflip(s, gVflip);
}

/* Try a hardware-JPEG init, honour the sensor's answer. */
static esp_err_t tryInit(pixformat_t fmt, framesize_t fs){
  camera_config_t cfg = buildCameraConfig(fmt, fs);
  esp_err_t err = esp_camera_init(&cfg);
  if (err == ESP_OK){
    gSwJpeg   = (fmt != PIXFORMAT_JPEG);
    gFrameSize = fs;
  }
  return err;
}

static bool cameraBegin(){
  if (!psramFound()) Serial.println("[cam] WARNING: no PSRAM found - using low res + DRAM buffers");

  /* ---- attempt 1: hardware JPEG (OV2640 / OV3660 / OV5640) -------------- */
  esp_err_t err = tryInit(PIXFORMAT_JPEG, defaultFrameSize(true));
  if (err == ESP_OK){
    Serial.println("[cam] hardware JPEG path ready");
  } else {
    Serial.printf("[cam] hardware JPEG init failed (err 0x%04X)\n", (unsigned)err);
    if (err == 0x106 || err == ESP_ERR_NOT_SUPPORTED){
      Serial.println("[cam] sensor reports 'JPEG format is not supported on this sensor'");
      Serial.println("[cam] -> RHYX M21-45 / GC2145 detected: switching to RGB565 + software JPEG");
    } else {
      Serial.println("[cam] -> retrying in RGB565 anyway");
    }
    esp_camera_deinit();
    delay(150);
    sensorPowerCycle();

#if !HAVE_IMG_CONVERTERS
    Serial.println("[cam] FATAL: img_converters.h missing - cannot software-encode JPEG");
    return false;
#else
    /* ---- attempt 2: RGB565 + software JPEG ----------------------------- */
    framesize_t fs = defaultFrameSize(false);
    err = tryInit(PIXFORMAT_RGB565, fs);
    if (err != ESP_OK && fs != FRAMESIZE_QVGA){
      esp_camera_deinit(); delay(120); sensorPowerCycle();
      err = tryInit(PIXFORMAT_RGB565, FRAMESIZE_QVGA);
    }
    if (err != ESP_OK){
      Serial.printf("[cam] RGB565 init failed too (err 0x%04X) - check the ribbon cable and 5V/2A supply\n", (unsigned)err);
      return false;
    }
    Serial.println("[cam] RGB565 + software JPEG path ready");
#endif
  }

  cameraDetectSensor();
  detectCaps();

  if (!caps.framesize){
    Serial.println("[cam] WARNING: this sensor exposes no set_framesize() - resolution control disabled");
  }
  applyStoredImagePrefs();

  /* warm-up: throw away the first few frames (AE/AWB convergence) */
  for (int i = 0; i < 3; i++){
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(60);
  }

  gCameraOk = true;
  Serial.printf("[cam] OK  sensor=%s  pid=0x%04X  path=%s  quality=%u  psram=%s\n",
                gSensorName.c_str(), (unsigned)gSensorPid,
                gSwJpeg ? "RGB565 -> software JPEG" : "hardware JPEG", (unsigned)gQuality,
                psramFound() ? bytesToHuman(ESP.getFreePsram()).c_str() : "n/a");
  return true;
}

/* frame-size helpers ------------------------------------------------------ */
struct FrameSizeOpt { int v; const char* n; };
static const FrameSizeOpt FS_TABLE[] = {
  { FRAMESIZE_QVGA, "QVGA 320x240" }, { FRAMESIZE_CIF,  "CIF 400x296" },
  { FRAMESIZE_HVGA, "HVGA 480x320" }, { FRAMESIZE_VGA,  "VGA 640x480" },
  { FRAMESIZE_SVGA, "SVGA 800x600" }, { FRAMESIZE_XGA,  "XGA 1024x768" },
  { FRAMESIZE_HD,   "HD 1280x720"  }, { FRAMESIZE_SXGA, "SXGA 1280x1024" },
  { FRAMESIZE_UXGA, "UXGA 1600x1200" },
};
static const int FS_TABLE_N = sizeof(FS_TABLE) / sizeof(FS_TABLE[0]);

/* biggest resolution that is comfortable with software JPEG */
static int maxFramesizeForMode(){
  if (!gSwJpeg) return FRAMESIZE_UXGA;
  return psramFound() ? FRAMESIZE_VGA : FRAMESIZE_QVGA;
}

static String frameSizeName(int v){
  for (int i = 0; i < FS_TABLE_N; i++) if (FS_TABLE[i].v == v) return String(FS_TABLE[i].n);
  return String("custom");
}
static String frameSizeListJson(){
  String j = "["; bool first = true;
  int cap = maxFramesizeForMode();
  for (int i = 0; i < FS_TABLE_N; i++){
    if (FS_TABLE[i].v > cap) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"v\":" + String(FS_TABLE[i].v) + ",\"n\":\"" + String(FS_TABLE[i].n) + "\"}";
  }
  j += "]";
  return j;
}

/*===========================================================================
  8. NETWORK (AP + STA together)
===========================================================================*/
/*---------------------------------------------------------------------------
  Wi-Fi events + the STA supervisor.

  * Logging is RATE LIMITED. A router that keeps refusing the join (wrong
    password, 5 GHz-only band, WPA3-only, MAC filter) fires a disconnection
    event every few seconds; printing one line per event buries everything
    else, which is exactly what happened in the first field test.
  * There is ONE reconnect state machine. The core's auto-reconnect and this
    sketch's retry tick both called begin()/disconnect() - two state machines
    fighting over the same radio. That is what produced the disconnect storm,
    the task-watchdog reset and a crash inside the Wi-Fi stack. Auto-reconnect
    is now off and staTick() owns the retries with a backoff.
---------------------------------------------------------------------------*/
static uint32_t gStaTries      = 0;                  // attempts since last success
static uint32_t gStaBackoffMs  = STA_BACKOFF_MIN;
static uint32_t gStaLastTry    = 0;
static uint32_t gStaLastLog    = 0;
static uint8_t  gStaReason     = 0;                  // last esp_wifi reason code

static const char* wifiReasonText(uint8_t r){
  switch (r){
    case 2:   return "authentication expired";
    case 15:  return "4-way handshake timeout - wrong password, or a WPA3-only router";
    case 200: return "beacon timeout - signal lost or a power dip";
    case 201: return "SSID not found - check the spelling; the ESP32 only sees 2.4 GHz networks";
    case 202: return "authentication failed - wrong password, or MAC filtering on the router";
    case 203: return "association failed - the router refused this client";
    case 204: return "handshake timeout - wrong password";
    case 205: return "connection failed - weak signal or the router refused";
    default:  return "reason not reported by the driver";
  }
}

static void wifiNoteEvent(int evId, uint8_t reason){
  if (evId == (int)ARDUINO_EVENT_WIFI_STA_GOT_IP){
    gStaTries = 0; gStaBackoffMs = STA_BACKOFF_MIN; gStaReason = 0;
    Serial.print("[wifi] STA connected, IP: "); Serial.println(WiFi.localIP());
    Serial.print("[wifi] open  ->  http://"); Serial.print(WiFi.localIP()); Serial.println("/");
    return;
  }
  if (evId == (int)ARDUINO_EVENT_WIFI_STA_DISCONNECTED){
    gStaReason = reason;
    uint32_t now = millis();
    bool firstFew = (gStaTries <= 2);
    if (firstFew || now - gStaLastLog > 30000UL){
      gStaLastLog = now;
      Serial.printf("[wifi] STA not connected (reason %u: %s) - AP stays up, retrying in the background\n",
                    (unsigned)reason, wifiReasonText(reason));
    }
  }
}

/* Core 3.x hands over the whole event record, core 2.x hands over the pair. */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void wifiEvents(arduino_event_t* e){
  wifiNoteEvent((int)e->event_id, e->event_info.wifi_sta_disconnected.reason);
}
#else
static void wifiEvents(WiFiEvent_t ev, WiFiEventInfo_t info){
  wifiNoteEvent((int)ev, info.wifi_sta_disconnected.reason);
}
#endif

static void networkStart(){
  /* register the event logger (also gives you the "got IP" line in the log) */
  WiFi.onEvent(wifiEvents, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(wifiEvents, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.persistent(false);           // protect flash from reconnect writes
  /* staTick() below is the single owner of STA retries - see the note above */
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);             // streaming needs the radio awake
  WiFi.setHostname(cfgHost.c_str());

  /* access point first: the dashboard is reachable even if the router is down */
  if (cfgApPass.length() >= 8) WiFi.softAP(cfgApSsid.c_str(), cfgApPass.c_str(), AP_CHANNEL, 0, AP_MAX_CLIENTS);
  else                         WiFi.softAP(cfgApSsid.c_str(), NULL,               AP_CHANNEL, 0, AP_MAX_CLIENTS);

  IPAddress apIp = WiFi.softAPIP();
  Serial.printf("[wifi] AP  \"%s\"  http://%u.%u.%u.%u/  (channel %d)\n",
                cfgApSsid.c_str(), apIp[0], apIp[1], apIp[2], apIp[3], AP_CHANNEL);

  if (cfgSsid.length() > 0){
    Serial.printf("[wifi] joining \"%s\" ...\n", cfgSsid.c_str());
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  } else {
    Serial.println("[wifi] no STA credentials stored - AP-only until you set them in the UI");
  }
}

/* Non-blocking STA supervisor ---------------------------------------------
   Never blocks the loop, never tears the AP down, never spams the log, and
   backs off instead of hammering a router that is refusing us. */
static void staTick(){
  if (cfgSsid.length() == 0) return;

  if (WiFi.status() == WL_CONNECTED){
    if (gStaTries){
      Serial.printf("[wifi] STA up after %lu attempt(s)\n", (unsigned long)gStaTries);
    }
    gStaTries = 0; gStaBackoffMs = STA_BACKOFF_MIN; gStaReason = 0;
    return;
  }

  uint32_t now = millis();
  if (now - gStaLastTry < gStaBackoffMs) return;
  gStaLastTry = now;
  gStaTries++;
  gStaBackoffMs = STA_BACKOFF_MIN << (gStaTries < 4 ? gStaTries : 4);   // 5,10,20,40,60 s
  if (gStaBackoffMs > STA_BACKOFF_MAX) gStaBackoffMs = STA_BACKOFF_MAX;

  bool logNow = (gStaTries <= 2) || (now - gStaLastLog > 30000UL);
  if (logNow){
    gStaLastLog = now;
    if (gStaReason)
      Serial.printf("[wifi] STA retry #%lu for \"%s\" - last failure: %s (reason %u)\n",
                    (unsigned long)gStaTries, cfgSsid.c_str(), wifiReasonText(gStaReason), (unsigned)gStaReason);
    else
      Serial.printf("[wifi] STA retry #%lu for \"%s\" (2.4 GHz + WPA2 only) - AP stays up\n",
                    (unsigned long)gStaTries, cfgSsid.c_str());
  }

  WiFi.reconnect();          // non-blocking: re-arms the join, AP stays untouched
}

static String wifiModeString(){
  if (WiFi.status() == WL_CONNECTED) return String("AP + STA (connected)");
  if (cfgSsid.length() > 0)          return String("AP + STA (STA down)");
  return String("AP only");
}

/*===========================================================================
  9. HTTP API  (port 80)
===========================================================================*/
static const char* uiHtml();   // defined with the embedded UI at the end

static void jsonSend(int code, const String& body){
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}
static void jsonOk(const String& extra = ""){
  String j = "{\"ok\":true";
  if (extra.length()) j += "," + extra;
  j += "}";
  jsonSend(200, j);
}
static void jsonFail(int code, const String& msg, bool unsupported = false){
  String j = "{\"ok\":false,\"msg\":\"" + jsonEsc(msg) + "\"";
  if (unsupported) j += ",\"unsupported\":true";
  j += "}";
  jsonSend(code, j);
}

/* ---- GET /status -------------------------------------------------------- */
static void handleStatus(){
  String j = "{";
  j += "\"ok\":true";
  j += ",\"fw\":\"" FW_VERSION "\"";
  j += ",\"owner\":\"" OWNER_NAME "\"";
  j += ",\"github\":\"" OWNER_GITHUB "\"";
  j += ",\"board\":\"" CAM_BOARD_NAME "\"";
  j += ",\"chip\":\"" + String(ESP.getChipModel()) + "\"";
  j += ",\"cpu\":" + String(ESP.getCpuFreqMHz());
  j += ",\"mac\":\"" + WiFi.macAddress() + "\"";
  j += ",\"sensor\":\"" + jsonEsc(gSensorName) + "\"";
  j += ",\"sensorPid\":" + String((int)gSensorPid);
  j += ",\"swJpeg\":" + String(gSwJpeg ? "true" : "false");
  j += ",\"cameraOk\":" + String(gCameraOk ? "true" : "false");
  j += ",\"cameraModel\":\"AI-THINKER\"";
  j += ",\"fs\":" + String(gFrameSize);
  j += ",\"fsName\":\"" + jsonEsc(frameSizeName(gFrameSize)) + "\"";
  j += ",\"fsList\":" + frameSizeListJson();
  j += ",\"quality\":" + String(gQuality);
  j += ",\"fpsCap\":" + String(gFpsCap);
  j += ",\"fps\":" + String((int)(gFps + 0.5f));
  j += ",\"frames\":" + String((unsigned long)frameCountGet());
  j += ",\"led\":" + String(gLedLevel);
  j += ",\"autoFlash\":" + String(gAutoFlash);
  j += ",\"mirror\":" + String(gMirror);
  j += ",\"vflip\":" + String(gVflip);

  sensor_t* s = esp_camera_sensor_get();
  if (s){
    j += ",\"brightness\":" + String(s->status.brightness);
    j += ",\"contrast\":"   + String(s->status.contrast);
    j += ",\"saturation\":" + String(s->status.saturation);
    j += ",\"effect\":"     + String(s->status.special_effect);
    j += ",\"wb\":"         + String(s->status.wb_mode ? 1 : 0);
    j += ",\"exposure\":"   + String(s->status.aec ? 1 : 0);
    j += ",\"gain\":"       + String(s->status.agc ? 1 : 0);
    j += ",\"agcGain\":"    + String(s->status.agc_gain);
  }

  j += ",\"ssid\":\"" + jsonEsc(cfgSsid) + "\"";
  j += ",\"sta\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
  j += ",\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\"";
  j += ",\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
  j += ",\"apSsid\":\"" + jsonEsc(cfgApSsid) + "\"";
  j += ",\"apIp\":\"" + WiFi.softAPIP().toString() + "\"";
  j += ",\"ap\":" + String(((int)WiFi.getMode() & WIFI_MODE_AP) ? "true" : "false");
  j += ",\"apClients\":" + String(WiFi.softAPgetStationNum());
  j += ",\"staTries\":" + String((unsigned long)gStaTries);
  j += ",\"staReason\":" + String((int)gStaReason);
  j += ",\"staReasonText\":\"" + jsonEsc(WiFi.status() == WL_CONNECTED ? String("") : String(wifiReasonText(gStaReason))) + "\"";
  j += ",\"mode\":\"" + wifiModeString() + "\"";
  j += ",\"host\":\"" + jsonEsc(cfgHost) + "\"";
  j += ",\"streamPort\":" + String(STREAM_PORT);
  j += ",\"webPort\":" + String(WEB_PORT);
  j += ",\"uptime\":" + String(uptimeSeconds());
  j += ",\"heap\":" + String((unsigned long)ESP.getFreeHeap());
  j += ",\"heapMin\":" + String((unsigned long)ESP.getMinFreeHeap());
  j += ",\"psram\":" + String(psramFound() ? "true" : "false");
  j += ",\"psramFree\":" + String((unsigned long)(psramFound() ? ESP.getFreePsram() : 0));
  j += ",\"psramMin\":" + String((unsigned long)(psramFound() ? ESP.getMinFreePsram() : 0));
  j += ",\"viewers\":" + String(__atomic_load_n(&gStreamClients, __ATOMIC_RELAXED));
  j += ",\"freeSketch\":" + String((unsigned long)ESP.getFreeSketchSpace());
  j += ",\"resetReason\":\"" + String(esp_reset_reason() == ESP_RST_POWERON ? "power-on" :
                                       esp_reset_reason() == ESP_RST_SW ? "software" :
                                       esp_reset_reason() == ESP_RST_PANIC ? "panic" : "other") + "\"";
  j += "}";
  jsonSend(200, j);
}

/* ---- GET /control?var=name&val=value ------------------------------------ */
static void handleControl(){
  if (!requireAuth()) return;
  if (!server.hasArg("var") || !server.hasArg("val")){ jsonFail(400, "need var and val"); return; }

  const String var = server.arg("var");
  const int    iv  = server.arg("val").toInt();
  sensor_t* s = esp_camera_sensor_get();
  bool unsupported = false;

  if (var == "led"){
    gLedLevel = (uint8_t)constrain(iv, 0, 255);
    ledWrite(gLedLevel);
    prefsSaveKV("led", gLedLevel);
    jsonOk("\"var\":\"led\",\"val\":" + String(gLedLevel)); return;
  }
  if (var == "fps"){
    gFpsCap = (uint8_t)constrain(iv, 1, 25);
    prefsSaveKV("fps", gFpsCap);
    jsonOk("\"var\":\"fps\",\"val\":" + String(gFpsCap)); return;
  }
  if (var == "autoflash"){
    gAutoFlash = iv ? 1 : 0;
    prefsSaveKV("aflash", gAutoFlash);
    jsonOk("\"var\":\"autoflash\",\"val\":" + String(gAutoFlash)); return;
  }
  if (var == "camera_defaults"){
    cameraResetDefaults();
    gQuality = DEFAULT_QUALITY;
    jsonOk("\"var\":\"camera_defaults\",\"val\":1"); return;
  }

  if (!s){ jsonFail(500, "camera not initialised"); return; }

  if (var == "framesize"){
    if (!caps.framesize){ unsupported = true; }
    else {
      int v = constrain(iv, 0, FS_TABLE[FS_TABLE_N-1].v);
      if (v > maxFramesizeForMode()) v = maxFramesizeForMode();
      s->set_framesize(s, (framesize_t)v);
      gFrameSize = v;
      /* let AE/AWB settle again at the new size */
      for (int i = 0; i < 2; i++){ camera_fb_t* fb = esp_camera_fb_get(); if (fb) esp_camera_fb_return(fb); }
      jsonOk("\"var\":\"framesize\",\"val\":" + String(v) + ",\"name\":\"" + jsonEsc(frameSizeName(v)) + "\"");
      return;
    }
  }
  else if (var == "quality"){
    gQuality = (uint8_t)constrain(iv, 4, 63);
    if (caps.quality) s->set_quality(s, gQuality);
    jsonOk("\"var\":\"quality\",\"val\":" + String(gQuality)); return;
  }
  else if (var == "brightness"){
    if (!caps.brightness) unsupported = true; else s->set_brightness(s, constrain(iv, -2, 2));
  }
  else if (var == "contrast"){
    if (!caps.contrast) unsupported = true; else s->set_contrast(s, constrain(iv, -2, 2));
  }
  else if (var == "saturation"){
    if (!caps.saturation) unsupported = true; else s->set_saturation(s, constrain(iv, -2, 2));
  }
  else if (var == "special_effect"){
    if (!caps.special) unsupported = true; else s->set_special_effect(s, constrain(iv, 0, 6));
  }
  else if (var == "wb"){
    if (!caps.wb) unsupported = true; else s->set_whitebal(s, iv ? 1 : 0);
  }
  else if (var == "awb_gain"){
    if (!caps.awbGain) unsupported = true; else s->set_awb_gain(s, iv ? 1 : 0);
  }
  else if (var == "exposure"){
    if (!caps.aec) unsupported = true; else s->set_exposure_ctrl(s, iv ? 1 : 0);
  }
  else if (var == "aec2"){
    if (!caps.aec2) unsupported = true; else s->set_aec2(s, iv ? 1 : 0);
  }
  else if (var == "aec_value"){
    if (!caps.aecValue) unsupported = true; else s->set_aec_value(s, constrain(iv, 0, 1200));
  }
  else if (var == "ae_level"){
    if (!caps.aeLevel) unsupported = true; else s->set_ae_level(s, constrain(iv, -2, 2));
  }
  else if (var == "gain_ctrl"){
    if (!caps.agc) unsupported = true; else s->set_gain_ctrl(s, iv ? 1 : 0);
  }
  else if (var == "agc_gain"){
    if (!caps.agcGain) unsupported = true; else s->set_agc_gain(s, constrain(iv, 0, 30));
  }
  else if (var == "gainceiling"){
    if (!caps.gainceiling) unsupported = true; else s->set_gainceiling(s, (gainceiling_t)constrain(iv, 0, 6));
  }
  else if (var == "bpc"){
    if (!caps.bpc) unsupported = true; else s->set_bpc(s, iv ? 1 : 0);
  }
  else if (var == "wpc"){
    if (!caps.wpc) unsupported = true; else s->set_wpc(s, iv ? 1 : 0);
  }
  else if (var == "lenc"){
    if (!caps.lenc) unsupported = true; else s->set_lenc(s, iv ? 1 : 0);
  }
  else if (var == "raw_gma"){
    if (!caps.rawGma) unsupported = true; else s->set_raw_gma(s, iv ? 1 : 0);
  }
  else if (var == "dcw"){
    if (!caps.dcw) unsupported = true; else s->set_dcw(s, iv ? 1 : 0);
  }
  else if (var == "colorbar"){
    if (!caps.colorbar) unsupported = true; else s->set_colorbar(s, iv ? 1 : 0);
  }
  else if (var == "hmirror"){
    if (!caps.hmirror) unsupported = true;
    else { gMirror = iv ? 1 : 0; s->set_hmirror(s, gMirror); prefsSaveKV("mirror", gMirror); }
  }
  else if (var == "vflip"){
    if (!caps.vflip) unsupported = true;
    else { gVflip = iv ? 1 : 0; s->set_vflip(s, gVflip); prefsSaveKV("vflip", gVflip); }
  }
  else {
    jsonFail(400, "unknown var '" + var + "'"); return;
  }

  if (unsupported){
    jsonSend(200, "{\"ok\":false,\"unsupported\":true,\"var\":\"" + var + "\"}");
    return;
  }
  jsonOk("\"var\":\"" + var + "\",\"val\":" + String(iv));
}

/* ---- GET /capture : one JPEG frame -------------------------------------- */
/* Returns the framebuffer exactly once, whether it is hardware or software
   encoded. Auto-flash is honoured here (useful at night / in dark rooms). */
static void handleCapture(){
  if (!requireAuth()) return;
  if (!gCameraOk){ jsonFail(503, "camera not ready"); return; }

  if (gAutoFlash && LED_GPIO_NUM >= 0){
    ledWrite(255);
    delay(120);
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb){
    if (gAutoFlash && LED_GPIO_NUM >= 0) ledWrite(gLedLevel);
    jsonFail(500, "capture failed");
    return;
  }

  uint8_t* jpg = NULL;
  size_t   len = 0;
  bool     owned = false;          // true -> we must free(jpg)

  if (fb->format == PIXFORMAT_JPEG){
    jpg = fb->buf; len = fb->len;
  } else {
#if HAVE_IMG_CONVERTERS
    owned = frame2jpg(fb, gQuality, &jpg, &len);
    if (!owned){ jpg = NULL; len = 0; }
#else
    owned = false;
#endif
  }

  if (!jpg || len == 0){
    if (fb) esp_camera_fb_return(fb);
    if (owned && jpg) free(jpg);
    if (gAutoFlash && LED_GPIO_NUM >= 0) ledWrite(gLedLevel);
    jsonFail(500, "JPEG conversion failed");
    return;
  }

  frameCountInc();
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Content-Disposition", "inline; filename=\"suryacam.jpg\"");
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)jpg, len);

  if (fb) esp_camera_fb_return(fb);
  if (owned) free(jpg);

  if (gAutoFlash && LED_GPIO_NUM >= 0){
    delay(40);
    ledWrite(gLedLevel);
  }
}

/* ---- POST /config : Wi-Fi + AP credentials ------------------------------ */
static void handleConfig(){
  if (!requireAuth()) return;

  /* actions */
  if (server.hasArg("action")){
    const String a = server.arg("action");
    if (a == "reconnect"){
      if (cfgSsid.length() == 0){ jsonFail(400, "no SSID stored"); return; }
      gStaTries = 0; gStaBackoffMs = STA_BACKOFF_MIN; gStaReason = 0;
      WiFi.disconnect(false, false);
      delay(60);
      WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
      jsonOk("\"action\":\"reconnect\"");
      return;
    }
    if (a == "forget"){
      cfgSsid = ""; cfgPass = "";
      prefsSaveKV("ssid", cfgSsid);
      prefsSaveKV("pass", cfgPass);
      WiFi.disconnect(true, true);
      jsonOk("\"action\":\"forget\"");
      return;
    }
    jsonFail(400, "unknown action");
    return;
  }

  if (server.method() != HTTP_POST){ jsonFail(405, "use POST"); return; }

  bool changed = false;

  if (server.hasArg("ssid")){
    String v = server.arg("ssid"); v.trim();
    if (v != cfgSsid){ cfgSsid = v; prefsSaveKV("ssid", cfgSsid); changed = true; }
  }
  if (server.hasArg("pass")){
    String v = server.arg("pass");
    if (v.length() && v != cfgPass){ cfgPass = v; prefsSaveKV("pass", cfgPass); changed = true; }
  }
  if (server.hasArg("host")){
    String v = server.arg("host"); v.trim();
    if (v.length() && v != cfgHost){ cfgHost = v; prefsSaveKV("host", cfgHost); }
  }
  if (server.hasArg("apssid")){
    String v = server.arg("apssid"); v.trim();
    if (v.length() >= 1 && v != cfgApSsid){ cfgApSsid = v; prefsSaveKV("apssid", cfgApSsid); }
  }
  if (server.hasArg("appass")){
    String v = server.arg("appass");
    if (v.length() >= 8 && v != cfgApPass){ cfgApPass = v; prefsSaveKV("appass", cfgApPass); }
  }

  jsonOk("\"saved\":true,\"ssid\":\"" + jsonEsc(cfgSsid) + "\"");

  if (changed && cfgSsid.length()){
    gStaTries = 0; gStaBackoffMs = STA_BACKOFF_MIN; gStaReason = 0;
    delay(150);
    WiFi.disconnect(false, false);
    delay(60);
    WiFi.begin(cfgSsid.c_str(), cfgPass.c_str());
  }
}

/* ---- /reboot  and  /factory -------------------------------------------- */
static void handleReboot(){
  if (!requireAuth()) return;
  jsonOk("\"rebooting\":true");
  server.client().flush();
  delay(300);
  ESP.restart();
}

static void handleFactory(){
  if (!requireAuth()) return;
  prefsClearAll();
  jsonOk("\"factory\":true");
  server.client().flush();
  delay(300);
  ESP.restart();
}

/* ---- UI ---------------------------------------------------------------- */
static void handleRoot(){
  if (!requireAuth()) return;
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", uiHtml());
}

static void handleFavicon(){
  server.send(204);
}

/* Captive-portal friendly 404: anything unknown lands back on the dashboard,
   which makes phones pop the "sign in to network" panel in AP mode. */
static void handleNotFound(){
  String path = server.uri();
  if (path.startsWith("/api")){ jsonFail(404, "not found"); return; }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "Redirecting to dashboard");
}

/*===========================================================================
 10. MJPEG STREAM SERVER (dedicated raw httpd on STREAM_PORT)
     Keeping the stream off port 80 means the control API never blocks,
     and a single slow client can never freeze the dashboard.
===========================================================================*/
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

/*----------------------------------------------------------------------------
  Reusable JPEG output buffer for the software-JPEG path.

  frame2jpg() mallocs a fresh buffer on EVERY frame and hands ownership to the
  caller, who must free() it. The first 1.0.0 build dropped that pointer
  instead of freeing it (jpgBuf = NULL), which leaked one buffer per frame --
  with PSRAM-backed malloc and QVGA RGB565 that is ~150 KB per frame, so the
  stream died after ~12 frames with "to_jpg: JPG buffer malloc failed".

  Instead of malloc/free churn we hand the encoder a callback that writes into
  ONE buffer, allocated once per stream client and reused for every frame.
  Nothing to forget: the buffer is released when the client disconnects.
----------------------------------------------------------------------------*/
#define JPG_BUF_MAX  (512u * 1024u)     // plenty for any streamed frame size

struct JpgSink {
  uint8_t* buf;
  size_t   cap;      // allocated bytes
  size_t   len;      // bytes written for the current frame
};

static size_t jpgSinkCb(void* arg, size_t index, const void* data, size_t len){
  JpgSink* s = (JpgSink*)arg;
  if (index + len > s->cap) return 0;          // overflow -> encoder aborts
  memcpy(s->buf + index, data, len);
  s->len = index + len;
  return len;
}

/* grow (or first-time allocate) the sink; prefers PSRAM, falls back to RAM */
/* Allocate / grow the sink buffer. Deliberately takes and returns only
   built-in types: the Arduino build system auto-generates a C++ prototype for
   every function in a .ino and drops it near the top of the file, so a
   signature mentioning a struct defined further down would not compile. */
static uint8_t* jpgSinkAlloc(uint8_t* oldBuf, size_t oldCap, size_t need, size_t* capOut){
  if (capOut) *capOut = oldCap;
  if (oldBuf && oldCap >= need) return oldBuf;       // reuse as-is
  if (oldBuf) heap_caps_free(oldBuf);                // wrong size -> reallocate
  size_t want = need + 8192;
  if (want > JPG_BUF_MAX) want = JPG_BUF_MAX;
  uint8_t* p = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = (uint8_t*)heap_caps_malloc(want, MALLOC_CAP_8BIT);
  if (!p){ if (capOut) *capOut = 0; return NULL; }
  if (capOut) *capOut = want;
  return p;
}

/* Make sure the sink can hold `need` bytes (never more than JPG_BUF_MAX) and
   return the capacity it ends up with, or 0 if nothing could be allocated.
   Capping BEFORE the comparison matters: on a frame bigger than the cap the
   naive "grow until cap >= frame size" test would reallocate every single
   frame instead of reusing the buffer. */
static size_t jpgSinkEnsure(uint8_t** buf, size_t* cap, size_t need){
  if (!buf || !cap) return 0;
  if (need > JPG_BUF_MAX) need = JPG_BUF_MAX;
  if (*buf && *cap >= need) return *cap;
  size_t newCap = 0;
  uint8_t* p = jpgSinkAlloc(*buf, *cap, need, &newCap);
  *buf = p;
  *cap = p ? newCap : 0;
  return *cap;
}

static void fpsTick(){
  uint32_t now = millis();
  if (now - gFpsMs >= 1000){
    uint32_t c = frameCountGet();
    gFps = (c - gFpsCount) * 1000.0f / (float)(now - gFpsMs);
    gFpsCount = c;
    gFpsMs    = now;
  }
}

static esp_err_t streamHandler(httpd_req_t* req){
#if WEB_AUTH_ENABLED
  if (req->method == HTTP_GET){
    char auth[128];
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth, sizeof(auth)) != ESP_OK){
      httpd_resp_set_status(req, "401 Unauthorized");
      httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"ESP32-CAM\"");
      httpd_resp_send(req, NULL, 0);
      return ESP_OK;
    }
  }
#endif

  if (!gCameraOk){ httpd_resp_send_500(req); return ESP_FAIL; }

  /* limit viewers: every client owns one JPEG buffer */
  if (__atomic_add_fetch(&gStreamClients, 1, __ATOMIC_SEQ_CST) > MAX_STREAM_CLIENTS){
    __atomic_sub_fetch(&gStreamClients, 1, __ATOMIC_SEQ_CST);
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Too many viewers - close another stream tab.\r\n", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res == ESP_OK) httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (res == ESP_OK) httpd_resp_set_hdr(req, "Cache-Control", "no-store");

  JpgSink      sink      = { NULL, 0, 0 };   // reused for every frame
  camera_fb_t* fb        = NULL;
  char         part[64];
  uint32_t     lastFrame = 0;
  int          failStreak = 0;

  while (res == ESP_OK){
    fb = esp_camera_fb_get();
    if (!fb){
      if (++failStreak >= 5){ res = ESP_FAIL; break; }
      Serial.println("[stream] frame grab failed - retrying");
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    const uint8_t* outBuf = NULL;
    size_t         outLen = 0;

    if (fb->format == PIXFORMAT_JPEG){
      outBuf = fb->buf;                       // driver-owned, sent as-is
      outLen = fb->len;
    }
#if HAVE_IMG_CONVERTERS
    else if (jpgSinkEnsure(&sink.buf, &sink.cap, fb->len + 1024u) > 0){
      sink.len = 0;
      if (frame2jpg_cb(fb, gQuality, jpgSinkCb, &sink) && sink.len > 0){
        outBuf = sink.buf;
        outLen = sink.len;
      }
    }
#endif

    if (!outBuf || outLen == 0){
      esp_camera_fb_return(fb); fb = NULL;    // ALWAYS hand the frame back
      if (++failStreak >= 10){ res = ESP_FAIL; break; }
      Serial.printf("[stream] JPEG conversion failed (heap=%s psram=%s) - retrying\n",
                    bytesToHuman(ESP.getFreeHeap()).c_str(),
                    psramFound() ? bytesToHuman(ESP.getFreePsram()).c_str() : "n/a");
      vTaskDelay(pdMS_TO_TICKS(30));
      continue;
    }

    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)outLen);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)outBuf, outLen);

    /* release: the framebuffer goes back to the driver, the JPEG buffer stays
       ours and is simply overwritten by the next frame (no leak, no churn) */
    esp_camera_fb_return(fb); fb = NULL;

    if (res != ESP_OK) break;                 // client vanished
    frameCountInc(); fpsTick(); failStreak = 0;

    /* FPS cap - keeps the ESP32 cool and the MJPEG smooth on weak phones */
    uint32_t frameMs = 1000 / (gFpsCap ? gFpsCap : 1);
    uint32_t elapsed = millis() - lastFrame;
    if (elapsed < frameMs) vTaskDelay(pdMS_TO_TICKS(frameMs - elapsed));
    else                   vTaskDelay(1);      // always yield: keeps the idle task
                                               // (and the task watchdog) alive
    lastFrame = millis();
  }

  if (sink.buf){ heap_caps_free(sink.buf); sink.buf = NULL; sink.cap = 0; }
  __atomic_sub_fetch(&gStreamClients, 1, __ATOMIC_SEQ_CST);
  Serial.printf("[stream] client disconnected (viewers now %d, psram %s free)\n",
                __atomic_load_n(&gStreamClients, __ATOMIC_RELAXED),
                psramFound() ? bytesToHuman(ESP.getFreePsram()).c_str() : "n/a");
  return res;
}

/* single frame on the stream port: handy for scripts / HA snapshots */
static esp_err_t frameHandler(httpd_req_t* req){
  if (!gCameraOk){ httpd_resp_send_500(req); return ESP_FAIL; }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb){ httpd_resp_send_500(req); return ESP_FAIL; }

  uint8_t* jpg = NULL; size_t len = 0; bool owned = false;
  if (fb->format == PIXFORMAT_JPEG){
    jpg = fb->buf; len = fb->len;
  }
#if HAVE_IMG_CONVERTERS
  else {
    owned = frame2jpg(fb, gQuality, &jpg, &len);
    if (!owned){ jpg = NULL; len = 0; }
  }
#endif

  esp_err_t res;
  if (!jpg || !len){
    res = httpd_resp_send_500(req);
  } else {
    frameCountInc();
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    res = httpd_resp_send(req, (const char*)jpg, len);
  }

  if (fb) esp_camera_fb_return(fb);
  if (owned) free(jpg);
  return res;
}

static bool startStreamServer(){
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port      = STREAM_PORT;
  cfg.ctrl_port        = STREAM_CTRL_PORT;   // MUST differ from the port-80 server
  cfg.stack_size       = 8192;               // frame2jpg needs some room
  cfg.lru_purge_enable = true;               // kick stale clients, not the ESP
  cfg.max_uri_handlers = 6;

  if (httpd_start(&streamServer, &cfg) != ESP_OK){
    Serial.printf("[stream] FATAL: could not start httpd on port %d\n", STREAM_PORT);
    streamServer = NULL;
    return false;
  }

  httpd_uri_t uriStream = {};
  uriStream.uri      = "/stream";
  uriStream.method   = HTTP_GET;
  uriStream.handler  = streamHandler;
  uriStream.user_ctx = NULL;
  httpd_register_uri_handler(streamServer, &uriStream);

  httpd_uri_t uriFrame = {};
  uriFrame.uri       = "/frame";
  uriFrame.method    = HTTP_GET;
  uriFrame.handler   = frameHandler;
  uriFrame.user_ctx  = NULL;
  httpd_register_uri_handler(streamServer, &uriFrame);

  Serial.printf("[stream] MJPEG server ready on port %d  (/stream, /frame)\n", STREAM_PORT);
  return true;
}

/*===========================================================================
 11. OTA + mDNS
===========================================================================*/
static void startOTA(){
  ArduinoOTA.setHostname(cfgHost.c_str());
  if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){ Serial.println("[ota] update starting"); });
  ArduinoOTA.onEnd([](){ Serial.println("\n[ota] done, rebooting"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
    Serial.printf("[ota] %u%%\r", t ? (p * 100) / t : 0);
  });
  ArduinoOTA.onError([](ota_error_t e){ Serial.printf("[ota] error %u\n", (unsigned)e); });
  ArduinoOTA.begin();
  Serial.printf("[ota] ready - password: %s\n", strlen(OTA_PASSWORD) ? OTA_PASSWORD : "(none)");
}

static void startMDNS(){
  if (MDNS.begin(cfgHost.c_str())){
    MDNS.addService("http", "tcp", WEB_PORT);
    MDNS.addService("http", "tcp", STREAM_PORT);
    Serial.printf("[mdns] http://%s.local\n", cfgHost.c_str());
  } else {
    Serial.println("[mdns] start failed (non fatal)");
  }
}

/*===========================================================================
 12. SETUP
===========================================================================*/
static void printBanner(){
  Serial.println();
  Serial.println("========================================================");
  Serial.println("  ESP32-CAM Control Center  ·  RHYX M21-45");
  Serial.println("  Firmware  : v" FW_VERSION "  (AP + STA)");
  Serial.println("  Author    : " OWNER_NAME);
  Serial.println("  GitHub    : " OWNER_GITHUB);
  Serial.println("  Board     : " CAM_BOARD_NAME);
  Serial.printf ("  Chip      : %s rev%u @ %u MHz\n", ESP.getChipModel(),
                 (unsigned)ESP.getChipRevision(), (unsigned)ESP.getCpuFreqMHz());
  Serial.printf ("  PSRAM     : %s\n", psramFound() ? bytesToHuman(ESP.getPsramSize()).c_str() : "not detected");
  Serial.printf ("  Flash     : %s\n", bytesToHuman(ESP.getFlashChipSize()).c_str());
  Serial.println("========================================================");
}

void setup(){
  Serial.begin(115200);
  delay(120);
  Serial.setDebugOutput(false);

  /* Brown-out detector: the camera's inrush current can trip it during WiFi
     TX on weak USB supplies. (Harmless - but use a 5V/2A supply regardless.) */
#ifdef RTC_CNTL_BROWN_OUT_REG
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  printBanner();

  prefsLoad();
  ledInit();
  ledWrite(gLedLevel);

  Serial.println("[boot] starting camera ...");
  if (!cameraBegin()){
    Serial.println("[boot] CAMERA FAILED - the web UI still starts so you can");
    Serial.println("[boot] inspect /status. Check the ribbon cable, 5V/2A power");
    Serial.println("[boot] and that Tools > PSRAM is enabled.");
  }

  Serial.println("[boot] starting network ...");
  networkStart();

  /* ---- routes ---- */
  server.on("/",            HTTP_GET,  handleRoot);
  server.on("/favicon.ico", HTTP_GET,  handleFavicon);
  server.on("/status",      HTTP_GET,  handleStatus);
  server.on("/control",     HTTP_GET,  handleControl);
  server.on("/capture",     HTTP_GET,  handleCapture);
  server.on("/config",      HTTP_GET,  handleConfig);     // action=reconnect|forget
  server.on("/config",      HTTP_POST, handleConfig);     // form body
  server.on("/reboot",      HTTP_GET,  handleReboot);
  server.on("/reboot",      HTTP_POST, handleReboot);
  server.on("/factory",     HTTP_GET,  handleFactory);
  server.on("/factory",     HTTP_POST, handleFactory);
  server.onNotFound(handleNotFound);
  server.enableCORS(true);
  server.begin();
  Serial.printf("[http] dashboard + API ready on port %d\n", WEB_PORT);

  startStreamServer();
  startMDNS();
  startOTA();

  /* Courtesy wait for the router - AFTER the servers are up, so the dashboard
     is already reachable while this runs. Whatever the outcome, staTick()
     keeps retrying with backoff once we fall through to loop(). */
  if (cfgSsid.length() > 0 && WiFi.status() != WL_CONNECTED){
    Serial.printf("[wifi] waiting up to %lus for \"%s\" (dashboard is already up)\n",
                  (unsigned long)(STA_CONNECT_WAIT / 1000UL), cfgSsid.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_CONNECT_WAIT){
      delay(250); Serial.print('.'); yield();
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED){
      Serial.printf("[wifi] no STA yet - sticking to AP mode; the reason appears in the log "
                    "(last: %s) and on the dashboard's Network tab\n",
                    wifiReasonText(gStaReason));
    }
  }

  gFpsMs = millis();
  Serial.println();
  Serial.println("--------------------------------------------------------");
  Serial.printf ("  Dashboard : http://%s/   (AP: http://%s/)\n",
                 WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
                 WiFi.softAPIP().toString().c_str());
  Serial.printf ("  Stream    : http://%s:%d/stream\n",
                 WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str(),
                 STREAM_PORT);
  Serial.printf ("  AP SSID   : %s   (password: %s)\n", cfgApSsid.c_str(),
                 cfgApPass.length() ? cfgApPass.c_str() : "open");
  Serial.println("--------------------------------------------------------");
  Serial.println();
}

/*===========================================================================
 13. LOOP
===========================================================================*/
static uint32_t lastHeapLog = 0;

void loop(){
  server.handleClient();     // port 80: API + UI
  ArduinoOTA.handle();       // OTA updates
  staTick();                 // watchdog for STA reconnects
  fpsTick();                 // live FPS statistic

  /* periodic health line (every 60 s) */
  uint32_t now = millis();
  if (now - lastHeapLog > 60000){
    lastHeapLog = now;
    Serial.printf("[health] up=%lus heap=%s psram=%s rssi=%d fps=%.1f frames=%lu clients=%d viewers=%d\n",
                  (unsigned long)uptimeSeconds(),
                  bytesToHuman(ESP.getFreeHeap()).c_str(),
                  psramFound() ? bytesToHuman(ESP.getFreePsram()).c_str() : "n/a",
                  WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
                  gFps, (unsigned long)frameCountGet(), WiFi.softAPgetStationNum(),
                  __atomic_load_n(&gStreamClients, __ATOMIC_RELAXED));
  }

  delay(2);                  // keep the RTOS scheduler + watchdogs happy
}

/*===========================================================================
 14. EMBEDDED DASHBOARD  (single-file edition)
     The dashboard is stored at the very bottom of this file inside a raw
     string literal, so this sketch is completely self-contained: copy/paste
     it into an empty Arduino sketch and it builds.

     The JavaScript uses ARROW functions on purpose. The Arduino builder runs
     an automatic C++ prototype generator over .ino files, and with classic
     "function foo(){" syntax it emitted a C++ prototype like
         function foo();
     OUTSIDE the string literal, which broke the build. Arrow functions are
     invisible to that generator, so everything stays inside the string.
===========================================================================*/


/*===========================================================================
 15. THE DASHBOARD  (the page served by handleRoot() above)
     Keep the delimiters intact:   R"SURYAUI(   ...   )SURYAUI"
     and never write the exact sequence   )SURYAUI"   inside the page.
===========================================================================*/
static const char INDEX_HTML[] PROGMEM = R"SURYAUI(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover" />
<meta name="theme-color" content="#070b14" />
<title>ESP32-CAM Control Center &middot; Surya Bajpai</title>
<style>
  *,*::before,*::after{box-sizing:border-box}
  :root{
    --bg:#070b14; --bg2:#0a1020; --card:#0e1626; --card2:#121c30; --card3:#16203a;
    --line:rgba(255,255,255,.08); --line2:rgba(255,255,255,.14);
    --txt:#e8eefb; --muted:#8d9dbb; --dim:#66748f;
    --acc:#22d3ee; --acc2:#a855f7; --ok:#22c55e; --warn:#f59e0b; --err:#ef4444;
    --r:16px; --r2:12px;
    --shadow:0 18px 40px -18px rgba(0,0,0,.85);
  }
  html,body{margin:0;padding:0}
  body{
    background:
      radial-gradient(1100px 600px at 12% -8%, rgba(34,211,238,.10), transparent 60%),
      radial-gradient(900px 520px at 92% 0%, rgba(168,85,247,.12), transparent 62%),
      var(--bg);
    color:var(--txt);
    font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
    font-size:15px; line-height:1.5; min-height:100vh;
    -webkit-font-smoothing:antialiased;
  }
  a{color:var(--acc);text-decoration:none}
  a:hover{text-decoration:underline}
  .wrap{max-width:1280px;margin:0 auto;padding:0 16px 40px}

  /* ---------- top bar ---------- */
  .topbar{
    position:sticky;top:0;z-index:40;
    backdrop-filter:blur(14px);-webkit-backdrop-filter:blur(14px);
    background:rgba(7,11,20,.82);border-bottom:1px solid var(--line);
  }
  .topbar-in{max-width:1280px;margin:0 auto;padding:12px 16px;display:flex;align-items:center;gap:14px;flex-wrap:wrap}
  .brand{display:flex;align-items:center;gap:12px;min-width:0}
  .logo{
    width:42px;height:42px;border-radius:12px;flex:0 0 auto;
    background:linear-gradient(140deg,var(--acc),#0ea5e9 45%,var(--acc2));
    display:grid;place-items:center;box-shadow:0 8px 22px -10px var(--acc);
  }
  .logo svg{width:24px;height:24px;fill:#04121a}
  .brand-t{min-width:0}
  .brand-t h1{margin:0;font-size:17px;font-weight:800;letter-spacing:.2px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
  .brand-t p{margin:1px 0 0;font-size:12px;color:var(--muted)}
  .brand-t p b{color:var(--txt);font-weight:700}
  .topbar-right{margin-left:auto;display:flex;align-items:center;gap:10px;flex-wrap:wrap}

  .pill{
    display:inline-flex;align-items:center;gap:8px;padding:7px 13px;border-radius:999px;
    background:var(--card);border:1px solid var(--line);font-size:12.5px;font-weight:600;color:var(--muted);
    white-space:nowrap;
  }
  .dot{width:9px;height:9px;border-radius:50%;background:var(--dim);flex:0 0 auto}
  .pill.live{color:#8ff0c0;border-color:rgba(34,197,94,.35);background:rgba(34,197,94,.10)}
  .pill.live .dot{background:var(--ok);box-shadow:0 0 0 0 rgba(34,197,94,.7);animation:pulse 1.8s infinite}
  .pill.off{color:#ffc9c9;border-color:rgba(239,68,68,.32);background:rgba(239,68,68,.10)}
  .pill.off .dot{background:var(--err)}
  @keyframes pulse{0%{box-shadow:0 0 0 0 rgba(34,197,94,.65)}70%{box-shadow:0 0 0 9px rgba(34,197,94,0)}100%{box-shadow:0 0 0 0 rgba(34,197,94,0)}}

  .btn{
    appearance:none;border:1px solid var(--line2);background:var(--card2);color:var(--txt);
    padding:9px 14px;border-radius:11px;font-size:13.5px;font-weight:650;cursor:pointer;
    display:inline-flex;align-items:center;gap:8px;transition:.16s transform,.16s background,.16s border-color;
    font-family:inherit;
  }
  .btn:hover{background:var(--card3);border-color:rgba(255,255,255,.24)}
  .btn:active{transform:translateY(1px)}
  .btn svg{width:16px;height:16px;fill:currentColor}
  .btn.primary{background:linear-gradient(135deg,var(--acc),#0ea5e9);border-color:transparent;color:#04121a}
  .btn.primary:hover{filter:brightness(1.08)}
  .btn.violet{background:linear-gradient(135deg,var(--acc2),#6366f1);border-color:transparent;color:#fff}
  .btn.danger{background:rgba(239,68,68,.14);border-color:rgba(239,68,68,.4);color:#ffb4b4}
  .btn.ghost{background:transparent}
  .btn.ghost:hover{background:var(--card2)}
  .btn[disabled]{opacity:.5;cursor:not-allowed}
  .btn.small{padding:7px 11px;font-size:12.5px}

  /* ---------- layout ---------- */
  .grid{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(0,1fr);gap:16px;margin-top:18px;align-items:start}
  @media (max-width:1000px){.grid{grid-template-columns:1fr}}
  .card{background:linear-gradient(180deg,var(--card),var(--bg2));border:1px solid var(--line);border-radius:var(--r);box-shadow:var(--shadow);overflow:hidden}
  .card-h{display:flex;align-items:center;gap:10px;padding:14px 16px;border-bottom:1px solid var(--line)}
  .card-h h2{margin:0;font-size:14.5px;font-weight:750;letter-spacing:.3px}
  .card-h .sub{margin-left:auto;font-size:12px;color:var(--muted);font-weight:600}
  .card-b{padding:16px}

  /* ---------- viewer ---------- */
  .frame{
    position:relative;background:#03060d;border-radius:var(--r2);overflow:hidden;
    border:1px solid var(--line);aspect-ratio:4/3;display:grid;place-items:center;
  }
  .frame img{width:100%;height:100%;object-fit:contain;display:block;background:#03060d}
  .frame img.hidden{display:none}
  .placeholder{position:absolute;inset:0;display:grid;place-items:center;text-align:center;padding:20px;color:var(--dim)}
  .placeholder.hidden{display:none}
  .placeholder .ico{width:64px;height:64px;margin:0 auto 12px;opacity:.5}
  .placeholder .ico svg{width:100%;height:100%;fill:currentColor}
  .placeholder b{display:block;color:var(--muted);font-size:14.5px;font-weight:700}
  .placeholder span{font-size:12.5px}
  .badges{position:absolute;top:10px;left:10px;display:flex;gap:7px;flex-wrap:wrap}
  .badge{
    font-size:11.5px;font-weight:750;letter-spacing:.4px;padding:5px 9px;border-radius:8px;
    background:rgba(4,10,20,.72);border:1px solid var(--line2);backdrop-filter:blur(6px);color:var(--muted);
  }
  .badge.live{color:#8ff0c0;border-color:rgba(34,197,94,.4)}
  .badges.right{left:auto;right:10px}
  .frame-foot{
    position:absolute;left:0;right:0;bottom:0;display:flex;gap:10px;flex-wrap:wrap;
    padding:9px 12px;font-size:11.5px;color:var(--muted);
    background:linear-gradient(0deg,rgba(3,6,13,.92),rgba(3,6,13,0));
  }
  .frame-foot b{color:var(--txt);font-weight:700}
  .actions{display:flex;gap:9px;flex-wrap:wrap;margin-top:14px}
  .sliders{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:14px;margin-top:16px}
  .slider{background:var(--card2);border:1px solid var(--line);border-radius:var(--r2);padding:11px 13px}
  .slider label{display:flex;justify-content:space-between;font-size:12px;font-weight:700;color:var(--muted);margin-bottom:8px}
  .slider label span{color:var(--acc);font-variant-numeric:tabular-nums}
  input[type=range]{
    -webkit-appearance:none;appearance:none;width:100%;height:5px;border-radius:99px;background:rgba(255,255,255,.14);outline:none;
  }
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:17px;height:17px;border-radius:50%;background:var(--acc);border:2px solid #04121a;cursor:pointer;box-shadow:0 0 0 3px rgba(34,211,238,.18)}
  input[type=range]::-moz-range-thumb{width:15px;height:15px;border-radius:50%;background:var(--acc);border:2px solid #04121a;cursor:pointer}

  /* ---------- tabs ---------- */
  .tabs{display:flex;gap:6px;padding:10px 12px;border-bottom:1px solid var(--line);background:rgba(255,255,255,.015);overflow-x:auto}
  .tab{
    border:1px solid transparent;background:transparent;color:var(--muted);font-family:inherit;
    padding:8px 13px;border-radius:10px;font-size:13px;font-weight:700;cursor:pointer;white-space:nowrap;
  }
  .tab:hover{color:var(--txt);background:var(--card2)}
  .tab.active{color:#04121a;background:linear-gradient(135deg,var(--acc),#38bdf8);border-color:transparent}
  .pane{display:none;padding:16px}
  .pane.active{display:block}

  .row{display:grid;grid-template-columns:1fr auto;gap:12px;align-items:center;padding:10px 0;border-bottom:1px dashed rgba(255,255,255,.07)}
  .row:last-child{border-bottom:0}
  .row .k{font-size:13px;font-weight:650}
  .row .k small{display:block;color:var(--dim);font-weight:500;font-size:11.5px}
  .row .v{font-size:13px;font-weight:750;font-variant-numeric:tabular-nums;text-align:right;color:var(--txt)}
  .row .v.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12.5px}

  select,input[type=text],input[type=password],input[type=number]{
    background:var(--card2);border:1px solid var(--line2);color:var(--txt);border-radius:10px;
    padding:8px 10px;font-size:13px;font-family:inherit;min-width:130px;max-width:100%;
  }
  select:focus,input:focus{outline:none;border-color:var(--acc);box-shadow:0 0 0 3px rgba(34,211,238,.13)}
  .field{display:block;margin-bottom:12px}
  .field span{display:block;font-size:12px;font-weight:700;color:var(--muted);margin-bottom:6px}
  .field input,.field select{width:100%}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
  @media (max-width:620px){.grid2{grid-template-columns:1fr}}

  .sw{position:relative;width:46px;height:26px;flex:0 0 auto}
  .sw input{opacity:0;width:0;height:0;position:absolute}
  .sw i{position:absolute;inset:0;background:rgba(255,255,255,.16);border-radius:99px;transition:.18s;cursor:pointer}
  .sw i::after{content:"";position:absolute;top:3px;left:3px;width:20px;height:20px;border-radius:50%;background:#fff;transition:.18s}
  .sw input:checked + i{background:linear-gradient(135deg,var(--acc),#22c55e)}
  .sw input:checked + i::after{transform:translateX(20px)}

  .info{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  @media (max-width:620px){.info{grid-template-columns:1fr}}
  .info .box{background:var(--card2);border:1px solid var(--line);border-radius:var(--r2);padding:11px 13px}
  .info .box .l{font-size:11px;letter-spacing:.6px;text-transform:uppercase;color:var(--dim);font-weight:750}
  .info .box .d{font-size:13.5px;font-weight:700;margin-top:3px;word-break:break-all}
  .info .box .d.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12.5px}
  .hint{font-size:12px;color:var(--dim);margin:10px 0 0}
  .note{
    font-size:12px;color:#ffd9a8;background:rgba(245,158,11,.10);border:1px solid rgba(245,158,11,.3);
    border-radius:var(--r2);padding:10px 12px;margin-bottom:14px;line-height:1.5;
  }
  .note b{color:#ffc46b}
  .divider{height:1px;background:var(--line);margin:16px 0}
  .linkbar{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}

  /* ---------- toast ---------- */
  #toasts{position:fixed;right:16px;bottom:16px;z-index:90;display:flex;flex-direction:column;gap:9px;max-width:min(92vw,360px)}
  .toast{
    background:var(--card3);border:1px solid var(--line2);border-left:4px solid var(--acc);
    border-radius:12px;padding:11px 13px;font-size:13px;font-weight:600;box-shadow:var(--shadow);
    animation:slidein .22s ease-out
  }
  .toast.ok{border-left-color:var(--ok)}
  .toast.err{border-left-color:var(--err)}
  .toast.warn{border-left-color:var(--warn)}
  @keyframes slidein{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:none}}

  footer{margin-top:26px;text-align:center;color:var(--dim);font-size:12.5px;line-height:1.9}
  footer .heart{color:#ff5c8a}
  footer b{color:var(--txt)}
  .github{
    display:inline-flex;align-items:center;gap:8px;padding:8px 14px;border-radius:11px;
    background:linear-gradient(135deg,rgba(168,85,247,.22),rgba(99,102,241,.22));
    border:1px solid rgba(168,85,247,.4);color:#e9dcff;font-weight:700;font-size:13px
  }
  .github:hover{text-decoration:none;filter:brightness(1.12)}
  .github svg{width:17px;height:17px;fill:currentColor}
  code.k{
    background:rgba(255,255,255,.07);border:1px solid var(--line);border-radius:7px;padding:2px 6px;
    font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12.5px
  }
</style>
</head>
<body>

<header class="topbar">
  <div class="topbar-in">
    <div class="brand">
      <div class="logo" aria-hidden="true">
        <svg viewBox="0 0 24 24"><path d="M9 3h6l1.2 2H20a2 2 0 0 1 2 2v11a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V7a2 2 0 0 1 2-2h3.8L9 3zm3 5.5A5.5 5.5 0 1 0 12 19.5 5.5 5.5 0 0 0 12 8.5zm0 2.4a3.1 3.1 0 1 1 0 6.2 3.1 3.1 0 0 1 0-6.2z"/></svg>
      </div>
      <div class="brand-t">
        <h1>ESP32-CAM Control Center</h1>
        <p>RHYX M21-45 &middot; <b>Surya Bajpai</b> &middot; AP + STA</p>
      </div>
    </div>
    <div class="topbar-right">
      <span class="pill off" id="statePill"><i class="dot"></i><span id="stateTxt">Connecting&hellip;</span></span>
      <a class="github" href="https://github.com/Surya-8948" target="_blank" rel="noopener noreferrer">
        <svg viewBox="0 0 16 16"><path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82a7.4 7.4 0 0 1 2-.27c.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.01 8.01 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
        Surya-8948
      </a>
    </div>
  </div>
</header>

<div class="wrap">
  <div class="grid">

    <!-- ================= VIEWER ================= -->
    <section class="card">
      <div class="card-h">
        <h2>Live View</h2>
        <span class="sub" id="viewSub">MJPEG stream</span>
      </div>
      <div class="card-b">
        <div class="frame" id="frame">
          <img id="stream" class="hidden" alt="Live camera stream" />
          <div class="placeholder" id="placeholder">
            <div>
              <div class="ico">
                <svg viewBox="0 0 24 24"><path d="M9 3h6l1.2 2H20a2 2 0 0 1 2 2v11a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V7a2 2 0 0 1 2-2h3.8L9 3zm3 5.5A5.5 5.5 0 1 0 12 19.5 5.5 5.5 0 0 0 12 8.5zm0 2.4a3.1 3.1 0 1 1 0 6.2 3.1 3.1 0 0 1 0-6.2z"/></svg>
              </div>
              <b>Stream stopped</b>
              <span>Press <b>Start Stream</b> to view the live feed</span>
            </div>
          </div>
          <div class="badges">
            <span class="badge" id="bLive">OFFLINE</span>
            <span class="badge" id="bRes">&mdash;</span>
          </div>
          <div class="badges right">
            <span class="badge" id="bFps">&mdash; FPS</span>
          </div>
          <div class="frame-foot">
            <span>Sensor <b id="fSensor">RHYX M21-45 / GC2145</b></span>
            <span>Mode <b id="fMode">AP + STA</b></span>
            <span>IP <b id="fIp" class="mono">&mdash;</b></span>
          </div>
        </div>

        <div class="actions">
          <button class="btn primary" id="btnStart">
            <svg viewBox="0 0 24 24"><path d="M8 5v14l11-7z"/></svg> Start Stream
          </button>
          <button class="btn" id="btnStop" disabled>
            <svg viewBox="0 0 24 24"><path d="M6 6h12v12H6z"/></svg> Stop
          </button>
          <button class="btn violet" id="btnSnap">
            <svg viewBox="0 0 24 24"><path d="M9 3h6l1.2 2H20a2 2 0 0 1 2 2v11a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V7a2 2 0 0 1 2-2h3.8L9 3zm3 5a5 5 0 1 0 0 10 5 5 0 0 0 0-10z"/></svg> Snapshot
          </button>
          <button class="btn ghost" id="btnFull">
            <svg viewBox="0 0 24 24"><path d="M3 3h7v2H5v5H3V3zm11 0h7v7h-2V5h-5V3zM3 14h2v5h5v2H3v-7zm16 0h2v7h-7v-2h5v-5z"/></svg> Fullscreen
          </button>
          <button class="btn ghost" id="btnFlash">
            <svg viewBox="0 0 24 24"><path d="M13 2 4.5 13H11l-1 9 8.5-11H12l1-9z"/></svg> Flash: Off
          </button>
        </div>

        <div class="sliders">
          <div class="slider">
            <label>Stream FPS cap <span id="vFps">10</span></label>
            <input type="range" id="sFps" min="1" max="25" value="10" />
          </div>
          <div class="slider">
            <label>JPEG quality <span id="vQual">12</span></label>
            <input type="range" id="sQual" min="4" max="63" value="12" />
          </div>
          <div class="slider">
            <label>Flash intensity <span id="vLed">0</span></label>
            <input type="range" id="sLed" min="0" max="255" value="0" />
          </div>
        </div>
        <p class="hint">Tip: the RHYX M21-45 (GC2145) sensor has no hardware JPEG encoder, so frames are converted in software. QVGA&ndash;VGA at 8&ndash;12&nbsp;FPS gives the smoothest result.</p>
      </div>
    </section>

    <!-- ================= SETTINGS ================= -->
    <aside class="card">
      <div class="tabs">
        <button class="tab active" data-pane="cam">Camera</button>
        <button class="tab" data-pane="net">Network</button>
        <button class="tab" data-pane="sys">System</button>
      </div>

      <!-- camera pane -->
      <div class="pane active" id="pane-cam">
        <div class="note" id="sensorNote">
          <b>Auto-detect:</b> if the sensor reports &ldquo;JPEG not supported&rdquo; (error 0x106) the firmware automatically
          restarts it in RGB565 and converts to JPEG in software. Works with RHYX M21-45, GC2145 and OV2640/OV3660.
        </div>

        <div class="field">
          <span>Resolution</span>
          <select id="selRes"></select>
        </div>

        <div class="row"><div class="k">Brightness <small>-2 &hellip; +2</small></div>
          <div class="v"><select id="vBrightness"><option>-2</option><option>-1</option><option>0</option><option>1</option><option>2</option></select></div></div>
        <div class="row"><div class="k">Contrast <small>-2 &hellip; +2</small></div>
          <div class="v"><select id="vContrast"><option>-2</option><option>-1</option><option>0</option><option>1</option><option>2</option></select></div></div>
        <div class="row"><div class="k">Saturation <small>-2 &hellip; +2</small></div>
          <div class="v"><select id="vSaturation"><option>-2</option><option>-1</option><option>0</option><option>1</option><option>2</option></select></div></div>
        <div class="row"><div class="k">Special effect</div>
          <div class="v"><select id="vEffect">
            <option value="0">None</option><option value="1">Negative</option><option value="2">Grayscale</option>
            <option value="3">Reddish</option><option value="4">Greenish</option><option value="5">Bluish</option></select></div></div>

        <div class="divider"></div>

        <div class="row"><div class="k">Horizontal mirror</div>
          <div class="v"><label class="sw"><input type="checkbox" id="swHmirror"><i></i></label></div></div>
        <div class="row"><div class="k">Vertical flip</div>
          <div class="v"><label class="sw"><input type="checkbox" id="swVflip"><i></i></label></div></div>
        <div class="row"><div class="k">White balance (AWB)</div>
          <div class="v"><label class="sw"><input type="checkbox" id="swWb" checked><i></i></label></div></div>
        <div class="row"><div class="k">Exposure control (AEC)</div>
          <div class="v"><label class="sw"><input type="checkbox" id="swExposure" checked><i></i></label></div></div>
        <div class="row"><div class="k">Gain control (AGC)</div>
          <div class="v"><label class="sw"><input type="checkbox" id="swGain" checked><i></i></label></div></div>
        <div class="row"><div class="k">Auto flash on snapshot</div>
          <div class="v"><label class="sw"><input type="checkbox" id="swAutoFlash"><i></i></label></div></div>

        <div class="divider"></div>
        <div class="field"><span>AGC gain <small style="font-weight:500;color:var(--dim)">(0&ndash;30)</small></span>
          <input type="range" id="sGain" min="0" max="30" value="0" /></div>
        <div class="actions">
          <button class="btn small" id="btnCamDefault">Reset camera defaults</button>
          <button class="btn small ghost" id="btnReloadCam">Reload values</button>
        </div>
      </div>

      <!-- network pane -->
      <div class="pane" id="pane-net">
        <div class="info">
          <div class="box"><div class="l">STA status</div><div class="d" id="nSta">&mdash;</div></div>
          <div class="box"><div class="l">STA IP</div><div class="d mono" id="nIp">&mdash;</div></div>
          <div class="box"><div class="l">Signal</div><div class="d" id="nRssi">&mdash;</div></div>
          <div class="box"><div class="l">AP SSID</div><div class="d mono" id="nApSsid">&mdash;</div></div>
          <div class="box"><div class="l">AP IP</div><div class="d mono" id="nApIp">192.168.4.1</div></div>
          <div class="box"><div class="l">AP clients</div><div class="d" id="nClients">0</div></div>
        </div>
        <div class="note warn" id="staNote" style="display:none"></div>

        <div class="divider"></div>
        <form id="netForm" autocomplete="off">
          <div class="grid2">
            <label class="field"><span>Wi-Fi SSID (2.4 GHz)</span>
              <input type="text" id="fSsid" placeholder="Your home Wi-Fi" maxlength="32" /></label>
            <label class="field"><span>Wi-Fi password</span>
              <input type="password" id="fPass" placeholder="Leave blank to keep current" maxlength="64" /></label>
            <label class="field"><span>Device hostname (mDNS)</span>
              <input type="text" id="fHost" placeholder="suryacam" maxlength="32" /></label>
            <label class="field"><span>AP SSID</span>
              <input type="text" id="fApSsid" placeholder="SuryaCAM-ESP32" maxlength="32" /></label>
            <label class="field"><span>AP password (min 8 chars)</span>
              <input type="password" id="fApPass" placeholder="Leave blank to keep current" maxlength="64" /></label>
          </div>
          <div class="actions">
            <button class="btn primary" type="submit">Save &amp; Apply</button>
            <button class="btn" type="button" id="btnReconnect">Reconnect Wi-Fi</button>
            <button class="btn danger" type="button" id="btnForget">Forget Wi-Fi</button>
          </div>
          <p class="hint">Credential changes are written to NVS flash and applied instantly &mdash; the AP stays up so you never lock yourself out. Use <code class="k">http://&lt;sta-ip&gt;/</code> or <code class="k">http://&lt;hostname&gt;.local/</code> or <code class="k">http://192.168.4.1/</code> (AP).</p>
        </form>
      </div>

      <!-- system pane -->
      <div class="pane" id="pane-sys">
        <div class="info">
          <div class="box"><div class="l">Owner</div><div class="d">Surya Bajpai</div></div>
          <div class="box"><div class="l">GitHub</div><div class="d"><a href="https://github.com/Surya-8948" target="_blank" rel="noopener noreferrer">github.com/Surya-8948</a></div></div>
          <div class="box"><div class="l">Firmware</div><div class="d mono" id="sFw">&mdash;</div></div>
          <div class="box"><div class="l">Board</div><div class="d" id="sBoard">&mdash;</div></div>
          <div class="box"><div class="l">Sensor</div><div class="d" id="sSensor">&mdash;</div></div>
          <div class="box"><div class="l">Pixel path</div><div class="d" id="sFormat">&mdash;</div></div>
          <div class="box"><div class="l">Uptime</div><div class="d" id="sUp">&mdash;</div></div>
          <div class="box"><div class="l">Wi-Fi RSSI</div><div class="d" id="sRssi">&mdash;</div></div>
          <div class="box"><div class="l">Free heap</div><div class="d mono" id="sHeap">&mdash;</div></div>
          <div class="box"><div class="l">Free PSRAM</div><div class="d mono" id="sPsram">&mdash;</div></div>
          <div class="box"><div class="l">Chip / CPU</div><div class="d" id="sChip">&mdash;</div></div>
          <div class="box"><div class="l">MAC</div><div class="d mono" id="sMac">&mdash;</div></div>
        </div>

        <div class="divider"></div>
        <div class="row"><div class="k">Stream URL <small>paste into VLC / OBS / Home Assistant</small></div>
          <div class="v mono" id="sStreamUrl">&mdash;</div></div>
        <div class="actions">
          <button class="btn small" id="btnCopy">Copy stream URL</button>
          <button class="btn small ghost" id="btnOpenCap">Open /capture</button>
          <button class="btn small ghost" id="btnRefresh">Refresh status</button>
        </div>
        <div class="divider"></div>
        <div class="actions">
          <button class="btn danger" id="btnReboot">Reboot device</button>
          <button class="btn danger ghost" id="btnFactory">Factory reset</button>
        </div>
        <p class="hint">OTA updates are enabled: the device appears as <code class="k" id="sOtaName">suryacam</code> in the Arduino IDE
        (Ports &rarr; Network ports) and as <code class="k">http://&lt;hostname&gt;.local/update</code> in ESPHome / espota.</p>
      </div>
    </aside>
  </div>

  <footer>
    <div>Designed &amp; developed with <span class="heart">&hearts;</span> by <b>Surya Bajpai</b></div>
    <div><a class="github" href="https://github.com/Surya-8948" target="_blank" rel="noopener noreferrer">
      <svg viewBox="0 0 16 16"><path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82a7.4 7.4 0 0 1 2-.27c.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.01 8.01 0 0 0 16 8c0-4.42-3.58-8-8-8z"/></svg>
      github.com/Surya-8948</a></div>
    <div>ESP32-CAM &middot; RHYX M21-45 &middot; firmware <span id="footFw">v1.0.0</span></div>
  </footer>
</div>

<div id="toasts"></div>

<script>
"use strict";
/* ============================================================
   ESP32-CAM Control Center  &middot;  Surya Bajpai
   Vanilla JS only — no frameworks, no CDN, works fully offline
   (required for AP mode where there is no internet access).
   ============================================================ */

var S = {
  streaming: false,
  online: false,
  primed: false,
  status: null,
  led: 0,
  fps: 10,
  quality: 12
};

/* MJPEG is served by a dedicated raw httpd instance on its own port so the
   control API on port 80 always stays responsive. The port is reported by
   /status and can be changed with the STREAM_PORT define. */
var SP = 81;
const streamURL = () => {
  return location.protocol + '//' + location.hostname + ':' + SP + '/stream?ts=' + Date.now();
}

const $ = (id) => { return document.getElementById(id); }

const toast = (msg, kind) => {
  var host = $('toasts');
  var d = document.createElement('div');
  d.className = 'toast ' + (kind || '');
  d.textContent = msg;
  host.appendChild(d);
  setTimeout(() => {
    d.style.transition = 'opacity .3s, transform .3s';
    d.style.opacity = '0';
    d.style.transform = 'translateY(8px)';
    setTimeout(() => { d.remove(); }, 320);
  }, 2600);
}

/* ---------- http helpers ---------- */
const api = async (path, opts) => {
  try{
    var ctl = new AbortController();
    var timer = setTimeout(() => { ctl.abort(); }, 7000);
    var r = await fetch(path, Object.assign({ cache:'no-store', signal: ctl.signal }, opts || {}));
    clearTimeout(timer);
    var txt = await r.text();
    if (!txt){ return { ok: r.ok }; }
    try { return JSON.parse(txt); } catch(e){ return { ok: r.ok, raw: txt }; }
  }catch(e){
    return null;
  }
}

const fmtBytes = (n) => {
  if (n === undefined || n === null || isNaN(n)) return '—';
  if (n < 1024) return n + ' B';
  if (n < 1048576) return (n/1024).toFixed(1) + ' KB';
  return (n/1048576).toFixed(2) + ' MB';
}
const fmtUptime = (s) => {
  if (s === undefined || s === null) return '—';
  var d = Math.floor(s/86400), h = Math.floor(s%86400/3600), m = Math.floor(s%3600/60), x = s%60;
  if (d) return d + 'd ' + h + 'h ' + m + 'm';
  if (h) return h + 'h ' + m + 'm ' + x + 's';
  if (m) return m + 'm ' + x + 's';
  return x + 's';
}
const stamp = () => {
  var d = new Date(), p = (n) => { return (n<10?'0':'')+n; };
  return d.getFullYear() + p(d.getMonth()+1) + p(d.getDate()) + '-' + p(d.getHours()) + p(d.getMinutes()) + p(d.getSeconds());
}

/* ---------- streaming ---------- */
const startStream = () => {
  var img = $('stream');
  $('btnStart').disabled = true;
  img.onload = () => {
    S.streaming = true;
    img.classList.remove('hidden');
    $('placeholder').classList.add('hidden');
    $('btnStop').disabled = false;
    $('bLive').textContent = 'LIVE';
    $('bLive').classList.add('live');
    $('statePill').className = 'pill live';
    $('stateTxt').textContent = 'Streaming';
    toast('Live stream started', 'ok');
  };
  img.onerror = () => {
    stopStream(true);
    toast('Stream failed — check Wi-Fi and retry', 'err');
  };
  img.src = streamURL();
}
const stopStream = (silent) => {
  var img = $('stream');
  img.onload = null; img.onerror = null;
  img.removeAttribute('src');
  img.classList.add('hidden');
  $('placeholder').classList.remove('hidden');
  S.streaming = false;
  $('btnStart').disabled = false;
  $('btnStop').disabled = true;
  $('bLive').textContent = 'OFFLINE';
  $('bLive').classList.remove('live');
  if ($('statePill').className.indexOf('off') < 0){
    $('statePill').className = 'pill';
    $('stateTxt').textContent = 'Idle';
  }
  if (!silent) toast('Stream stopped');
}

/* ---------- control ---------- */
const setVar = async (name, val, quiet) => {
  var r = await api('/control?var=' + encodeURIComponent(name) + '&val=' + encodeURIComponent(val));
  if (r === null){
    if (!quiet) toast('Device unreachable', 'err');
    return false;
  }
  if (r.ok === false && r.unsupported){
    if (!quiet) toast(name + ' is not supported by this sensor', 'warn');
    return false;
  }
  if (r.ok === false){
    if (!quiet) toast('Rejected: ' + (r.msg || name), 'err');
    return false;
  }
  if (!quiet) toast(name + ' = ' + val, 'ok');
  return true;
}

/* ---------- status ---------- */
const buildResList = (list, current) => {
  var sel = $('selRes');
  if (list && list.length){
    var wanted = JSON.stringify(list);
    if (sel.dataset.sig !== wanted){
      sel.dataset.sig = wanted;
      sel.innerHTML = '';
      list.forEach((o) => {
        var op = document.createElement('option');
        op.value = o.v; op.textContent = o.n;
        sel.appendChild(op);
      });
    }
    if (document.activeElement !== sel) sel.value = String(current);
  }
}

const setOnline = (st) => {
  S.online = true; S.status = st;
  if (document.activeElement !== $('selRes')) { /* handled in buildResList */ }
  $('statePill').className = 'pill ' + (S.streaming ? 'live' : '');
  $('stateTxt').textContent = S.streaming ? 'Streaming' : (st.ap ? 'AP ready' : 'Online');
  if (st.streamPort) SP = st.streamPort;
  $('bRes').textContent = st.fsName || '—';
  $('bFps').textContent = (st.fps || 0) + ' FPS';
  $('fSensor').textContent = st.sensor || '—';
  $('fMode').textContent = st.mode || '—';
  $('fIp').textContent = st.ip || st.apIp || '—';

  buildResList(st.fsList, st.fs);

  $('nSta').textContent  = st.sta ? 'Connected' : (st.ssid ? 'Disconnected' : 'Not configured');
  {
    const sn = $('staNote');
    if (!st.ssid || st.sta) {
      sn.style.display = 'none';
    } else {
      sn.style.display = '';
      sn.textContent = 'STA not connected' +
        (st.staReasonText ? ' - last failure: ' + st.staReasonText + (st.staReason ? ' (code ' + st.staReason + ')' : '') : '') +
        '. The AP stays up and retries continue in the background (tries so far: ' + (st.staTries || 0) + '). ' +
        'Heads-up: the ESP32 radio is 2.4 GHz only and needs a WPA2 or WPA2/WPA3-mixed network.';
    }
  }
  $('nIp').textContent   = st.ip || '—';
  $('nRssi').textContent = st.sta ? (st.rssi + ' dBm (' + Math.max(0, Math.min(100, 2*(st.rssi+100))) + '%)') : '—';
  $('nApSsid').textContent = st.apSsid || '—';
  $('nApIp').textContent   = st.apIp || '—';
  $('nClients').textContent = st.clients || 0;

  $('sFw').textContent     = 'v' + (st.fw || '?');
  $('sBoard').textContent  = st.board || '—';
  $('sSensor').textContent = st.sensor || '—';
  $('sFormat').textContent = st.swJpeg ? 'RGB565 → software JPEG' : 'Hardware JPEG';
  $('sUp').textContent     = fmtUptime(st.uptime);
  $('sRssi').textContent   = st.sta ? (st.rssi + ' dBm') : '—';
  $('sHeap').textContent   = fmtBytes(st.heap);
  $('sPsram').textContent  = st.psram ? fmtBytes(st.psramFree) : 'not present';
  $('sChip').textContent   = (st.chip || '—') + ' @ ' + (st.cpu || '?') + ' MHz';
  $('sMac').textContent    = st.mac || '—';
  $('sStreamUrl').textContent = 'http://' + (st.ip || st.apIp || '192.168.4.1') + ':' + SP + '/stream';
  $('sOtaName').textContent = (st.host || 'suryacam') + '  /  ' + (st.host || 'suryacam') + '.local';
  $('footFw').textContent  = 'v' + (st.fw || '1.0.0');

  if (!S.primed){
    S.primed = true;
    if (st.vflip !== undefined) $('swVflip').checked = !!st.vflip;
    if (st.hmirror !== undefined) $('swHmirror').checked = !!st.hmirror;
    if (st.quality !== undefined){ S.quality = st.quality; $('sQual').value = st.quality; $('vQual').textContent = st.quality; }
    if (st.fpsCap !== undefined){ S.fps = st.fpsCap; $('sFps').value = st.fpsCap; $('vFps').textContent = st.fpsCap; }
    if (st.brightness !== undefined) $('vBrightness').value = st.brightness;
    if (st.contrast !== undefined) $('vContrast').value = st.contrast;
    if (st.saturation !== undefined) $('vSaturation').value = st.saturation;
    if (st.wb !== undefined) $('swWb').checked = !!st.wb;
    if (st.exposure !== undefined) $('swExposure').checked = !!st.exposure;
    if (st.gain !== undefined) $('swGain').checked = !!st.gain;
    if (st.led !== undefined){ S.led = st.led; $('sLed').value = st.led; $('vLed').textContent = st.led; $('btnFlash').innerHTML = flashIcon() + ' Flash: ' + (st.led ? 'On' : 'Off'); }
    if (st.autoFlash !== undefined) $('swAutoFlash').checked = !!st.autoFlash;
    if (st.ssid) $('fSsid').value = st.ssid;
    if (st.host) $('fHost').value = st.host;
    if (st.apSsid) $('fApSsid').value = st.apSsid;
  }
}

const setOffline = () => {
  S.online = false;
  if (!S.streaming){
    $('statePill').className = 'pill off';
    $('stateTxt').textContent = 'Device offline';
  }
}

const poll = async () => {
  var st = await api('/status');
  if (st && st.ok !== false && st.fw){ setOnline(st); } else { setOffline(); }
}

/* ---------- ui wiring ---------- */
const flashIcon = () => {
  return '<svg viewBox="0 0 24 24"><path d="M13 2 4.5 13H11l-1 9 8.5-11H12l1-9z"/></svg>';
}

document.querySelectorAll('.tab').forEach((t) => {
  t.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach((x) => { x.classList.remove('active'); });
    document.querySelectorAll('.pane').forEach((x) => { x.classList.remove('active'); });
    t.classList.add('active');
    $('pane-' + t.dataset.pane).classList.add('active');
  });
});

$('btnStart').addEventListener('click', startStream);
$('btnStop').addEventListener('click', () => { stopStream(false); });

$('btnSnap').addEventListener('click', () => {
  var a = document.createElement('a');
  a.href = '/capture?ts=' + Date.now();
  a.download = 'surya-cam-' + stamp() + '.jpg';
  document.body.appendChild(a);
  a.click();
  a.remove();
  toast('Snapshot saved', 'ok');
});

$('btnFull').addEventListener('click', () => {
  var f = $('frame');
  if (document.fullscreenElement){ document.exitFullscreen(); return; }
  if (f.requestFullscreen) f.requestFullscreen();
  else if (f.webkitRequestFullscreen) f.webkitRequestFullscreen();
});

$('btnFlash').addEventListener('click', () => {
  S.led = S.led > 0 ? 0 : 255;
  $('sLed').value = S.led;
  $('vLed').textContent = S.led;
  $('btnFlash').innerHTML = flashIcon() + ' Flash: ' + (S.led ? 'On' : 'Off');
  setVar('led', S.led, true);
  toast('Flash ' + (S.led ? 'on' : 'off'), S.led ? 'ok' : '');
});

var ledTimer = null;
$('sLed').addEventListener('input', (e) => {
  S.led = parseInt(e.target.value, 10);
  $('vLed').textContent = S.led;
  $('btnFlash').innerHTML = flashIcon() + ' Flash: ' + (S.led ? 'On' : 'Off');
  clearTimeout(ledTimer);
  ledTimer = setTimeout(() => { setVar('led', S.led, true); }, 180);
});

[$('sFps'), $('sQual'), $('sGain')].forEach((el) => {
  el.addEventListener('input', () => { 
    if (el === $('sFps')) $('vFps').textContent = el.value;
    if (el === $('sQual')) $('vQual').textContent = el.value;
  });
});

$('sFps').addEventListener('change', (e) => { S.fps = parseInt(e.target.value,10); setVar('fps', S.fps, true); });
$('sQual').addEventListener('change', (e) => { S.quality = parseInt(e.target.value,10); setVar('quality', S.quality, true); });
$('sGain').addEventListener('change', (e) => { setVar('agc_gain', e.target.value, true); });

$('selRes').addEventListener('change', async (e) => {
  var ok = await setVar('framesize', e.target.value);
  if (ok) toast('Resolution set — restart the stream', 'ok');
});

['vBrightness','vContrast','vSaturation','vEffect'].forEach((id) => {
  $(id).addEventListener('change', (e) => {
    var map = { vBrightness:'brightness', vContrast:'contrast', vSaturation:'saturation', vEffect:'special_effect' };
    setVar(map[id], e.target.value);
  });
});

var swMap = {
  swHmirror:  { v:'hmirror',   d:0 },
  swVflip:    { v:'vflip',     d:0 },
  swWb:       { v:'wb',        d:1 },
  swExposure: { v:'exposure',  d:1 },
  swGain:     { v:'gain_ctrl', d:1 },
  swAutoFlash:{ v:'autoflash', d:0 }
};
Object.keys(swMap).forEach((id) => {
  $(id).addEventListener('change', (e) => {
    setVar(swMap[id].v, e.target.checked ? 1 : 0, true);
  });
});

$('btnCamDefault').addEventListener('click', async () => {
  await setVar('camera_defaults', 1, true);
  toast('Camera defaults restored', 'ok');
  S.primed = false;
  setTimeout(poll, 900);
});
$('btnReloadCam').addEventListener('click', () => { S.primed = false; poll(); toast('Values reloaded', 'ok'); });

/* network form */
$('netForm').addEventListener('submit', async (e) => {
  e.preventDefault();
  var body = new URLSearchParams();
  body.set('ssid', $('fSsid').value.trim());
  body.set('pass', $('fPass').value);
  body.set('host', $('fHost').value.trim());
  body.set('apssid', $('fApSsid').value.trim());
  body.set('appass', $('fApPass').value);
  var r = await api('/config', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: body.toString()
  });
  if (r && r.ok){
    toast('Saved — connecting to ' + ($('fSsid').value.trim() || 'AP only'), 'ok');
    $('fPass').value = ''; $('fApPass').value = '';
    setTimeout(poll, 2500);
    setTimeout(poll, 7000);
  } else {
    toast('Save failed — device unreachable', 'err');
  }
});
$('btnReconnect').addEventListener('click', async () => {
  var r = await api('/config?action=reconnect', { method:'POST' });
  toast(r && r.ok ? 'Reconnecting Wi-Fi…' : 'Failed', r && r.ok ? 'ok' : 'err');
});
$('btnForget').addEventListener('click', async () => {
  if (!confirm('Forget saved Wi-Fi credentials? The device keeps running in AP mode.')) return;
  var r = await api('/config?action=forget', { method:'POST' });
  toast(r && r.ok ? 'Credentials cleared' : 'Failed', r && r.ok ? 'ok' : 'err');
  if (r && r.ok) setTimeout(poll, 2500);
});

/* system */
$('btnCopy').addEventListener('click', () => {
  var t = $('sStreamUrl').textContent;
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(t).then(() => { toast('Stream URL copied', 'ok'); },
      () => { toast(t, 'warn'); });
  } else { toast(t, 'warn'); }
});
$('btnOpenCap').addEventListener('click', () => { window.open('/capture?ts=' + Date.now(), '_blank'); });
$('btnRefresh').addEventListener('click', () => { poll(); toast('Status refreshed', 'ok'); });
$('btnReboot').addEventListener('click', async () => {
  if (!confirm('Reboot the ESP32-CAM now?')) return;
  toast('Rebooting…', 'warn');
  await api('/reboot');
  S.primed = false;
  setTimeout(poll, 9000);
});
$('btnFactory').addEventListener('click', async () => {
  if (!confirm('Factory reset erases Wi-Fi settings and camera preferences. Continue?')) return;
  toast('Factory reset — rebooting…', 'warn');
  await api('/factory');
  S.primed = false;
  setTimeout(poll, 12000);
});

/* lifecycle */
window.addEventListener('beforeunload', () => {
  var img = $('stream');
  if (img) { img.removeAttribute('src'); }
});
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) poll();
});

/* boot */
(() => {
  var def = [
    { v:5, n:'QVGA 320x240' }, { v:6, n:'CIF 400x296' }, { v:7, n:'HVGA 480x320' },
    { v:8, n:'VGA 640x480' },  { v:9, n:'SVGA 800x600' }
  ];
  buildResList(def, 5);
  poll();
  setInterval(() => { if (!document.hidden) poll(); }, 5000);
})();
</script>
</body>
</html>
)SURYAUI";

/* The one-liner the rest of the sketch calls to serve the dashboard. It is
   declared near the top (forward declaration) and defined here because the
   string it points to lives at the bottom of the file. */
static const char* uiHtml(){ return INDEX_HTML; }
