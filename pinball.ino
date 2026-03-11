/**
 * Pinball Bot Firmware
 *
 * ---------------------------------
 * v0.6.2
 * ---------------------------------
 * GPIO pins changed to 2,3 from 4,5 for easier wiring/mounting
 *
 * ---------------------------------
 * v0.6.1
 * ---------------------------------
 * OTA OFF by default
 *
 * ---------------------------------
 * v0.6.0
 * ---------------------------------
 *  OTA (A/B partitions):
 *   1) ArduinoOTA enabled (TCP port 3232).
 *   2) Requires partition scheme with otadata + ota_0 + ota_1 (A/B).
 *   3) Safety: on OTA start -> AUTO OFF + relays OFF (same as /kill).
 *   4) /ota endpoint: enable/disable OTA at runtime (optional safety gate).
 *   5) /otaInfo: show running partition + next update partition.
 *
 *  NOTE:
 *   - ArduinoOTA provides convenience + basic auth (password).
 *   - NOT strong security by itself. See notes near OTA config.
 *
 * ---------------------------------
 * v0.5.3
 * ---------------------------------
 *  1) /roi UI improvements:
 *     - Larger display via zoom (2x/3x/4x/5x) with correct coordinate mapping.
 *     - Overlay all defined zones with distinct colors + labels.
 *     - Selecting a zone loads its current side/en/tap/roi fields.
 *  2) Persist config across reset/power cycle:
 *     - Zones + Motion knobs + Auto state stored in NVS (Preferences).
 *     - Save on /zonesSet, /zonesClear, /motion, /auto, /kill.
 *  3) Added /cfgReset to wipe stored config and reboot.
 *
 * ---------------------------------
 * v0.5.2
 * ---------------------------------
 *  - Runtime-configurable motion processing knobs and presets (mode 0/1/2)
 *  - /motion endpoint to set mode and individual knobs
 *  - All key vision “tweaks” are togglable: cooldown, bbox, down requirement, direction heuristic, gate
 *  - “Down requirement” is now independent from direction heuristic
 *  - Added per-second delta counters (dframes) and “pre_gate” counters to diagnose where triggers are lost
 *  - Zone0 default ON with full-frame ROI (0,0,160,120)
 *  - GI change reject configurable (default OFF)
 *
 * ---------------------------------
 * v0.5.1
 * ---------------------------------
 * - ROI setting web GUI
 *
 * ---------------------------------
 * v0.5.0
 * ---------------------------------
 * 1) Camera orientation fix:
 *    - Apply vertical flip so the playfield matches program coordinates.
 * 2) Auto tuning then lock:
 *    - Start with Auto Exposure / Auto Gain / Auto White Balance enabled.
 *    - After warmup + stability, lock the tuned values (disable auto) to keep diff/ROI stable.
 *
 * ---------------------------------
 * v0.4 (Major Architecture Update)
 * ---------------------------------
 * - Added background vision task (FreeRTOS) pinned to Core 1.
 * - Implemented robust single-camera-owner design:
 *     * Only vision task calls esp_camera_fb_get()/return().
 *     * Frames copied immediately into shared latest_frame buffer.
 *     * HTTP endpoints no longer access camera directly.
 * - Added ROI-based motion zones with automatic flipper tap triggering.
 * - Added /auto endpoint to enable/disable automatic flipper control.
 * - Implemented non-blocking relay scheduling (removed delay-based tap/hold).
 * - Added cooldown gating to prevent flipper feedback loops.
 * - Added global-change reject and ROI mean normalization to reduce false triggers.
 * - Improved stability against cam_hal FB-OVF errors.
 * - Version string centralized via FW_VERSION.
 * - Web UI (/): add one-click example buttons/links for each GET endpoint
 *
 * ---------------------------------
 * v0.3
 * ---------------------------------
 * 1. /diff: replace avg-diff-only with thresholded motion mask stats (changed pixels, ratio, centroid, bbox)
 * 2. /diff: add ROI mean normalization to reduce false triggers from GI/score brightness changes
 * 3. /diff: add global-change reject (ratio too high) to ignore lighting flicker/camera shake/flipper sweeps
 * 4. /diff: add cooldown gating after relay actuation to prevent flipper-motion infinite loops
 * 5. /diff: return richer JSON for tuning (thresholds, mean_delta, bbox, motion_ok)
 *
 * ---------------------------------
 * v0.2
 * ---------------------------------
 * 1. support 2 relays: left GPIO4, right GPIO5
 * 2. add reusable tap/hold function (for future CV trigger)
 * 3. new HTTP endpoints for left/right tap + hold
 * 4. pull-up relay ctrl GPIO pins for active LOW
 *
 * ---------------------------------
 * v0.1
 * ---------------------------------
 * 1. better implementation of LED showing relay status (blinks faster when relay is ON)
 * 2. web query of relay (without ?on=0 or 1) will not reset relay status (to the default 0)
 * 3. changed relay pin to GPIO #4 (avoid #1 used by serial)
 * 4. tested on physical Songle relay, works
 *
 * ---------------------------------
 * v0
 * ---------------------------------
 * 1. basic functions:
 *      WiFi, web server, status LED, image capture, diff/ROI calculation, mock relay
 * 2. allocation of functions
 *      events -> core-0, Arduino -> core-1
 */

#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <Preferences.h>

// ---- v0.6.0 OTA additions ----
#include <ArduinoOTA.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
// ------------------------------

#include "pinball_types.h"
#include "camera_pins_xiao_esp32s3_sense.h"

// ---------------------------
// Firmware version
// ---------------------------
static const char* FW_VERSION = "v0.6.2";

// ---------------------------
// OTA (v0.6.0)
// ---------------------------
/*
 * ArduinoOTA notes:
 * - Default ArduinoOTA port is 3232. Explicitly set it anyway.
 * - This is "LAN OTA" style. On SoftAP it is reachable from clients on user's AP.
 * - Security: password is a shared secret; traffic is not encrypted. Consider keeping OTA disabled
 * - OFF by default and enabled only when needed (/ota?on=1&mins=10).
 */
static const uint16_t OTA_PORT = 3232;
static const char* OTA_HOSTNAME = "myesp32";
static const char* OTA_PASSWORD = "********"; // TODO: change

// Optional safety gate: OTA can be disabled at runtime.
static volatile bool g_ota_enabled = false;
// Optional timed window: if enabled via /ota?on=1&mins=N, auto-disables after N minutes.
static uint32_t g_ota_disable_at_ms = 0;

// Forward declarations for OTA endpoints
static void handle_ota();
static void handle_otaInfo();
static void ota_setup();
static void ota_handle_loop();
static void ota_force_safe_state(const char* why);

// ---------------------------
// Board GPIOs
// ---------------------------
static const int LED_GPIO = 21;
static const bool LED_ACTIVE_LOW = true;

// Relay outputs
static const int RELAY_L_GPIO = 2;
static const int RELAY_R_GPIO = 3;
static const int RELAY_ON_LEVEL = LOW;

// Defaults
static const uint32_t DEFAULT_TAP_MS  = 80;
static const uint32_t DEFAULT_HOLD_MS = 300;

// ---------------------------
// State (single source of truth)
// ---------------------------
static volatile bool relay_l_state = false;
static volatile bool relay_r_state = false;

// LED heartbeat
static unsigned long lastBlink = 0;
static bool ledState = false;
static bool lastAnyRelayStateForBlink = false;

// ---------------------------
// Wi-Fi (SoftAP)
// ---------------------------
static const char* AP_SSID = "myesp32";
static const char* AP_PASS = "********";  // TODO: change

WebServer server(80);

// ---------------------------
// Previous-frame storage for diff engine (owned by vision task)
// ---------------------------
static uint8_t* prev_frame = nullptr;
static size_t   prev_len   = 0;
static int      prev_w     = 0;
static int      prev_h     = 0;

// Latest frame snapshot for /frame.pgm and ROI tool
static uint8_t*  latest_frame = nullptr;
static size_t    latest_len   = 0;
static int       latest_w     = 0;
static int       latest_h     = 0;
static uint32_t  latest_frame_ms = 0;

static portMUX_TYPE g_frame_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_stats_mux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------
// Persistence (NVS Preferences)
// ---------------------------
static Preferences g_prefs;
static const char* PREF_NS = "pinball";
static const uint32_t CFG_MAGIC = 0x50424F54; // 'PBOT'
static const uint16_t CFG_VER   = 1;

struct PersistHeader {
  uint32_t magic = CFG_MAGIC;
  uint16_t ver   = CFG_VER;
  uint16_t rsvd  = 0;
};

struct ZonePersist {
  uint8_t  enabled = 0;
  uint8_t  side = 0;        // 0=L 1=R
  uint16_t tap_ms = 120;
  int16_t  x = 0;
  int16_t  y = 0;
  int16_t  w = 0;
  int16_t  h = 0;
};

struct MotionPersist {
  int32_t mode;

  uint8_t  pix_thr;
  uint32_t min_changed;
  uint32_t max_changed;
  float    global_ratio_reject;

  uint8_t  gi_reject_en;
  int32_t  gi_reject_abs_mean_thr;

  uint8_t  bbox_en;
  int32_t  bbox_max_w;
  int32_t  bbox_max_h;

  uint8_t  down_required;

  uint8_t  dir_en;
  int32_t  dir_mode;

  uint8_t  cooldown_en;
  uint32_t cooldown_ms;

  uint8_t  gate_en;
  uint32_t gate_ms;
};

static bool prefs_write_blob(const char* key, const void* data, size_t len) {
  size_t n = g_prefs.putBytes(key, data, len);
  return (n == len);
}

static bool prefs_read_blob(const char* key, void* data, size_t len) {
  size_t n = g_prefs.getBytes(key, data, len);
  return (n == len);
}

// ---------------------------
// Helpers
// ---------------------------
static inline uint8_t absdiff_u8(uint8_t a, uint8_t b) { return (a > b) ? (a - b) : (b - a); }

static void clamp_roi(ROI& r, int w, int h) {
  if (w <= 0 || h <= 0) { r = ROI{0,0,0,0}; return; }
  if (r.w < 0) r.w = 0;
  if (r.h < 0) r.h = 0;
  if (r.x < 0) { r.w += r.x; r.x = 0; }
  if (r.y < 0) { r.h += r.y; r.y = 0; }
  if (r.x > w) r.x = w;
  if (r.y > h) r.y = h;
  if (r.x + r.w > w) r.w = w - r.x;
  if (r.y + r.h > h) r.h = h - r.y;
  if (r.w < 0) r.w = 0;
  if (r.h < 0) r.h = 0;
}

static int parseIntArgOrDefault(const char* name, int def) {
  if (!server.hasArg(name)) return def;
  return server.arg(name).toInt();
}

static uint32_t parseMsArgOrDefault(const char* name, uint32_t def, uint32_t minv, uint32_t maxv) {
  if (!server.hasArg(name)) return def;
  long v = server.arg(name).toInt();
  if (v < (long)minv) v = (long)minv;
  if (v > (long)maxv) v = (long)maxv;
  return (uint32_t)v;
}

// Strictly accept only "0" or "1" if present
static bool parseOnArgStrict(bool* out_on) {
  if (!server.hasArg("on")) return false;
  String s = server.arg("on");
  if (s == "0") { *out_on = false; return true; }
  if (s == "1") { *out_on = true;  return true; }
  return false;
}

// Parse ROI from query parameter "roi" formatted as "x,y,w,h"
static ROI parse_roi() {
  ROI r;
  if (!server.hasArg("roi")) return r;
  String s = server.arg("roi");
  int vals[4] = {0,0,0,0};
  int vi = 0;
  int last = 0;
  for (int i = 0; i <= (int)s.length() && vi < 4; i++) {
    if (i == (int)s.length() || s[i] == ',' || s[i] == ' ' || s[i] == ';') {
      String part = s.substring(last, i);
      part.trim();
      vals[vi++] = part.toInt();
      last = i + 1;
    }
  }
  r.x = vals[0]; r.y = vals[1]; r.w = vals[2]; r.h = vals[3];
  return r;
}

// ---------------------------
// LED + Relay helpers
// ---------------------------
static void writeLed(bool on) {
  if (LED_ACTIVE_LOW) digitalWrite(LED_GPIO, on ? LOW : HIGH);
  else                digitalWrite(LED_GPIO, on ? HIGH : LOW);
}

static inline bool anyRelayOn() { return relay_l_state || relay_r_state; }

// Global “vision cooldown” (used to avoid flipper self-trigger). Now configurable.
static volatile uint32_t last_actuation_ms = 0;

static void applyRelayPin(int gpio, volatile bool &stateVar, bool on, const char* why) {
  (void)why;
  stateVar = on;
  digitalWrite(gpio, on ? RELAY_ON_LEVEL : !RELAY_ON_LEVEL);
  if (on) last_actuation_ms = millis();
}

static void setLeftRelay(bool on, const char* why)  { applyRelayPin(RELAY_L_GPIO, relay_l_state, on, why); }
static void setRightRelay(bool on, const char* why) { applyRelayPin(RELAY_R_GPIO, relay_r_state, on, why); }

// ---------------------------
// Relay scheduler (non-blocking)
// ---------------------------
struct RelayAction {
  bool    is_on = false;
  bool    hold = false;
  bool    pulse_active = false;
  uint32_t off_at_ms = 0;
};

static RelayAction relay_action[2];

static void relay_hw_set(FlipperSide side, bool on, const char* why) {
  if (side == FLIPPER_LEFT)  setLeftRelay(on, why);
  else                       setRightRelay(on, why);
  relay_action[(int)side].is_on = on;
  if (on) last_actuation_ms = millis();
}

static void flipper_release(FlipperSide side, const char* why) {
  RelayAction &a = relay_action[(int)side];
  a.hold = false;
  a.pulse_active = false;
  a.off_at_ms = 0;
  relay_hw_set(side, false, why);
}

static void flipper_tap_ms(FlipperSide side, uint32_t tap_ms) {
  if (tap_ms < 1) tap_ms = 1;
  if (tap_ms > 2000) tap_ms = 2000;

  RelayAction &a = relay_action[(int)side];
  a.hold = false;
  a.pulse_active = true;
  a.off_at_ms = millis() + tap_ms;
  relay_hw_set(side, true, "tap");
}

static void flipper_hold_ms(FlipperSide side, uint32_t hold_ms) {
  if (hold_ms < 1) hold_ms = 1;
  if (hold_ms > 30000) hold_ms = 30000;

  RelayAction &a = relay_action[(int)side];
  a.hold = false;
  a.pulse_active = true;
  a.off_at_ms = millis() + hold_ms;
  relay_hw_set(side, true, "hold_ms");
}

static void flipper_tap_hold_ms(FlipperSide side, uint32_t tap_ms, uint32_t hold_ms) {
  (void)tap_ms;
  flipper_hold_ms(side, hold_ms);
}

static void relay_actions_update() {
  uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    RelayAction &a = relay_action[i];
    if (a.hold) continue;
    if (a.pulse_active && a.is_on) {
      if ((int32_t)(now - a.off_at_ms) >= 0) {
        a.pulse_active = false;
        a.off_at_ms = 0;
        relay_hw_set((FlipperSide)i, false, "sched_off");
      }
    }
  }
}

// ---------------------------
// Zones + stats
// ---------------------------
static const int MAX_ZONES = 4;
static Zone g_zones[MAX_ZONES];
static volatile bool g_auto_enabled = false;

struct ZoneStats {
  bool have_prev = false;
  bool armed = false;

  // Raw motion measures
  uint32_t changed = 0;
  uint32_t sum_changed = 0;
  float ratio = 0.0f;
  int cx = -1, cy = -1;
  int mean_delta = 0;

  // bbox
  int minx = 0, miny = 0, maxx = -1, maxy = -1;
  int bw = 0, bh = 0;

  // direction / down heuristic
  bool down = false;

  // Check outcomes
  bool size_ok = false;
  bool global_ok = false;
  bool bbox_ok = false;
  bool down_ok = false;     // independent down requirement
  bool dir_ok = false;      // independent direction heuristic
  bool cooldown_ok = false;
  bool pre_gate_ok = false; // “would trigger” before per-zone gate
  bool motion_ok = false;   // after gate + auto

  // Gating
  uint32_t last_trigger_ms = 0;
  uint32_t trigger_count = 0;

  // Debug counters (ever)
  uint32_t ok_count = 0;        // count of motion_ok evaluations that passed (post-gate)
  uint32_t pre_gate_count = 0;  // count of pre_gate_ok (passed checks, before gate)
};

static ZoneStats g_zone_stats[MAX_ZONES];
static int g_last_cy[MAX_ZONES];
static int g_last_cy2[MAX_ZONES];

static TaskHandle_t g_vision_task = nullptr;

// ---------------------------
// Camera warmup/lock
// ---------------------------
static volatile bool g_cam_locked = false;
static uint32_t g_cam_warmup_start_ms = 0;

static const uint32_t CAM_WARMUP_MIN_MS = 1500;
static const int CAM_STABLE_FRAMES_REQUIRED = 12;
static const int CAM_MEAN_STABLE_EPS = 2;

static void apply_camera_orientation() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_vflip(s, 1);
  // s->set_hmirror(s, 1); // do not apply by default
}

static void camera_enable_auto() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;

  s->set_gain_ctrl(s, 1);      // AGC
  s->set_exposure_ctrl(s, 1);  // AEC
  s->set_aec2(s, 1);           // AEC2
  s->set_whitebal(s, 1);       // AWB
  s->set_awb_gain(s, 1);       // AWB gain

  s->set_special_effect(s, 0);
  s->set_wb_mode(s, 0);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  s->set_gainceiling(s, GAINCEILING_16X);
}

static void camera_lock_current_params() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;

  int agc_gain  = s->status.agc_gain;
  int aec_value = s->status.aec_value;
  int wb_mode   = s->status.wb_mode;

  s->set_gain_ctrl(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  s->set_aec2(s, 0);

  s->set_wb_mode(s, wb_mode);
  s->set_agc_gain(s, agc_gain);
  s->set_aec_value(s, aec_value);

  s->set_special_effect(s, 0);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  g_cam_locked = true;
  Serial.printf("Camera locked: agc_gain=%d aec_value=%d wb_mode=%d\n", agc_gain, aec_value, wb_mode);
}

// ---------------------------
// Motion config
// ---------------------------
struct MotionCfg {
  int mode = 0;

  uint8_t  pix_thr = 12;
  uint32_t min_changed = 40;
  uint32_t max_changed = 1200;
  float    global_ratio_reject = 0.25f;

  bool gi_reject_en = false;
  int  gi_reject_abs_mean_thr = 30;

  bool bbox_en = true;
  int  bbox_max_w = 70;
  int  bbox_max_h = 70;

  bool down_required = true;

  bool dir_en = false;
  int  dir_mode = 0;

  bool cooldown_en = true;
  uint32_t cooldown_ms = 350;

  bool gate_en = true;
  uint32_t gate_ms = 250;
};

static MotionCfg g_motion;

static void motion_apply_mode(int mode) {
  if (mode < 0) mode = 0;
  if (mode > 2) mode = 2;
  g_motion.mode = mode;

  if (mode == 0) {
    g_motion.pix_thr = 12;
    g_motion.min_changed = 40;
    g_motion.max_changed = 1200;
    g_motion.global_ratio_reject = 0.25f;

    g_motion.bbox_en = true;
    g_motion.bbox_max_w = 70;
    g_motion.bbox_max_h = 70;

    g_motion.down_required = true;

    g_motion.dir_en = false;
    g_motion.dir_mode = 0;

    g_motion.cooldown_en = true;
    g_motion.cooldown_ms = 350;

    g_motion.gate_en = true;
    g_motion.gate_ms = 250;

    g_motion.gi_reject_en = false;
    g_motion.gi_reject_abs_mean_thr = 30;
  } else if (mode == 1) {
    g_motion.pix_thr = 12;
    g_motion.min_changed = 20;
    g_motion.max_changed = 5000;
    g_motion.global_ratio_reject = 0.40f;

    g_motion.bbox_en = false;

    g_motion.down_required = false;

    g_motion.dir_en = false;
    g_motion.dir_mode = 0;

    g_motion.cooldown_en = true;
    g_motion.cooldown_ms = 350;

    g_motion.gate_en = true;
    g_motion.gate_ms = 250;

    g_motion.gi_reject_en = false;
    g_motion.gi_reject_abs_mean_thr = 30;
  } else {
    g_motion.pix_thr = 12;
    g_motion.min_changed = 20;
    g_motion.max_changed = 20000;
    g_motion.global_ratio_reject = 0.95f;

    g_motion.bbox_en = false;

    g_motion.down_required = false;

    g_motion.dir_en = false;
    g_motion.dir_mode = 0;

    g_motion.cooldown_en = true;
    g_motion.cooldown_ms = 350;

    g_motion.gate_en = true;
    g_motion.gate_ms = 250;

    g_motion.gi_reject_en = false;
    g_motion.gi_reject_abs_mean_thr = 30;
  }
}

// ---------------------------
// Global stats + dframes deltas
// ---------------------------
struct GlobalStats {
  uint32_t frames = 0;
  uint32_t have_prev = 0;

  uint32_t gi_rejected = 0;
  uint32_t cooldown_blocked = 0;

  int last_frame_mean_delta = 0;

  uint32_t relay_taps = 0;
  uint32_t relay_holds = 0;
  uint32_t relay_releases = 0;
};

static GlobalStats g_stats;
static GlobalStats g_stats_last;
static ZoneStats   g_zone_last[MAX_ZONES];

// ---------------------------
// Persistence: save/load helpers
// ---------------------------
static void persist_save_all();
static void persist_load_all();

static void persist_save_header() {
  PersistHeader hdr;
  prefs_write_blob("hdr", &hdr, sizeof(hdr));
}

static bool persist_valid_header() {
  PersistHeader hdr;
  if (!prefs_read_blob("hdr", &hdr, sizeof(hdr))) return false;
  return (hdr.magic == CFG_MAGIC && hdr.ver == CFG_VER);
}

static void persist_save_zones() {
  for (int i = 0; i < MAX_ZONES; i++) {
    ZonePersist zp;
    Zone z = g_zones[i];
    zp.enabled = z.enabled ? 1 : 0;
    zp.side = (z.side == FLIPPER_RIGHT) ? 1 : 0;
    zp.tap_ms = (uint16_t)constrain((int)z.tap_ms, 10, 2000);
    zp.x = (int16_t)z.roi.x;
    zp.y = (int16_t)z.roi.y;
    zp.w = (int16_t)z.roi.w;
    zp.h = (int16_t)z.roi.h;

    char key[8];
    snprintf(key, sizeof(key), "z%d", i);
    prefs_write_blob(key, &zp, sizeof(zp));
  }
}

static void persist_load_zones() {
  for (int i = 0; i < MAX_ZONES; i++) {
    char key[8];
    snprintf(key, sizeof(key), "z%d", i);

    ZonePersist zp;
    if (!prefs_read_blob(key, &zp, sizeof(zp))) continue;

    Zone z = g_zones[i];
    z.enabled = (zp.enabled != 0);
    z.side = (zp.side != 0) ? FLIPPER_RIGHT : FLIPPER_LEFT;
    z.tap_ms = (uint32_t)zp.tap_ms;
    z.roi = ROI{(int)zp.x, (int)zp.y, (int)zp.w, (int)zp.h};
    g_zones[i] = z;
  }
}

static void persist_save_motion() {
  MotionPersist mp{};
  mp.mode = g_motion.mode;

  mp.pix_thr = g_motion.pix_thr;
  mp.min_changed = g_motion.min_changed;
  mp.max_changed = g_motion.max_changed;
  mp.global_ratio_reject = g_motion.global_ratio_reject;

  mp.gi_reject_en = g_motion.gi_reject_en ? 1 : 0;
  mp.gi_reject_abs_mean_thr = g_motion.gi_reject_abs_mean_thr;

  mp.bbox_en = g_motion.bbox_en ? 1 : 0;
  mp.bbox_max_w = g_motion.bbox_max_w;
  mp.bbox_max_h = g_motion.bbox_max_h;

  mp.down_required = g_motion.down_required ? 1 : 0;

  mp.dir_en = g_motion.dir_en ? 1 : 0;
  mp.dir_mode = g_motion.dir_mode;

  mp.cooldown_en = g_motion.cooldown_en ? 1 : 0;
  mp.cooldown_ms = g_motion.cooldown_ms;

  mp.gate_en = g_motion.gate_en ? 1 : 0;
  mp.gate_ms = g_motion.gate_ms;

  prefs_write_blob("motion", &mp, sizeof(mp));
}

static void persist_load_motion() {
  MotionPersist mp{};
  if (!prefs_read_blob("motion", &mp, sizeof(mp))) return;

  // Start from preset, then overwrite so fields stay coherent
  motion_apply_mode((int)mp.mode);

  g_motion.pix_thr = mp.pix_thr;
  g_motion.min_changed = mp.min_changed;
  g_motion.max_changed = mp.max_changed;
  g_motion.global_ratio_reject = mp.global_ratio_reject;

  g_motion.gi_reject_en = (mp.gi_reject_en != 0);
  g_motion.gi_reject_abs_mean_thr = (int)mp.gi_reject_abs_mean_thr;

  g_motion.bbox_en = (mp.bbox_en != 0);
  g_motion.bbox_max_w = (int)mp.bbox_max_w;
  g_motion.bbox_max_h = (int)mp.bbox_max_h;

  g_motion.down_required = (mp.down_required != 0);

  g_motion.dir_en = (mp.dir_en != 0);
  g_motion.dir_mode = (int)mp.dir_mode;

  g_motion.cooldown_en = (mp.cooldown_en != 0);
  g_motion.cooldown_ms = mp.cooldown_ms;

  g_motion.gate_en = (mp.gate_en != 0);
  g_motion.gate_ms = mp.gate_ms;
}

static void persist_save_auto() {
  g_prefs.putUChar("auto", g_auto_enabled ? 1 : 0);
}

static void persist_load_auto() {
  if (!g_prefs.isKey("auto")) return;
  uint8_t v = g_prefs.getUChar("auto", 0);
  g_auto_enabled = (v != 0);
}

static void persist_save_all() {
  if (!g_prefs.begin(PREF_NS, false)) return;
  persist_save_header();
  persist_save_zones();
  persist_save_motion();
  persist_save_auto();
  g_prefs.end();
}

static void persist_load_all() {
  if (!g_prefs.begin(PREF_NS, true)) return;
  bool ok = persist_valid_header();
  if (ok) {
    persist_load_zones();
    persist_load_motion();
    persist_load_auto();
  }
  g_prefs.end();
}

static void persist_clear_all_and_reboot() {
  if (g_prefs.begin(PREF_NS, false)) {
    g_prefs.clear();
    g_prefs.end();
  }
  delay(100);
  ESP.restart();
}

// ---------------------------
// v0.5.2: Vision task
// ---------------------------
static void vision_task(void* arg) {
  (void)arg;

  for (int i = 0; i < MAX_ZONES; i++) { g_last_cy[i] = -1; g_last_cy2[i] = -1; }

  const TickType_t period = pdMS_TO_TICKS(50); // ~20Hz schedule
  TickType_t last_wake = xTaskGetTickCount();

  int stable_count = 0;
  int last_mean = -1;

  while (true) {
    vTaskDelayUntil(&last_wake, period);

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;
    if (fb->format != PIXFORMAT_GRAYSCALE) { esp_camera_fb_return(fb); continue; }

    bool have_prev = (prev_frame && prev_len == fb->len && prev_w == fb->width && prev_h == fb->height);
    if (!prev_frame || prev_len != fb->len) {
      free(prev_frame);
      prev_frame = (uint8_t*)malloc(fb->len);
      prev_len = fb->len;
      if (!prev_frame) { esp_camera_fb_return(fb); continue; }
      memcpy(prev_frame, fb->buf, fb->len);
      prev_w = fb->width;
      prev_h = fb->height;
      have_prev = true;
    }

    if (!latest_frame || latest_len != fb->len) {
      portENTER_CRITICAL(&g_frame_mux);
      free(latest_frame);
      latest_frame = (uint8_t*)malloc(fb->len);
      latest_len = fb->len;
      latest_w = fb->width;
      latest_h = fb->height;
      portEXIT_CRITICAL(&g_frame_mux);
      if (!latest_frame) { esp_camera_fb_return(fb); continue; }
    }

    portENTER_CRITICAL(&g_frame_mux);
    memcpy(latest_frame, fb->buf, fb->len);
    latest_w = fb->width;
    latest_h = fb->height;
    latest_frame_ms = millis();
    portEXIT_CRITICAL(&g_frame_mux);

    esp_camera_fb_return(fb);

    const uint8_t* curr_buf = latest_frame;
    const size_t   curr_len = latest_len;
    const int      W = latest_w;
    const int      H = latest_h;

    uint32_t now_ms = millis();

    portENTER_CRITICAL(&g_stats_mux);
    g_stats.frames++;
    if (have_prev) g_stats.have_prev++;
    portEXIT_CRITICAL(&g_stats_mux);

    if (!g_cam_locked) {
      if ((uint32_t)(now_ms - g_cam_warmup_start_ms) >= CAM_WARMUP_MIN_MS) {
        uint32_t sum = 0;
        for (size_t i = 0; i < curr_len; i++) sum += curr_buf[i];
        int mean = (curr_len > 0) ? (int)(sum / curr_len) : 0;

        if (last_mean >= 0 && abs(mean - last_mean) <= CAM_MEAN_STABLE_EPS) stable_count++;
        else stable_count = 0;

        last_mean = mean;

        if (stable_count >= CAM_STABLE_FRAMES_REQUIRED) camera_lock_current_params();
      } else {
        stable_count = 0;
        last_mean = -1;
      }
    }

    int frame_mean_delta = 0;
    if (have_prev && curr_len > 0) {
      uint32_t sumc = 0, sump = 0;
      for (size_t i = 0; i < curr_len; i++) { sumc += curr_buf[i]; sump += prev_frame[i]; }
      int meanc = (int)(sumc / curr_len);
      int meanp = (int)(sump / curr_len);
      frame_mean_delta = meanp - meanc; // positive => frame got darker
    }

    bool gi_rejected = false;
    if (g_motion.gi_reject_en && have_prev) {
      if (abs(frame_mean_delta) >= g_motion.gi_reject_abs_mean_thr) gi_rejected = true;
    }

    portENTER_CRITICAL(&g_stats_mux);
    g_stats.last_frame_mean_delta = frame_mean_delta;
    if (gi_rejected) g_stats.gi_rejected++;
    portEXIT_CRITICAL(&g_stats_mux);

    bool cooldown_ok = true;
    if (g_motion.cooldown_en) {
      cooldown_ok = ((uint32_t)(now_ms - (uint32_t)last_actuation_ms) > g_motion.cooldown_ms);
      if (!cooldown_ok) {
        portENTER_CRITICAL(&g_stats_mux);
        g_stats.cooldown_blocked++;
        portEXIT_CRITICAL(&g_stats_mux);
      }
    }

    for (int zi = 0; zi < MAX_ZONES; zi++) {
      Zone z = g_zones[zi];

      ZoneStats out;
      out.have_prev = have_prev;
      out.armed = cooldown_ok && !gi_rejected;

      out.minx = 9999; out.miny = 9999; out.maxx = -1; out.maxy = -1;

      ROI roi = z.roi;
      clamp_roi(roi, W, H);
      uint32_t roi_pixels = (uint32_t)roi.w * (uint32_t)roi.h;

      ZoneStats prevS;
      portENTER_CRITICAL(&g_stats_mux);
      prevS = g_zone_stats[zi];
      portEXIT_CRITICAL(&g_stats_mux);

      out.last_trigger_ms = prevS.last_trigger_ms;
      out.trigger_count   = prevS.trigger_count;
      out.ok_count        = prevS.ok_count;
      out.pre_gate_count  = prevS.pre_gate_count;

      if (z.enabled && have_prev && roi_pixels > 0) {
        uint32_t sum_curr = 0, sum_prev = 0;
        for (int yy = roi.y; yy < roi.y + roi.h; yy++) {
          int row = yy * W;
          for (int xx = roi.x; xx < roi.x + roi.w; xx++) {
            size_t idx = (size_t)row + (size_t)xx;
            sum_curr += curr_buf[idx];
            sum_prev += prev_frame[idx];
          }
        }
        int delta = (int)(sum_prev / roi_pixels) - (int)(sum_curr / roi_pixels);
        out.mean_delta = delta;

        int64_t sx = 0, sy = 0;

        for (int yy = roi.y; yy < roi.y + roi.h; yy++) {
          int row = yy * W;
          for (int xx = roi.x; xx < roi.x + roi.w; xx++) {
            size_t idx = (size_t)row + (size_t)xx;

            int curr = (int)curr_buf[idx] + delta;
            if (curr < 0) curr = 0;
            if (curr > 255) curr = 255;

            uint8_t d = absdiff_u8((uint8_t)curr, prev_frame[idx]);
            if (d >= g_motion.pix_thr) {
              out.changed++;
              out.sum_changed += d;
              sx += xx; sy += yy;
              if (xx < out.minx) out.minx = xx;
              if (yy < out.miny) out.miny = yy;
              if (xx > out.maxx) out.maxx = xx;
              if (yy > out.maxy) out.maxy = yy;
            }
          }
        }

        out.ratio = (roi_pixels > 0) ? ((float)out.changed / (float)roi_pixels) : 0.0f;
        if (out.changed) {
          out.cx = (int)(sx / (int64_t)out.changed);
          out.cy = (int)(sy / (int64_t)out.changed);
        }

        if (out.cy >= 0 && g_last_cy[zi] >= 0) {
          out.down = (out.cy > g_last_cy[zi] + 1) || (out.cy > g_last_cy2[zi] + 1);
        }

        if (out.maxx >= 0) {
          out.bw = (out.maxx - out.minx + 1);
          out.bh = (out.maxy - out.miny + 1);
        }

        out.size_ok   = (out.changed >= g_motion.min_changed && out.changed <= g_motion.max_changed);
        out.global_ok = (out.ratio <= g_motion.global_ratio_reject);

        if (!g_motion.bbox_en) out.bbox_ok = true;
        else out.bbox_ok = (out.maxx < 0) ? true : (out.bw < g_motion.bbox_max_w && out.bh < g_motion.bbox_max_h);

        out.down_ok = (!g_motion.down_required) || out.down;

        if (!g_motion.dir_en || g_motion.dir_mode == 0) {
          out.dir_ok = true;
        } else if (g_motion.dir_mode == 1) {
          out.dir_ok = out.down;
        } else if (g_motion.dir_mode == 2) {
          out.dir_ok = (out.cy >= 0 && g_last_cy[zi] >= 0) ? (out.cy < g_last_cy[zi] - 1) : false;
        } else {
          out.dir_ok = true;
        }

        out.cooldown_ok = cooldown_ok && !gi_rejected;
        out.pre_gate_ok = out.armed && out.size_ok && out.global_ok && out.bbox_ok && out.down_ok && out.dir_ok;

        if (out.pre_gate_ok) out.pre_gate_count++;

        bool gate_ok = true;
        if (g_motion.gate_en) {
          gate_ok = ((uint32_t)(now_ms - prevS.last_trigger_ms) > g_motion.gate_ms);
        }

        out.motion_ok = out.pre_gate_ok && gate_ok;

        g_last_cy2[zi] = g_last_cy[zi];
        g_last_cy[zi]  = out.cy;

        if (g_auto_enabled && out.motion_ok) {
          flipper_tap_ms(z.side, z.tap_ms);
          out.last_trigger_ms = now_ms;
          out.trigger_count = prevS.trigger_count + 1;
          out.ok_count = prevS.ok_count + 1;

          portENTER_CRITICAL(&g_stats_mux);
          g_stats.relay_taps++;
          portEXIT_CRITICAL(&g_stats_mux);
        }
      }

      portENTER_CRITICAL(&g_stats_mux);
      g_zone_stats[zi] = out;
      portEXIT_CRITICAL(&g_stats_mux);
    }

    memcpy(prev_frame, curr_buf, prev_len);
    prev_w = W;
    prev_h = H;
  }
}

// ---------------------------
// HTTP handlers
// ---------------------------
static void handle_root() {
  String page;
  page.reserve(9000);

  page += "<!doctype html><html><head><meta charset='utf-8'>\n";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>\n";
  page += "<title>Pinball Bot</title>\n";
  page += "<style>\n";
  page += "body{font-family:system-ui,Arial,sans-serif;margin:16px;max-width:980px}\n";
  page += "h1{margin:0 0 8px 0}\n";
  page += "h2{margin:16px 0 8px 0}\n";
  page += ".row{display:flex;flex-wrap:wrap;gap:8px;margin:8px 0 14px}\n";
  page += "a.btn{display:inline-block;text-decoration:none;border:1px solid #888;border-radius:10px;padding:8px 10px;background:#f7f7f7;color:#111}\n";
  page += "a.btn:active{transform:translateY(1px)}\n";
  page += "code{background:#f0f0f0;padding:2px 4px;border-radius:6px}\n";
  page += ".section{border-top:1px solid #ddd;padding-top:12px;margin-top:14px}\n";
  page += ".muted{color:#555;font-size:0.95em}\n";
  page += "</style></head><body>\n";

  page += "<h1>Pinball Bot</h1>\n";
  page += "<div>Firmware: <b>" + String(FW_VERSION) + "</b></div>\n";
  page += "<div>Camera locked: <b>" + String(g_cam_locked ? "true" : "false") + "</b></div>\n";
  page += "<div>Auto: <b>" + String(g_auto_enabled ? "true" : "false") + "</b> (persisted)</div>\n";

  // OTA status
  page += "<div>OTA: <b>" + String(g_ota_enabled ? "enabled" : "disabled") + "</b> (port " + String(OTA_PORT) + ")</div>\n";
  page += "<div class='muted'>Endpoints: <code>/ota</code>, <code>/otaInfo</code></div>\n";

  page += "<div class='section'><h2>Status</h2><div class='row'>\n";
  page += "<a class='btn' href='/version'>/version</a>\n";
  page += "<a class='btn' href='/caminfo'>/caminfo</a>\n";
  page += "<a class='btn' href='/relays'>/relays</a>\n";
  page += "<a class='btn' href='/diff'>/diff</a>\n";
  page += "<a class='btn' href='/frame.pgm'>/frame.pgm</a>\n";
  page += "<a class='btn' href='/roi'>/roi (ROI picker)</a>\n";
  page += "</div><div class='muted'>Tip: poll <code>/diff</code> from a script while tuning zones.</div></div>\n";

  page += "<div class='section'><h2>Safety</h2><div class='row'>\n";
  page += "<a class='btn' href='/kill'>/kill (AUTO OFF + relays OFF)</a>\n";
  page += "<a class='btn' href='/cfgReset' onclick='return confirm(\"Reset saved config and reboot?\")'>/cfgReset (wipe saved config)</a>\n";
  page += "</div></div>\n";

  page += "<div class='section'><h2>OTA</h2><div class='row'>\n";
  page += "<a class='btn' href='/otaInfo'>/otaInfo</a>\n";
  page += "<a class='btn' href='/ota?on=1&mins=10'>/ota?on=1&mins=10</a>\n";
  page += "<a class='btn' href='/ota?on=0'>/ota?on=0</a>\n";
  page += "</div><div class='muted'>Recommended: enable OTA only briefly when flashing.</div></div>\n";

  page += "<div class='section'><h2>Manual Flipper Control</h2><div class='row'>\n";
  page += "<a class='btn' href='/tapL?ms=120'>/tapL?ms=120</a>\n";
  page += "<a class='btn' href='/tapR?ms=120'>/tapR?ms=120</a>\n";
  page += "<a class='btn' href='/holdL?ms=300'>/holdL?ms=300</a>\n";
  page += "<a class='btn' href='/holdR?ms=300'>/holdR?ms=300</a>\n";
  page += "</div></div>\n";

  page += "<div class='section'><h2>Auto + Zones</h2><div class='row'>\n";
  page += "<a class='btn' href='/auto?on=1'>/auto?on=1</a>\n";
  page += "<a class='btn' href='/auto?on=0'>/auto?on=0</a>\n";
  page += "<a class='btn' href='/zones'>/zones</a>\n";
  page += "</div>\n";
  page += "<div class='muted'>ROI format: <code>x,y,w,h</code> in the 160x120 frame.</div></div>\n";

  page += "<div class='section'><h2>Motion knobs</h2><div class='row'>\n";
  page += "<a class='btn' href='/motion?mode=0'>/motion?mode=0</a>\n";
  page += "<a class='btn' href='/motion?mode=1'>/motion?mode=1</a>\n";
  page += "<a class='btn' href='/motion?mode=2'>/motion?mode=2</a>\n";
  page += "</div>\n";
  page += "<div class='muted'>Example: <code>/motion?mode=1&amp;cooldown_en=0&amp;gate_en=0</code> (persisted)</div>\n";
  page += "</div>\n";

  page += "</body></html>\n";
  server.send(200, "text/html", page);
}

static void handle_caminfo() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) { server.send(500, "application/json", "{\"error\":\"no_sensor\"}"); return; }
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{"
           "\"pid\":%u,\"ver\":%u,\"midh\":%u,\"midl\":%u,"
           "\"locked\":%s,"
           "\"agc_gain\":%d,\"aec_value\":%d,\"wb_mode\":%d,"
           "\"hmirror\":%d,\"vflip\":%d"
           "}",
           s->id.PID, s->id.VER, s->id.MIDH, s->id.MIDL,
           g_cam_locked ? "true" : "false",
           s->status.agc_gain, s->status.aec_value, s->status.wb_mode,
           s->status.hmirror, s->status.vflip);
  server.send(200, "application/json", buf);
}

static void handle_relays_readall() {
  String json = "{";
  json += "\"left\":" + String(relay_l_state ? "1" : "0") + ",";
  json += "\"right\":" + String(relay_r_state ? "1" : "0");
  json += "}";
  server.send(200, "application/json", json);
}

static void handle_relayL() {
  bool on = false;
  if (server.hasArg("on")) {
    if (!parseOnArgStrict(&on)) { server.send(400, "application/json", "{\"error\":\"bad_on_use_0_or_1\"}"); return; }
    setLeftRelay(on, "http");
  }
  server.send(200, "application/json", String("{\"left\":") + (relay_l_state ? "1" : "0") + "}");
}

static void handle_relayR() {
  bool on = false;
  if (server.hasArg("on")) {
    if (!parseOnArgStrict(&on)) { server.send(400, "application/json", "{\"error\":\"bad_on_use_0_or_1\"}"); return; }
    setRightRelay(on, "http");
  }
  server.send(200, "application/json", String("{\"right\":") + (relay_r_state ? "1" : "0") + "}");
}

static void handle_tapL() {
  uint32_t ms = parseMsArgOrDefault("ms", DEFAULT_TAP_MS, 10, 2000);
  flipper_tap_ms(FLIPPER_LEFT, ms);
  portENTER_CRITICAL(&g_stats_mux); g_stats.relay_taps++; portEXIT_CRITICAL(&g_stats_mux);
  server.send(200, "application/json", String("{\"tapL_ms\":") + ms + "}");
}

static void handle_tapR() {
  uint32_t ms = parseMsArgOrDefault("ms", DEFAULT_TAP_MS, 10, 2000);
  flipper_tap_ms(FLIPPER_RIGHT, ms);
  portENTER_CRITICAL(&g_stats_mux); g_stats.relay_taps++; portEXIT_CRITICAL(&g_stats_mux);
  server.send(200, "application/json", String("{\"tapR_ms\":") + ms + "}");
}

static void handle_holdL() {
  uint32_t ms = parseMsArgOrDefault("ms", DEFAULT_HOLD_MS, 10, 10000);
  flipper_hold_ms(FLIPPER_LEFT, ms);
  portENTER_CRITICAL(&g_stats_mux); g_stats.relay_holds++; portEXIT_CRITICAL(&g_stats_mux);
  server.send(200, "application/json", String("{\"holdL_ms\":") + ms + "}");
}

static void handle_holdR() {
  uint32_t ms = parseMsArgOrDefault("ms", DEFAULT_HOLD_MS, 10, 10000);
  flipper_hold_ms(FLIPPER_RIGHT, ms);
  portENTER_CRITICAL(&g_stats_mux); g_stats.relay_holds++; portEXIT_CRITICAL(&g_stats_mux);
  server.send(200, "application/json", String("{\"holdR_ms\":") + ms + "}");
}

static void handle_tapHoldL() {
  uint32_t tap  = parseMsArgOrDefault("tap",  DEFAULT_TAP_MS,  10, 2000);
  uint32_t hold = parseMsArgOrDefault("hold", DEFAULT_HOLD_MS, 10, 10000);
  flipper_tap_hold_ms(FLIPPER_LEFT, tap, hold);
  portENTER_CRITICAL(&g_stats_mux); g_stats.relay_holds++; portEXIT_CRITICAL(&g_stats_mux);
  server.send(200, "application/json", String("{\"tapHoldL\":{\"tap_ms\":") + tap + ",\"hold_ms\":" + hold + "}}");
}

static void handle_tapHoldR() {
  uint32_t tap  = parseMsArgOrDefault("tap",  DEFAULT_TAP_MS,  10, 2000);
  uint32_t hold = parseMsArgOrDefault("hold", DEFAULT_HOLD_MS, 10, 10000);
  flipper_tap_hold_ms(FLIPPER_RIGHT, tap, hold);
  portENTER_CRITICAL(&g_stats_mux); g_stats.relay_holds++; portEXIT_CRITICAL(&g_stats_mux);
  server.send(200, "application/json", String("{\"tapHoldR\":{\"tap_ms\":") + tap + ",\"hold_ms\":" + hold + "}}");
}

static void handle_frame_pgm() {
  portENTER_CRITICAL(&g_frame_mux);
  uint8_t* buf = latest_frame;
  size_t len = latest_len;
  int w = latest_w;
  int h = latest_h;
  uint32_t ts = latest_frame_ms;
  portEXIT_CRITICAL(&g_frame_mux);

  if (!buf || len == 0 || w <= 0 || h <= 0) {
    server.send(503, "text/plain", "No frame yet (vision task warming up)");
    return;
  }

  String header = "P5\n" + String(w) + " " + String(h) + "\n255\n";
  server.setContentLength(header.length() + len);
  server.sendHeader("Connection", "close");
  server.sendHeader("X-Frame-Millis", String(ts));
  server.send(200, "image/x-portable-graymap", "");

  WiFiClient client = server.client();
  client.write((const uint8_t*)header.c_str(), header.length());

  const size_t CHUNK = 1024;
  size_t off = 0;
  while (off < len) {
    size_t n = (len - off > CHUNK) ? CHUNK : (len - off);
    client.write(buf + off, n);
    off += n;
  }
}

static void handle_auto() {
  bool on;
  if (parseOnArgStrict(&on)) {
    g_auto_enabled = on;
    persist_save_all();
  }
  server.send(200, "application/json", String("{\"auto\":") + (g_auto_enabled ? "true" : "false") + "}");
}

static void handle_kill() {
  g_auto_enabled = false;

  flipper_release(FLIPPER_LEFT,  "kill");
  flipper_release(FLIPPER_RIGHT, "kill");

  relay_action[(int)FLIPPER_LEFT].hold = false;
  relay_action[(int)FLIPPER_LEFT].pulse_active = false;
  relay_action[(int)FLIPPER_LEFT].off_at_ms = 0;

  relay_action[(int)FLIPPER_RIGHT].hold = false;
  relay_action[(int)FLIPPER_RIGHT].pulse_active = false;
  relay_action[(int)FLIPPER_RIGHT].off_at_ms = 0;

  persist_save_all();
  server.send(200, "application/json", "{\"ok\":true,\"auto\":false,\"left\":0,\"right\":0}");
}

static void handle_cfgReset() {
  server.send(200, "application/json", "{\"ok\":true,\"resetting\":true}");
  delay(50);
  persist_clear_all_and_reboot();
}

static void handle_zones() {
  String json = "{";
  json += "\"auto\":" + String(g_auto_enabled ? "true" : "false") + ",";
  json += "\"zones\":[";
  for (int i = 0; i < MAX_ZONES; i++) {
    if (i) json += ",";
    Zone z = g_zones[i];

    ZoneStats s;
    portENTER_CRITICAL(&g_stats_mux);
    s = g_zone_stats[i];
    portEXIT_CRITICAL(&g_stats_mux);

    json += "{";
    json += "\"i\":" + String(i) + ",";
    json += "\"en\":" + String(z.enabled ? "1" : "0") + ",";
    json += "\"side\":\"" + String(sideName(z.side)) + "\",";
    json += "\"roi\":[" + String(z.roi.x) + "," + String(z.roi.y) + "," + String(z.roi.w) + "," + String(z.roi.h) + "],";
    json += "\"tap_ms\":" + String(z.tap_ms) + ",";
    json += "\"last_trigger_ms\":" + String(s.last_trigger_ms) + ",";
    json += "\"trigger_count\":" + String(s.trigger_count);
    json += "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// /zonesSet?i=0&roi=x,y,w,h&side=L|R&en=0|1&tap=120
static void handle_zonesSet() {
  int i = parseIntArgOrDefault("i", 0);
  if (i < 0) i = 0;
  if (i >= MAX_ZONES) i = MAX_ZONES - 1;

  Zone z = g_zones[i];

  if (server.hasArg("roi")) z.roi = parse_roi();
  if (server.hasArg("side")) {
    String s = server.arg("side"); s.toUpperCase();
    if (s == "R") z.side = FLIPPER_RIGHT;
    else if (s == "L") z.side = FLIPPER_LEFT;
  }
  if (server.hasArg("en")) {
    String e = server.arg("en");
    if (e == "0") z.enabled = false;
    if (e == "1") z.enabled = true;
  }
  if (server.hasArg("tap")) z.tap_ms = parseMsArgOrDefault("tap", z.tap_ms, 10, 2000);

  g_zones[i] = z;
  persist_save_all();

  server.send(200, "application/json",
    String("{\"ok\":true,\"i\":") + i +
    ",\"en\":" + (z.enabled ? "true":"false") +
    ",\"side\":\"" + String(sideName(z.side)) + "\"" +
    ",\"tap_ms\":" + String(z.tap_ms) +
    ",\"roi\":{\"x\":" + z.roi.x + ",\"y\":" + z.roi.y + ",\"w\":" + z.roi.w + ",\"h\":" + z.roi.h + "}" +
    "}"
  );
}

// /zonesClear?i=0 (if no i, clears all)
static void handle_zonesClear() {
  if (server.hasArg("i")) {
    int i = parseIntArgOrDefault("i", 0);
    if (i < 0) i = 0;
    if (i >= MAX_ZONES) i = MAX_ZONES - 1;
    g_zones[i].enabled = false;
    g_zones[i].roi = ROI{0,0,0,0};
    portENTER_CRITICAL(&g_stats_mux);
    g_zone_stats[i] = ZoneStats{};
    portEXIT_CRITICAL(&g_stats_mux);
    persist_save_all();
    server.send(200, "application/json", String("{\"cleared\":") + i + "}");
  } else {
    for (int i = 0; i < MAX_ZONES; i++) {
      g_zones[i].enabled = false;
      g_zones[i].roi = ROI{0,0,0,0};
      portENTER_CRITICAL(&g_stats_mux);
      g_zone_stats[i] = ZoneStats{};
      portEXIT_CRITICAL(&g_stats_mux);
    }
    persist_save_all();
    server.send(200, "application/json", "{\"cleared\":\"all\"}");
  }
}

static void handle_motion() {
  if (server.hasArg("mode")) motion_apply_mode(server.arg("mode").toInt());

  if (server.hasArg("pix_thr")) g_motion.pix_thr = (uint8_t)constrain(server.arg("pix_thr").toInt(), 0, 255);

  if (server.hasArg("min"))     g_motion.min_changed = (uint32_t)max(0L, server.arg("min").toInt());
  if (server.hasArg("max"))     g_motion.max_changed = (uint32_t)max(0L, server.arg("max").toInt());

  if (server.hasArg("global"))  g_motion.global_ratio_reject = server.arg("global").toFloat();

  if (server.hasArg("bbox_en")) g_motion.bbox_en = (server.arg("bbox_en") == "1");
  if (server.hasArg("bbox_w"))  g_motion.bbox_max_w = (int)max(1L, server.arg("bbox_w").toInt());
  if (server.hasArg("bbox_h"))  g_motion.bbox_max_h = (int)max(1L, server.arg("bbox_h").toInt());

  if (server.hasArg("down_required")) g_motion.down_required = (server.arg("down_required") == "1");

  if (server.hasArg("dir_en"))   g_motion.dir_en = (server.arg("dir_en") == "1");
  if (server.hasArg("dir_mode")) g_motion.dir_mode = server.arg("dir_mode").toInt();

  if (server.hasArg("cooldown_en")) g_motion.cooldown_en = (server.arg("cooldown_en") == "1");
  if (server.hasArg("cooldown_ms")) g_motion.cooldown_ms = (uint32_t)max(0L, server.arg("cooldown_ms").toInt());

  if (server.hasArg("gate_en")) g_motion.gate_en = (server.arg("gate_en") == "1");
  if (server.hasArg("gate_ms")) g_motion.gate_ms = (uint32_t)max(0L, server.arg("gate_ms").toInt());

  if (server.hasArg("gi_en"))  g_motion.gi_reject_en = (server.arg("gi_en") == "1");
  if (server.hasArg("gi_thr")) g_motion.gi_reject_abs_mean_thr = (int)max(0L, server.arg("gi_thr").toInt());

  persist_save_all();

  String json = "{";
  json += "\"mode\":" + String(g_motion.mode) + ",";
  json += "\"pix_thr\":" + String(g_motion.pix_thr) + ",";
  json += "\"min_changed\":" + String(g_motion.min_changed) + ",";
  json += "\"max_changed\":" + String(g_motion.max_changed) + ",";
  json += "\"global_ratio_reject\":" + String(g_motion.global_ratio_reject, 4) + ",";
  json += "\"bbox_en\":" + String(g_motion.bbox_en ? "true":"false") + ",";
  json += "\"bbox_max_w\":" + String(g_motion.bbox_max_w) + ",";
  json += "\"bbox_max_h\":" + String(g_motion.bbox_max_h) + ",";
  json += "\"down_required\":" + String(g_motion.down_required ? "true":"false") + ",";
  json += "\"dir_en\":" + String(g_motion.dir_en ? "true":"false") + ",";
  json += "\"dir_mode\":" + String(g_motion.dir_mode) + ",";
  json += "\"cooldown_en\":" + String(g_motion.cooldown_en ? "true":"false") + ",";
  json += "\"cooldown_ms\":" + String(g_motion.cooldown_ms) + ",";
  json += "\"gate_en\":" + String(g_motion.gate_en ? "true":"false") + ",";
  json += "\"gate_ms\":" + String(g_motion.gate_ms) + ",";
  json += "\"gi_reject_en\":" + String(g_motion.gi_reject_en ? "true":"false") + ",";
  json += "\"gi_reject_abs_mean_thr\":" + String(g_motion.gi_reject_abs_mean_thr);
  json += "}";
  server.send(200, "application/json", json);
}

static void handle_diff() {
  GlobalStats gs;
  portENTER_CRITICAL(&g_stats_mux);
  gs = g_stats;
  portEXIT_CRITICAL(&g_stats_mux);

  String json = "{";
  json += "\"auto\":" + String(g_auto_enabled ? "true" : "false") + ",";
  json += "\"cam_locked\":" + String(g_cam_locked ? "true" : "false") + ",";
  json += "\"w\":" + String(latest_w) + ",";
  json += "\"h\":" + String(latest_h) + ",";

  json += "\"motion\":{";
  json += "\"mode\":" + String(g_motion.mode) + ",";
  json += "\"pix_thr\":" + String(g_motion.pix_thr) + ",";
  json += "\"min_changed\":" + String(g_motion.min_changed) + ",";
  json += "\"max_changed\":" + String(g_motion.max_changed) + ",";
  json += "\"global_ratio_reject\":" + String(g_motion.global_ratio_reject, 4) + ",";
  json += "\"bbox_en\":" + String(g_motion.bbox_en ? "true":"false") + ",";
  json += "\"bbox_max_w\":" + String(g_motion.bbox_max_w) + ",";
  json += "\"bbox_max_h\":" + String(g_motion.bbox_max_h) + ",";
  json += "\"down_required\":" + String(g_motion.down_required ? "true":"false") + ",";
  json += "\"dir_en\":" + String(g_motion.dir_en ? "true":"false") + ",";
  json += "\"dir_mode\":" + String(g_motion.dir_mode) + ",";
  json += "\"cooldown_en\":" + String(g_motion.cooldown_en ? "true":"false") + ",";
  json += "\"cooldown_ms\":" + String(g_motion.cooldown_ms) + ",";
  json += "\"gate_en\":" + String(g_motion.gate_en ? "true":"false") + ",";
  json += "\"gate_ms\":" + String(g_motion.gate_ms) + ",";
  json += "\"gi_reject_en\":" + String(g_motion.gi_reject_en ? "true":"false") + ",";
  json += "\"gi_thr\":" + String(g_motion.gi_reject_abs_mean_thr);
  json += "},";

  json += "\"stats\":{";
  json += "\"frames\":" + String(gs.frames) + ",";
  json += "\"have_prev\":" + String(gs.have_prev) + ",";
  json += "\"gi_rej\":" + String(gs.gi_rejected) + ",";
  json += "\"cooldown_blk\":" + String(gs.cooldown_blocked) + ",";
  json += "\"mean_d\":" + String(gs.last_frame_mean_delta) + ",";
  json += "\"relay_taps\":" + String(gs.relay_taps) + ",";
  json += "\"relay_holds\":" + String(gs.relay_holds) + ",";
  json += "\"relay_releases\":" + String(gs.relay_releases);
  json += "},";

  json += "\"zones\":[";
  for (int i = 0; i < MAX_ZONES; i++) {
    Zone z = g_zones[i];
    ZoneStats s;
    portENTER_CRITICAL(&g_stats_mux);
    s = g_zone_stats[i];
    portEXIT_CRITICAL(&g_stats_mux);

    if (i) json += ",";
    json += "{";
    json += "\"i\":" + String(i) + ",";
    json += "\"en\":" + String(z.enabled ? "true" : "false") + ",";
    json += "\"side\":\"" + String(sideName(z.side)) + "\",";
    json += "\"tap_ms\":" + String(z.tap_ms) + ",";
    json += "\"roi\":{";
    json += "\"x\":" + String(z.roi.x) + ",\"y\":" + String(z.roi.y) + ",\"w\":" + String(z.roi.w) + ",\"h\":" + String(z.roi.h);
    json += "},";

    json += "\"armed\":" + String(s.armed ? "true" : "false") + ",";
    json += "\"changed\":" + String(s.changed) + ",";
    json += "\"ratio\":" + String(s.ratio, 4) + ",";
    json += "\"sum_changed\":" + String(s.sum_changed) + ",";
    json += "\"mean_delta\":" + String(s.mean_delta) + ",";
    json += "\"cx\":" + String(s.cx) + ",";
    json += "\"cy\":" + String(s.cy) + ",";
    json += "\"down\":" + String(s.down ? "true" : "false") + ",";

    json += "\"bbox\":{";
    json += "\"minx\":" + String(s.minx) + ",\"miny\":" + String(s.miny) + ",\"maxx\":" + String(s.maxx) + ",\"maxy\":" + String(s.maxy) + ",\"w\":" + String(s.bw) + ",\"h\":" + String(s.bh);
    json += "},";

    json += "\"checks\":{";
    json += "\"size_ok\":" + String(s.size_ok ? "true":"false") + ",";
    json += "\"global_ok\":" + String(s.global_ok ? "true":"false") + ",";
    json += "\"bbox_ok\":" + String(s.bbox_ok ? "true":"false") + ",";
    json += "\"down_ok\":" + String(s.down_ok ? "true":"false") + ",";
    json += "\"dir_ok\":" + String(s.dir_ok ? "true":"false") + ",";
    json += "\"cooldown_ok\":" + String(s.cooldown_ok ? "true":"false") + ",";
    json += "\"pre_gate_ok\":" + String(s.pre_gate_ok ? "true":"false");
    json += "},";

    json += "\"pre_gate_count\":" + String(s.pre_gate_count) + ",";
    json += "\"motion_ok\":" + String(s.motion_ok ? "true" : "false") + ",";
    json += "\"last_trigger_ms\":" + String(s.last_trigger_ms) + ",";
    json += "\"trigger_count\":" + String(s.trigger_count) + ",";
    json += "\"ok_count\":" + String(s.ok_count);
    json += "}";
  }
  json += "]";

  json += "}";
  server.send(200, "application/json", json);
}

// ---------------------------
// /roi tool (v0.5.3 updated: zoom + overlays + load zone fields)
// ---------------------------
static void handle_roi_tool() {
  String page;
  page.reserve(12000);

  page += R"HTML(<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>ROI Tool</title>
<style>
  body{font-family:system-ui,Arial,sans-serif;margin:14px;max-width:1200px}
  .row{display:flex;flex-wrap:wrap;gap:10px;align-items:center;margin:10px 0}
  canvas{border:1px solid #888;border-radius:12px;touch-action:none;image-rendering:pixelated}
  .card{border:1px solid #ddd;border-radius:12px;padding:12px}
  label{font-size:14px}
  input,select,button{font-size:14px;padding:6px 8px}
  button{border:1px solid #888;border-radius:10px;background:#f7f7f7}
  code{background:#f0f0f0;padding:2px 4px;border-radius:6px}
  .muted{color:#555}
  .warn{color:#a30}
  .small{font-size:12px}
</style>
</head>
<body>
<h2>ROI Picker</h2>
<div class="muted">
Drag a rectangle on the image. It will fill <code>x,y,w,h</code> (in 160×120 coordinates). Click Apply to save.
</div>

<div class="row card">
  <label>Zone:
    <select id="zone">
      <option value="0">0</option>
      <option value="1">1</option>
      <option value="2">2</option>
      <option value="3">3</option>
    </select>
  </label>

  <label>Side:
    <select id="side">
      <option value="L">L</option>
      <option value="R">R</option>
    </select>
  </label>

  <label>Enable:
    <select id="en">
      <option value="1">1</option>
      <option value="0">0</option>
    </select>
  </label>

  <label>Tap ms:
    <input id="tap" type="number" min="10" max="2000" value="120" style="width:90px"/>
  </label>

  <label>Zoom:
    <select id="zoom">
      <option value="2">2×</option>
      <option value="3">3×</option>
      <option value="4" selected>4×</option>
      <option value="5">5×</option>
    </select>
  </label>

  <button id="apply">Apply to /zonesSet</button>
  <button id="clear">Clear zone</button>
</div>

<div class="row card">
  <label>x <input id="x" type="number" min="0" value="0" style="width:80px"/></label>
  <label>y <input id="y" type="number" min="0" value="0" style="width:80px"/></label>
  <label>w <input id="w" type="number" min="0" value="0" style="width:80px"/></label>
  <label>h <input id="h" type="number" min="0" value="0" style="width:80px"/></label>
  <span class="muted">ROI = <code id="roiText">0,0,0,0</code></span>
</div>

<div class="row">
  <div class="card">
    <div class="row" style="margin:0;justify-content:space-between">
      <div class="muted">Live frame (source 160×120, displayed scaled)</div>
      <div>
        <button id="once">Refresh</button>
        <label class="muted">Auto <input id="auto" type="checkbox" checked/></label>
      </div>
    </div>
    <canvas id="cv"></canvas>
    <div class="muted small" style="margin-top:6px">
      Tip: define zones near the flipper “strike” area, not too big. All defined zones are overlaid.
    </div>
    <div id="status" class="muted"></div>
    <div id="err" class="warn"></div>
  </div>

  <div class="card" style="flex:1;min-width:320px">
    <div class="muted">Current zones</div>
    <pre id="zones" style="white-space:pre-wrap;font-size:12px;margin:8px 0 0 0"></pre>
  </div>
</div>

<script>
const SRC_W = 160;
const SRC_H = 120;

const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');

let drag = false;
let x0=0, y0=0, x1=0, y1=0;

let zonesCache = null;

const zoneColors = [
  {stroke:"#00ff00", fill:"rgba(0,255,0,0.10)"},
  {stroke:"#ff00ff", fill:"rgba(255,0,255,0.10)"},
  {stroke:"#00ffff", fill:"rgba(0,255,255,0.10)"},
  {stroke:"#ffff00", fill:"rgba(255,255,0,0.10)"},
];

function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

function setROI(ax, ay, aw, ah){
  document.getElementById('x').value = ax;
  document.getElementById('y').value = ay;
  document.getElementById('w').value = aw;
  document.getElementById('h').value = ah;
  document.getElementById('roiText').textContent = `${ax},${ay},${aw},${ah}`;
}

function roiFromInputs(){
  const x = parseInt(document.getElementById('x').value||"0",10);
  const y = parseInt(document.getElementById('y').value||"0",10);
  const w = parseInt(document.getElementById('w').value||"0",10);
  const h = parseInt(document.getElementById('h').value||"0",10);
  return {x,y,w,h};
}

['x','y','w','h'].forEach(id=>{
  document.getElementById(id).addEventListener('input', ()=>{
    const r = roiFromInputs();
    setROI(r.x,r.y,r.w,r.h);
    redrawOverlays();
  });
});

function getZoom(){
  const z = parseInt(document.getElementById('zoom').value,10);
  return (isFinite(z) && z>=1) ? z : 4;
}

function setCanvasZoom(){
  const z = getZoom();
  cv.width = SRC_W * z;
  cv.height = SRC_H * z;
  cv.style.width = (SRC_W * z) + "px";
  cv.style.height = (SRC_H * z) + "px";
}

document.getElementById('zoom').addEventListener('change', async ()=>{
  setCanvasZoom();
  await refreshOnce();
});

// --- PGM P5 decoder ---
async function fetchPGM(){
  const res = await fetch(`/frame.pgm?_=${Date.now()}`, {cache:'no-store'});
  if(!res.ok) throw new Error(`HTTP ${res.status}`);
  const buf = new Uint8Array(await res.arrayBuffer());

  let i=0;
  function readToken(){
    while(i < buf.length && (buf[i]===10 || buf[i]===13 || buf[i]===9 || buf[i]===32)) i++;
    if(i < buf.length && buf[i]===35){
      while(i < buf.length && buf[i]!==10) i++;
      return readToken();
    }
    let s='';
    while(i < buf.length && !(buf[i]===10 || buf[i]===13 || buf[i]===9 || buf[i]===32)){
      s += String.fromCharCode(buf[i]); i++;
    }
    return s;
  }
  const magic = readToken();
  const w = parseInt(readToken(),10);
  const h = parseInt(readToken(),10);
  const maxv = parseInt(readToken(),10);
  while(i < buf.length && (buf[i]===10 || buf[i]===13 || buf[i]===9 || buf[i]===32)) { i++; break; }

  if(magic !== 'P5') throw new Error(`Bad magic ${magic}`);
  if(maxv !== 255) throw new Error(`Unsupported maxval ${maxv}`);

  const pixels = buf.subarray(i, i + w*h);
  if(pixels.length < w*h) throw new Error(`Truncated pixel data`);
  return {w,h,pixels};
}

function drawFrameScaled(w,h,pixels){
  const z = getZoom();
  if(w !== SRC_W || h !== SRC_H) throw new Error(`Unexpected frame size ${w}x${h}`);
  const off = document.createElement('canvas');
  off.width = SRC_W;
  off.height = SRC_H;
  const octx = off.getContext('2d');
  const id = octx.createImageData(SRC_W,SRC_H);
  for(let p=0, j=0; p < SRC_W*SRC_H; p++, j+=4){
    const v = pixels[p];
    id.data[j+0]=v; id.data[j+1]=v; id.data[j+2]=v; id.data[j+3]=255;
  }
  octx.putImageData(id,0,0);

  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(off, 0,0, SRC_W,SRC_H, 0,0, SRC_W*z, SRC_H*z);
}

function drawZoneRect(i, r, selected=false){
  if(!r || r.w<=0 || r.h<=0) return;
  const z = getZoom();
  const c = zoneColors[i % zoneColors.length];
  ctx.save();
  ctx.lineWidth = selected ? 3 : 2;
  ctx.strokeStyle = c.stroke;
  ctx.fillStyle = c.fill;

  const x = r.x * z;
  const y = r.y * z;
  const w = r.w * z;
  const h = r.h * z;

  ctx.fillRect(x, y, w, h);
  ctx.strokeRect(x + 0.5, y + 0.5, w, h);

  ctx.fillStyle = c.stroke;
  ctx.font = `${12*z/4}px system-ui, Arial`;
  ctx.fillText(`Z${i}`, x + 4, y + 14);

  ctx.restore();
}

function redrawOverlays(){
  if(zonesCache && zonesCache.zones){
    const sel = parseInt(document.getElementById('zone').value,10);
    zonesCache.zones.forEach(zz=>{
      const r = {x:zz.roi[0], y:zz.roi[1], w:zz.roi[2], h:zz.roi[3]};
      const isSel = (zz.i === sel);
      if(r.w>0 && r.h>0){
        drawZoneRect(zz.i, r, isSel);
      }
    });
  }

  const r = roiFromInputs();
  if((drag) || (r.w>0 && r.h>0)){
    const z = getZoom();
    ctx.save();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "white";
    ctx.setLineDash([6,4]);
    ctx.strokeRect(r.x*z + 0.5, r.y*z + 0.5, r.w*z, r.h*z);
    ctx.restore();
  }
}

function posFromEvent(ev){
  const rect = cv.getBoundingClientRect();
  const cx = (ev.clientX - rect.left) * (cv.width / rect.width);
  const cy = (ev.clientY - rect.top) * (cv.height / rect.height);
  const z = getZoom();
  const sx = clamp(Math.round(cx / z), 0, SRC_W-1);
  const sy = clamp(Math.round(cy / z), 0, SRC_H-1);
  return {x:sx, y:sy};
}

cv.addEventListener('pointerdown', (ev)=>{
  cv.setPointerCapture(ev.pointerId);
  drag=true;
  const p = posFromEvent(ev);
  x0=p.x; y0=p.y; x1=p.x; y1=p.y;
});

cv.addEventListener('pointermove', (ev)=>{
  if(!drag) return;
  const p = posFromEvent(ev);
  x1=p.x; y1=p.y;
  const ax = Math.min(x0,x1), ay = Math.min(y0,y1);
  const w = Math.abs(x1-x0), h = Math.abs(y1-y0);
  setROI(ax, ay, w, h);
  document.getElementById('err').textContent = '';
  refreshOnce(false);
});

cv.addEventListener('pointerup', ()=>{
  drag=false;
  document.getElementById('err').textContent = '';
  refreshOnce(false);
});

async function refreshZones(){
  const z = await (await fetch('/zones', {cache:'no-store'})).json();
  zonesCache = z;
  document.getElementById('zones').textContent = JSON.stringify(z, null, 2);
  return z;
}

function loadZoneFieldsFromCache(i){
  if(!zonesCache || !zonesCache.zones) return;
  const zz = zonesCache.zones.find(o=>o.i===i);
  if(!zz) return;

  document.getElementById('side').value = zz.side;
  document.getElementById('en').value = String(zz.en);
  document.getElementById('tap').value = String(zz.tap_ms);

  const r = {x:zz.roi[0], y:zz.roi[1], w:zz.roi[2], h:zz.roi[3]};
  setROI(r.x,r.y,r.w,r.h);
}

document.getElementById('zone').addEventListener('change', ()=>{
  const i = parseInt(document.getElementById('zone').value,10);
  loadZoneFieldsFromCache(i);
  refreshOnce(false);
});

async function refreshOnce(fetchZonesToo=true){
  const st = document.getElementById('status');
  const er = document.getElementById('err');
  try{
    er.textContent='';
    const t0 = performance.now();

    if(fetchZonesToo){
      await refreshZones();
      const i = parseInt(document.getElementById('zone').value,10);
      loadZoneFieldsFromCache(i);
    }

    const {w,h,pixels} = await fetchPGM();
    setCanvasZoom();
    drawFrameScaled(w,h,pixels);
    redrawOverlays();

    const t1 = performance.now();
    st.textContent = `Frame ok (${w}x${h}) total ${(t1-t0).toFixed(1)} ms (zoom ${getZoom()}×)`;
  }catch(e){
    er.textContent = String(e);
  }
}

document.getElementById('once').addEventListener('click', ()=>refreshOnce(true));

document.getElementById('apply').addEventListener('click', async ()=>{
  const i = document.getElementById('zone').value;
  const side = document.getElementById('side').value;
  const en = document.getElementById('en').value;
  const tap = document.getElementById('tap').value;
  const r = roiFromInputs();
  const roi = `${r.x},${r.y},${r.w},${r.h}`;
  const url = `/zonesSet?i=${encodeURIComponent(i)}&roi=${encodeURIComponent(roi)}&side=${encodeURIComponent(side)}&en=${encodeURIComponent(en)}&tap=${encodeURIComponent(tap)}`;
  try{
    await (await fetch(url, {cache:'no-store'})).json();
    document.getElementById('status').textContent = `Applied: ${url}`;
    await refreshOnce(true);
  }catch(e){
    document.getElementById('err').textContent = `Apply failed: ${e}`;
  }
});

document.getElementById('clear').addEventListener('click', async ()=>{
  const i = document.getElementById('zone').value;
  const url = `/zonesClear?i=${encodeURIComponent(i)}`;
  try{
    await fetch(url, {cache:'no-store'});
    document.getElementById('status').textContent = `Cleared zone ${i}`;
    await refreshOnce(true);
  }catch(e){
    document.getElementById('err').textContent = `Clear failed: ${e}`;
  }
});

async function loop(){
  if(document.getElementById('auto').checked){
    await refreshOnce(true);
  } else {
    await refreshOnce(false);
  }
  setTimeout(loop, 450);
}

setCanvasZoom();
setROI(0,0,0,0);
refreshOnce(true);
loop();
</script>
</body>
</html>)HTML";

  server.send(200, "text/html", page);
}

// ---------------------------
// v0.6.0 OTA endpoints
// ---------------------------
static void handle_ota() {
  // /ota?on=0|1&mins=10
  bool on;
  bool hasOn = parseOnArgStrict(&on);

  if (hasOn) {
    g_ota_enabled = on;
    if (!on) {
      g_ota_disable_at_ms = 0;
    } else {
      int mins = parseIntArgOrDefault("mins", 0);
      if (mins > 0) {
        uint32_t dur_ms = (uint32_t)mins * 60UL * 1000UL;
        g_ota_disable_at_ms = millis() + dur_ms;
      } else {
        g_ota_disable_at_ms = 0;
      }
    }
  }

  String json = "{";
  json += "\"ota_enabled\":" + String(g_ota_enabled ? "true" : "false") + ",";
  json += "\"port\":" + String(OTA_PORT) + ",";
  json += "\"hostname\":\"" + String(OTA_HOSTNAME) + "\",";
  if (g_ota_disable_at_ms) {
    int32_t remain = (int32_t)(g_ota_disable_at_ms - millis());
    if (remain < 0) remain = 0;
    json += "\"auto_disable_ms\":" + String(remain);
  } else {
    json += "\"auto_disable_ms\":0";
  }
  json += "}";
  server.send(200, "application/json", json);
}

static void handle_otaInfo() {
  const esp_partition_t* run = esp_ota_get_running_partition();
  const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);

  String json = "{";
  json += "\"ota_enabled\":" + String(g_ota_enabled ? "true" : "false") + ",";
  json += "\"port\":" + String(OTA_PORT) + ",";
  json += "\"running\":{";
  if (run) {
    json += "\"label\":\"" + String(run->label) + "\",";
    json += "\"subtype\":" + String((int)run->subtype) + ",";
    json += "\"offset\":" + String((uint32_t)run->address) + ",";
    json += "\"size\":" + String((uint32_t)run->size);
  } else {
    json += "\"label\":\"?\"";
  }
  json += "},";

  json += "\"next\":{";
  if (next) {
    json += "\"label\":\"" + String(next->label) + "\",";
    json += "\"subtype\":" + String((int)next->subtype) + ",";
    json += "\"offset\":" + String((uint32_t)next->address) + ",";
    json += "\"size\":" + String((uint32_t)next->size);
  } else {
    json += "\"label\":\"?\"";
  }
  json += "}";

  json += "}";
  server.send(200, "application/json", json);
}

static void ota_force_safe_state(const char* why) {
  (void)why;
  // Same semantics as /kill, but no HTTP response needed.
  g_auto_enabled = false;

  flipper_release(FLIPPER_LEFT,  "ota");
  flipper_release(FLIPPER_RIGHT, "ota");

  relay_action[(int)FLIPPER_LEFT].hold = false;
  relay_action[(int)FLIPPER_LEFT].pulse_active = false;
  relay_action[(int)FLIPPER_LEFT].off_at_ms = 0;

  relay_action[(int)FLIPPER_RIGHT].hold = false;
  relay_action[(int)FLIPPER_RIGHT].pulse_active = false;
  relay_action[(int)FLIPPER_RIGHT].off_at_ms = 0;

  // Persist AUTO=off so reboot after OTA stays safe
  persist_save_all();
}

static void ota_setup() {
  ArduinoOTA.setPort(OTA_PORT);                 // default is 3232
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA
    .onStart([]() {
      ota_force_safe_state("ota_start");

      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch";
      else type = "filesystem";

      const esp_partition_t* run = esp_ota_get_running_partition();
      const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);

      Serial.printf("OTA Start: %s\n", type.c_str());
      if (run)  Serial.printf("  running: %s @0x%08lx size=%lu\n", run->label, (unsigned long)run->address, (unsigned long)run->size);
      if (next) Serial.printf("  next:    %s @0x%08lx size=%lu\n", next->label, (unsigned long)next->address, (unsigned long)next->size);
    })
    .onEnd([]() {
      Serial.println("OTA End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      static uint32_t lastPrint = 0;
      uint32_t now = millis();
      if (now - lastPrint > 500) {
        lastPrint = now;
        uint32_t pct = (total > 0) ? (progress * 100U / total) : 0;
        Serial.printf("OTA Progress: %u%%\n", (unsigned)pct);
      }
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA Error[%u]: ", (unsigned)error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      else Serial.println("Unknown");
    });

  ArduinoOTA.begin();
  Serial.printf("OTA ready: %s:%u (enabled=%c)\n", OTA_HOSTNAME, (unsigned)OTA_PORT, g_ota_enabled ? 'Y' : 'N');
}

static void ota_handle_loop() {
  if (!g_ota_enabled) return;

  if (g_ota_disable_at_ms) {
    if ((int32_t)(millis() - g_ota_disable_at_ms) >= 0) {
      g_ota_enabled = false;
      g_ota_disable_at_ms = 0;
      Serial.println("OTA auto-disabled (timeout).");
      return;
    }
  }
  ArduinoOTA.handle();
}

// ---------------------------
// Camera init
// ---------------------------
static bool init_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA; // 160x120
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) config.fb_location = CAMERA_FB_IN_PSRAM;
  else              config.fb_location = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return false;

  apply_camera_orientation();
  camera_enable_auto();
  g_cam_locked = false;

  return true;
}

// ---------------------------
// Arduino entry points
// ---------------------------
void setup() {
  Serial.begin(921600);
  delay(200);
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());

  // Relays: deterministic boot state
  pinMode(RELAY_L_GPIO, OUTPUT);
  pinMode(RELAY_R_GPIO, OUTPUT);
  digitalWrite(RELAY_L_GPIO, HIGH);
  digitalWrite(RELAY_R_GPIO, HIGH);

  gpio_pullup_en((gpio_num_t)RELAY_L_GPIO);
  gpio_pullup_en((gpio_num_t)RELAY_R_GPIO);
  gpio_pulldown_dis((gpio_num_t)RELAY_L_GPIO);
  gpio_pulldown_dis((gpio_num_t)RELAY_R_GPIO);

  setLeftRelay(false, "boot");
  setRightRelay(false, "boot");

  // LED
  pinMode(LED_GPIO, OUTPUT);
  writeLed(false);
  for (int i = 0; i < 3; i++) { writeLed(true); delay(20); writeLed(false); delay(60); }

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("SoftAP: %s (%s)\n", AP_SSID, ok ? "OK" : "FAIL");
  Serial.print("IP: "); Serial.println(ip);

  // Motion defaults
  motion_apply_mode(0);
  g_motion.gi_reject_en = false;

  // Zones defaults
  for (int i = 0; i < MAX_ZONES; i++) {
    g_zones[i].enabled = false;
    g_zones[i].side = (i % 2 == 0) ? FLIPPER_LEFT : FLIPPER_RIGHT;
    g_zones[i].tap_ms = 120;
    g_zones[i].roi = ROI{0,0,0,0};
    g_zone_stats[i] = ZoneStats{};
  }
  g_zones[0].enabled = true;
  g_zones[0].side = FLIPPER_LEFT;
  g_zones[0].tap_ms = 120;
  g_zones[0].roi = ROI{0,0,160,120};

  // Load persisted config (overrides defaults if present)
  persist_load_all();

  if (!init_camera()) {
    Serial.println("Camera init failed. Check board selection, PSRAM, and camera connection.");
  } else {
    Serial.println("Camera init OK (GRAYSCALE QQVGA). Auto enabled; will lock after stable.");
    g_cam_warmup_start_ms = millis();
  }

  // Routes
  server.on("/", handle_root);
  server.on("/version", [](){ server.send(200, "text/plain", FW_VERSION); });

  server.on("/relays", handle_relays_readall);
  server.on("/relayL", handle_relayL);
  server.on("/relayR", handle_relayR);

  server.on("/tapL", handle_tapL);
  server.on("/tapR", handle_tapR);
  server.on("/holdL", handle_holdL);
  server.on("/holdR", handle_holdR);
  server.on("/tapHoldL", handle_tapHoldL);
  server.on("/tapHoldR", handle_tapHoldR);

  server.on("/frame.pgm", handle_frame_pgm);
  server.on("/diff", handle_diff);
  server.on("/auto", handle_auto);
  server.on("/kill", handle_kill);
  server.on("/zones", handle_zones);
  server.on("/zonesSet", handle_zonesSet);
  server.on("/zonesClear", handle_zonesClear);
  server.on("/caminfo", handle_caminfo);
  server.on("/roi", handle_roi_tool);
  server.on("/motion", handle_motion);
  server.on("/cfgReset", handle_cfgReset);

  // OTA endpoints (v0.6.0)
  server.on("/ota", handle_ota);
  server.on("/otaInfo", handle_otaInfo);

  server.begin();
  Serial.println("HTTP server started.");

  if (esp_camera_sensor_get() != nullptr) {
    BaseType_t okTask = xTaskCreatePinnedToCore(
      vision_task,
      "vision_task",
      8192,
      nullptr,
      2,
      &g_vision_task,
      1
    );
    Serial.printf("Vision task start: %s\n", (okTask == pdPASS) ? "OK" : "FAIL");
  }

  // Ensure current config is stored at least once (creates header on first run)
  persist_save_all();

  // Start ArduinoOTA (v0.6.0)
  ota_setup();

  // TODO remove after OTA tested
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* next    = esp_ota_get_next_update_partition(NULL);
  
  Serial.printf("Running: %s @0x%06lx size=0x%06lx\n",
                running->label,
                (unsigned long)running->address,
                (unsigned long)running->size);
  
  Serial.printf("Next:    %s @0x%06lx size=0x%06lx\n",
                next->label,
                (unsigned long)next->address,
                (unsigned long)next->size);
}

void loop() {
  server.handleClient();
  relay_actions_update();

  // OTA servicing (v0.6.0)
  ota_handle_loop();

  unsigned long now = millis();
  bool anyOn = anyRelayOn();
  unsigned long period = anyOn ? 200 : 500;

  if (anyOn != lastAnyRelayStateForBlink) {
    lastAnyRelayStateForBlink = anyOn;
    lastBlink = now;
    ledState = false;
    writeLed(false);
  }

  if (now - lastBlink >= period) {
    lastBlink = now;
    ledState = !ledState;
    writeLed(ledState);
  }

  // Print STAT once per second with deltas ("dframes")
  static uint32_t lastStatMs = 0;
  if (now - lastStatMs >= 1000) {
    lastStatMs = now;

    GlobalStats gs;
    portENTER_CRITICAL(&g_stats_mux);
    gs = g_stats;
    portEXIT_CRITICAL(&g_stats_mux);

    uint32_t d_frames = gs.frames - g_stats_last.frames;
    uint32_t d_have   = gs.have_prev - g_stats_last.have_prev;
    uint32_t d_gi     = gs.gi_rejected - g_stats_last.gi_rejected;
    uint32_t d_cd     = gs.cooldown_blocked - g_stats_last.cooldown_blocked;

    Serial.printf("STAT frames=%lu(+%lu) have_prev=%lu(+%lu) gi_rej=%lu(+%lu) cooldown_blk=%lu(+%lu) mean_d=%d auto=%c mode=%d | ",
      (unsigned long)gs.frames, (unsigned long)d_frames,
      (unsigned long)gs.have_prev, (unsigned long)d_have,
      (unsigned long)gs.gi_rejected, (unsigned long)d_gi,
      (unsigned long)gs.cooldown_blocked, (unsigned long)d_cd,
      gs.last_frame_mean_delta,
      g_auto_enabled ? 'Y':'N',
      g_motion.mode
    );

    for (int i = 0; i < MAX_ZONES; i++) {
      ZoneStats zs;
      portENTER_CRITICAL(&g_stats_mux);
      zs = g_zone_stats[i];
      portEXIT_CRITICAL(&g_stats_mux);

      uint32_t d_ok   = zs.ok_count - g_zone_last[i].ok_count;
      uint32_t d_trig = zs.trigger_count - g_zone_last[i].trigger_count;
      uint32_t d_pre  = zs.pre_gate_count - g_zone_last[i].pre_gate_count;

      Serial.printf("Z%d(pre=%lu +%lu ok=%lu +%lu trig=%lu +%lu) ",
        i,
        (unsigned long)zs.pre_gate_count, (unsigned long)d_pre,
        (unsigned long)zs.ok_count, (unsigned long)d_ok,
        (unsigned long)zs.trigger_count, (unsigned long)d_trig
      );

      g_zone_last[i] = zs;
    }

    Serial.printf("| relay(tap=%lu +%lu hold=%lu +%lu rel=%lu +%lu) | OTA=%c\n",
      (unsigned long)gs.relay_taps, (unsigned long)(gs.relay_taps - g_stats_last.relay_taps),
      (unsigned long)gs.relay_holds, (unsigned long)(gs.relay_holds - g_stats_last.relay_holds),
      (unsigned long)gs.relay_releases, (unsigned long)(gs.relay_releases - g_stats_last.relay_releases),
      g_ota_enabled ? 'Y' : 'N'
    );

    g_stats_last = gs;
  }

  // Heartbeat every 2s
  static uint32_t lastHb = 0;
  if (now - lastHb > 2000) {
    lastHb = now;
    Serial.printf("HB ms=%lu wifi=%s ip=%s relays=%c%c cam_locked=%c ota=%c\n",
                  (unsigned long)now,
                  (WiFi.getMode() == WIFI_AP) ? "AP" : "?",
                  WiFi.softAPIP().toString().c_str(),
                  relay_l_state ? 'L' : '-',
                  relay_r_state ? 'R' : '-',
                  g_cam_locked ? 'Y' : 'N',
                  g_ota_enabled ? 'Y' : 'N');
  }
}
