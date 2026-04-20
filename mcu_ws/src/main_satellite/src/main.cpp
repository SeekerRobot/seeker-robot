/**
 * @file main_satellite/src/main.cpp
 * @brief Seeker Robot — camera + mic satellite sketch.
 *
 * Runs on a second XIAO ESP32-S3 Sense board that offloads the OV2640 camera
 * and the PDM microphone from the primary MCU. The main board runs with
 * `-DENABLE_CAM=0 -DENABLE_MIC=0` (see the `esp32s3sense_offload` env) so the
 * host-side stack can point its camera proxy and mic consumer at this board's
 * static IP instead.
 *
 * Motivation: camera MJPEG + mic PCM + speaker TTS + OLED framebuffer + gait
 * + micro-ROS all on one ESP32-S3 exhausts the internal-SRAM PBUF pool under
 * concurrent load (sendPacket() errno 12). Splitting the two heaviest HTTP
 * endpoints onto a dedicated board keeps both sides well below the PBUF
 * starvation threshold.
 *
 * Endpoints (same shape as the primary board — the host-side nodes don't need
 * to change, only their configured IP):
 *   GET http://<SATELLITE_IP>:80/cam    — MJPEG video stream
 *   GET http://<SATELLITE_IP>:81/audio  — raw 16-bit PCM, 16 kHz, mono
 *
 * Production bring-up:
 *   - Safe-mode arbitration: 5 consecutive PANIC/WDT boots in under 30 s drops
 *     the sketch into BLE + WiFi + OTA, skipping camera/mic init so a broken
 *     firmware can always be recovered over the air.
 *   - BLE Nordic-UART debug transport (optional — serial-only if it fails).
 *   - WiFi static-IP bring-up with ArduinoOTA handler.
 *   - Onboard LED acts as status indicator: 500 ms toggle during normal
 *     operation (BlinkSubsystem default), 100 ms toggle (~5 Hz) in safe mode
 *     so the operator can tell at a glance when OTA recovery is required.
 *     The satellite runs with only the sense board connected — no external
 *     LED chain, no OLED, no I2C peripherals.
 *
 * Network config — expects SATELLITE_IP, WIFI_SSID, WIFI_PASSWORD, GATEWAY,
 * and SUBNET in network_config.ini. Reuses WIFI_SSID/GATEWAY/SUBNET with the
 * primary board.
 *
 * Pin layout matches the S3 Sense `Config::pdm_clk` / `Config::pdm_data`
 * (mic) and the bundled `camera_pins.h` layout for the OV2640 module that
 * ships with the Seeed board.
 */
#include <Arduino.h>
#include <BleDebugSubsystem.h>
#include <BlinkSubsystem.h>
#include <CameraSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <MicSubsystem.h>
#include <Preferences.h>
#include <RobotConfig.h>
#include <WiFiUdp.h>
#include <camera_pins.h>
#include <esp_system.h>

// ---------------------------------------------------------------------------
// Network config — SATELLITE_IP is a dedicated macro so the satellite doesn't
// collide with the primary board's STATIC_IP. WIFI_SSID / WIFI_PASSWORD /
// GATEWAY / SUBNET are shared.
// ---------------------------------------------------------------------------
#ifndef SATELLITE_IP
#error "SATELLITE_IP not defined — add `satellite_ip = { a, b, c, d }` to network_config.ini"
#endif

static IPAddress static_ip SATELLITE_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

// ---------------------------------------------------------------------------
// Safe-mode state — identical mechanism to the primary main sketch so a
// brick-loop satellite can always be recovered over OTA.
// ---------------------------------------------------------------------------
static constexpr uint8_t kSafeModeThreshold = 5;
static constexpr uint32_t kStableRunMs = 30000;
static constexpr char kSafeModeNs[] = "seeker_sm";
static constexpr char kSafeModeCount[] = "count";
static constexpr char kSafeModeReason[] = "last_reason";

// UDP reset channel — same contract as the primary main sketch, so a single
// host-side tool can recover either board.
// Trigger: `echo -n "SEEKER_CLEAR_SM" | nc -u -w1 <SAT_IP> 4210`.
static constexpr uint16_t kSafeModeUdpPort = 4210;
static constexpr char kSafeModeUdpMagic[] = "SEEKER_CLEAR_SM";

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_RESET";
    case ESP_RST_SW:        return "SW_RESTART";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static uint8_t s_boot_count = 0;
static esp_reset_reason_t s_reset_reason = ESP_RST_UNKNOWN;
static char s_prev_reason[16] = {};

static uint8_t safeModeTickOnBoot() {
  s_reset_reason = esp_reset_reason();
  Preferences p;
  p.begin(kSafeModeNs, /*readOnly=*/false);
  s_boot_count = p.getUChar(kSafeModeCount, 0) + 1;
  p.getString(kSafeModeReason, s_prev_reason, sizeof(s_prev_reason));
  p.putUChar(kSafeModeCount, s_boot_count);
  p.putString(kSafeModeReason, resetReasonStr(s_reset_reason));
  p.end();
  return s_boot_count;
}

static void safeModeClearCounter() {
  Preferences p;
  p.begin(kSafeModeNs, /*readOnly=*/false);
  p.clear();
  p.end();
}

static void safeModeScheduleClear() {
  xTaskCreatePinnedToCore(
      [](void*) {
        vTaskDelay(pdMS_TO_TICKS(kStableRunMs));
        safeModeClearCounter();
        Debug::printf(Debug::Level::INFO,
                      "[Sat] Firmware stable for %u ms — cleared boot count",
                      (unsigned)kStableRunMs);
        vTaskDelete(nullptr);
      },
      "sm_clear", 2048, nullptr, 1, nullptr, 1);
}

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
// BlinkSubsystem's update() toggles LED_BUILTIN once per update() call, so
// the value passed to beginThreadedPinned() *is* the toggle period. 500 ms
// is the normal-operation heartbeat; safe mode re-starts blink at 100 ms.
static constexpr uint32_t kBlinkNormalMs = 500;
static constexpr uint32_t kBlinkSafeModeMs = 100;

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::BleDebugSetup ble_setup("SeekerSatellite");

static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "sat_wifi", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

// Satellite has no micro-ROS stack competing for the lwIP PBUF pool, so it
// can afford a bigger frame and a lower (better) jpeg quality number. Stream
// cap stays at 20 Hz (50 ms) — well below the OV2640's ceiling but matches
// the main board for consumer-side parity.
static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80,
                                        /*ctrl_port=*/32768,
                                        /*frame_interval_ms=*/50,
                                        /*jpeg_quality=*/10,
                                        /*frame_size=*/FRAMESIZE_VGA);

static Subsystem::MicSetup mic_setup(I2S_NUM_0,
                                     /*sample_rate=*/16000,
                                     /*clk_pin=*/Config::pdm_clk,
                                     /*data_pin=*/Config::pdm_data,
                                     /*gain=*/4,
                                     /*http_port=*/81);

static const char* wifiStateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED: return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:   return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:    return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING: return "RECONNECTING";
    case Subsystem::WifiState::FAILED:       return "FAILED";
  }
  return "UNKNOWN";
}

[[noreturn]] static void runSafeMode() {
  Debug::printf(Debug::Level::ERROR,
                "[SafeMode] %u consecutive boots (prev_reason=%s, "
                "this_reason=%s) — skipping cam/mic bring-up, awaiting OTA.",
                s_boot_count, s_prev_reason[0] ? s_prev_reason : "n/a",
                resetReasonStr(s_reset_reason));

  // 5 Hz onboard-LED flash — visually distinct from the 1 Hz normal-mode
  // heartbeat so the operator can tell at a glance the board is awaiting OTA.
  blink.beginThreadedPinned(2048, 1, kBlinkSafeModeMs, 1);

  auto& ble = Subsystem::BleDebugSubsystem::getInstance(ble_setup);
  if (ble.init()) ble.beginThreadedPinned(8192, 2, 50, 0);

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR,
                  "[SafeMode] WiFi init failed — BLE-only, no OTA");
  } else {
    wifi.beginThreadedPinned(4096, 3, 100, 1);
  }

  WiFiUDP reset_udp;
  bool udp_bound = false;

  uint32_t last_status = 0;
  while (true) {
    if (wifi.isConnected() && !udp_bound) {
      udp_bound = reset_udp.begin(kSafeModeUdpPort);
      Debug::printf(udp_bound ? Debug::Level::INFO : Debug::Level::ERROR,
                    "[SafeMode] UDP reset listener on :%u %s",
                    kSafeModeUdpPort, udp_bound ? "ready" : "bind failed");
    }
    if (udp_bound) {
      int len = reset_udp.parsePacket();
      if (len > 0) {
        char buf[32];
        int n = reset_udp.read(reinterpret_cast<uint8_t*>(buf),
                               sizeof(buf) - 1);
        if (n > 0) buf[n] = '\0';
        constexpr int kMagicLen =
            static_cast<int>(sizeof(kSafeModeUdpMagic)) - 1;
        if (n == kMagicLen &&
            memcmp(buf, kSafeModeUdpMagic, kMagicLen) == 0) {
          reset_udp.beginPacket(reset_udp.remoteIP(), reset_udp.remotePort());
          reset_udp.print("OK_RESTARTING");
          reset_udp.endPacket();
          Debug::printf(Debug::Level::ERROR,
                        "[SafeMode] UDP reset from %s:%u — clearing counter "
                        "and rebooting",
                        reset_udp.remoteIP().toString().c_str(),
                        reset_udp.remotePort());
          safeModeClearCounter();
          vTaskDelay(pdMS_TO_TICKS(200));
          ESP.restart();
        }
      }
    }
    uint32_t now = millis();
    if (now - last_status >= 5000) {
      last_status = now;
      const char* wifi_str =
          wifi.isConnected() ? wifi.getLocalIP().toString().c_str()
                             : "waiting";
      Debug::printf(Debug::Level::ERROR,
                    "[SafeMode] boot=%u reason=%s prev=%s wifi=%s — OTA or "
                    "UDP :%u reset to recover",
                    s_boot_count, resetReasonStr(s_reset_reason),
                    s_prev_reason[0] ? s_prev_reason : "n/a", wifi_str,
                    kSafeModeUdpPort);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

static void haltOnFail(const char* subsystem) {
  Debug::printf(Debug::Level::ERROR, "[Sat] %s init FAILED — halting",
                subsystem);
  while (true) vTaskDelay(portMAX_DELAY);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(500);

  uint8_t boots = safeModeTickOnBoot();
  Debug::printf(Debug::Level::INFO,
                "[Sat] Boot #%u  this_reason=%s  prev_reason=%s", boots,
                resetReasonStr(s_reset_reason),
                s_prev_reason[0] ? s_prev_reason : "n/a");
  if (boots >= kSafeModeThreshold) {
    runSafeMode();  // never returns
  }

  Debug::printf(Debug::Level::INFO, "[Sat] Seeker satellite booting…");

  // --- Blink heartbeat (onboard LED only — satellite has no LED chain) ---
  blink.beginThreadedPinned(2048, 1, kBlinkNormalMs, 1);

  // --- BLE debug (optional) ---
  auto& ble = Subsystem::BleDebugSubsystem::getInstance(ble_setup);
  if (ble.init()) {
    ble.beginThreadedPinned(8192, 2, 50, 0);
    Debug::printf(Debug::Level::INFO, "[Sat] BLE advertising as \"%s\"",
                  ble_setup.deviceName);
  } else {
    Debug::printf(Debug::Level::WARN,
                  "[Sat] BLE init failed — Serial-only debug");
  }

  // --- WiFi (+ ArduinoOTA handler) ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) haltOnFail("WiFi");
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Sat] WiFi connecting to \"%s\"",
                WIFI_SSID);

  // --- Camera (OV2640, MJPEG on :80) — non-fatal ---
  // Init runs inside the ThreadedSubsystem task after begin(); the subsystem
  // logs its own failure. We start it even if cam hardware is missing so the
  // mic half of the board still comes up.
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  cam.beginThreadedPinned(8192, 2, 5000, 1);

  // --- Mic (PDM on I2S_NUM_0, PCM on :81, Core 0) — non-fatal ---
  // Core-pinned to 0 so camera_task (Core 1) can't starve i2s_channel_read.
  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  mic.beginThreadedPinned(4096, 2, 5000, 0);

  safeModeScheduleClear();

  Debug::printf(Debug::Level::INFO, "[Sat] Boot complete");
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (wifi.isConnected()) {
    Debug::printf(Debug::Level::INFO,
                  "[Loop] wifi=CONNECTED  ip=%-15s  rssi=%d dBm",
                  wifi.getLocalIP().toString().c_str(), wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%s",
                  wifiStateStr(wifi.getState()));
  }
  delay(2000);
}
