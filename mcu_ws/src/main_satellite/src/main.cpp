/**
 * @file main_satellite/src/main.cpp
 * @brief Seeker Robot — satellite sketch (camera offload board).
 *
 * Runs on either an AI-Thinker ESP32-CAM (default, `esp32cam_satellite`) or a
 * second XIAO ESP32-S3 Sense (`esp32s3sense_satellite`). The main board runs
 * with ENABLE_CAM=0 (`esp32s3sense_offload` env) so the MJPEG endpoint moves
 * here while mic, speaker, OLED, and gait stay on the main sketch. Gyro is
 * disabled across both boards for now (ENABLE_GYRO=0 everywhere).
 *
 * Motivation: camera MJPEG on the main ESP32-S3 competes with micro-ROS, mic
 * PCM, speaker TTS, OLED framebuffer, and gait control for the internal-SRAM
 * PBUF pool and starts dropping packets (sendPacket() errno 12) under
 * concurrent load. Moving the MJPEG server onto its own board removes the
 * largest single PBUF consumer.
 *
 * Endpoints (unchanged from earlier satellite revisions — host-side nodes
 * just point at this board's static IP):
 *   GET http://<SATELLITE_IP>:80/cam    — MJPEG video stream
 *
 * micro-ROS topics (when BRIDGE_ENABLE_* flags set in the env):
 *   /mcu/heartbeat2  — satellite liveness (main publishes heartbeat1)
 *
 * Compile-time hardware gates (set per env in platformio.ini):
 *   ENABLE_CAM   — OV2640 MJPEG server (default 1)
 *   ENABLE_MIC   — PDM microphone PCM server (default 0 — mic lives on main)
 *   ENABLE_GYRO  — BNO085 IMU (default 1; off on both satellite envs for now)
 * Each maps to a `#if ENABLE_FOO` block below; set to 0 via -D to skip.
 *
 * Production bring-up:
 *   - Safe-mode arbitration: 5 consecutive PANIC/WDT boots in under 30 s
 *     drops the sketch into WiFi + OTA + UDP reset listener on :4211,
 *     skipping every subsystem so bricked firmware can be recovered OTA.
 *   - WiFi static-IP bring-up with ArduinoOTA handler.
 *   - Onboard LED: 500 ms toggle in normal mode, 100 ms (~5 Hz) in safe mode.
 *
 * Network config — expects SATELLITE_IP, WIFI_SSID, WIFI_PASSWORD, GATEWAY,
 * SUBNET, AGENT_IP, AGENT_PORT in network_config.ini.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <Preferences.h>
#include <RobotConfig.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <hal_thread.h>

// ---------------------------------------------------------------------------
// Hardware enable flags — default values for when the env doesn't set them.
// ESP32-CAM has no onboard mic, so ENABLE_MIC must stay 0 on that board.
// ---------------------------------------------------------------------------
#ifndef ENABLE_CAM
#define ENABLE_CAM 1
#endif
#ifndef ENABLE_MIC
#define ENABLE_MIC 0
#endif
#ifndef ENABLE_GYRO
#define ENABLE_GYRO 1
#endif
#ifndef ENABLE_MICROROS
#define ENABLE_MICROROS 1
#endif

#if ENABLE_CAM
#include <CameraSubsystem.h>
#include <camera_pins.h>
#endif
#if ENABLE_MIC
#include <MicSubsystem.h>
#endif
#if ENABLE_GYRO
#include <GyroSubsystem.h>
#endif

#if ENABLE_MICROROS
#include <MicroRosBridge.h>
#include <microros_manager_robot.h>
#endif

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

// UDP reset channel — same magic string as the primary main sketch, on a
// distinct port so the two boards can be reset independently. Main uses 4210,
// satellite uses 4211 (overridden via -DSAFE_MODE_UDP_PORT in platformio.ini).
// Trigger: `echo -n "SEEKER_CLEAR_SM" | nc -u -w1 <SAT_IP> <port>`.
#ifndef SAFE_MODE_UDP_PORT
#define SAFE_MODE_UDP_PORT 4211
#endif
static constexpr uint16_t kSafeModeUdpPort = SAFE_MODE_UDP_PORT;
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
      "sm_clear", 4096, nullptr, 1, nullptr, 1);
}

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
// BlinkSubsystem's update() toggles LED_BUILTIN once per update() call, so
// the value passed to beginThreadedPinned() *is* the toggle period. 500 ms
// is the normal-operation heartbeat; safe mode re-starts blink at 100 ms.
static constexpr uint32_t kBlinkNormalMs = 500;
static constexpr uint32_t kBlinkSafeModeMs = 100;

static Threads::Mutex i2c_mutex;  // shared by gyro (BNO085) on Wire

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "sat_wifi", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

#if ENABLE_MICROROS
static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "seeker_satellite_node");
static Subsystem::MicrorosManager manager(manager_setup);
#endif

#if ENABLE_CAM
// Stream cap stays at 20 Hz (50 ms) — well below the OV2640's ceiling but
// matches the main board for consumer-side parity.
static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80,
                                        /*ctrl_port=*/32768,
                                        /*frame_interval_ms=*/1,
                                        /*jpeg_quality=*/10,
                                        /*frame_size=*/FRAMESIZE_VGA);
#endif

#if ENABLE_MIC
static Subsystem::MicSetup mic_setup(I2S_NUM_0,
                                     /*sample_rate=*/16000,
                                     /*clk_pin=*/Config::pdm_clk,
                                     /*data_pin=*/Config::pdm_data,
                                     /*gain=*/4,
                                     /*http_port=*/81);
#endif

#if ENABLE_GYRO
// Pass SDA/SCL explicitly so GyroSubsystem::init() pins Wire to the
// SD-slot lines instead of the ESP32-CAM's default 21/22, which are the
// camera's Y5 and PCLK. The S3-Sense variant's default Wire pins already
// match Config::sda/scl, but pinning explicitly everywhere is harmless and
// keeps the two envs symmetric.
static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int, Config::sda,
                                       Config::scl);
#endif

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

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR,
                  "[SafeMode] WiFi init failed — Serial-only, no OTA");
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
  delay(2000);

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

#if ENABLE_CAM
  // --- Camera: hardware init BEFORE WiFi ---
  // esp_camera_init() claims a 16 KB internal-DRAM DMA buffer that has to be
  // contiguous; running after WiFi-driver heap fragmentation often fails with
  // ESP_ERR_NO_MEM. The task thread (started later) only starts the HTTP
  // server, which needs lwIP up.
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  if (!cam.init()) {
    Debug::printf(Debug::Level::WARN,
                  "[Sat] Camera init failed — /cam endpoint disabled");
  }
#endif

  // --- WiFi (+ ArduinoOTA handler) ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) haltOnFail("WiFi");
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Sat] WiFi connecting to \"%s\"",
                WIFI_SSID);

#if ENABLE_GYRO
  // --- Gyro (BNO085 on I2C, non-fatal if it fails) ---
  // gyro_setup carries the SDA/SCL pins, so GyroSubsystem::init() pins Wire
  // itself — pinning here too would just get overridden.
  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  const bool gyro_ready = gyro.init();
  if (gyro_ready) {
    gyro.beginThreadedPinned(4096, 5, 0, 1);
  } else {
    Debug::printf(Debug::Level::WARN,
                  "[Sat] Gyro init failed — IMU topic disabled");
  }
#endif

#if ENABLE_CAM
  // --- Camera HTTP server (begin() starts the MJPEG server on :80) ---
  // WiFi is up by now, so lwIP is ready for httpd_start. begin() no-ops if
  // the earlier init() failed, keeping the rest of the board alive.
  cam.beginThreadedPinned(8192, 2, 5000, 1);
#endif

#if ENABLE_MIC
  // --- Mic (PDM on I2S_NUM_0, PCM on :81, Core 0) — non-fatal ---
  // Core-pinned to 0 so camera_task (Core 1) can't starve i2s_channel_read.
  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  mic.beginThreadedPinned(4096, 2, 5000, 0);
#endif

#if ENABLE_MICROROS
  // --- MicroRosBridge (gyro → /mcu/imu, heartbeat → /mcu/heartbeat2) ---
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.heartbeat_topic = "mcu/heartbeat2";
#if ENABLE_GYRO
  bridge_setup.gyro = gyro_ready ? &gyro : nullptr;
#endif
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  // --- MicrorosManager ---
  manager.registerParticipant(&bridge);
  if (!manager.init()) haltOnFail("MicroRosManager");
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Sat] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  manager.beginThreadedPinned(8192, 3, 10, 1);
#else
  Debug::printf(Debug::Level::INFO,
                "[Sat] micro-ROS disabled (ENABLE_MICROROS=0)");
#endif

  safeModeScheduleClear();

  Debug::printf(Debug::Level::INFO, "[Sat] Boot complete");
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
#if ENABLE_MICROROS
  const char* uros_state = manager.getStateStr();
#else
  const char* uros_state = "DISABLED";
#endif
  if (wifi.isConnected()) {
    Debug::printf(
        Debug::Level::INFO,
        "[Loop] wifi=CONNECTED  microros=%-18s  ip=%-15s  rssi=%d dBm",
        uros_state, wifi.getLocalIP().toString().c_str(), wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%-12s  microros=%s",
                  wifiStateStr(wifi.getState()), uros_state);
  }
  delay(2000);
}
