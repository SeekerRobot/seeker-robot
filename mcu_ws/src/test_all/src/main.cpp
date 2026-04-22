/**
 * @file test_all/src/main.cpp
 * @author Tal Avital
 * @date 4/7/2026
 * @brief Full integration test: micro-ROS bridge + MJPEG camera stream + PDM
 *        audio stream + I2S speaker, all running concurrently on the Seeed
 *        XIAO ESP32-S3 Sense.
 *
 *   GET http://<static_ip>:80/cam    — MJPEG video stream
 *   GET http://<static_ip>:81/audio  — raw 16-bit PCM, 16kHz, mono (mic)
 *
 * Verify on host:
 *   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 *   ros2 topic echo /mcu/heartbeat
 *   ros2 topic hz   /mcu/imu        # ~50 Hz
 *   ros2 topic echo /mcu/battery_voltage
 *   ros2 topic hz   /mcu/scan       # ~6 Hz
 *   ros2 topic echo /mcu/log
 *
 * Speaker / TTS (seeker_tts node on host, port 8383):
 *   FISH_API_KEY=xxx ros2 run seeker_tts tts_node
 *   ros2 topic pub /audio_tts_input std_msgs/String "data: 'hello'" --once
 *   ros2 topic pub /audio_play_file std_msgs/String "data:
 * '/path/to/sound.wav'" --once
 */

// Compile-time subsystem gates. Default ON; flip to 0 via -DENABLE_X=0 in
// platformio.ini build_flags to binary-search PBUF starvation triage.
#ifndef ENABLE_CAM
#define ENABLE_CAM 1
#endif
#ifndef ENABLE_MIC
#define ENABLE_MIC 1
#endif
#ifndef ENABLE_SPK
#define ENABLE_SPK 1
#endif
#ifndef ENABLE_OLED
#define ENABLE_OLED 1
#endif

#include <Arduino.h>
#include <BatterySubsystem.h>
#if ENABLE_BLE_DEBUG
#include <BleDebugSubsystem.h>
#endif
#include <BlinkSubsystem.h>
#if ENABLE_CAM
#include <CameraSubsystem.h>
#endif
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <GaitController.h>
#include <GaitRosParticipant.h>
#include <GyroSubsystem.h>
#include <HexapodConfig.h>
#include <HexapodKinematics.h>
#include <LedSubsystem.h>
#include <LidarSubsystem.h>
#if ENABLE_MIC
#include <MicSubsystem.h>
#endif
#include <MicroRosBridge.h>
#if ENABLE_OLED
#include <OledSubsystem.h>
#endif
#include <Preferences.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>
#if ENABLE_SPK
#include <SpeakerSubsystem.h>
#endif
#include <WiFiUdp.h>
#include <Wire.h>
#if ENABLE_CAM
// camera_pins.h pulls in esp_camera.h → sensor.h which typedefs sensor_t,
// conflicting with Adafruit_Sensor.h used by the BNO085 gyro driver. Only
// needed when the camera is actually being built.
#include <camera_pins.h>
#endif
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;
static IPAddress agent_ip(AGENT_IP);

static constexpr Subsystem::BatteryCalibration kBattCalibration(
    /*raw_lo=*/2432, /*volt_lo=*/11.52f,
    /*raw_hi=*/2648, /*volt_hi=*/12.62f);

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

#if ENABLE_BLE_DEBUG
static Subsystem::BleDebugSetup ble_setup("SeekerDebug");
#endif

static Subsystem::LedSetup led_setup(/*num_leds=*/5);
static Subsystem::LedSubsystem<Config::rgb_data> leds(led_setup);

static Threads::Mutex i2c_mutex;

static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);
static Subsystem::BatterySetup battery_setup(Config::batt, kBattCalibration,
                                             /*num_samples=*/16);
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);
#if ENABLE_OLED
static Subsystem::OledSetup oled_setup(i2c_mutex, agent_ip, 8390);
#endif
static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "test_all_node");
static Subsystem::MicrorosManager manager(manager_setup);

#if ENABLE_CAM
static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80);
#endif
#if ENABLE_MIC
static Subsystem::MicSetup mic_setup(I2S_NUM_0,
                                     /*sample_rate=*/16000,
                                     /*clk_pin=*/Config::pdm_clk,
                                     /*data_pin=*/Config::pdm_data,
                                     /*gain=*/4,
                                     /*http_port=*/81);
#endif

// ---------------------------------------------------------------------------
// Safe mode — after N consecutive boots without a kStableRunMs "healthy run"
// the sketch skips every risky subsystem and comes up in LED+WiFi+OTA mode
// so the robot can be reflashed over the air. WiFi subsystem handles the
// ArduinoOTA handler itself (ENABLE_ARDUINO_OTA=1 in platformio.ini).
// ---------------------------------------------------------------------------
static constexpr uint8_t kSafeModeThreshold = 5;
static constexpr uint32_t kStableRunMs = 10000;
static constexpr uint16_t kSafeModeUdpPort = 8889;
static constexpr char kSafeModeNs[] = "seeker_sm";
static constexpr char kSafeModeCount[] = "count";
static constexpr char kSafeModeReason[] = "last_reason";

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXT_RESET";
    case ESP_RST_SW:
      return "SW_RESTART";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    default:
      return "UNKNOWN";
  }
}

static uint8_t s_boot_count = 0;
static esp_reset_reason_t s_reset_reason = ESP_RST_UNKNOWN;
static char s_prev_reason[16] = {};

// Records this boot in NVS and returns the new consecutive-boot count. Must
// run very early in setup() — before any subsystem that could crash.
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

// One-shot task: after kStableRunMs of uptime without a reboot, clear the
// counter — signals that the firmware is trustworthy.
static void safeModeScheduleClear() {
  xTaskCreatePinnedToCore(
      [](void*) {
        vTaskDelay(pdMS_TO_TICKS(kStableRunMs));
        safeModeClearCounter();
        Debug::printf(Debug::Level::INFO,
                      "[Main] Firmware stable for %u ms — cleared boot count",
                      (unsigned)kStableRunMs);
        vTaskDelete(nullptr);
      },
      "sm_clear", 2048, nullptr, 1, nullptr, 1);
}

// Enter safe mode: Blink + LEDs (magenta chase) + WiFi with OTA. Never
// returns — loops forever broadcasting status every 5 s. Recover from host:
//     echo clear | nc -u -w1 <static_ip> 8889
[[noreturn]] static void runSafeMode() {
  Debug::printf(Debug::Level::ERROR,
                "[SafeMode] %u consecutive boots (prev_reason=%s, "
                "this_reason=%s) — skipping hardware bring-up, awaiting OTA.",
                s_boot_count, s_prev_reason[0] ? s_prev_reason : "n/a",
                resetReasonStr(s_reset_reason));

  blink.beginThreadedPinned(2048, 2, 500, 1);

  if (leds.init()) {
    leds.beginThreadedPinned(2048, 2, 20, 1);
    leds.setBrightness(128);
    leds.setEffect(Subsystem::LedMode::CHASE, CRGB::Magenta, 180);
  } else {
    Debug::printf(Debug::Level::WARN,
                  "[SafeMode] LED init failed — blink-only indicator");
  }

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR,
                  "[SafeMode] WiFi init failed — no OTA path");
  } else {
    wifi.beginThreadedPinned(4096, 3, 100, 1);
  }

  WiFiUDP sm_udp;
  bool udp_bound = false;
  uint32_t last_status = 0;
  while (true) {
    if (!udp_bound && wifi.isConnected()) {
      udp_bound = sm_udp.begin(kSafeModeUdpPort);
      Debug::printf(Debug::Level::INFO,
                    "[SafeMode] UDP escape listening on :%u (%s)",
                    kSafeModeUdpPort, udp_bound ? "ok" : "BIND FAILED");
    }
    if (udp_bound && sm_udp.parsePacket() > 0) {
      while (sm_udp.available()) sm_udp.read();
      safeModeClearCounter();
      Debug::printf(Debug::Level::ERROR,
                    "[SafeMode] UDP escape received — cleared counter, "
                    "rebooting in 250 ms");
      delay(250);
      ESP.restart();
    }
    uint32_t now = millis();
    if (now - last_status >= 5000) {
      last_status = now;
      String wifi_str =
          wifi.isConnected() ? wifi.getLocalIP().toString() : String("waiting");
      Debug::printf(Debug::Level::ERROR,
                    "[SafeMode] boot=%u reason=%s prev=%s wifi=%s — OTA or "
                    "`nc -u <ip> %u` to recover",
                    s_boot_count, resetReasonStr(s_reset_reason),
                    s_prev_reason[0] ? s_prev_reason : "n/a", wifi_str.c_str(),
                    kSafeModeUdpPort);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ---------------------------------------------------------------------------
// LED phase FSM — flips the chain between BOOT / WIFI_WAIT / IDLE effects.
// Palette mirrors StatusLedController::applyState so visuals stay consistent
// when the controller is added later.
// ---------------------------------------------------------------------------
enum class LedPhase : uint8_t { BOOT, WIFI_WAIT, IDLE };
static LedPhase s_led_phase = LedPhase::BOOT;
static uint32_t s_boot_phase_until_ms = 0;

static void ledPhaseApply(LedPhase next) {
  if (next == s_led_phase) return;
  switch (next) {
    case LedPhase::BOOT:
      leds.setEffect(Subsystem::LedMode::RAINBOW_PULSE, CRGB::White, 200);
      break;
    case LedPhase::WIFI_WAIT:
      leds.setEffect(Subsystem::LedMode::CHASE, CRGB::Red, 256);
      break;
    case LedPhase::IDLE:
      leds.setEffect(Subsystem::LedMode::PULSE, CRGB::Cyan, 64);
      break;
  }
  s_led_phase = next;
}

// ---------------------------------------------------------------------------
// Servo / gait defaults (mirror test_sub_movement)
// ---------------------------------------------------------------------------
static constexpr uint16_t kDefaultMinPwm = 120;
static constexpr uint16_t kDefaultMaxPwm = 590;
static constexpr float kDefaultVelocity = 720.0f;
static constexpr float kDefaultAccel = 1000.0f;
static constexpr float kDefaultBudget = 10000.0f;
static constexpr float kDefaultFreqHz = 50.0f;
static constexpr float kDefaultTotalAngle = 180.0f;

// ---------------------------------------------------------------------------
// Stack high-water-mark telemetry. We keep a small table of our own
// beginThreadedPinned()'d task handles and their reserved stack sizes, then
// periodically log `uxTaskGetStackHighWaterMark()` for each — the minimum
// free stack ever seen. Lets us shrink oversized stacks back to what they
// actually use.
//
// `uxTaskGetStackHighWaterMark` units: BYTES on ESP-IDF/xtensa (StackType_t
// is uint8_t), not words. `uxTaskGetSystemState` would be simpler but needs
// CONFIG_FREERTOS_USE_TRACE_FACILITY, which arduino-esp32 2.0.17 ships off.
// ---------------------------------------------------------------------------
struct StackTrack {
  const char* name;
  TaskHandle_t handle;
  uint32_t reserved_bytes;
};

static constexpr size_t kMaxTrackedTasks = 16;
static StackTrack s_stack_tracks[kMaxTrackedTasks];
static size_t s_stack_track_count = 0;

static void trackStack(const char* name, TaskHandle_t h, uint32_t reserved) {
  if (!h || s_stack_track_count >= kMaxTrackedTasks) return;
  s_stack_tracks[s_stack_track_count++] = {name, h, reserved};
}

static void logTaskStackHighWater() {
  for (size_t i = 0; i < s_stack_track_count; i++) {
    const auto& t = s_stack_tracks[i];
    uint32_t free_bytes = uxTaskGetStackHighWaterMark(t.handle);
    uint32_t used =
        t.reserved_bytes > free_bytes ? t.reserved_bytes - free_bytes : 0;
    Debug::printf(Debug::Level::INFO,
                  "[StackHWM] %-10s  used=%5u / reserved=%5u B  free=%5u",
                  t.name, (unsigned)used, (unsigned)t.reserved_bytes,
                  (unsigned)free_bytes);
  }
}

// ---------------------------------------------------------------------------
// PBUF watchdog — 100 ms cadence on Core 1. Catches transient drops in
// internal / DMA heap that the 1 s [Loop] telemetry would miss. Fires when
// dma_largest < 8 KB or internal free < 20 KB, which is where lwIP tends to
// start failing sendto() with ENOMEM. Rate-limited to 2 Hz of log output.
// Thresholds come from the migration plan's "dma largest ≥ 20 KB" target.
// ---------------------------------------------------------------------------
static void pbufWatchdogTask(void*) {
  static uint32_t last_low_log_ms = 0;
  for (;;) {
    size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    size_t int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t now = millis();
    if ((dma_largest < 8192 || int_free < 20000) &&
        now - last_low_log_ms > 500) {
      Debug::printf(Debug::Level::WARN, "[PBUF-LOW] dma_largest=%u int_free=%u",
                    (unsigned)dma_largest, (unsigned)int_free);
      last_low_log_ms = now;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// M-port ↔ HexapodConfig::kServoConfigs index. Index 0 is unused (M-ports
// start at 1); index 13 has no physical header.
static constexpr uint8_t kMPortToHexIdx[14] = {
    255,  //  [0]  — unused
    0,    //  M1  → FL Hip
    4,    //  M2  → ML Hip
    8,    //  M3  → RL Hip
    10,   //  M4  → RR Hip
    6,    //  M5  → MR Hip
    2,    //  M6  → FR Hip
    11,   //  M7  → RR Knee
    1,    //  M8  → FL Knee
    5,    //  M9  → ML Knee
    9,    //  M10 → RL Knee
    7,    //  M11 → MR Knee
    3,    //  M12 → FR Knee
    255,  //  M13 — unassigned
};

// Leg index → ServoSubsystem index in the 13-entry M-port layout.
static constexpr uint8_t kLegServoHip[6] = {0, 5, 1, 4, 2, 3};
static constexpr uint8_t kLegServoKnee[6] = {7, 11, 8, 10, 9, 6};

// NVS namespace — shared with test_sub_servo / test_sub_movement so calibration
// and gait tuning carry over without re-entering them.
static constexpr char kPrefsNs[] = "srvtest";
static constexpr char kPrefsCfgKey[] = "cfg";
static constexpr char kPrefsBudgetKey[] = "budget";
static constexpr char kPrefsStepHKey[] = "g_step_h";
static constexpr char kPrefsCycleKey[] = "g_cycle";
static constexpr char kPrefsScaleKey[] = "g_scale";
static constexpr char kPrefsHeightKey[] = "m_height";
static constexpr char kPrefsMaxVxKey[] = "m_maxvx";
static constexpr char kPrefsMaxVyKey[] = "m_maxvy";
static constexpr char kPrefsMaxWzKey[] = "m_maxwz";
static constexpr char kPrefsMaxHvelKey[] = "m_maxhvel";
static Preferences prefs;

static Subsystem::ServoConfig servo_configs[13];
static float body_height_mm = 0.0f;

// Velocity caps — mirror test_sub_movement defaults so `ros2 topic pub`
// produces the same motion as its `forward/move` console commands. Any of
// these that are present in NVS (under the m_max* keys) override at boot.
static float max_vx = 0.15f;    // m/s
static float max_vy = 0.15f;    // m/s
static float max_wz = 1.00f;    // rad/s
static float max_hvel = 0.15f;  // m/s — combined |vx,vy| cap

static void buildDefaults(float& budget_out, Gait::GaitConfig& gc_out,
                          float& height_out) {
  budget_out = kDefaultBudget;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi != 255) {
      servo_configs[m - 1] = HexapodConfig::kServoConfigs[hi];
      const float total = servo_configs[m - 1].total_angle_deg;
      const float scale = (kDefaultMaxPwm - kDefaultMinPwm) / total;
      if (servo_configs[m - 1].min_angle < 0.0f) {
        const float center = (kDefaultMinPwm + kDefaultMaxPwm) / 2.0f;
        servo_configs[m - 1].min_pwm =
            (uint16_t)roundf(center + servo_configs[m - 1].min_angle * scale);
        servo_configs[m - 1].max_pwm =
            (uint16_t)roundf(center + servo_configs[m - 1].max_angle * scale);
      } else {
        servo_configs[m - 1].min_pwm = (uint16_t)roundf(
            kDefaultMinPwm + servo_configs[m - 1].min_angle * scale);
        servo_configs[m - 1].max_pwm = (uint16_t)roundf(
            kDefaultMinPwm + servo_configs[m - 1].max_angle * scale);
      }
    } else {
      servo_configs[m - 1] = {
          .channel = Config::mPort(m),
          .min_angle = 0.0f,
          .max_angle = kDefaultTotalAngle,
          .min_pwm = kDefaultMinPwm,
          .max_pwm = kDefaultMaxPwm,
          .inverted = false,
          .max_velocity = kDefaultVelocity,
          .max_accel = kDefaultAccel,
          .total_angle_deg = kDefaultTotalAngle,
      };
    }
  }

  gc_out = HexapodConfig::kGaitConfig;
  for (uint8_t i = 0; i < 6; i++) {
    gc_out.leg_servo_hip[i] = kLegServoHip[i];
    gc_out.leg_servo_knee[i] = kLegServoKnee[i];
  }
  height_out = 0.0f;
}

static bool loadFromPrefs(float& budget_out, Gait::GaitConfig& gc_out,
                          float& height_out) {
  prefs.begin(kPrefsNs, /*readOnly=*/true);
  bool have_cfg = prefs.isKey(kPrefsCfgKey);
  if (have_cfg) {
    size_t n =
        prefs.getBytes(kPrefsCfgKey, servo_configs, sizeof(servo_configs));
    if (n != sizeof(servo_configs)) have_cfg = false;
  }
  if (have_cfg) {
    budget_out = prefs.getFloat(kPrefsBudgetKey, kDefaultBudget);
  }
  gc_out.step_height_mm = prefs.getFloat(kPrefsStepHKey, gc_out.step_height_mm);
  gc_out.cycle_time_s = prefs.getFloat(kPrefsCycleKey, gc_out.cycle_time_s);
  gc_out.step_scale = prefs.getFloat(kPrefsScaleKey, gc_out.step_scale);
  height_out = prefs.getFloat(kPrefsHeightKey, 0.0f);
  max_vx = prefs.getFloat(kPrefsMaxVxKey, max_vx);
  max_vy = prefs.getFloat(kPrefsMaxVyKey, max_vy);
  max_wz = prefs.getFloat(kPrefsMaxWzKey, max_wz);
  max_hvel = prefs.getFloat(kPrefsMaxHvelKey, max_hvel);
  prefs.end();
  return have_cfg;
}

void setup() {
  Serial.begin(921600);
  delay(2000);

  if (psramFound()) {
    Debug::printf(Debug::Level::INFO, "[Main] PSRAM: %u bytes",
                  ESP.getPsramSize());
  } else {
    Debug::printf(Debug::Level::WARN, "[Main] PSRAM not found");
  }

  // --- Safe-mode arbitration (must run before any risky bring-up) ---
  uint8_t boots = safeModeTickOnBoot();
  Debug::printf(Debug::Level::INFO,
                "[Main] Boot #%u  this_reason=%s  prev_reason=%s", boots,
                resetReasonStr(s_reset_reason),
                s_prev_reason[0] ? s_prev_reason : "n/a");
  if (boots >= kSafeModeThreshold) {
    runSafeMode();  // never returns
  }

  trackStack("blink", blink.beginThreadedPinned(2048, 2, 500, 1), 2048);

  // --- LED chain — come up immediately so boot state is visible ---
  if (!leds.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] LED init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("leds", leds.beginThreadedPinned(2048, 2, 20, 1), 2048);
  leds.setBrightness(96);
  leds.setEffect(Subsystem::LedMode::RAINBOW_PULSE, CRGB::White, 200);
  s_led_phase = LedPhase::BOOT;
  s_boot_phase_until_ms = millis() + 2000;  // hold BOOT effect ≥2 s

#if ENABLE_BLE_DEBUG
  // BLE Nordic UART debug — init BEFORE WiFi/camera/I2S so the BT controller
  // (~10 KB internal DRAM funcs table) can allocate contiguous memory while
  // internal DRAM is still whole. Deferring to end-of-setup() causes
  // `BLE_INIT: Malloc failed / esp_bt_controller_init -2`.
  // NimBLE host allocations go to PSRAM via
  // CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL. Non-fatal: a failed init is
  // logged and ignored so the safe-mode counter still clears on the 10 s
  // stable-run timer at the end of setup().
  auto& ble = Subsystem::BleDebugSubsystem::getInstance(ble_setup);
  if (!ble.init()) {
    Debug::printf(Debug::Level::WARN,
                  "[Main] BleDebug init FAILED — continuing without BLE");
  } else {
    trackStack("ble", ble.beginThreadedPinned(8192, 2, 50, 0), 8192);
  }
#endif

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("wifi", wifi.beginThreadedPinned(4096, 3, 100, 1), 4096);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi started, connecting to \"%s\"",
                WIFI_SSID);

  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  if (!gyro.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gyro init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // 4096 B: arduino-esp32 3.x's BNO085 SH2 callback chain (wasReset ->
  // setReports -> sh2_setSensorConfig) uses noticeably more stack than 2.x.
  // 2048 B tripped the stack canary on first `wasReset()` after boot.
  trackStack("gyro", gyro.beginThreadedPinned(4096, 5, 0, 1), 4096);

  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  if (!batt.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Battery init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("batt", batt.beginThreadedPinned(4096, 2, 50, 1), 4096);

  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Lidar init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("lidar", lidar.beginThreadedPinned(4096, 4, 1, 0), 4096);

  // Order matters: Mic + Speaker I2S DMA descriptors must be allocated BEFORE
  // Camera grabs its framebuffers. esp_camera_init() consumes large chunks of
  // internal DMA-capable RAM and fragments the pool; if I2S allocates after,
  // i2s_alloc_dma_desc() fails (observed on IDF 5.3.2 / arduino-esp32 3.1.3).
  //
  // audioStreamTask is pinned to Core 0 internally — keeps I2S reads off
  // Core 1 where camera_task (from esp_camera_init) runs.
#if ENABLE_MIC
  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  if (!mic.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Mic init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("mic", mic.beginThreadedPinned(4096, 2, 5000, 1), 4096);
#else
  Debug::printf(Debug::Level::WARN, "[Main] Mic DISABLED (ENABLE_MIC=0)");
#endif

#if ENABLE_SPK
  // Speaker fetches PCM from seeker_tts on the host (AGENT_IP:8383). Uses
  // I2S_NUM_1 (mic uses I2S_NUM_0), pinned to Core 1. Host-side mic gating
  // during playback lives in seeker_tts (/audio_speaker_active).
  static Subsystem::SpeakerSetup speaker_setup(
      I2S_NUM_1, 16000, Config::spk_bclk, Config::spk_lrclk, Config::spk_dout,
      agent_ip, 8383, 2048);
  auto& spk = Subsystem::SpeakerSubsystem::getInstance(speaker_setup);
  if (!spk.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Speaker init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("spk", spk.beginThreadedPinned(4096, 2, 100, 1), 4096);
#else
  Debug::printf(Debug::Level::WARN, "[Main] Speaker DISABLED (ENABLE_SPK=0)");
#endif

#if ENABLE_CAM
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  if (!cam.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Camera init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("cam", cam.beginThreadedPinned(4096, 2, 5000, 1), 4096);
#else
  Debug::printf(Debug::Level::WARN, "[Main] Camera DISABLED (ENABLE_CAM=0)");
#endif

#if ENABLE_OLED
  auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);
  if (!oled.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] OLED init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  trackStack("oled", oled.beginThreadedPinned(4096, 2, 100, 1), 4096);
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "test_all");
  oled.setOverlay(1, 4, 52, "connecting...");
  Debug::printf(Debug::Level::INFO, "[Main] OLED started");
#else
  Debug::printf(Debug::Level::WARN, "[Main] OLED DISABLED (ENABLE_OLED=0)");
#endif

  float budget = kDefaultBudget;
  Gait::GaitConfig gc;
  buildDefaults(budget, gc, body_height_mm);
  bool cfg_from_prefs = loadFromPrefs(budget, gc, body_height_mm);
  Debug::printf(Debug::Level::INFO, "[Main] Servo config: %s",
                cfg_from_prefs ? "NVS (srvtest)" : "defaults");

  static Subsystem::ServoSetup servo_setup(Wire, Config::pca_addr,
                                           Config::servo_en, servo_configs, 13,
                                           budget, kDefaultFreqHz);
  auto& servos = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  for (uint8_t m = 1; m <= 12; m++) servos.attach(m - 1);
  if (!servos.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Servo init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // 25 Hz (40 ms). GaitController + ServoSubsystem both compute dt from
  // micros() deltas so motion speed is Hz-agnostic — lowering cadence
  // only relaxes how finely the velocity ramp is resampled. Prio 3 (below
  // micro-ROS manager) because brief preemption just shows up as jitter in
  // the next tick — not motion drift.
  trackStack("servos", servos.beginThreadedPinned(4096, 3, 40, 1), 4096);

  static Kinematics::HexapodKinematics kin(HexapodConfig::kLegConfigs);

  static Gait::GaitSetup gait_setup(gc, &kin, &servos);
  static Gait::GaitController gait_ctrl(gait_setup);
  if (!gait_ctrl.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Gait init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  // 25 Hz (40 ms). Gait integrates body pose + phase with micros()-derived
  // dt, so cycle_time_s and velocity setpoints are unchanged by this rate.
  // Prio 3 so micro-ROS manager (prio 4) isn't preempted by IK recompute.
  trackStack("gait", gait_ctrl.beginThreadedPinned(8192, 3, 40, 1), 8192);

  if (body_height_mm > 0.01f && body_height_mm < HexapodConfig::kL2) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (kin.setStandHeight(body_height_mm)) {
      Kinematics::SolveResult r = kin.standNeutral();
      for (uint8_t i = 0; i < Gait::kNumLegs; i++) {
        if (!r.valid[i]) continue;
        servos.setAngle(kLegServoHip[i], r.legs[i].hip);
        servos.setAngle(kLegServoKnee[i], r.legs[i].knee);
      }
      Debug::printf(Debug::Level::INFO, "[Main] Restored body height %.1f mm",
                    body_height_mm);
    }
  }

  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = &gyro;
  bridge_setup.battery = &batt;
  bridge_setup.lidar = &lidar;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  static Gait::GaitRosParticipantSetup gait_ros_setup{};
  gait_ros_setup.gait = &gait_ctrl;
  gait_ros_setup.max_vx = max_vx;
  gait_ros_setup.max_vy = max_vy;
  gait_ros_setup.max_wz = max_wz;
  gait_ros_setup.max_hvel = max_hvel;
  static Gait::GaitRosParticipant gait_ros(gait_ros_setup);

  manager.registerParticipant(&bridge);
  manager.registerParticipant(&gait_ros);
  if (!manager.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Manager init FAILED — halting");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  trackStack("manager", manager.beginThreadedPinned(6144, 4, 10, 1), 6144);

  // PBUF watchdog — pinned to Core 1 alongside lwIP / HTTP servers so it sees
  // the same heap the transmit path allocates from. Prio 2 so it can't starve
  // gait/micro-ROS. 4 KB stack: heap_caps_* walks multi-heap free-lists and
  // Debug::printf pulls in printf formatting — 2 KB triggers the canary on
  // arduino-esp32 3.x / IDF 5.x.
  xTaskCreatePinnedToCore(pbufWatchdogTask, "pbuf_wd", 4096, nullptr, 2,
                          nullptr, 1);

  // --- Safe-mode watchdog clear — after kStableRunMs of uptime without
  //     rebooting, wipe the NVS boot counter so only NEW crashes count.
  safeModeScheduleClear();
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);

  // LED phase FSM — hold BOOT for a short fuse, then track WiFi state.
  if (millis() >= s_boot_phase_until_ms) {
    ledPhaseApply(wifi.isConnected() ? LedPhase::IDLE : LedPhase::WIFI_WAIT);
  }

  // Heap snapshot — ENOMEM (errno 12) from sendPacket means lwIP couldn't
  // grab a PBUF. Watch internal free + min-ever + largest DMA block here to
  // correlate drops with streaming bursts / gait activity. PSRAM tracked too
  // so we can see if WiFi/lwIP fallback allocations are fragmenting PSRAM.
  size_t heap_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t heap_int_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  size_t heap_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
  size_t heap_dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
  size_t heap_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t heap_psram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
  size_t heap_psram_largest =
      heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

  if (wifi.isConnected()) {
    Debug::printf(
        Debug::Level::INFO,
        "[Loop] wifi=CONNECTED  microros=%-18s  ip=%-15s  rssi=%d dBm  "
        "heap_int=%u min=%u  dma=%u largest=%u  psram=%u min=%u largest=%u",
        manager.getStateStr(), wifi.getLocalIP().toString().c_str(),
        wifi.getRSSI(), (unsigned)heap_int, (unsigned)heap_int_min,
        (unsigned)heap_dma, (unsigned)heap_dma_largest, (unsigned)heap_psram,
        (unsigned)heap_psram_min, (unsigned)heap_psram_largest);
  } else {
    Debug::printf(
        Debug::Level::INFO,
        "[Loop] wifi=%-12s  microros=%s  heap_int=%u min=%u  dma=%u largest=%u"
        "  psram=%u min=%u largest=%u",
        wifi.getState() == Subsystem::WifiState::CONNECTING ? "CONNECTING"
                                                            : "DISCONNECTED",
        manager.getStateStr(), (unsigned)heap_int, (unsigned)heap_int_min,
        (unsigned)heap_dma, (unsigned)heap_dma_largest, (unsigned)heap_psram,
        (unsigned)heap_psram_min, (unsigned)heap_psram_largest);
  }

  // Stack HWM dump every ~10 s (loop cadence is 1 s now, so every 10 loops).
  static uint8_t hwm_tick = 0;
  if (++hwm_tick >= 10) {
    hwm_tick = 0;
    logTaskStackHighWater();
  }

  delay(1000);
}
