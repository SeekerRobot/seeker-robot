/**
 * @file main_add/src/main.cpp
 * @brief Seeker Robot — incremental rebuild of the main integration sketch.
 *
 * Phase 1–6 scope (this file): safe-mode boot counter + Blink + LED chain +
 * WiFi/ArduinoOTA + Gyro (BNO085, I2C) + Battery
 * (ADC, non-fatal) + Lidar (LD14P on Serial2, Core 0) + Camera/Mic/Speaker
 * (S3 only: OV2640 MJPEG :80, PDM mic PCM :81, I2S speaker from host :8383)
 * + OLED (SSD1306, HTTP framebuffer from host :8390) + Servo (PCA9685)
 * + HexapodKinematics + GaitController (arms + stands neutral at boot)
 * + MicroRosBridge (heartbeat, gyro, battery, lidar, debug log) +
 * MicrorosManager (WiFi UDP transport to agent :8888) + GaitRosParticipant
 * (/cmd_vel → gait) + diagnostics loop.
 *
 * Boot sequence:
 *   safeModeTickOnBoot() → (safe mode? → runSafeMode(), never returns)
 *   → blink → LEDs (rainbow pulse) → WiFi → schedule 30 s safe-mode clear
 *   → loop() flips LEDs on WiFi state + logs every 2 s
 *
 * Safe-mode recovery: 5 consecutive boots without a 30 s healthy run →
 * magenta chase LEDs + WiFi + ArduinoOTA only. Reflash over WiFi to clear.
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
// NOTE: CameraSubsystem.h (→ esp_camera.h → sensor.h) must come before
// GyroSubsystem.h (→ Adafruit_BNO08x.h → Adafruit_Sensor.h). Both headers
// define `sensor_t`; Adafruit guards its typedef with __SENSOR_H__ (the
// esp32-camera guard), so esp32-camera has to win the race.
#ifdef ENV_ESP32S3SENSE
#include <CameraSubsystem.h>
#include <MicSubsystem.h>
#include <SpeakerSubsystem.h>
#include <camera_pins.h>
#endif
#include <ESP32WifiSubsystem.h>
#include <GaitController.h>
#include <GaitRosParticipant.h>
#include <GyroSubsystem.h>
#include <HexapodConfig.h>
#include <HexapodKinematics.h>
#include <LedSubsystem.h>
#include <LidarSubsystem.h>
#include <MicroRosBridge.h>
#include <OledSubsystem.h>
#include <Preferences.h>
#include <RobotConfig.h>
#include <ServoSubsystem.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_system.h>
#include <hal_thread.h>
#include <microros_manager_robot.h>

// ---------------------------------------------------------------------------
// Network config — injected from network_config.ini via platformio.ini
// ---------------------------------------------------------------------------
static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;
static IPAddress agent_ip(AGENT_IP);

// ---------------------------------------------------------------------------
// Battery — 3S LiPo calibration lifted from src/main
// ---------------------------------------------------------------------------
static constexpr Subsystem::BatteryCalibration kBattCalibration(
    /*raw_lo=*/2432, /*volt_lo=*/11.52f,
    /*raw_hi=*/2648, /*volt_hi=*/12.62f);

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

// M-port ↔ HexapodConfig::kServoConfigs index (lifted from test_sub_movement).
// Index 0 is unused (M-ports start at 1); index 13 has no physical header.
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
// and gait tuning carry over to this sketch without re-entering them.
static constexpr char kPrefsNs[] = "srvtest";
static constexpr char kPrefsCfgKey[] = "cfg";
static constexpr char kPrefsBudgetKey[] = "budget";
static constexpr char kPrefsStepHKey[] = "g_step_h";
static constexpr char kPrefsCycleKey[] = "g_cycle";
static constexpr char kPrefsScaleKey[] = "g_scale";
static constexpr char kPrefsHeightKey[] = "m_height";
static Preferences prefs;

// Filled in during setup(); consumed by servo/gait bring-up.
static Subsystem::ServoConfig servo_configs[13];
static float body_height_mm = 0.0f;

// ---------------------------------------------------------------------------
// Globals — constructed at static init
// ---------------------------------------------------------------------------
static Threads::Mutex i2c_mutex;  // shared by Gyro + Servo + OLED

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::LedSetup led_setup(/*num_leds=*/5);
static Subsystem::LedSubsystem<Config::rgb_data> leds(led_setup);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("wifi", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "main_add_node");
static Subsystem::MicrorosManager manager(manager_setup);

static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);
static Subsystem::BatterySetup battery_setup(Config::batt, kBattCalibration,
                                             /*num_samples=*/16);
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);

static Subsystem::OledSetup oled_setup(i2c_mutex, agent_ip, 8390);

#ifdef ENV_ESP32S3SENSE
static Subsystem::CameraSetup cam_setup(camera_config, /*port=*/80);
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
// returns — loops forever broadcasting status every 5 s.
[[noreturn]] static void runSafeMode() {
  Debug::printf(Debug::Level::ERROR,
                "[SafeMode] %u consecutive boots (prev_reason=%s, "
                "this_reason=%s) — skipping hardware bring-up, awaiting OTA.",
                s_boot_count, s_prev_reason[0] ? s_prev_reason : "n/a",
                resetReasonStr(s_reset_reason));

  blink.beginThreadedPinned(2048, 1, 500, 1);

  if (leds.init()) {
    leds.beginThreadedPinned(2048, 2, 20, 1);
    leds.setBrightness(128);
    // Distinct safe-mode signal — magenta chase, easy to tell apart from
    // any normal-mode LED state.
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
    // WiFi subsystem calls ArduinoOTA.begin()/handle() internally when
    // ENABLE_ARDUINO_OTA=1, so once this task connects OTA is live.
    wifi.beginThreadedPinned(4096, 3, 100, 1);
  }

  // UDP "touch" escape — any packet on kSafeModeUdpPort clears the NVS
  // counter and reboots so the next start comes up in normal mode without
  // requiring OTA. Recover from the host with:
  //     echo clear | nc -u -w1 <static_ip> 8889
  WiFiUDP sm_udp;
  bool udp_bound = false;

  uint32_t last_status = 0;
  while (true) {
    // Bind lazily once WiFi is up (UDP bind before association silently
    // fails on some ESP32 stacks).
    if (!udp_bound && wifi.isConnected()) {
      udp_bound = sm_udp.begin(kSafeModeUdpPort);
      Debug::printf(Debug::Level::INFO,
                    "[SafeMode] UDP escape listening on :%u (%s)",
                    kSafeModeUdpPort, udp_bound ? "ok" : "BIND FAILED");
    }

    if (udp_bound && sm_udp.parsePacket() > 0) {
      // Drain the packet — contents don't matter, arrival is the signal.
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
// Servo / gait config — build factory defaults, then overlay any NVS-saved
// calibration. Logic is copied verbatim from test_sub_movement so this sketch
// picks up the same "srvtest" namespace transparently.
// ---------------------------------------------------------------------------
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
  // Override leg_servo_*[] to index into our 13-entry M-port layout.
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
  prefs.end();
  return have_cfg;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void haltOnFail(const char* subsystem) {
  Debug::printf(Debug::Level::ERROR, "[Main] %s init FAILED — halting",
                subsystem);
  while (true) vTaskDelay(portMAX_DELAY);
}

static const char* wifiStateStr(Subsystem::WifiState s) {
  switch (s) {
    case Subsystem::WifiState::DISCONNECTED:
      return "DISCONNECTED";
    case Subsystem::WifiState::CONNECTING:
      return "CONNECTING";
    case Subsystem::WifiState::CONNECTED:
      return "CONNECTED";
    case Subsystem::WifiState::RECONNECTING:
      return "RECONNECTING";
    case Subsystem::WifiState::FAILED:
      return "FAILED";
  }
  return "UNKNOWN";
}

// Loop-side LED FSM — flips the chain between "waiting for WiFi" and
// "idle/connected" effects. StatusLedController will take over in a later
// phase once MicrorosManager exists; palette choices here mirror
// StatusLedController::applyState so the visual language stays consistent.
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
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(
      2000);  // long delay to allow serial monitors to attach before any output

  // --- Safe-mode arbitration (must run before any risky bring-up) ---
  uint8_t boots = safeModeTickOnBoot();
  Debug::printf(Debug::Level::INFO,
                "[Main] Boot #%u  this_reason=%s  prev_reason=%s", boots,
                resetReasonStr(s_reset_reason),
                s_prev_reason[0] ? s_prev_reason : "n/a");
  if (boots >= kSafeModeThreshold) {
    runSafeMode();  // never returns
  }

  Debug::printf(Debug::Level::INFO, "[Main] Seeker Robot (main_add) booting…");

  // --- Blink heartbeat (onboard LED_BUILTIN) ---
  blink.beginThreadedPinned(2048, 1, 500, 1);

  // --- LED chain — come up immediately so boot state is visible ---
  if (!leds.init()) haltOnFail("LED");
  leds.beginThreadedPinned(2048, 2, 20, 1);
  leds.setBrightness(96);
  leds.setEffect(Subsystem::LedMode::RAINBOW_PULSE, CRGB::White, 200);
  s_led_phase = LedPhase::BOOT;
  s_boot_phase_until_ms = millis() + 2000;  // hold BOOT effect ≥2 s

#ifdef ENV_ESP32S3SENSE
  // --- Camera / Mic / Speaker HW init — ALL DMA-heavy and must run BEFORE
  //     BLE/WiFi so internal DRAM is still unfragmented. HTTP endpoints are
  //     deferred to beginThreadedPinned() later because they need lwIP.
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  if (!cam.init()) {
    Debug::printf(Debug::Level::WARN,
                  "[Main] Camera HW init failed — MJPEG stream disabled");
  }

  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  if (!mic.init()) {
    Debug::printf(Debug::Level::WARN,
                  "[Main] Mic HW init failed — audio stream disabled");
  }

  static Subsystem::SpeakerSetup speaker_setup(
      I2S_NUM_1, 16000, Config::spk_bclk, Config::spk_lrclk, Config::spk_dout,
      agent_ip, 8383, 2048);
  auto& spk = Subsystem::SpeakerSubsystem::getInstance(speaker_setup);
  if (!spk.init()) {
    Debug::printf(Debug::Level::WARN,
                  "[Main] Speaker HW init failed — playback disabled");
  }
#endif

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) haltOnFail("WiFi");
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi connecting to \"%s\"",
                WIFI_SSID);

  // --- Gyro (BNO085 on I2C, fatal) ---
  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  if (!gyro.init()) haltOnFail("Gyro");
  gyro.beginThreadedPinned(4096, 5, 0, 1);

  // --- Battery (ADC, non-fatal so bench runs without the 3S pack work) ---
  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  Subsystem::BatterySubsystem* batt_ptr = nullptr;
  if (batt.init()) {
    batt.beginThreadedPinned(4096, 2, 50, 1);
    batt_ptr = &batt;
  } else {
    Debug::printf(Debug::Level::WARN,
                  "[Main] Battery init failed — voltage telemetry disabled");
  }

  // --- Lidar (LD14P on Serial2, Core 0, 1 ms cadence) ---
  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) haltOnFail("Lidar");
  lidar.beginThreadedPinned(6144, 4, 1, 0);

#ifdef ENV_ESP32S3SENSE
  // --- Start Camera/Mic/Speaker tasks — WiFi is up, lwIP is ready for their
  //     HTTP endpoints; I2S/DMA was already claimed in their init() above.
  cam.beginThreadedPinned(8192, 2, 5000, 1);
  mic.beginThreadedPinned(4096, 2, 5000, 0);
  spk.beginThreadedPinned(8192, 2, 100, 1);
#endif  // ENV_ESP32S3SENSE

  // --- OLED (SSD1306, HTTP fetch from host :8390) ---
  auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);
  if (!oled.init()) haltOnFail("OLED");
  oled.beginThreadedPinned(4096, 2, 100, 1);
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "main_add");
  oled.setOverlay(1, 4, 52, "connecting...");

  // --- Movement: servo configs (defaults + NVS overlay from "srvtest" ns) ---
  float budget = kDefaultBudget;
  Gait::GaitConfig gc;
  buildDefaults(budget, gc, body_height_mm);
  bool cfg_from_prefs = loadFromPrefs(budget, gc, body_height_mm);
  Debug::printf(Debug::Level::INFO, "[Main] Servo config: %s",
                cfg_from_prefs ? "NVS (srvtest)" : "defaults");

  // --- Servo subsystem (PCA9685 on I2C, 13-entry M-port layout). Attach
  //     M1..M12 so GaitController::begin() can standNeutral() + arm().
  static Subsystem::ServoSetup servo_setup(Wire, Config::pca_addr,
                                           Config::servo_en, servo_configs, 13,
                                           budget, kDefaultFreqHz);
  auto& servos = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  for (uint8_t m = 1; m <= 12; m++) servos.attach(m - 1);
  if (!servos.init()) haltOnFail("Servo");
  servos.beginThreadedPinned(4096, 2, 10, 1);

  // --- Kinematics (sync, no task) ---
  static Kinematics::HexapodKinematics kin(HexapodConfig::kLegConfigs);

  // --- GaitController — begin() auto-arms + moves to standNeutral. Sits in
  //     IDLE afterward; walking is enable()'d by a future /cmd_vel bridge.
  static Gait::GaitSetup gait_setup(gc, &kin, &servos);
  static Gait::GaitController gait_ctrl(gait_setup);
  if (!gait_ctrl.init()) haltOnFail("Gait");
  gait_ctrl.beginThreadedPinned(8192, 2, 10, 1);

  // If a body height was saved in NVS, apply it once gait begin() has had a
  // chance to run its own standNeutral + arm.
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

  // --- MicroRosBridge — single owner of heartbeat/gyro/battery/lidar/debug
  //     publishers. Subsystems stay ROS-unaware; the bridge reads their
  //     thread-safe getters at the cadences it owns.
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = &gyro;
  bridge_setup.battery = batt_ptr;
  bridge_setup.lidar = &lidar;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  // --- GaitRosParticipant — subscribes to /cmd_vel and forwards the twist
  //     to gait_ctrl.setVelocity(), auto-enabling out of dead-band.
  static Gait::GaitRosParticipantSetup gait_ros_setup{&gait_ctrl};
  static Gait::GaitRosParticipant gait_ros(gait_ros_setup);

  manager.registerParticipant(&bridge);
  manager.registerParticipant(&gait_ros);
  if (!manager.init()) haltOnFail("MicroRosManager");
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  manager.beginThreadedPinned(8192, 4, 10, 1);

  // --- Safe-mode watchdog clear — after kStableRunMs of uptime without
  //     rebooting, wipe the NVS boot counter so only NEW crashes count.
  safeModeScheduleClear();

  Debug::printf(Debug::Level::INFO, "[Main] Phase-6 boot complete");
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);

  // LED phase FSM — hold BOOT for a short fuse, then track WiFi state.
  if (millis() >= s_boot_phase_until_ms) {
    ledPhaseApply(wifi.isConnected() ? LedPhase::IDLE : LedPhase::WIFI_WAIT);
  }

  if (wifi.isConnected()) {
    Debug::printf(
        Debug::Level::INFO,
        "[Loop] wifi=CONNECTED  microros=%-18s  ip=%-15s  rssi=%d dBm",
        manager.getStateStr(), wifi.getLocalIP().toString().c_str(),
        wifi.getRSSI());
  } else {
    Debug::printf(Debug::Level::INFO, "[Loop] wifi=%-12s  microros=%s",
                  wifiStateStr(wifi.getState()), manager.getStateStr());
  }

  delay(2000);
}
