/**
 * @file main/src/main.cpp
 * @brief Seeker Robot — main integration sketch.
 *
 * Brings up every subsystem the MCU owns and drives the onboard SK6812 chain
 * as a robot-state indicator via StatusLedController.
 *
 * Subsystems (all threaded, most pinned to Core 1):
 *   Blink heartbeat, LED chain, WiFi, BNO085
 *   gyro, battery ADC, LD14P lidar (core 0), PCA9685 servos, HexapodKinematics,
 *   GaitController, OV2640 camera (MJPEG :80), PDM mic (PCM :81, core 0),
 *   I2S speaker (PCM from host :8383), SSD1306 OLED (framebuffer from host
 *   :8390), micro-ROS bridge (heartbeat + IMU + battery + lidar + log) and
 *   GaitRosParticipant (subscribes to /cmd_vel).
 *
 * Feature gates (S3 Sense only — non-S3 boards skip cam/mic/spk unconditionally):
 *   ENABLE_CAM  — OV2640 MJPEG server (default 1)
 *   ENABLE_MIC  — PDM microphone PCM server (default 1)
 *   ENABLE_SPK  — I2S speaker HTTP client (default 1)
 * Set any of these to 0 via -D in platformio.ini to offload that subsystem to
 * a satellite board. The `esp32s3sense_offload` env ships cam + mic off while
 * keeping the speaker on the main board.
 *
 * Boot LED sequence (visible state ladder):
 *   rainbow pulse → red chase (wifi) → yellow pulse (micro-ROS) →
 *   cyan slow pulse (idle). Red fast pulse at any point = low battery.
 *
 * ROS verify (after `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`):
 *   ros2 topic echo /mcu/heartbeat
 *   ros2 topic hz   /mcu/imu              # ~50 Hz
 *   ros2 topic echo /mcu/battery_voltage
 *   ros2 topic hz   /mcu/scan             # ~6 Hz
 *   ros2 topic echo /mcu/log
 *   ros2 topic pub  /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}}' --once
 */
#include <Arduino.h>
#include <BatterySubsystem.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
// Camera/mic/speaker are wired only on S3 Sense; off-board variants never
// compile them in. Within S3 Sense each is individually gated so a
// satellite-offload build can drop cam+mic without touching the rest of the
// sketch. Defaults preserve the full-featured main build.
//
// NOTE: CameraSubsystem.h (→ esp_camera.h → sensor.h) must come before
// GyroSubsystem.h (→ Adafruit_BNO08x.h → Adafruit_Sensor.h). Both headers
// define `sensor_t`; Adafruit guards its typedef with __SENSOR_H__ (the
// esp32-camera guard), so esp32-camera has to win the race.
#ifdef ENV_ESP32S3SENSE
#ifndef ENABLE_CAM
#define ENABLE_CAM 1
#endif
#ifndef ENABLE_MIC
#define ENABLE_MIC 1
#endif
#ifndef ENABLE_SPK
#define ENABLE_SPK 1
#endif
#else
#define ENABLE_CAM 0
#define ENABLE_MIC 0
#define ENABLE_SPK 0
#endif

#if ENABLE_CAM
#include <CameraSubsystem.h>
#include <camera_pins.h>
#endif
#if ENABLE_MIC
#include <MicSubsystem.h>
#endif
#if ENABLE_SPK
#include <SpeakerSubsystem.h>
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
#include <RobotPersistence.h>
#include <ServoSubsystem.h>
#include <StatusLedController.h>
#include <Wire.h>
#include <WiFiUdp.h>
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
// Battery — 3S LiPo calibration lifted from test_all
// ---------------------------------------------------------------------------
static constexpr Subsystem::BatteryCalibration kBattCalibration(
    /*raw_lo=*/2432, /*volt_lo=*/11.52f,
    /*raw_hi=*/2648, /*volt_hi=*/12.62f);

// ---------------------------------------------------------------------------
// Servo defaults (mirror test_sub_movement)
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
    255,              //  [0]  — unused
    0,                //  M1  → FL Hip
    4,                //  M2  → ML Hip
    8,                //  M3  → RL Hip
    10,               //  M4  → RR Hip
    6,                //  M5  → MR Hip
    2,                //  M6  → FR Hip
    11,               //  M7  → RR Knee
    1,                //  M8  → FL Knee
    5,                //  M9  → ML Knee
    9,                //  M10 → RL Knee
    7,                //  M11 → MR Knee
    3,                //  M12 → FR Knee
    255,              //  M13 — unassigned
};

// Leg index → ServoSubsystem index in the 13-entry M-port layout.
static constexpr uint8_t kLegServoHip[6] = {0, 5, 1, 4, 2, 3};
static constexpr uint8_t kLegServoKnee[6] = {7, 11, 8, 10, 9, 6};

// ---------------------------------------------------------------------------
// Globals — constructed at static init
// ---------------------------------------------------------------------------
static Threads::Mutex i2c_mutex;  // shared by Gyro, Servo, OLED

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

static Subsystem::LedSetup led_setup(/*num_leds=*/5);
static Subsystem::LedSubsystem<Config::rgb_data> leds(led_setup);

static Subsystem::GyroSetup gyro_setup(Wire, Config::gyro_addr,
                                       Config::gyro_int);
static Subsystem::BatterySetup battery_setup(Config::batt, kBattCalibration,
                                             /*num_samples=*/16);
static Subsystem::LidarSetup lidar_setup(Serial2, Config::rx, Config::tx,
                                         /*rx_buf_size=*/512,
                                         /*scan_freq_hz=*/6.0f);
static Subsystem::OledSetup oled_setup(i2c_mutex, agent_ip, 8390);

static Subsystem::ESP32WifiSubsystemSetup wifi_setup(
    "wifi", WIFI_SSID, WIFI_PASSWORD, static_ip, gateway, subnet);

static Subsystem::MicrorosManagerSetup manager_setup("microros",
                                                     "seeker_main_node");
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

// Movement state filled in during setup().
static Subsystem::ServoConfig servo_configs[13];
static float body_height_mm = 0.0f;

// ---------------------------------------------------------------------------
// Safe mode — after N consecutive boots without a kStableRunMs "healthy run"
// the sketch skips every risky subsystem and comes up in WiFi+OTA mode so
// the robot can be reflashed over the air. WiFi subsystem handles the OTA
// handler itself (ENABLE_ARDUINO_OTA=1 in platformio.ini) — we just need
// WiFi to come up and keep running.
// ---------------------------------------------------------------------------
// Defaults come from platformio.ini build flags so a rebuild is the only
// knob — no NVS override layer. Guarded so an in-tree override still builds.
#ifndef SAFE_MODE_THRESHOLD
#define SAFE_MODE_THRESHOLD 5
#endif
#ifndef SAFE_MODE_STABLE_MS
#define SAFE_MODE_STABLE_MS 10000
#endif
static constexpr uint8_t kSafeModeThreshold = SAFE_MODE_THRESHOLD;
static constexpr uint32_t kStableRunMs = SAFE_MODE_STABLE_MS;
static constexpr char kSafeModeNs[] = "seeker_sm";
static constexpr char kSafeModeCount[] = "count";
static constexpr char kSafeModeReason[] = "last_reason";

// UDP reset channel — lets a host on the LAN clear the safe-mode counter
// without reflashing. Only bound in runSafeMode(), so normal-mode firmware
// never listens here. Magic string keeps stray packets from clearing state.
// Trigger from host: `echo -n "SEEKER_CLEAR_SM" | nc -u -w1 <MCU_IP> 4210`.
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
      "sm_clear", 4096, nullptr, 1, nullptr, 1);
}

// Enter safe mode: bring up Blink + LEDs (magenta chase) + WiFi (with its
// built-in ArduinoOTA handler), then loop forever broadcasting status.
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
    // Distinct safe-mode signal — magenta chase, easy to tell apart from any
    // normal-mode LED state.
    leds.setEffect(Subsystem::LedMode::CHASE, CRGB::Magenta, 180);
  }

  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR,
                  "[SafeMode] WiFi init failed — Serial-only, no OTA");
  } else {
    // WiFi subsystem calls ArduinoOTA.begin()/handle() internally when
    // ENABLE_ARDUINO_OTA=1, so once this task connects OTA is live.
    wifi.beginThreadedPinned(4096, 3, 100, 1);
  }

  WiFiUDP reset_udp;
  bool udp_bound = false;

  // Broadcast the reason every 5 s so whoever connects (serial or tailing a
  // log later) sees it immediately.
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

// ---------------------------------------------------------------------------
// Servo / gait config helpers (mirror test_sub_movement)
// ---------------------------------------------------------------------------
static Persistence::RobotPrefs buildDefaults() {
  Persistence::RobotPrefs d{};
  d.budget = kDefaultBudget;
  for (uint8_t m = 1; m <= 13; m++) {
    uint8_t hi = kMPortToHexIdx[m];
    if (hi != 255) {
      d.servos[m - 1] = HexapodConfig::kServoConfigs[hi];
      const float total = d.servos[m - 1].total_angle_deg;
      const float scale = (kDefaultMaxPwm - kDefaultMinPwm) / total;
      if (d.servos[m - 1].min_angle < 0.0f) {
        const float center = (kDefaultMinPwm + kDefaultMaxPwm) / 2.0f;
        d.servos[m - 1].min_pwm =
            (uint16_t)roundf(center + d.servos[m - 1].min_angle * scale);
        d.servos[m - 1].max_pwm =
            (uint16_t)roundf(center + d.servos[m - 1].max_angle * scale);
      } else {
        d.servos[m - 1].min_pwm = (uint16_t)roundf(
            kDefaultMinPwm + d.servos[m - 1].min_angle * scale);
        d.servos[m - 1].max_pwm = (uint16_t)roundf(
            kDefaultMinPwm + d.servos[m - 1].max_angle * scale);
      }
    } else {
      d.servos[m - 1] = {
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

  d.gait = HexapodConfig::kGaitConfig;
  for (uint8_t i = 0; i < 6; i++) {
    d.gait.leg_servo_hip[i] = kLegServoHip[i];
    d.gait.leg_servo_knee[i] = kLegServoKnee[i];
  }

  d.height_mm = 0.0f;
  return d;
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

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(500);

  // --- Safe-mode arbitration (must run before any risky bring-up) ---
  uint8_t boots = safeModeTickOnBoot();
  Debug::printf(Debug::Level::INFO,
                "[Main] Boot #%u  this_reason=%s  prev_reason=%s", boots,
                resetReasonStr(s_reset_reason),
                s_prev_reason[0] ? s_prev_reason : "n/a");
  if (boots >= kSafeModeThreshold) {
    runSafeMode();  // never returns
  }

  Debug::printf(Debug::Level::INFO, "[Main] Seeker Robot booting…");

  // --- Blink heartbeat (onboard LED_BUILTIN) ---
  blink.beginThreadedPinned(2048, 1, 500, 1);

  // --- LED chain — come up immediately so boot state is visible ---
  if (!leds.init()) haltOnFail("LED");
  leds.beginThreadedPinned(2048, 2, 20, 1);

  // --- WiFi ---
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) haltOnFail("WiFi");
  wifi.beginThreadedPinned(4096, 3, 100, 1);
  Debug::printf(Debug::Level::INFO, "[Main] WiFi connecting to \"%s\"",
                WIFI_SSID);

  // --- Gyro (BNO085 on I2C, non-fatal if it fails) ---
  auto& gyro = Subsystem::GyroSubsystem::getInstance(gyro_setup, i2c_mutex);
  Subsystem::GyroSubsystem* gyro_ptr = nullptr;
  if (gyro.init()) {
    gyro.beginThreadedPinned(4096, 5, 0, 1);
    gyro_ptr = &gyro;
  } else {
    Debug::printf(Debug::Level::WARN,
                  "[Main] Gyro init failed — IMU topic disabled");
  }

  // --- Battery (ADC, non-fatal if it fails) ---
  auto& batt = Subsystem::BatterySubsystem::getInstance(battery_setup);
  Subsystem::BatterySubsystem* batt_ptr = nullptr;
  if (batt.init()) {
    batt.beginThreadedPinned(4096, 2, 1000, 1);
    batt_ptr = &batt;
  } else {
    Debug::printf(Debug::Level::WARN,
                  "[Main] Battery init failed — low-batt LED disabled");
  }

  // --- Lidar (LD14P on Serial2, Core 0) ---
  auto& lidar = Subsystem::LidarSubsystem::getInstance(lidar_setup);
  if (!lidar.init()) haltOnFail("Lidar");
  lidar.beginThreadedPinned(6144, 4, 1, 0);

  // --- Movement: servo configs (defaults + NVS overlay) ---
  Persistence::RobotPrefs defaults = buildDefaults();
  Persistence::RobotPrefs cfg = defaults;
  bool cfg_from_prefs = Persistence::loadAll(cfg, defaults);
  for (uint8_t i = 0; i < 13; i++) servo_configs[i] = cfg.servos[i];
  body_height_mm = cfg.height_mm;
  Debug::printf(Debug::Level::INFO, "[Main] Servo config: %s",
                cfg_from_prefs ? "NVS" : "defaults");

  // --- Servo subsystem (PCA9685 on I2C, M-port 13-entry layout) ---
  static Subsystem::ServoSetup servo_setup(Wire, Config::pca_addr,
                                           Config::servo_en, servo_configs, 13,
                                           cfg.budget, kDefaultFreqHz);
  auto& servos = Subsystem::ServoSubsystem::getInstance(servo_setup, i2c_mutex);
  for (uint8_t m = 1; m <= 12; m++) servos.attach(m - 1);  // skip M13
  if (!servos.init()) haltOnFail("Servo");
  servos.beginThreadedPinned(4096, 5, 10, 1);

  // --- Kinematics (sync) ---
  static Kinematics::HexapodKinematics kin(HexapodConfig::kLegConfigs);

  // --- GaitController ---
  static Gait::GaitSetup gait_setup(cfg.gait, &kin, &servos);
  static Gait::GaitController gait_ctrl(gait_setup);
  if (!gait_ctrl.init()) haltOnFail("Gait");
  gait_ctrl.beginThreadedPinned(8192, 4, 10, 1);

  // Apply saved body height after gait begin() has had a chance to stand
  // neutral and arm (test_sub_movement pattern).
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

#if ENABLE_CAM
  // --- Camera (OV2640, MJPEG on :80) — non-fatal ---
  auto& cam = Subsystem::CameraSubsystem::getInstance(cam_setup);
  cam.beginThreadedPinned(8192, 2, 5000, 1);
#endif

#if ENABLE_MIC
  // --- Mic (PDM on I2S_NUM_0, PCM on :81, Core 0) — non-fatal ---
  auto& mic = Subsystem::MicSubsystem::getInstance(mic_setup);
  mic.beginThreadedPinned(4096, 2, 5000, 0);
#endif

#if ENABLE_SPK
  // --- Speaker (I2S_NUM_1, PCM from host :8383) ---
  // Host-side mic gating during playback lives in seeker_tts
  // (/audio_speaker_active topic); see SpeakerSubsystem.h.
  static Subsystem::SpeakerSetup speaker_setup(
      I2S_NUM_1, 16000, Config::spk_bclk, Config::spk_lrclk, Config::spk_dout,
      agent_ip, 8383, 2048);
  auto& spk = Subsystem::SpeakerSubsystem::getInstance(speaker_setup);
  if (!spk.init()) haltOnFail("Speaker");
  spk.beginThreadedPinned(8192, 2, 100, 1);
#endif  // ENABLE_SPK

  // --- OLED (I2C, HTTP fetch from host :8390) ---
  auto& oled = Subsystem::OledSubsystem::getInstance(oled_setup);
  if (!oled.init()) haltOnFail("OLED");
  oled.beginThreadedPinned(4096, 2, 100, 1);
  oled.setFrame("boot");
  oled.setOverlay(0, 4, 40, "seeker_main");
  oled.setOverlay(1, 4, 52, "connecting...");

  // --- MicroRosBridge (sensors → topics) ---
  static Subsystem::MicroRosBridgeSetup bridge_setup;
  bridge_setup.gyro = gyro_ptr;
  bridge_setup.battery = batt_ptr;
  bridge_setup.lidar = &lidar;
  static Subsystem::MicroRosBridge bridge(bridge_setup);

  // --- GaitRosParticipant (subscribe /cmd_vel) ---
  static Gait::GaitRosParticipantSetup gait_ros_setup{&gait_ctrl};
  static Gait::GaitRosParticipant gait_ros(gait_ros_setup);

  // --- MicrorosManager ---
  manager.registerParticipant(&bridge);
  manager.registerParticipant(&gait_ros);
  if (!manager.init()) haltOnFail("MicroRosManager");
  manager.setStateCallback([](bool connected) {
    Debug::printf(Debug::Level::INFO, "[Main] micro-ROS agent %s",
                  connected ? "CONNECTED" : "DISCONNECTED");
  });
  manager.beginThreadedPinned(8192, 3, 10, 1);

  // --- StatusLedController (last — it references everything above) ---
  static Subsystem::StatusLedSetup slc_setup;
  slc_setup.leds = &leds;
  slc_setup.battery = batt_ptr;
  slc_setup.wifi = &wifi;
  slc_setup.manager = &manager;
  slc_setup.gait = &gait_ctrl;
  static Subsystem::StatusLedController slc(slc_setup);
  if (!slc.init()) haltOnFail("StatusLed");
  slc.beginThreadedPinned(3072, 2, 100, 1);

  // --- Safe-mode watchdog clear — after kStableRunMs of uptime without
  //     rebooting, wipe the NVS boot counter so only NEW crashes count.
  safeModeScheduleClear();

  Debug::printf(Debug::Level::INFO, "[Main] Boot complete");
}

void loop() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
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
