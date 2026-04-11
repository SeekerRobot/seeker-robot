/**
 * @file test_sub_speaker.cpp
 * @brief SpeakerSubsystem test — connects to WiFi, then long-polls
 *        the ROS 2 host (AGENT_IP) for PCM audio and plays it over I2S.
 *
 * Pair with the seeker_tts ROS 2 node on the host:
 *   FISH_API_KEY=xxx ros2 run seeker_tts tts_node
 *   ros2 topic pub /audio_tts_input std_msgs/String "data: 'hello'" --once
 *   ros2 topic pub /audio_play_file std_msgs/String "data:
 * '/path/to/sound.wav'" --once
 *
 * Commands (serial monitor at 921600 baud):
 *   info      Print subsystem status
 *   help      Show this help
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>
#include <CustomDebug.h>
#include <ESP32WifiSubsystem.h>
#include <RobotConfig.h>
#include <SpeakerSubsystem.h>

// ---------------------------------------------------------------------------
// WiFi (reuses network_config.ini macros)
// ---------------------------------------------------------------------------
static IPAddress static_ip STATIC_IP;
static IPAddress gateway GATEWAY;
static IPAddress subnet SUBNET;

static Subsystem::ESP32WifiSubsystemSetup wifi_setup("WifiSubsystem", WIFI_SSID,
                                                     WIFI_PASSWORD, static_ip,
                                                     gateway, subnet);

// ---------------------------------------------------------------------------
// Speaker — fetches PCM from http://<AGENT_IP>:8383/audio_out
// ---------------------------------------------------------------------------
static IPAddress agent_ip(AGENT_IP);

static Subsystem::SpeakerSetup speaker_setup(I2S_NUM_1, 16000, Config::spk_bclk,
                                             Config::spk_lrclk,
                                             Config::spk_dout, agent_ip, 8383,
                                             2048, nullptr);

// ---------------------------------------------------------------------------
// Heartbeat blink
// ---------------------------------------------------------------------------
static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

// ---------------------------------------------------------------------------
// Serial parser
// ---------------------------------------------------------------------------
static constexpr size_t kLineBufSize = 64;
static char line_buf[kLineBufSize];
static uint8_t line_pos = 0;

static void printHelp() {
  Serial.println(
      "\r\n"
      "===== Speaker Subsystem Test ============================\r\n"
      "info       Print speaker + WiFi status\r\n"
      "help / ?   Show this help\r\n"
      "=========================================================\r\n"
      "Audio is fetched automatically from the TTS host.\r\n"
      "Publish on the ROS 2 side:\r\n"
      "  TTS:  ros2 topic pub /audio_tts_input std_msgs/String "
      "\"data: 'hello'\" --once\r\n"
      "  File: ros2 topic pub /audio_play_file std_msgs/String "
      "\"data: '/path/to/sound.wav'\" --once\r\n");
}

static void cmdInfo() {
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  auto& spk = Subsystem::SpeakerSubsystem::getInstance(speaker_setup);

  Serial.printf("WiFi: %s", wifi.isConnected() ? "CONNECTED" : "DISCONNECTED");
  if (wifi.isConnected()) {
    Serial.printf("  IP=%s  RSSI=%d dBm", wifi.getLocalIP().toString().c_str(),
                  wifi.getRSSI());
  }
  Serial.println();
  Serial.printf("Speaker: I2S %s  host=%s:%u\r\n",
                spk.isI2sReady() ? "ready" : "not ready",
                agent_ip.toString().c_str(), 8383);
}

static void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line_pos > 0) {
        line_buf[line_pos] = '\0';
        Serial.printf("> %s\r\n", line_buf);
        // Lowercase the command.
        for (uint8_t i = 0; i < line_pos; i++) {
          if (line_buf[i] >= 'A' && line_buf[i] <= 'Z') line_buf[i] += 32;
        }
        if (strcmp(line_buf, "info") == 0)
          cmdInfo();
        else if (strcmp(line_buf, "help") == 0 || strcmp(line_buf, "?") == 0)
          printHelp();
        else
          Serial.printf("ERR: unknown command '%s'. Type 'help'.\r\n",
                        line_buf);
        line_pos = 0;
      }
    } else if (line_pos < kLineBufSize - 1) {
      line_buf[line_pos++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  delay(500);

  blink.beginThreadedPinned(2048, 1, 500, 1);

  // Start WiFi first — speaker needs network to reach the host.
  auto& wifi = Subsystem::ESP32WifiSubsystem::getInstance(wifi_setup);
  if (!wifi.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] WiFi init FAILED");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  wifi.beginThreadedPinned(4096, 3, 100, 1);

  Debug::printf(Debug::Level::INFO, "[Main] Connecting to \"%s\"...",
                WIFI_SSID);

  // Block until WiFi is up so the speaker has a route to the host.
  while (!wifi.isConnected()) {
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  Debug::printf(Debug::Level::INFO, "[Main] WiFi connected — IP %s",
                wifi.getLocalIP().toString().c_str());

  // Start speaker — will poll host for audio in its update() loop.
  auto& spk = Subsystem::SpeakerSubsystem::getInstance(speaker_setup);
  if (!spk.init()) {
    Debug::printf(Debug::Level::ERROR, "[Main] Speaker init FAILED");
    while (true) vTaskDelay(portMAX_DELAY);
  }
  spk.beginThreadedPinned(8192, 2, 100, 1);

  Serial.println("\r\n=== Speaker Subsystem Test ===");
  Serial.printf("Polling http://%s:8383/audio_out for PCM audio\r\n",
                agent_ip.toString().c_str());
  printHelp();
}

void loop() {
  processSerial();
  delay(50);
}
