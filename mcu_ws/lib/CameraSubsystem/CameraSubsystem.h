/**
 * @file CameraSubsystem.h
 * @author Tal Avital
 * @date 4/4/26
 * @brief Manages OV2640 initialization and an MJPEG HTTP stream server.
 *        The IDF httpd server drives the actual streaming; update() only logs
 *        periodic status.
 */
#pragma once

#include <CustomDebug.h>
#include <ThreadedSubsystem.h>
#include <esp_camera.h>
#include <esp_http_server.h>

namespace Subsystem {

class CameraSetup : public Classes::BaseSetup {
 public:
  CameraSetup() = delete;

  /// @param config             Camera hardware config (pins, pixel format,
  /// etc.)
  /// @param port               HTTP port for the /cam endpoint (default 80).
  /// @param ctrl_port          httpd internal control socket port — must be
  ///                           unique across all httpd instances.
  /// @param frame_interval_ms  Per-frame delay in the stream loop. 50 ms caps
  ///                           the sender at ~20 fps, which keeps the lwIP PBUF
  ///                           pool from starving when the board also runs a
  ///                           micro-ROS UDP stack. Satellite boards can leave
  ///                           it here (no micro-ROS contention) or lower it.
  /// @param jpeg_quality       0 = auto-select by PSRAM (12 w/ PSRAM). Non-zero
  ///                           forces the value (lower = better quality).
  /// @param frame_size         FRAMESIZE_INVALID = auto (QVGA w/ PSRAM, QQVGA
  ///                           otherwise). Override to taste on boards that can
  ///                           spend more bandwidth.
  CameraSetup(const camera_config_t& config, uint16_t port = 80,
              uint16_t ctrl_port = 32768, uint32_t frame_interval_ms = 50,
              uint8_t jpeg_quality = 0,
              framesize_t frame_size = FRAMESIZE_INVALID)
      : Classes::BaseSetup("CameraSubsystem"),
        config_(config),
        port_(port),
        ctrl_port_(ctrl_port),
        frame_interval_ms_(frame_interval_ms),
        jpeg_quality_(jpeg_quality),
        frame_size_(frame_size) {}

  const camera_config_t config_;
  const uint16_t port_;
  const uint16_t ctrl_port_;
  const uint32_t frame_interval_ms_;
  const uint8_t jpeg_quality_;
  const framesize_t frame_size_;
};

/**
 * @brief Initializes OV2640 and serves an MJPEG stream over HTTP.
 *
 * Runs as a FreeRTOS task. Camera init and httpd startup happen in begin().
 * Frame-buffer handling is done inside streamHandler(), which is invoked by
 * the IDF httpd task — not this update loop. update() logs periodic status.
 *
 * PSRAM availability is checked at begin() time and used to choose the
 * appropriate frame_size, fb_count, and fb_location.
 *
 * Usage:
 * @code
 * static CameraSetup cam_setup(camera_config);
 * auto& cam = CameraSubsystem::getInstance(cam_setup);
 * cam.beginThreadedPinned(4096, 2, 5000, 1);
 * @endcode
 */
class CameraSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  CameraSubsystem(const CameraSubsystem&) = delete;
  CameraSubsystem& operator=(const CameraSubsystem&) = delete;

  static CameraSubsystem& getInstance(const CameraSetup& setup) {
    static CameraSubsystem instance(setup);
    return instance;
  }

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override { return setup_.getId(); }

  bool isCameraReady() const { return camera_ready_; }
  bool isServerRunning() const { return stream_httpd_ != nullptr; }

 private:
  explicit CameraSubsystem(const CameraSetup& setup)
      : ThreadedSubsystem(setup), setup_(setup) {}

  void startServer();
  void stopServer();
  static esp_err_t streamHandler(httpd_req_t* req);

  const CameraSetup setup_;
  httpd_handle_t stream_httpd_ = nullptr;
  bool camera_ready_ = false;
  uint32_t last_log_ms_ = 0;

  static constexpr uint32_t kLogIntervalMs = 1000;
};

}  // namespace Subsystem
