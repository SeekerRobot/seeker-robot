# libs_external

External libraries that can't go in `lib/` (e.g., `esp32/micro_ros_platformio`). Keeping micro-ROS here ensures it is built once and shared across all ESP32 sketches.

This directory is seeded into a Docker named volume by the `init-bootstrap` service so that build artifacts stay off the host.
