/**
 * @file hal_thread.h
 * @brief Abstract away platform-specific threading and make a single class!
 * Current scheme flags:
 * THREADING_SCHEME_ESPIDF, implements FreeRTOS
 * THREADING_SCHEME_ESP_MODERN, implements the C++ variants
 * Original found at
 * https://github.com/IEEE-UCF/SEC26Mirror/blob/master/mcu_ws/lib/rtos_compat/FreeRTOSCompat.h
 * @author Aldem Pido
 * @date 3/24/26
 */
#pragma once

#ifdef THREADING_SCHEME_ESPIDF

namespace Threads {
class Mutex {
 public:
  // Constructor
  Mutex() : handle_(xSemaphoreCreateRecursiveMutex()) {}

  // Deconstructor
  ~Mutex() {
    if (handle_) vSemaphoreDelete(handle_);
  }

  // Force singleton (only one can exist at once!)
  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;

  /// @brief Locks the thread recursively.
  void lock() {
    if (handle_) xSemaphoreTakeRecursive(handle_, portMAX_DELAY);
  }

  /// @brief Releases the thread recursively.
  void unlock() {
    if (handle_) xSemaphoreGiveRecursive(handle_);
  }

 private:
  SemaphoreHandle_t handle_;
};

class Scope {
 public:
  /// @brief Creates a scope
  /// @param m Reference to Mutex
  explicit Scope(Mutex& m) : m_(m) { m_.lock(); }
  ~Scope() { m_.unlock(); }

  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;

 private:
  Mutex& m_;
};
};  // namespace Threads

#endif

#ifdef THREADING_SCHEME_ESP_MODERN

#include <mutex>

namespace Threads {
class Mutex {
 public:
  Mutex() = default;
  ~Mutex() = default;

  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;

  void lock() { mtx_.lock(); }
  void unlock() { mtx_.unlock(); }

 private:
  std::recursive_mutex mtx_;
};

using Scope = std::lock_guard<Mutex>;
}  // namespace Threads

#endif  // THREADING_SCHEME_ESP_MODERN
