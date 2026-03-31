/**
 * @file CustomDebug.h
 * @author Aldem Pido
 * @date 3/24/26
 * @brief Routes all debug statements into a centralized place with proper
 * handling for each transport.
 */

// @TODO: actually implement
#pragma once

#include <hal_thread.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>

#ifdef DEBUG_TRANSPORT_SERIAL
#include <Arduino.h>
#endif

#ifdef DEBUG_TRANSPORT_BLUETOOTH
#include <BleDebugSubsystem.h>
#endif

#ifdef DEBUG_TRANSPORT_MICROROS
#endif

namespace Debug {

inline Threads::Mutex& serialMutex() {
  static Threads::Mutex mtx;
  return mtx;
}

enum class Level : uint8_t {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3,
  VERBOSE = 4,
};

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 2  // Default: INFO
#endif

inline const char* levelPrefix(Level level) {
  switch (level) {
    case Level::ERROR:
      return "[E]";
    case Level::WARN:
      return "[W]";
    case Level::INFO:
      return "[I]";
    case Level::DEBUG:
      return "[D]";
    case Level::VERBOSE:
      return "[V]";
    default:
      return "[?]";
  }
}

inline void printf(Level level, const char* fmt, ...) {
  if (static_cast<uint8_t>(level) > DEBUG_LEVEL) return;

  char    buf[256];
  int     offset = snprintf(buf, sizeof(buf), "%s ", levelPrefix(level));
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf + offset, sizeof(buf) - offset, fmt, args);
  va_end(args);

#ifdef DEBUG_TRANSPORT_SERIAL
  {
    Threads::Scope lock(serialMutex());
    Serial.println(buf);
  }
#endif

#ifdef DEBUG_TRANSPORT_BLUETOOTH
  Subsystem::BleDebugSubsystem::writeIfReady(buf);
#endif

#if !defined(DEBUG_TRANSPORT_SERIAL) && !defined(DEBUG_TRANSPORT_BLUETOOTH)
  (void)fmt;
#endif
}

inline void printf(const char* fmt, ...) {
  char    buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

#ifdef DEBUG_TRANSPORT_SERIAL
  {
    Threads::Scope lock(serialMutex());
    Serial.println(buf);
  }
#endif

#ifdef DEBUG_TRANSPORT_BLUETOOTH
  Subsystem::BleDebugSubsystem::writeIfReady(buf);
#endif

#if !defined(DEBUG_TRANSPORT_SERIAL) && !defined(DEBUG_TRANSPORT_BLUETOOTH)
  (void)fmt;
#endif
}

inline void flush() {
#ifdef DEBUG_TRANSPORT_SERIAL
  Serial.flush();
#endif
}

}  // namespace Debug