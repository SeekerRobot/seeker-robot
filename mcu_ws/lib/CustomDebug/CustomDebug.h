/**
 * @file CustomDebug.h
 * @author Aldem Pido
 * @date 3/24/26
 * @brief Routes all debug statements into a centralized place with proper
 * handling for each transport.
 */

// @TODO: actually implement
#pragma once

#include <cstdarg>

#ifdef DEBUG_TRANSPORT_BLUETOOTH
#endif

#ifdef DEBUG_TRANSPORT_SERIAL
#endif

#ifdef DEBUG_TRANSPORT_MICROROS
#endif

namespace Debug {

inline void printf(const char* fmt, ...) { (void)fmt; }

inline void flush() {}

}  // namespace Debug