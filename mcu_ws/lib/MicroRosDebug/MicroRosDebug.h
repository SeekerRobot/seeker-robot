/**
 * @file MicroRosDebug.h
 * @brief Thin interface for the micro-ROS debug log transport.
 *
 * Included by CustomDebug.h when DEBUG_TRANSPORT_MICROROS is defined.
 * Deliberately has NO micro-ROS headers — callers only need the enqueue
 * declaration. The queue and publisher live in MicroRosDebug.cpp.
 *
 * Usage:
 *   - Enable BRIDGE_ENABLE_DEBUG=1 so MicroRosBridge creates the publisher.
 *   - Enable DEBUG_TRANSPORT_MICROROS in build_flags so CustomDebug routes
 *     Debug::printf output through enqueue().
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace MicroRosDebug {

/// Maximum message length (bytes), excluding the null terminator.
static constexpr uint16_t kMsgLen = 240;

/// Enqueue a formatted debug string for publishing over micro-ROS.
/// Non-blocking: drops silently if the queue is full or not yet initialised.
/// Safe to call from any FreeRTOS task context.
void enqueue(const char* text);

/// Called by MicroRosBridge::onCreate() once the publisher is ready.
void open();

/// Called by MicroRosBridge::onDestroy() to stop accepting messages.
void close();

/// Dequeue one message into buf (max len bytes including NUL).
/// Returns true if a message was available.
bool dequeue(char* buf, size_t len);

}  // namespace MicroRosDebug
