/**
 * @file MicroRosDebug.cpp
 * @brief Thread-safe ring-buffer queue bridging Debug::printf to the
 *        MicroRosBridge debug publisher.
 *
 * open()/close() are called by MicroRosBridge::onCreate()/onDestroy().
 * enqueue() is called from any task via CustomDebug::printf.
 * dequeue() is called from the manager task via MicroRosBridge::publishAll().
 *
 * The Threads::Mutex is recursive (xSemaphoreCreateRecursiveMutex), so a
 * Debug::printf call originating inside publishAll() won't deadlock.
 */
#include "MicroRosDebug.h"

#include <hal_thread.h>
#include <string.h>

namespace {

static constexpr uint8_t kQueueDepth = 8;

struct Entry {
  char text[MicroRosDebug::kMsgLen + 1];
};

static Entry s_queue[kQueueDepth];
static uint8_t s_head = 0;
static uint8_t s_tail = 0;
static Threads::Mutex s_mutex;
static bool s_open = false;

}  // namespace

namespace MicroRosDebug {

void open() {
  Threads::Scope lock(s_mutex);
  s_head = 0;
  s_tail = 0;
  s_open = true;
}

void close() {
  Threads::Scope lock(s_mutex);
  s_open = false;
}

void enqueue(const char* text) {
  Threads::Scope lock(s_mutex);
  if (!s_open) return;
  uint8_t next = (s_head + 1) % kQueueDepth;
  if (next == s_tail) return;  // full — drop silently
  strncpy(s_queue[s_head].text, text, MicroRosDebug::kMsgLen);
  s_queue[s_head].text[MicroRosDebug::kMsgLen] = '\0';
  s_head = next;
}

bool dequeue(char* buf, size_t len) {
  if (!buf || len == 0) return false;
  Threads::Scope lock(s_mutex);
  if (!s_open || s_tail == s_head) return false;
  strncpy(buf, s_queue[s_tail].text, len - 1);
  buf[len - 1] = '\0';
  s_tail = (s_tail + 1) % kQueueDepth;
  return true;
}

}  // namespace MicroRosDebug
