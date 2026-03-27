/**
 * @file test_threaded_blink.cpp
 * @author Aldem Pido
 * @date 3/26/26
 * @brief Simplest possible test to determine if target is alive.
 */
#include <Arduino.h>
#include <BlinkSubsystem.h>

static Classes::BaseSetup blink_setup("blink");
static Subsystem::BlinkSubsystem blink(blink_setup);

void setup() { blink.beginThreadedPinned(2048, 1, 500, 1); }

void loop() {}
