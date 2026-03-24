#include <Arduino.h>
#include <ThreadedSubsystem.h>

class BlinkSubsystem : public Subsystem::ThreadedSubsystem {
 public:
  explicit BlinkSubsystem(const Classes::BaseSetup& setup)
      : ThreadedSubsystem(setup) {}

  void begin() override { 
    pinMode(LED_BUILTIN, OUTPUT); 
    initSuccess_ = true;
  }

  void update() override {
    if(!initSuccess_) return;
    led_state_ = !led_state_;
    digitalWrite(LED_BUILTIN, led_state_ ? HIGH : LOW);
  }

  bool init() override { return true; }
  const char* getInfo() override { return "BlinkSubsystem"; }
  void pause() override {}
  void reset() override {}

 private:
  bool led_state_ = false;
};

static Classes::BaseSetup blink_setup("blink");
static BlinkSubsystem blink(blink_setup);

void setup() {
  blink.beginThreadedPinned(2048, 1, 500, 1);
}

void loop() {}
