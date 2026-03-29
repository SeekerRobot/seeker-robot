#include <Arduino.h>
#include <pthread.h>

#define BUTTON_0 35  // GPIO34/35 are input-only on ESP32 (OK for buttons)
#define BUTTON_1 34
#define BUTTON_0POWER 33  // power pins (if you’re actually powering buttons this way)
#define BUTTON_1POWER 32

#define LED0 26
#define LED1 25

struct button_and_led {
  int kButton;
  int kLed;
};

static pthread_mutex_t g_mutex;
static volatile int g_mutex_holder = 0;  // 0 = none, else holds BUTTON pin number

// IMPORTANT: these must stay valid after setup() returns -> make them global/static
static button_and_led system0{ BUTTON_0, LED0 };
static button_and_led system1{ BUTTON_1, LED1 };

void* readDigital(void* arg) {
  button_and_led* pins = static_cast<button_and_led*>(arg);

  Serial.printf("Thread started: Button=%d, LED=%d\n", pins->kButton, pins->kLed);

  pinMode(pins->kButton, INPUT_PULLDOWN);  // Use external pullup/pulldown with GPIO34/35 (no internal pullups)
  pinMode(pins->kLed, OUTPUT);
  digitalWrite(pins->kLed, LOW);

  int prev = digitalRead(pins->kButton);

  while (true) {
    int curr = digitalRead(pins->kButton);
    if (curr == HIGH) {
      int res = pthread_mutex_trylock(&g_mutex);
      if (res == 0) {
        Serial.println("Mutex was free");
        digitalWrite(pins->kLed, HIGH);
        while (curr == HIGH) {
          curr = digitalRead(pins->kButton);
          delay(50);
        }
        digitalWrite(pins->kLed, LOW);
        pthread_mutex_unlock(&g_mutex);
      } else {
        Serial.println("Mutex was not free");
      }
    }
    delay(50);
    /*if (prev != curr) {
      if (curr == HIGH) {
        // Button pressed: try to acquire ownership
        pthread_mutex_lock(&g_mutex);

        

        pthread_mutex_unlock(&g_mutex);
      } else {
        // Button released: release ownership if we are the owner
        pthread_mutex_lock(&g_mutex);

        if (g_mutex_holder == pins->kButton) {
          digitalWrite(pins->kLed, LOW);
          g_mutex_holder = 0;
        }

        pthread_mutex_unlock(&g_mutex);
      }

      prev = curr;
    }*/

    //delay(50); // debounce-ish + yield
  }

  return nullptr;
}

void setup() {
  Serial.begin(9600);

  // Init mutex
  if (pthread_mutex_init(&g_mutex, nullptr) != 0) {
    Serial.println("Mutex init failed");
    while (true) { delay(1000); }
  }

  pthread_t id_button0;
  pthread_t id_button1;

  int r0 = pthread_create(&id_button0, nullptr, readDigital, &system0);
  int r1 = pthread_create(&id_button1, nullptr, readDigital, &system1);

  Serial.printf("pthread_create results: button0=%d button1=%d (0 means success)\n", r0, r1);
}

void loop() {
  // Nothing here; work is done by the threads
  delay(1000);
}
