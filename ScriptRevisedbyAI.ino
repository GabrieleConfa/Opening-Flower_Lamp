#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ==========================================
// 1. HARDWARE & PIN DEFINITIONS
// ==========================================
const uint8_t PIN_BUTTON   = 2;
const uint8_t PIN_NEOPIXEL = 8;
const uint8_t NUM_SERVOS   = 5;
const uint8_t SERVO_PINS[NUM_SERVOS] = {3, 4, 5, 6, 7};

// ==========================================
// 2. SERVO & KINEMATIC CONSTANTS
// ==========================================
const int     DELTA_OPEN_DEG  = 105;
const float   ACCEL_DEG_S2    = 70.0f;
const uint8_t UPDATE_PERIOD_MS = 25;

int CLOSED_ANGLE[NUM_SERVOS]   = {27, 37, 27, 19, 32};
const int8_t SERVO_DIR[NUM_SERVOS] = {+1, +1, +1, +1, +1};
float VEL_MAX_DEG_S[NUM_SERVOS]    = {45, 48, 43, 46, 44};
const uint16_t SERVO_PHASE_MS[NUM_SERVOS] = {0, 120, 240, 120, 0};

// ==========================================
// 3. LED CONSTANTS
// ==========================================
const uint16_t NUM_LEDS = 12;
const uint8_t  LED_BRIGHT_MAX = 110;
const uint8_t  LED_WARM_R = 255;
const uint8_t  LED_WARM_G = 120;
const uint8_t  LED_WARM_B = 150;
const uint16_t NEO_SHOW_MIN_INTERVAL_MS = 40;

// ==========================================
// 4. SYSTEM STATE & OBJECTS
// ==========================================
Servo servos[NUM_SERVOS];
Adafruit_NeoPixel strip(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

enum Mode { CLOSED, OPENING, OPEN, CLOSING };
Mode mode = CLOSED;

struct ServoState {
  float currentAngle;
  float targetAngle;
  float velocity;
  unsigned long startTimeMs;
  bool active;
} sState[NUM_SERVOS];

// Global Tracking Variables
float ledProgress = 0.0f; 
bool  ledDirty    = false;
bool  ledActive   = false;
unsigned long lastNeoShowMs = 0;
unsigned long lastUpdateMs  = 0;

// ==========================================
// 5. UTILITY & MATH FUNCTIONS
// ==========================================
float clampf(float x, float a, float b) { 
  return x < a ? a : (x > b ? b : x); 
}

float ease01(float t) { 
  return (t < 0.5f) ? 4.0f * t * t * t : 1.0f - powf(-2.0f * t + 2.0f, 3.0f) * 0.5f; 
}

uint8_t lerp8(uint8_t a, uint8_t b, float t) { 
  return (uint8_t)(a + (b - a) * t); 
}

uint8_t clamp8(float x) { 
  return (x < 0 ? 0 : (x > 255 ? 255 : (uint8_t)x)); 
}

// ==========================================
// 6. LED MANAGEMENT
// ==========================================
void ledBufferUpdate() {
  float e = ease01(clampf(ledProgress, 0.0f, 1.0f));

  uint8_t startR = 180;
  uint8_t startG = 30;
  uint8_t startB = 50;

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    float t = (i < NUM_LEDS/2) ? (float)i / (NUM_LEDS/2) : (float)(NUM_LEDS-1-i)/(NUM_LEDS/2);

    uint8_t r = lerp8(startR, LED_WARM_R, e * (1.0 - 0.3*t));
    uint8_t g = lerp8(startG, LED_WARM_G, e * (1.0 - 0.3*t));
    uint8_t b = lerp8(startB, LED_WARM_B + 20*t, e * (1.0 - 0.3*t));

    // Optional subtle glitter effect during opening
    if (mode == OPENING && random(0, 100) < 5) {
      r = clamp8(r + 50);
      g = clamp8(g + 50);
      b = clamp8(b + 60);
    }
    strip.setPixelColor(i, strip.Color(r, g, b));
  }

  strip.setBrightness((uint8_t)(LED_BRIGHT_MAX * e));
  ledDirty = true;
}

void ledMaybeShow() {
  unsigned long now = millis();
  if (ledDirty && (now - lastNeoShowMs >= NEO_SHOW_MIN_INTERVAL_MS)) {
    strip.show();
    lastNeoShowMs = now;
    ledDirty = false;
  }
}

// ==========================================
// 7. KINEMATIC MOVEMENT ENGINE
// ==========================================
void startMovement(bool toOpen) {
  unsigned long t0 = millis();

  if (toOpen) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(80, 0, 30));
    }
    strip.setBrightness(30);
    strip.show();
    delay(80); // Brief initial setup delay allowed here before loop starts
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    float base = CLOSED_ANGLE[i];
    float tgt  = base + (toOpen ? SERVO_DIR[i] * DELTA_OPEN_DEG : 0);

    sState[i].targetAngle = clampf(tgt, 0, 180);
    sState[i].velocity    = 0;
    sState[i].active      = true;

    int jitter = random(-20, 20);
    sState[i].startTimeMs = t0 + SERVO_PHASE_MS[i] + jitter;

    if (!servos[i].attached()) servos[i].attach(SERVO_PINS[i]);
  }

  ledProgress = toOpen ? 0.05f : 1.0f;
  ledActive   = true;
  mode        = toOpen ? OPENING : CLOSING;

  ledBufferUpdate();
}

void updateServos() {
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) * 0.001f;
  lastUpdateMs = now;

  bool allDone = true;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (!sState[i].active) continue;
    if (now < sState[i].startTimeMs) { allDone = false; continue; }

    float err = sState[i].targetAngle - sState[i].currentAngle;
    if (fabs(err) < 1.0f) {
      sState[i].currentAngle = sState[i].targetAngle;
      sState[i].velocity = 0;
      sState[i].active = false;
      continue;
    }

    allDone = false;
    float sign = err > 0 ? 1.0f : -1.0f;
    float dist = fabs(err);

    // Calculate velocity required to stop at target
    float vStop = sqrtf(2.0f * ACCEL_DEG_S2 * dist);
    
    // Apply acceleration or deceleration
    float a = (fabs(sState[i].velocity) > vStop) ? -ACCEL_DEG_S2 * sign : ACCEL_DEG_S2 * sign;

    sState[i].velocity += a * dt;
    sState[i].velocity  = clampf(sState[i].velocity, -VEL_MAX_DEG_S[i], VEL_MAX_DEG_S[i]);
    sState[i].currentAngle += sState[i].velocity * dt;
    
    servos[i].write((int)round(sState[i].currentAngle));
  }

  // Sync LED progress to the main servo's velocity
  if (ledActive) {
    float rate = VEL_MAX_DEG_S[0] / 90.0f;
    ledProgress += (mode == OPENING ? +rate : -rate) * dt;
    ledProgress  = clampf(ledProgress, 0, 1);
    ledBufferUpdate();
  }

  ledMaybeShow();

  // Handle state transition when movement completes
  if (allDone) {
    if (mode == OPENING) {
      mode = OPEN;
      ledActive = false;
      ledDirty = true;
      ledMaybeShow();
    } else if (mode == CLOSING) {
      mode = CLOSED;
      ledActive = false;
      strip.setBrightness(0);
      for (uint16_t i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, 0);
      ledDirty = true;
      ledMaybeShow();
    }
  }
}

// ==========================================
// 8. MAIN SETUP & LOOP
// ==========================================
void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  randomSeed(analogRead(A0));

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    sState[i].currentAngle = CLOSED_ANGLE[i];
    sState[i].active = false;
    servos[i].write(CLOSED_ANGLE[i]);
  }

  strip.begin();
  strip.setBrightness(0);
  strip.show();

  lastNeoShowMs = millis();
  lastUpdateMs  = millis();
}

void loop() {
  bool wantOpen = (digitalRead(PIN_BUTTON) == LOW);

  if (wantOpen && (mode == CLOSED || mode == CLOSING)) startMovement(true);
  if (!wantOpen && (mode == OPEN || mode == OPENING)) startMovement(false);

  if (millis() - lastUpdateMs >= UPDATE_PERIOD_MS) {
    updateServos();
  }
}