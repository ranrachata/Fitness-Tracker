#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
#define MPU_ADDR      0x68
#define SDA_PIN       6
#define SCL_PIN       7
#define BUZZER_PIN    5
#define BUTTON_PIN    10

// ── Push-up thresholds ──────────────────────────────────────────────────────
// Device: flat on upper back, face-down.
// az ≈ +1.0 g at top (arms extended); drops below DOWN when chest nears floor.
#define PUSHUP_DOWN_THR   1.2f   // az < this  → "down" phase detected
#define PUSHUP_UP_THR     1.3f   // az > this  → "up" phase detected → count

// ── Sit-up thresholds ───────────────────────────────────────────────────────
// Device: flat on chest/abdomen.
// |ay| ≈ 0 when lying flat; rises toward 1 g when torso is upright.
#define SITUP_UP_THR      0.60f   // |ay| > this → "up" phase detected
#define SITUP_DOWN_THR    0.25f   // |ay| < this → back to flat → count

#define MIN_REP_MS  600           // minimum ms between counted reps (debounce)
#define SMOOTH_A    0.25f         // EMA smoothing factor (0 = no update, 1 = raw)

// ── State ───────────────────────────────────────────────────────────────────
enum Mode     { PUSHUP, SITUP };
enum RepState { WAIT_DOWN, WAIT_UP };

Mode          mode      = PUSHUP;
RepState      repState  = WAIT_DOWN;   // push-up starts waiting for DOWN first
int           repCount  = 0;
bool          lastBtn   = HIGH;
unsigned long lastRepTime = 0;

float smoothAz = 1.0f;   // initial estimate: device lying flat (az = 1 g)
float smoothAy = 0.0f;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── I2C / hardware helpers ───────────────────────────────────────────────────
void i2c_clear_bus() {
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(5);
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
  pinMode(SDA_PIN, INPUT_PULLUP);
  delay(10);
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool initAll() {
  Wire.end(); delay(20);
  i2c_clear_bus();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) return false;
  initMPU();
  return true;
}

void readMPU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  int16_t rawAx = Wire.read() << 8 | Wire.read();
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  int16_t rawGy = Wire.read() << 8 | Wire.read();
  int16_t rawGz = Wire.read() << 8 | Wire.read();
  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;
  gx = rawGx / 131.0f;
  gy = rawGy / 131.0f;
  gz = rawGz / 131.0f;
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(1000);
  i2c_clear_bus();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED failed"); for (;;) yield();
  }
  initMPU();
  Serial.println("Ready");
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  // ── Button: toggle mode + reset ──────────────────────────────────────────
  bool btn = digitalRead(BUTTON_PIN);
  if (btn == LOW && lastBtn == HIGH) {
    delay(50);  // debounce
    mode      = (mode == PUSHUP) ? SITUP : PUSHUP;
    repCount  = 0;
    repState  = WAIT_DOWN;
    smoothAz  = 1.0f;
    smoothAy  = 0.0f;
    lastRepTime = 0;
  }
  lastBtn = btn;

  // ── I2C watchdog ─────────────────────────────────────────────────────────
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    if (!initAll()) { delay(500); return; }
  }

  // ── Read & smooth sensor ─────────────────────────────────────────────────
  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);
  smoothAz = SMOOTH_A * az + (1.0f - SMOOTH_A) * smoothAz;
  smoothAy = SMOOTH_A * ay + (1.0f - SMOOTH_A) * smoothAy;

  unsigned long now     = millis();
  bool          counted = false;

  // ── Rep detection ─────────────────────────────────────────────────────────
  if (mode == PUSHUP) {
    // Sequence: arms-up (az~1) → chest-down (az<0.75) → push-up (az>1.15) → count
    if (repState == WAIT_DOWN && smoothAz < PUSHUP_DOWN_THR) {
      repState = WAIT_UP;
    } else if (repState == WAIT_UP && smoothAz > PUSHUP_UP_THR) {
      if (now - lastRepTime > MIN_REP_MS) {
        repCount++;
        lastRepTime = now;
        counted = true;
      }
      repState = WAIT_DOWN;
    }
  } else {  // SITUP
    // Sequence: lying (|ay|~0) → sit-up (|ay|>0.6) → lie back (|ay|<0.25) → count
    if (repState == WAIT_UP && abs(smoothAy) > SITUP_UP_THR) {
      repState = WAIT_DOWN;
    } else if (repState == WAIT_DOWN && abs(smoothAy) < SITUP_DOWN_THR) {
      if (now - lastRepTime > MIN_REP_MS) {
        repCount++;
        lastRepTime = now;
        counted = true;
      }
      repState = WAIT_UP;
    }
  }

  if (counted) {
    tone(BUZZER_PIN, 1800, 120);
  }

  // ── Display ──────────────────────────────────────────────────────────────
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Mode header
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(mode == PUSHUP ? "  >> PUSH-UP <<  " : "  >>  SIT-UP <<  ");

  // Big rep counter (centred)
  display.setTextSize(4);
  int cx = (repCount < 10)  ? 52 :
           (repCount < 100) ? 28 : 4;
  display.setCursor(cx, 14);
  display.print(repCount);

  // Status line: live sensor value + what we're waiting for
  display.setTextSize(1);
  display.setCursor(0, 56);
  if (mode == PUSHUP) {
    display.printf("az:%.2f  wait:%s", smoothAz,
                   repState == WAIT_DOWN ? "DOWN" : "UP  ");
  } else {
    display.printf("ay:%.2f  wait:%s", smoothAy,
                   repState == WAIT_UP   ? "UP  " : "DOWN");
  }

  display.display();
  delay(50);
  yield();
}