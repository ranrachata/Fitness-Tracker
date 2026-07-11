#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ── WiFi / MQTT config ────────────────────────────────────────────────────────
#define WIFI_SSID     "ranrachata"
#define WIFI_PASS     "29Nov2009"
#define MQTT_SERVER   "13.213.18.235"
#define MQTT_PORT     1883

// ── Hardware pins ─────────────────────────────────────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C
#define MPU_ADDR      0x68
#define SDA_PIN       6
#define SCL_PIN       7
#define BUZZER_PIN    5
#define BUTTON_PIN    10

// ── Push-up thresholds ────────────────────────────────────────────────────────
// Device: flat on upper back, face-down.
#define PUSHUP_DOWN_THR   0.85f
#define PUSHUP_UP_THR     1.05f

// ── Sit-up thresholds ─────────────────────────────────────────────────────────
// Device: flat on chest/abdomen.
#define SITUP_UP_THR      0.60f
#define SITUP_DOWN_THR    0.25f

#define MIN_REP_MS  600
#define SMOOTH_A    0.25f

// ── State ─────────────────────────────────────────────────────────────────────
enum Mode     { PUSHUP, SITUP };
enum RepState { WAIT_DOWN, WAIT_UP };

Mode          mode      = PUSHUP;
RepState      repState  = WAIT_DOWN;
int           repCount  = 0;
bool          lastBtn   = HIGH;
unsigned long lastRepTime   = 0;
unsigned long lastMqttRetry = 0;

float smoothAz = 1.0f;
float smoothAy = 0.0f;

String mqttTopic;   // "situp/<device_id>"
String deviceId;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Network helpers ───────────────────────────────────────────────────────────
void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t = millis();
  int frame = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);  display.print("Connecting WiFi");
    display.setCursor(0, 12); display.print(WIFI_SSID);
    display.setCursor(0, 26);
    for (int i = 0; i <= (frame % 4); i++) display.print(".");
    display.setCursor(0, 40);
    display.printf("%.1fs", (millis() - t) / 1000.0f);
    display.display();
    frame++;
    delay(400);
  }

  display.clearDisplay();
  display.setTextSize(1);
  if (WiFi.status() == WL_CONNECTED) {
    display.setCursor(0, 0);  display.print("WiFi Connected!");
    display.setCursor(0, 14); display.print(WiFi.localIP().toString());
  } else {
    display.setCursor(0, 0);  display.print("WiFi FAILED");
    display.setCursor(0, 14); display.print("Running offline");
  }
  display.display();
  delay(1800);
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  unsigned long now = millis();
  if (now - lastMqttRetry < 5000) return;
  lastMqttRetry = now;
  mqtt.connect(deviceId.c_str());
}

void publishState() {
  if (!mqtt.connected()) return;
  char payload[80];
  snprintf(payload, sizeof(payload),
           "{\"device\":\"%s\",\"mode\":\"%s\",\"count\":%d}",
           deviceId.c_str(),
           mode == PUSHUP ? "PUSH-UP" : "SIT-UP",
           repCount);
  mqtt.publish(mqttTopic.c_str(), payload, true);  // retained
}

// ── I2C / hardware helpers ────────────────────────────────────────────────────
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

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  delay(500);
  i2c_clear_bus();
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED failed"); for (;;) yield();
  }
  initMPU();

  // WiFi + derive device ID from MAC
  connectWifi();
  String mac = WiFi.macAddress();   // "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  deviceId  = "esp32_" + mac.substring(6);  // last 6 hex chars
  mqttTopic = "situp/" + deviceId;

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setKeepAlive(30);
  reconnectMQTT();

  Serial.printf("Device: %s  Topic: %s\n",
                deviceId.c_str(), mqttTopic.c_str());
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  mqtt.loop();
  reconnectMQTT();

  // ── Button: toggle mode + reset ──────────────────────────────────────────
  bool btn = digitalRead(BUTTON_PIN);
  if (btn == LOW && lastBtn == HIGH) {
    delay(50);
    mode      = (mode == PUSHUP) ? SITUP : PUSHUP;
    repCount  = 0;
    repState  = WAIT_DOWN;
    smoothAz  = 1.0f;
    smoothAy  = 0.0f;
    lastRepTime = 0;
    publishState();   // announce mode change immediately
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
  } else {
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
    publishState();
  }

  // ── Display ───────────────────────────────────────────────────────────────
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  bool mqttOk = mqtt.connected();

  // Row 0 — mode
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(mode == PUSHUP ? " >> PUSH-UP <<" : " >>  SIT-UP <<");

  // Row 1 — connection status (always visible, refreshes every loop)
  display.setCursor(0, 10);
  display.printf("W:%s M:%s  %s",
                 wifiOk ? "OK" : "--",
                 mqttOk ? "OK" : "--",
                 deviceId.c_str());

  // Divider
  display.drawFastHLine(0, 20, 128, SSD1306_WHITE);

  // Row 2 — rep counter (size 3, centred)
  display.setTextSize(3);
  int cx = (repCount < 10)  ? 52 :
           (repCount < 100) ? 34 : 16;
  display.setCursor(cx, 23);
  display.print(repCount);

  // Divider
  display.drawFastHLine(0, 49, 128, SSD1306_WHITE);

  // Row 3 — sensor + wait state
  display.setTextSize(1);
  display.setCursor(0, 52);
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
