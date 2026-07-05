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

#define GYRO_X_THRESHOLD  100.0   // °/s
#define GYRO_Z_THRESHOLD  100.0   // °/s

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool useZAxis = false;          // false = X axis, true = Z axis
bool lastButtonState = HIGH;

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
  Wire.read(); Wire.read();
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  int16_t rawGy = Wire.read() << 8 | Wire.read();
  int16_t rawGz = Wire.read() << 8 | Wire.read();
  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;
  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;
}

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

void loop() {
  // Button toggle (debounced on falling edge)
  bool btnState = digitalRead(BUTTON_PIN);
  if (btnState == LOW && lastButtonState == HIGH) {
    useZAxis = !useZAxis;
    delay(50); // debounce
  }
  lastButtonState = btnState;

  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    if (!initAll()) { delay(500); return; }
  }

  float ax, ay, az, gx, gy, gz;
  readMPU(ax, ay, az, gx, gy, gz);

  bool triggered = useZAxis ? (abs(gz) > GYRO_Z_THRESHOLD)
                             : (abs(gx) > GYRO_X_THRESHOLD);

  if (triggered) {
    tone(BUZZER_PIN, 1500, 80);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(25, 0);
  display.print("=  MPU-6050  =");

  display.setCursor(0,  12); display.print("Acc (g)");
  display.setCursor(72, 12); display.print("Gyro(d/s)");
  display.drawFastVLine(66, 10, 44, SSD1306_WHITE);

  display.setCursor(0, 23); display.printf("X %+.2f", ax);
  display.setCursor(0, 33); display.printf("Y %+.2f", ay);
  display.setCursor(0, 43); display.printf("Z %+.2f", az);

  display.setCursor(72, 23); display.printf("%+6.1f", gx);
  display.setCursor(72, 33); display.printf("%+6.1f", gy);
  display.setCursor(72, 43); display.printf("%+6.1f", gz);

  display.setCursor(0, 56);
  if (useZAxis) {
    if (triggered) display.printf("** Gz %.0f d/s **", gz);
    else           display.printf("   Gz %.0f d/s   ", gz);
  } else {
    if (triggered) display.printf("** Gx %.0f d/s **", gx);
    else           display.printf("   Gx %.0f d/s   ", gx);
  }

  display.display();
  delay(80);
  yield();
}