/*
  Preliminary DAS test: Arduino UNO R4 WiFi + Grove shield.

  Sensors:
    - MCP9808  temperature   I2C 0x18
    - ADXL345  accelerometer I2C 0x53
    - MMA7660  accelerometer I2C 0x4C
    - Grove loudness sensor  analog LOUD_PIN

  Accelerations are in g, temperature in degC, loudness is the peak-to-peak
  of a short burst of ADC readings (0-1023).
  Lines starting with '#' are comments (boot messages, help).
  Sensors that are not found at boot are omitted from the JSON.

  Serial commands (same convention as the MADS sketch): <value><command>
    10p  sampling period in ms     x  toggle pause     ?  help
*/
#include <Arduino.h>
#include <Wire.h>

#define VERSION "1.0.0"
#define BAUD_RATE 115200
#define DEFAULT_PERIOD_MS 20UL  // 50 Hz
#define LOUD_PIN A0             // Grove shield analog port A0 (SIG on A0)
#define LOUD_SAMPLES 32

// I2C addresses
#define MCP9808_ADDR 0x18
#define ADXL345_ADDR 0x53
#define MMA7660_ADDR 0x4C

bool has_mcp = false, has_adxl = false, has_mma = false;

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// MCP9808
bool mcpBegin() {
  uint8_t b[2];
  if (!readRegs(MCP9808_ADDR, 0x06, b, 2) || b[0] != 0x00 || b[1] != 0x54) return false;
  writeReg(MCP9808_ADDR, 0x08, 0x03);  // 0.0625 degC
  return true;
}

bool mcpRead(float &t) {
  uint8_t b[2];
  if (!readRegs(MCP9808_ADDR, 0x05, b, 2)) return false;
  uint8_t upper = b[0] & 0x1F;
  if (upper & 0x10) {
    upper &= 0x0F;
    t = upper * 16.0f + b[1] / 16.0f - 256.0f;
  } else {
    t = upper * 16.0f + b[1] / 16.0f;
  }
  return true;
}

// ADXL345
bool adxlBegin() {
  uint8_t id;
  if (!readRegs(ADXL345_ADDR, 0x00, &id, 1) || id != 0xE5) return false;
  writeReg(ADXL345_ADDR, 0x2C, 0x0B);  // BW_RATE: 200 Hz output data rate
  writeReg(ADXL345_ADDR, 0x31, 0x0B);  // DATA_FORMAT: full resolution, +-16 g
  writeReg(ADXL345_ADDR, 0x2D, 0x08);  // POWER_CTL: measure
  return true;
}

bool adxlRead(float g[3]) {
  uint8_t b[6];
  if (!readRegs(ADXL345_ADDR, 0x32, b, 6)) return false;
  for (int i = 0; i < 3; i++) {
    int16_t raw = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
    g[i] = raw * 0.0039f;  // 3.9 mg/LSB in full mode
  }
  return true;
}

// MMA7660
bool mmaBegin() {
  writeReg(MMA7660_ADDR, 0x07, 0x00);  // MODE: standby 
  writeReg(MMA7660_ADDR, 0x08, 0x00);  // SR: 120 samples/s
  writeReg(MMA7660_ADDR, 0x07, 0x01);  // MODE: active
  uint8_t b[3];
  return readRegs(MMA7660_ADDR, 0x00, b, 3);
}

bool mmaRead(float g[3]) {
  uint8_t b[3];
  for (int tries = 0; tries < 5; tries++) {
    if (!readRegs(MMA7660_ADDR, 0x00, b, 3)) return false;
    if (!((b[0] | b[1] | b[2]) & 0x40)) {
      for (int i = 0; i < 3; i++) {
        int8_t v = b[i] & 0x3F;
        if (v > 31) v -= 64;
        g[i] = v / 21.33f;
      }
      return true;
    }
  }
  return false;
}

// Loudness
int loudRead() {
  int lo = 1023, hi = 0;
  for (int i = 0; i < LOUD_SAMPLES; i++) {
    int v = analogRead(LOUD_PIN);
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return hi - lo;
}

void printVec(const char *name, const float v[3], uint8_t dec) {
  Serial.print(",\"");
  Serial.print(name);
  Serial.print("\":[");
  for (int i = 0; i < 3; i++) {
    if (i) Serial.print(',');
    Serial.print(v[i], dec);
  }
  Serial.print(']');
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.print("# Starting DAS test v" VERSION "\n");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LOUD_PIN, INPUT);
  analogReadResolution(10);

  Wire.begin();
  Wire.setClock(400000);

  has_mcp = i2cPresent(MCP9808_ADDR) && mcpBegin();
  has_adxl = i2cPresent(ADXL345_ADDR) && adxlBegin();
  has_mma = i2cPresent(MMA7660_ADDR) && mmaBegin();
  Serial.print(has_mcp ? "# MCP9808 ok\n" : "# MCP9808 NOT found\n");
  Serial.print(has_adxl ? "# ADXL345 ok\n" : "# ADXL345 NOT found\n");
  Serial.print(has_mma ? "# MMA7660 ok\n" : "# MMA7660 NOT found\n");
}

void loop() {
  static unsigned long period_ms = DEFAULT_PERIOD_MS;
  static unsigned long prev = 0;
  static unsigned long acc = 0;
  static bool pause = false, led = false;

  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case '0' ... '9':
        acc = acc * 10 + (ch - '0');
        break;
      case 'p':
        period_ms = constrain(acc, 5UL, 10000UL);
        acc = 0;
        break;
      case 'x':
        pause = !pause;
        break;
      case '?':
        Serial.print("# Version: " VERSION "\n");
        Serial.print("# 10p set period to 10 ms (now ");
        Serial.print(period_ms);
        Serial.print(" ms), x toggle pause\n");
        break;
      case '\n':
      case '\r':
        break;
      default:
        acc = 0;
    }
  }

  if (pause) return;

  unsigned long now = millis();
  if (now - prev < period_ms) return;
  prev = now;

  digitalWrite(LED_BUILTIN, led);
  led = !led;

  float adxl[3], mma[3], temp;
  bool ok_adxl = has_adxl && adxlRead(adxl);
  bool ok_mma = has_mma && mmaRead(mma);
  bool ok_mcp = has_mcp && mcpRead(temp);
  int loud = loudRead();

  Serial.print("{\"millis\":");
  Serial.print(now);
  Serial.print(",\"data\":{\"loud\":");
  Serial.print(loud);
  if (ok_adxl) printVec("adxl", adxl, 3);
  if (ok_mma) printVec("mma", mma, 2);
  if (ok_mcp) {
    Serial.print(",\"temp\":");
    Serial.print(temp, 2);
  }
  Serial.print("}}\n");
}
