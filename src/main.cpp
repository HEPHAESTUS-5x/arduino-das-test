/*
  Preliminary DAS test: Arduino UNO R4 WiFi + Grove shield.

  Sensors:
    - MCP9808  temperature   I2C 0x18
    - ADXL345  accelerometer I2C 0x53  (hardware FIFO, 800 Hz by default)
    - MMA7660  accelerometer I2C 0x4C  (polled, ~100 Hz)
    - Grove loudness sensor  analog LOUD_PIN

  Output: one JSON line per batch (not per sample), so the serial/database rate
  stays at ~50 lines/s while the ADXL345 runs at its I2C-limited rate:
    {"millis":T,"data":{"fs":800,"n0":N,"ovr":K,"adxl":[x,y,z,x,y,z,...],
                        "mma":[x,y,z,...],"mma_ms":[t,...],"loud":L,"temp":C}}
  - adxl: raw LSB, 3.9 mg/LSB (full resolution, +-16 g), interleaved x,y,z.
  - n0: index (since boot) of the first ADXL sample of the batch; the sample
    time is n0/fs, not the jittery millis. ovr counts FIFO overruns (lost data).
  - mma: raw counts, 21.33 LSB/g, interleaved x,y,z, timestamps in mma_ms.
  - loud, temp (degC): sent about once per second only.
  Lines starting with '#' are comments (boot messages, help).
  Sensors that are not found at boot are omitted from the JSON.

  Serial commands: <value><command>
    13r  ADXL345 rate code: 10=100 Hz 11=200 Hz 12=400 Hz 13=800 Hz 14=1600 Hz
    x    toggle pause     ?  help
*/
#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>

#define VERSION "2.0.0"
#define BAUD_RATE 115200
#define DEFAULT_ADXL_RATE 13    // 3200 / 2^(15-code) Hz -> 800 Hz
#define ADXL_FIFO_MIN 16        // drain the FIFO when it holds at least this many samples
#define ADXL_BATCH_MAX 32       // FIFO depth
#define MMA_PERIOD_MS 10UL      // MMA7660 polling (the sensor runs at 120 Hz)
#define MMA_BATCH_MAX 32
#define SLOW_PERIOD_MS 1000UL   // loudness + temperature
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
uint8_t adxl_rate = DEFAULT_ADXL_RATE;

int adxlFs() { return 3200 >> (15 - adxl_rate); }

void adxlFifoReset() {
  writeReg(ADXL345_ADDR, 0x38, 0x00);  // FIFO_CTL: bypass (clears the FIFO)
  writeReg(ADXL345_ADDR, 0x38, 0x80);  // FIFO_CTL: stream mode
}

bool adxlBegin() {
  uint8_t id;
  if (!readRegs(ADXL345_ADDR, 0x00, &id, 1) || id != 0xE5) return false;
  writeReg(ADXL345_ADDR, 0x2D, 0x00);       // POWER_CTL: standby while configuring
  writeReg(ADXL345_ADDR, 0x2C, adxl_rate);  // BW_RATE: output data rate
  writeReg(ADXL345_ADDR, 0x31, 0x0B);       // DATA_FORMAT: full resolution, +-16 g
  adxlFifoReset();
  writeReg(ADXL345_ADDR, 0x2D, 0x08);       // POWER_CTL: measure
  return true;
}

uint8_t adxlFifoLevel() {
  uint8_t s = 0;
  readRegs(ADXL345_ADDR, 0x39, &s, 1);  // FIFO_STATUS
  return s & 0x3F;
}

bool adxlOverrun() {
  uint8_t s = 0;
  readRegs(ADXL345_ADDR, 0x30, &s, 1);  // INT_SOURCE (reading clears it)
  return s & 0x01;
}

// pops one sample (raw LSB) from the FIFO
bool adxlRead(int16_t v[3]) {
  uint8_t b[6];
  if (!readRegs(ADXL345_ADDR, 0x32, b, 6)) return false;
  for (int i = 0; i < 3; i++) v[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
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

// raw counts (6 bit signed, 21.33 LSB/g)
bool mmaRead(int8_t v[3]) {
  uint8_t b[3];
  for (int tries = 0; tries < 5; tries++) {
    if (!readRegs(MMA7660_ADDR, 0x00, b, 3)) return false;
    if (!((b[0] | b[1] | b[2]) & 0x40)) {
      for (int i = 0; i < 3; i++) {
        v[i] = b[i] & 0x3F;
        if (v[i] > 31) v[i] -= 64;
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

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.print("# Starting DAS test v" VERSION "\n");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LOUD_PIN, INPUT);
  analogReadResolution(10);

  Wire.begin();
  Wire.setClock(400000);  // ADXL345 at 800 Hz keeps the bus ~25% busy

  has_mcp = i2cPresent(MCP9808_ADDR) && mcpBegin();
  has_adxl = i2cPresent(ADXL345_ADDR) && adxlBegin();
  has_mma = i2cPresent(MMA7660_ADDR) && mmaBegin();
  Serial.print(has_mcp ? "# MCP9808 ok\n" : "# MCP9808 NOT found\n");
  Serial.print(has_adxl ? "# ADXL345 ok\n" : "# ADXL345 NOT found\n");
  Serial.print(has_mma ? "# MMA7660 ok\n" : "# MMA7660 NOT found\n");
}

static char out[2048];

static size_t add(size_t len, const char *fmt, ...) {
  if (len >= sizeof(out)) return len;
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(out + len, sizeof(out) - len, fmt, ap);
  va_end(ap);
  return n < 0 ? len : min(len + (size_t)n, sizeof(out));
}

void loop() {
  static unsigned long acc = 0;
  static bool pause = false, led = false;
  static unsigned long prev_mma = 0, prev_slow = 0;
  static uint32_t n_adxl = 0, n_ovr = 0;
  static int16_t adxl[ADXL_BATCH_MAX * 3];
  static int8_t mma[MMA_BATCH_MAX * 3];
  static uint32_t mma_ms[MMA_BATCH_MAX];
  static uint8_t n_mma = 0;

  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case '0' ... '9':
        acc = acc * 10 + (ch - '0');
        break;
      case 'r':
        adxl_rate = constrain(acc, 10UL, 14UL);
        acc = 0;
        if (has_adxl) adxlBegin();
        break;
      case 'x':
        pause = !pause;
        if (!pause && has_adxl) adxlFifoReset();
        break;
      case '?':
        Serial.print("# Version: " VERSION "\n");
        Serial.print("# 13r set ADXL345 rate code 10..14 (now ");
        Serial.print(adxl_rate);
        Serial.print(" = ");
        Serial.print(adxlFs());
        Serial.print(" Hz), x toggle pause\n");
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

  // MMA7660: slow sensor, poll it and keep the samples for the next batch
  if (has_mma && now - prev_mma >= MMA_PERIOD_MS && n_mma < MMA_BATCH_MAX) {
    prev_mma = now;
    if (mmaRead(&mma[n_mma * 3])) mma_ms[n_mma++] = now;
  }

  // ADXL345: wait for enough samples in the FIFO, then drain it in one go
  uint8_t n = 0;
  if (has_adxl) {
    uint8_t level = adxlFifoLevel();
    if (level >= ADXL_FIFO_MIN) {
      if (level > ADXL_BATCH_MAX) level = ADXL_BATCH_MAX;
      while (n < level && adxlRead(&adxl[n * 3])) n++;
      if (adxlOverrun()) n_ovr++;
    }
  }

  bool slow = now - prev_slow >= SLOW_PERIOD_MS;
  // without the ADXL345 there is no FIFO to pace the output: send MMA data in groups
  if (!(n > 0 || (!has_adxl && (n_mma >= 4 || slow)))) return;

  size_t len = add(0, "{\"millis\":%lu,\"data\":{\"fs\":%d,\"n0\":%lu,\"ovr\":%lu",
                   now, adxlFs(), (unsigned long)n_adxl, (unsigned long)n_ovr);
  if (n) {
    len = add(len, ",\"adxl\":[");
    for (int i = 0; i < n * 3; i++) len = add(len, i ? ",%d" : "%d", adxl[i]);
    len = add(len, "]");
    n_adxl += n;
  }
  if (n_mma) {
    len = add(len, ",\"mma\":[");
    for (int i = 0; i < n_mma * 3; i++) len = add(len, i ? ",%d" : "%d", mma[i]);
    len = add(len, "],\"mma_ms\":[");
    for (int i = 0; i < n_mma; i++) len = add(len, i ? ",%lu" : "%lu", (unsigned long)mma_ms[i]);
    len = add(len, "]");
    n_mma = 0;
  }
  if (slow) {
    prev_slow = now;
    len = add(len, ",\"loud\":%d", loudRead());
    float temp;
    if (has_mcp && mcpRead(temp)) {
      int c = (int)(temp * 100 + (temp < 0 ? -0.5f : 0.5f));
      len = add(len, ",\"temp\":%s%d.%02d", c < 0 ? "-" : "", abs(c) / 100, abs(c) % 100);
    }
  }
  len = add(len, "}}\n");
  Serial.write((const uint8_t *)out, len);

  digitalWrite(LED_BUILTIN, led);
  led = !led;
}
