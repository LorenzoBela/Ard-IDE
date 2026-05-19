/**
 * BatteryMonitor.cpp — Dual-channel battery voltage monitoring
 *
 * Channel A (GPIO35): 7.5V 2S Li-ion pack
 * Channel B (GPIO34): 12V  3S Li-ion pack
 * Both via standard "DC 0-25V Voltage Sensor Module" (ratio 5.0).
 */

#include "BatteryMonitor.h"
#include "LogTee.h"

#define Serial LogSerial

// ── Per-channel state ──
struct BattState {
  uint8_t pin;
  float   minV;
  float   maxV;
  unsigned long capMah;
  float   voltage;
  int     lastAdcRaw;
  int     lastPinMv;
  bool    emaInit;
};

static BattState ch[BATT_CH_COUNT];

// ── SOC lookup point ──
struct SocPoint { float v; int pct; };

// 2S Li-ion OCV curve (channel A — 7.5V pack)
static const SocPoint kSoc2S[] = {
  {8.40f, 100},
  {8.20f,  90},
  {7.95f,  80},
  {7.75f,  70},
  {7.55f,  60},
  {7.35f,  50},
  {7.15f,  40},
  {6.95f,  30},
  {6.75f,  20},
  {6.45f,  10},
  {6.00f,   0},
};

// 3S Li-ion OCV curve (channel B — 12V pack)
static const SocPoint kSoc3S[] = {
  {12.60f, 100},
  {12.30f,  90},
  {11.93f,  80},
  {11.63f,  70},
  {11.33f,  60},
  {11.03f,  50},
  {10.73f,  40},
  {10.43f,  30},
  {10.13f,  20},
  { 9.68f,  10},
  { 9.00f,   0},
};

// ── Generic SOC interpolation ──
static int socFromCurve(float v, const SocPoint *curve, int n) {
  if (v >= curve[0].v)     return 100;
  if (v <= curve[n - 1].v) return 0;

  for (int i = 0; i < (n - 1); i++) {
    const SocPoint &hi = curve[i];
    const SocPoint &lo = curve[i + 1];
    if (v <= hi.v && v >= lo.v) {
      float span = hi.v - lo.v;
      if (span <= 0.0f) return lo.pct;
      float t   = (v - lo.v) / span;
      float pct = lo.pct + t * (float)(hi.pct - lo.pct);
      if (pct <   0.0f) pct =   0.0f;
      if (pct > 100.0f) pct = 100.0f;
      return (int)(pct + 0.5f);
    }
  }
  return 0;
}

static int socForChannel(BattChannel idx) {
  float v = ch[idx].voltage;
  if (v < ch[idx].minV) v = ch[idx].minV;
  if (v > ch[idx].maxV) v = ch[idx].maxV;

  if (idx == BATT_CH_A) {
    return socFromCurve(v, kSoc2S, (int)(sizeof(kSoc2S) / sizeof(kSoc2S[0])));
  } else {
    return socFromCurve(v, kSoc3S, (int)(sizeof(kSoc3S) / sizeof(kSoc3S[0])));
  }
}

// ── Read one channel (averaged) ──
static float readChannelAveraged(BattChannel idx) {
  uint32_t sumRaw = 0;
  uint32_t sumMv  = 0;

  (void)analogRead(ch[idx].pin);

  for (int i = 0; i < BATT_SAMPLES; i++) {
    int raw = analogRead(ch[idx].pin);
    int mv  = analogReadMilliVolts(ch[idx].pin);
    if (raw < 0) raw = 0;
    if (mv  < 0) mv  = 0;
    sumRaw += (uint32_t)raw;
    sumMv  += (uint32_t)mv;
  }

  ch[idx].lastAdcRaw = (int)(sumRaw / (uint32_t)BATT_SAMPLES);
  ch[idx].lastPinMv  = (int)(sumMv  / (uint32_t)BATT_SAMPLES);

  float pinV = (float)ch[idx].lastPinMv / 1000.0f;

  // Fallback for cores where analogReadMilliVolts is missing or uncalibrated.
  if (ch[idx].lastPinMv <= 0 ||
      (ch[idx].lastPinMv < BATT_SENSOR_MIN_PIN_MV && ch[idx].lastAdcRaw > 0)) {
    pinV = ((float)ch[idx].lastAdcRaw / 4095.0f) * BATT_ADC_REF;
    ch[idx].lastPinMv = (int)(pinV * 1000.0f + 0.5f);
  }

  float batV = (pinV * BATT_DIVIDER_RATIO * BATT_CAL_GAIN) + BATT_CAL_OFFSET;
  return batV;
}

// ── Public API ──

void batteryBegin() {
  // Channel A — 7.5V on GPIO35
  ch[BATT_CH_A].pin     = BATT_PIN_A;
  ch[BATT_CH_A].minV    = BATT_A_MIN_VOLTAGE;
  ch[BATT_CH_A].maxV    = BATT_A_MAX_VOLTAGE;
  ch[BATT_CH_A].capMah  = BATT_A_CAPACITY_MAH;
  ch[BATT_CH_A].voltage = 0.0f;
  ch[BATT_CH_A].lastAdcRaw = 0;
  ch[BATT_CH_A].lastPinMv  = 0;
  ch[BATT_CH_A].emaInit    = false;

  // Channel B — 12V on GPIO34
  ch[BATT_CH_B].pin     = BATT_PIN_B;
  ch[BATT_CH_B].minV    = BATT_B_MIN_VOLTAGE;
  ch[BATT_CH_B].maxV    = BATT_B_MAX_VOLTAGE;
  ch[BATT_CH_B].capMah  = BATT_B_CAPACITY_MAH;
  ch[BATT_CH_B].voltage = 0.0f;
  ch[BATT_CH_B].lastAdcRaw = 0;
  ch[BATT_CH_B].lastPinMv  = 0;
  ch[BATT_CH_B].emaInit    = false;

  analogReadResolution(12);
  analogSetAttenuation(BATT_ADC_ATTEN);

  // Configure both ADC pins
  for (int i = 0; i < BATT_CH_COUNT; i++) {
    adcAttachPin(ch[i].pin);
    pinMode(ch[i].pin, INPUT);
    analogSetPinAttenuation(ch[i].pin, BATT_ADC_ATTEN);
  }

  // Prime readings
  for (int i = 0; i < BATT_CH_COUNT; i++) {
    float v = readChannelAveraged((BattChannel)i);
    ch[i].voltage = v;
    ch[i].emaInit = true;
  }

  Serial.printf("[BATT] Init CH-A (7.5V): %.2fV pin=%dmV raw=%d valid=%d\n",
                ch[0].voltage, ch[0].lastPinMv, ch[0].lastAdcRaw,
                (ch[0].lastPinMv >= BATT_SENSOR_MIN_PIN_MV) ? 1 : 0);
  Serial.printf("[BATT] Init CH-B (12V):  %.2fV pin=%dmV raw=%d valid=%d\n",
                ch[1].voltage, ch[1].lastPinMv, ch[1].lastAdcRaw,
                (ch[1].lastPinMv >= BATT_SENSOR_MIN_PIN_MV) ? 1 : 0);
}

void batteryUpdate() {
  for (int i = 0; i < BATT_CH_COUNT; i++) {
    float v = readChannelAveraged((BattChannel)i);

    // Skip floating / disconnected sensor
    if (ch[i].lastPinMv < BATT_SENSOR_MIN_PIN_MV) continue;

    if (!ch[i].emaInit) {
      ch[i].voltage = v;
      ch[i].emaInit = true;
    } else {
      ch[i].voltage = (BATT_EMA_ALPHA * v) + ((1.0f - BATT_EMA_ALPHA) * ch[i].voltage);
    }

    // Upper hard guard
    float hardMax = ch[i].maxV * 1.5f;
    if (ch[i].voltage > hardMax) ch[i].voltage = hardMax;
  }
}

float batteryGetVoltage(BattChannel idx) {
  if (idx >= BATT_CH_COUNT) return 0.0f;
  return ch[idx].voltage;
}

int batteryGetPercentage(BattChannel idx) {
  if (idx >= BATT_CH_COUNT) return 0;
  if (!batterySensorLooksValid(idx)) return 0;
  return socForChannel(idx);
}

unsigned long batteryGetCapacityMah(BattChannel idx) {
  if (idx >= BATT_CH_COUNT) return 0;
  return ch[idx].capMah;
}

unsigned long batteryGetRemainingMah(BattChannel idx) {
  int pct = batteryGetPercentage(idx);
  unsigned long cap = batteryGetCapacityMah(idx);
  return (unsigned long)((cap * (unsigned long)pct + 50UL) / 100UL);
}

int batteryGetPinMilliVolts(BattChannel idx) {
  if (idx >= BATT_CH_COUNT) return 0;
  return ch[idx].lastPinMv;
}

int batteryGetAdcRaw(BattChannel idx) {
  if (idx >= BATT_CH_COUNT) return 0;
  return ch[idx].lastAdcRaw;
}

bool batterySensorLooksValid(BattChannel idx) {
  if (idx >= BATT_CH_COUNT) return false;
  return ch[idx].lastPinMv >= BATT_SENSOR_MIN_PIN_MV;
}

bool batteryIsLow(BattChannel idx) {
  return batterySensorLooksValid(idx) && (batteryGetPercentage(idx) < BATT_LOW_PCT);
}

bool batteryIsCritical(BattChannel idx) {
  return batterySensorLooksValid(idx) && (batteryGetPercentage(idx) < BATT_CRITICAL_PCT);
}
