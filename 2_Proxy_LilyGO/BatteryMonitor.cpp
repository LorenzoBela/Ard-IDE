/**
 * BatteryMonitor.cpp — Battery voltage monitoring implementation
 */

#include "BatteryMonitor.h"

static float samples[BATT_SAMPLES];
static int   sampleIdx  = 0;
static float voltage    = 0.0f;

void batteryBegin() {
  pinMode(BATT_PIN, INPUT);
  analogReadResolution(12);

  float initial = (analogRead(BATT_PIN) / 4095.0f) * BATT_ADC_REF * BATT_DIVIDER_RATIO;
  for (int i = 0; i < BATT_SAMPLES; i++) samples[i] = initial;
  voltage = initial;
}

float batteryUpdate() {
  int raw = analogRead(BATT_PIN);
  float v = (raw / 4095.0f) * BATT_ADC_REF * BATT_DIVIDER_RATIO;

  samples[sampleIdx] = v;
  sampleIdx = (sampleIdx + 1) % BATT_SAMPLES;

  float sum = 0;
  for (int i = 0; i < BATT_SAMPLES; i++) sum += samples[i];
  voltage = sum / BATT_SAMPLES;
  return voltage;
}

float batteryGetVoltage()    { return voltage; }

int batteryGetPercentage() {
  if (voltage <= BATT_MIN_VOLTAGE) return 0;
  if (voltage >= BATT_MAX_VOLTAGE) return 100;
  return (int)((voltage - BATT_MIN_VOLTAGE) / (BATT_MAX_VOLTAGE - BATT_MIN_VOLTAGE) * 100.0f);
}

bool batteryIsLow()      { return batteryGetPercentage() < BATT_LOW_PCT; }
bool batteryIsCritical()  { return batteryGetPercentage() < BATT_CRITICAL_PCT; }
