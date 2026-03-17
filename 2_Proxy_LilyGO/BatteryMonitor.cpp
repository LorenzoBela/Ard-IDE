/**
 * BatteryMonitor.cpp — Battery voltage monitoring implementation
 */

#include "BatteryMonitor.h"

static float voltage = 0.0f;
static bool emaInitialized = false;

void batteryBegin() {
  pinMode(BATT_PIN, INPUT);
  analogReadResolution(12);

  float initial = (analogRead(BATT_PIN) / 4095.0f) * BATT_ADC_REF * BATT_DIVIDER_RATIO;
  voltage = initial;
  emaInitialized = true;
}

float batteryUpdate() {
  int raw = analogRead(BATT_PIN);
  float v = (raw / 4095.0f) * BATT_ADC_REF * BATT_DIVIDER_RATIO;

  if (!emaInitialized) {
    voltage = v;
    emaInitialized = true;
  } else {
    voltage = (BATT_EMA_ALPHA * v) + ((1.0f - BATT_EMA_ALPHA) * voltage);
  }

  if (voltage < BATT_MIN_VOLTAGE) voltage = BATT_MIN_VOLTAGE;
  if (voltage > BATT_MAX_VOLTAGE) voltage = BATT_MAX_VOLTAGE;
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
