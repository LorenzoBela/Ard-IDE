/**
 * BatteryMonitor.h — Battery voltage monitoring for 1S 18650 Li-Ion cell
 *
 * Hardware:
 *   GPIO 35 (ADC1_CH7) — LILYGO T-SIM built-in battery sense
 *   On-board divider: 100k + 100k  → ratio 2.0
 *   1S 18650: 4.2V full → 3.0V cutoff
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

#define BATT_PIN              35
#define BATT_DIVIDER_RATIO    2.0f    // LilyGO on-board 100k/100k divider
#define BATT_MIN_VOLTAGE      3.0f    // empty 18650 cutoff
#define BATT_MAX_VOLTAGE      4.2f    // full 18650
#define BATT_SAMPLES          10
#define BATT_ADC_REF          3.3f
#define BATT_LOW_PCT          20
#define BATT_CRITICAL_PCT     10

void     batteryBegin();
float    batteryUpdate();         // Read + smooth, returns voltage
float    batteryGetVoltage();
int      batteryGetPercentage();
bool     batteryIsLow();          // < 20%
bool     batteryIsCritical();     // < 10%

#endif // BATTERY_MONITOR_H
