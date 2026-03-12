/**
 * BatteryMonitor.h — Battery voltage monitoring for 3S Li-Ion pack
 *
 * Adapted from PlatformIO hardware/lib/BatteryMonitor.
 *
 * Hardware:
 *   GPIO 35 (ADC1_CH7) — LILYGO T-SIM built-in battery sense
 *   External divider: 100k (R1) + 30k (R2/GND) → ratio 4.33
 *   3S pack: 12.6V full → 9.0V cutoff
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

#define BATT_PIN              35
#define BATT_DIVIDER_RATIO    4.33f
#define BATT_MIN_VOLTAGE      9.0f    // 3.0V per cell
#define BATT_MAX_VOLTAGE      12.6f   // 4.2V per cell
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
