/**
 * BatteryMonitor.h — Dual-channel battery voltage monitoring
 *
 * Hardware (this project setup):
 *   Channel A (GPIO34): 7.5V 2S Li-ion pack via voltage sensor module
 *   Channel B (GPIO35): 12V  3S Li-ion pack via voltage sensor module
 *
 * Both use standard "DC 0-25V Voltage Sensor Modules" (30kΩ/7.5kΩ divider,
 * multiply-back ratio = 5.0).  The modules output 0–5V proportional to
 * 0–25V input, but ESP32 ADC at 11 dB reads 0–3.3V so practical max ≈ 16.5V.
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// ── Channel identifiers ──
enum BattChannel : uint8_t {
  BATT_CH_A = 0,   // 7.5V pack on GPIO34
  BATT_CH_B = 1,   // 12V pack on GPIO35
  BATT_CH_COUNT = 2
};

// ── Pin assignments ──
#define BATT_PIN_A            34
#define BATT_PIN_B            35

// ── Voltage sensor module divider ratio ──
// Module: 30kΩ (R1) + 7.5kΩ (R2) → ratio = (30+7.5)/7.5 = 5.0
#define BATT_DIVIDER_RATIO    5.00f

// ── Channel A: 7.5V 2S Li-ion thresholds ──
#define BATT_A_MIN_VOLTAGE    6.00f    // 2S cutoff  (3.0V/cell)
#define BATT_A_MAX_VOLTAGE    8.40f    // 2S full    (4.2V/cell)
#define BATT_A_CAPACITY_MAH   24000UL

// ── Channel B: 12V 3S Li-ion thresholds ──
#define BATT_B_MIN_VOLTAGE    9.00f    // 3S cutoff  (3.0V/cell)
#define BATT_B_MAX_VOLTAGE    12.60f   // 3S full    (4.2V/cell)
#define BATT_B_CAPACITY_MAH   24000UL  // Adjust if different pack

// ── ADC settings (shared) ──
#define BATT_SAMPLES          16
#define BATT_ADC_REF          3.3f
#define BATT_ADC_ATTEN        ADC_11db

// ── Calibration knobs (tune per channel with DMM if needed) ──
#define BATT_CAL_GAIN         1.000f
#define BATT_CAL_OFFSET       0.00f

// ── Validity & smoothing ──
// With ratio 5.0, a valid 6V pack gives pin = 1200mV.
// Anything below 400mV is floating noise.
#define BATT_SENSOR_MIN_PIN_MV 400
#define BATT_EMA_ALPHA        0.15f

// ── Warning thresholds (percentage) ──
#define BATT_LOW_PCT          20
#define BATT_CRITICAL_PCT     10

// ── Public API ──
void     batteryBegin();
void     batteryUpdate();              // Read + smooth both channels

// Per-channel getters
float    batteryGetVoltage(BattChannel ch);
int      batteryGetPercentage(BattChannel ch);
unsigned long batteryGetCapacityMah(BattChannel ch);
unsigned long batteryGetRemainingMah(BattChannel ch);
int      batteryGetPinMilliVolts(BattChannel ch);
int      batteryGetAdcRaw(BattChannel ch);
bool     batterySensorLooksValid(BattChannel ch);
bool     batteryIsLow(BattChannel ch);
bool     batteryIsCritical(BattChannel ch);

// Legacy single-channel wrappers (default = channel A / 7.5V)
// Keeps existing call sites working without modification.
inline float    batteryGetVoltage()          { return batteryGetVoltage(BATT_CH_A); }
inline int      batteryGetPercentage()       { return batteryGetPercentage(BATT_CH_A); }
inline unsigned long batteryGetCapacityMah() { return batteryGetCapacityMah(BATT_CH_A); }
inline unsigned long batteryGetRemainingMah(){ return batteryGetRemainingMah(BATT_CH_A); }
inline int      batteryGetPinMilliVolts()    { return batteryGetPinMilliVolts(BATT_CH_A); }
inline int      batteryGetAdcRaw()           { return batteryGetAdcRaw(BATT_CH_A); }
inline bool     batterySensorLooksValid()    { return batterySensorLooksValid(BATT_CH_A); }
inline bool     batteryIsLow()               { return batteryIsLow(BATT_CH_A); }
inline bool     batteryIsCritical()          { return batteryIsCritical(BATT_CH_A); }

#endif // BATTERY_MONITOR_H
