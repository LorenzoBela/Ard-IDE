/**
 * BatteryMonitor.h — Battery voltage monitoring for VBUS-powered LilyGO
 *
 * Hardware (this project setup):
 *   - 7.4V 2S Li-ion pack (24000 mAh) → buck converter → ~4.9V
 *   - Buck output wired to VBUS + GND on LilyGO T-SIM A7670E
 *   - LilyGO onboard voltage divider (2× 100kΩ) on GPIO35
 *     reads the power rail: pin_mV ≈ VBUS / 2
 *
 * Note:
 *   Because a buck converter holds its output steady until the input
 *   drops below its minimum operating voltage, the VBUS reading is
 *   mostly flat (~4.9V) and then drops sharply near end-of-charge.
 *   We map the useful range (VBUS 3.3V–5.1V) to 0–100 % SOC.
 *   For precise pack-level monitoring, route a sense wire from BAT+
 *   (pre-buck) through an external divider into a free ADC pin.
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

// ── ADC pin ──
#define BATT_PIN              35

// ── Voltage divider ──
// LilyGO onboard: 100k / 100k = ratio 2.0
#define BATT_DIVIDER_RATIO    2.00f

// ── VBUS voltage thresholds (post-buck) ──
// Buck converter nominally outputs 4.9V.
// As the 2S pack discharges below the converter's dropout the output sags.
//   Full   = 5.10V  (buck running comfortably)
//   Empty  = 3.30V  (ESP32 brownout imminent)
#define BATT_MIN_VOLTAGE      3.30f
#define BATT_MAX_VOLTAGE      5.10f

// ── Pack capacity (informational) ──
#define BATT_CAPACITY_MAH     24000UL

// ── ADC settings ──
#define BATT_SAMPLES          16       // More samples → smoother reading
#define BATT_ADC_REF          3.3f
#define BATT_ADC_ATTEN        ADC_11db

// ── Calibration knobs (tune with DMM) ──
#define BATT_CAL_GAIN         1.000f
#define BATT_CAL_OFFSET       0.00f

// ── Validity & smoothing ──
// A real VBUS reading through the 2:1 divider gives 1650–2450 mV on the pin.
// A floating GPIO35 reads 50–300 mV of noise.  Set threshold to 1000 mV
// so floating-pin noise is always rejected and the AT+CBC modem fallback fires.
#define BATT_SENSOR_MIN_PIN_MV 1000
#define BATT_EMA_ALPHA        0.15f    // Slower filter (less jitter)

// ── Warning thresholds ──
#define BATT_LOW_PCT          20
#define BATT_CRITICAL_PCT     10

// ── Public API ──
void     batteryBegin();
float    batteryUpdate();              // Read + smooth, returns voltage
float    batteryGetVoltage();
int      batteryGetPercentage();
unsigned long batteryGetCapacityMah();
unsigned long batteryGetRemainingMah();
int      batteryGetPinMilliVolts();
int      batteryGetAdcRaw();
bool     batterySensorLooksValid();
bool     batteryIsLow();               // < 20 %
bool     batteryIsCritical();          // < 10 %

// Fallback: feed voltage from an external source (e.g. modem AT+CBC)
// when GPIO35 is floating / not connected to the power rail.
void     batterySetExternalVoltage(float volts);

#endif // BATTERY_MONITOR_H
