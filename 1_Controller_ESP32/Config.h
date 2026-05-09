/**
 * Config.h — All constants, pins, and timing for Controller ESP32
 *
 * This file is automatically included by the Arduino IDE as a tab.
 * Centralises every magic number so the main sketch stays clean.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==================== NETWORK ====================
// WiFi -- auto-discovered by scanning for SmartTopBox_AP_* SSIDs
extern char WIFI_SSID[24];
static const char WIFI_PASSWORD[] = "topbox123";

// Proxy endpoints on LilyGO (default SoftAP gateway IP)
static const char PROXY_HOST[] = "192.168.4.1";
static const int  PROXY_PORT   = 8080;

// Hardware ID (derived from proxy's AP SSID at runtime)
extern char HARDWARE_ID[12];

// ==================== UART (ESP32-CAM fallback) ====================
#define CAM_UART_RX   16
#define CAM_UART_TX   17
#define CAM_UART_BAUD 9600

// ==================== PINS ====================
#define LOCK_PIN          32    // Relay / MOSFET control pin
#define LOCK_PIN_ON       LOW   // Active-Low relay: LOW = ON  (solenoid powered)
#define LOCK_PIN_OFF      HIGH  // Active-Low relay: HIGH = OFF (solenoid unpowered, safe during boot)

#define REED_SWITCH_PIN   4     // Lock position sensor (input-only GPIO)

// Reed polarity/config:
// - If magnet-closed reads HIGH, set REED_CLOSED_IS_HIGH to 1.
// - If magnet-closed reads LOW, set REED_CLOSED_IS_HIGH to 0.
#define REED_CLOSED_IS_HIGH      1
// For NO reed wired pin->switch->GND, use INPUT_PULLUP to avoid floating input.
#define REED_USE_INTERNAL_PULLUP 1

// ==================== KEYPAD ====================
static const uint8_t KP_ROWS = 4;
static const uint8_t KP_COLS = 3;

// ==================== TIMING ====================
#define WIFI_FIRST_RETRY_DELAY_MS 500
#define WIFI_RETRY_BASE_MS        1000
#define WIFI_RETRY_MAX_MS         32000
#define WIFI_RESCAN_INTERVAL_MS   45000
#define WIFI_RETRY_JITTER_MS      500
#define WIFI_UI_DISCONNECT_GRACE_MS 5000
#define LCD_MESSAGE_DURATION      2000
#define FACE_CHECK_TIMEOUT        15000
#define FACE_CHECK_WIFI_TIMEOUT_MS 8000
#define FACE_CHECK_UART_TIMEOUT_MS 10000
#define FACE_CHECK_MAX_ATTEMPTS   3
#define FACE_RETRY_DELAY_MS       1500
#define DELIVERY_CONTEXT_FETCH_MS 1000
#define PERSONAL_PIN_TIMEOUT_MS   15000
#define PERSONAL_PIN_MAX_LEN      6

// ==================== LIVENESS (MVP DEFAULTS) ====================
#define LIVENESS_BASE_INTERVAL_MS      10000
#define LIVENESS_RECOVERY_INTERVAL_MS   5000
#define LIVENESS_DEGRADED_MISSES           2
#define LIVENESS_DOWN_MISSES               3
#define LIVENESS_RECOVERY_SUCCESSES        2
#define CONTROLLER_PEER_STALE_MS       30000

// ==================== UTILITY DIAGNOSTICS MODE ====================
#define CONTROLLER_DIAG_UTILITY_TIMEOUT_MS 5000
#define CONTROLLER_DIAG_STANDBY_REFRESH_MS 30000
#define CONTROLLER_DIAG_IDLE_REFRESH_MS 10000
#define CONTROLLER_DIAG_UTILITY_REFRESH_MS 1000
#define CONTROLLER_DIAG_STALE_MS 15000
#define CONTROLLER_DIAG_HTTP_TIMEOUT_MS 300
#define CONTROLLER_DIAG_RETRY_BASE_MS 1000
#define CONTROLLER_DIAG_RETRY_MAX_MS 8000

// ==================== LOCK SAFETY (EC-21/22/95/96) ====================
#define LOCK_MAX_ACTIVE_MS        10000   // Hard thermal cutoff (10 sec hold)
#define LOCK_RETRY_MAX            3       // EC-21: Max unlock retry attempts
#define LOCK_RETRY_DELAY_MS       1000    // Delay between retries (increased from 200 to give user time to open lid)
#define LOCK_DEBOUNCE_MS          50      // EC-95: Reed switch debounce
#define TAMPER_COOLDOWN_MS        60000   // Suppress repeat tamper reports for 60 s
#define TAMPER_BOOT_GRACE_MS      8000    // Ignore tamper briefly after boot/reset (sensor settle)
#define TAMPER_OPEN_CONFIRM_MS    1500    // Reed must stay OPEN this long before tamper latches
#define LOCK_THERMAL_MAX_TEMP     80.0f   // EC-96: Modeled coil temp ceiling (C)
#define LOCK_HEAT_PER_ACTUATION   15.0f   // C added per actuation
#define LOCK_COOLING_RATE         0.05f   // C per ms of cooling
#define LOCK_CLOSE_ASSIST_HOLD_MS 5000    // '#' close-assist retract window (5 s)
#define LOCK_CLOSE_ASSIST_MAX_ATTEMPTS 3  // Max close-assist retries before alert
#define LOCK_AWAIT_CLOSE_WARN_MS   60000  // Warn if lid remains open for too long

// ==================== OTP LOCKOUT (EC-04) ====================
#define MAX_OTP_ATTEMPTS          5
#define OTP_LOCKOUT_MS            300000  // 5 minutes
#define OTP_ATTEMPT_COOLDOWN_MS   1000    // 1 s debounce between attempts

// ==================== OFFLINE OTP RECOVERY (MVP) ====================
#define OFFLINE_OTP_UPTIME_TTL_MS       300000 // 5 min cached OTP window while uptime is continuous
#define OFFLINE_BOOT_FALLBACK_ENTER_MS    8000 // allow degraded unlock mode after boot when still offline
#define OFFLINE_RECOVERY_TOKEN_ATTEMPTS      1 // bounded offline attempts after reboot without trusted time

// ==================== DISPLAY HEALTH (EC-86) ====================
#define DISPLAY_HEALTH_CHECK_MS   10000   // Check LCD I2C every 10 s
#define DISPLAY_I2C_ADDR          0x27    // PCF8574 backpack
#define DISPLAY_MAX_FAILURES      3       // Consecutive I2C fails -> FAILED
#define CONTROLLER_CAM_POWER_CMD_COOLDOWN_MS 15000 // Avoid command spam

// ==================== KEYPAD HEALTH (EC-82) ====================
#define KEYPAD_STUCK_THRESHOLD_MS 30000   // 30 s continuous hold = stuck

// ==================== ADMIN OVERRIDE (EC-77) ====================
#define ADMIN_OVERRIDE_TIMEOUT_MS 10000   // Auto-expire if not acted on

// ==================== BOOT SELF-TEST (POST) ====================
#define SELFTEST_PULSE_DURATION_MS    150   // Solenoid ON time per pulse (brief click)
#define SELFTEST_INTERVAL_MS          1500  // 1.5 s settle between pulses (5x thermal margin)
#define SELFTEST_PULSE_COUNT          3     // Number of test pulses

// ==================== BOOT RIDER AUTH ====================
#define BOOT_AUTH_IDLE_TIMEOUT_MS     300000 // 5 min idle → LCD dims (power-save)
#define BOOT_AUTH_MAX_LEN             6      // Max PIN length (matches personal PIN)

// ==================== UNJAM UTILITY (KEY 6) ====================
#define UNJAM_PULSE_DURATION_MS       300   // Solenoid ON per unjam pulse (longer than POST)
#define UNJAM_COOLDOWN_MS             2000  // Min 2 s between consecutive pulses
#define UNJAM_MAX_PULSES              5     // Max pulses before auto-lockout
#define UNJAM_COUNTER_RESET_MS        10000 // Auto-reset counter after 10 s idle

// ==================== OPEN-DOOR SAFETY ====================
#define DOOR_OPEN_LCD_WARN_INTERVAL_MS 30000  // LCD warning flash every 30 s
#define DOOR_OPEN_CRITICAL_MS          300000 // 5 min → escalated DOOR_OPEN_CRITICAL alert

// ==================== LCD AUTO-RECOVERY ====================
#define LCD_RECOVERY_POST_SOLENOID_MS  200    // Re-init LCD 200ms after solenoid de-energise (EMI settle)
#define LCD_PERIODIC_RESYNC_MS         60000  // Watchdog: re-init LCD every 60s to prevent gibberish

// ==================== UDP LOGGING ====================
#define UDP_LOG_PORT 5114

#endif // CONFIG_H
