/**
 * ProxyClient.h — HTTP communication with the LilyGO proxy
 *
 * Encapsulates all network I/O:
 *   - fetchDeliveryContext()  → GET /otp
 *   - reportEventToProxy()   → POST /event
 *   - requestFaceCheck()     → GET /face-check (+ UART fallback)
 *   - netLog()               → UDP debug broadcast
 *   - WiFi reconnect with exponential backoff (Article 2.4)
 */

#ifndef PROXY_CLIENT_H
#define PROXY_CLIENT_H

#include <Arduino.h>
#include "Config.h"

// ── Delivery context (shared with main sketch) ──
extern char  currentOtp[8];
extern char  activeDeliveryId[64];
extern bool  hasActiveDelivery;

// ── Last parsed status command from proxy ("UNLOCKING" / "LOCKED" / "") ──
extern String lastStatusCommand;

// ── Public API ──

/** printf-style logging to Serial + UDP broadcast. */
void netLog(const char *format, ...);

/** Scan WiFi for SmartTopBox_AP_* SSID, populate WIFI_SSID + HARDWARE_ID.
 *  Returns true if a matching AP was found. */
bool scanForProxy();

/** Non-blocking WiFi connection manager. Call from loop(). */
void maintainWiFiConnection(unsigned long now);

/** Start initial WiFi connection (call from setup()). */
void startWiFiConnection();

/** Fetch OTP + delivery_id + optional status from proxy GET /otp.
 *  Updates currentOtp, activeDeliveryId, hasActiveDelivery, lastStatusCommand. */
void fetchDeliveryContext();

/** Report lock event to proxy POST /event → Firebase.
 *  Returns true on HTTP 200/201. */
bool reportEventToProxy(bool otpValid, bool faceDetected, bool unlocked, bool thermalCutoff = false);

/**
 * Report a safety/health alert to proxy POST /event → Firebase.
 * Used by EC-04 (lockout), EC-21/22 (solenoid), EC-82 (keypad), EC-86 (display).
 *
 *   alertType: "LOCKOUT", "SOLENOID_STUCK", "THERMAL_CUTOFF",
 *              "DISPLAY_FAILED", "KEYPAD_STUCK", "ADMIN_OVERRIDE"
 *   details:   Short description (e.g. "STUCK_CLOSED after 3 retries")
 */
bool reportAlertToProxy(const char *alertType, const char *details);

/** Request face check via proxy GET /face-check, with UART Serial2 fallback.
 *  Returns: 1 = face detected, 0 = no face, -1 = error/timeout. */
int requestFaceCheck();

/** Report reed switch tamper (unauthorized lid-open) to Proxy.
 *  Proxy writes tamper state to Firebase, triggering push notifications. */
bool reportTamperToProxy();

#endif // PROXY_CLIENT_H
