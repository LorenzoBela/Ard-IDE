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

// ── Live geofence state (parsed from proxy response) ──
extern int16_t geoDistMeters;    // Distance in meters to current target (-1 = unknown)
extern bool    geoInsideFence;   // True if inside the outer geofence radius
extern bool    isReturning;      // True if return-to-sender mode is active
extern char    deliveryPhase[12]; // PICKUP | IN_TRANSIT | DROPOFF | RETURN | NONE
extern bool    pickupInsideFence; // Inside pickup geofence
extern bool    dropoffInsideFence; // Inside dropoff geofence

struct ControllerDiagData {
	int battPct;
	float battVoltage;
	int rssi;
	int csq;
	bool gpsFix;
	bool lteConnected;
	bool modemOk;
	bool timeSynced;
	bool camUp;
	bool controllerUp;
	bool commandPending;
	int firebaseFailures;
	int commandStage;
	int connectivityState;
	bool photoUploadActive;
	int photoUploadProgress;
	unsigned long camAgeMs;
	unsigned long controllerAgeMs;
	unsigned long lteReconnectMs;
	unsigned long photoUploadAgeMs;
	unsigned long proxyUptimeMs;
};

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
 *  Updates currentOtp, activeDeliveryId, hasActiveDelivery, lastStatusCommand,
 *  and phase/geofence fields. */
void fetchDeliveryContext();

/** Force proxy to refresh delivery context from Firebase (GET /refresh-context). */
bool requestContextRefresh();

/** Lightweight keep-awake heartbeat to LilyGO (GET /ping). */
bool requestProxyPing();

/** Fetch cached diagnostics from proxy GET /diag (non-blocking budget). */
bool fetchDiagnostics(ControllerDiagData &out);

/** Report lock event to proxy POST /event → Firebase.
 *  Returns true on HTTP 200/201. */
bool reportEventToProxy(bool otpValid,
						bool faceDetected,
						bool unlocked,
						bool thermalCutoff = false,
						uint8_t faceAttempts = 0,
						bool faceRetryExhausted = false,
						bool fallbackRequired = false,
						const char *failureReason = "",
						unsigned long unlockLatencyMs = 0);

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
 *  Returns: 1 = face detected, 2 = low-light no-face,
 *           0 = no face, -1 = error/timeout. */
int requestFaceCheck();

/**
 * Request camera power mode via proxy GET /cam-power?mode=<sleep|wake>.
 * Returns true when proxy confirms command delivery.
 */
bool requestCameraPowerMode(bool wakeMode);

/** Report reed switch tamper (unauthorized lid-open) to Proxy.
 *  Proxy writes tamper state to Firebase, triggering push notifications. */
bool reportTamperToProxy();

/** Report admin command handling result to proxy POST /command-ack.
 *  status examples: accepted, rejected_state, executed, failed_unlock */
bool reportCommandAckToProxy(const char *command, const char *status, const char *details);

/** Verify personal PIN and request lock toggle authorization from proxy.
 *  Returns: 1 = allow unlock, 2 = allow relock, 0 = wrong PIN,
 *           -1 = PIN unavailable/stale context, -2 = HTTP error. */
int requestPersonalPinToggle(const char *pin, bool currentlyLocked);

#endif // PROXY_CLIENT_H
