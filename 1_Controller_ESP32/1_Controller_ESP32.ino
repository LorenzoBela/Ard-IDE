/**
 * 1_Controller_ESP32.ino — Parcel-Safe Controller (State Machine)
 *
 * This is the MAIN SKETCH uploaded to the ESP32 DevKit via Arduino IDE.
 * All hardware I/O and safety logic live in their own tab files:
 *   - Config.h          → Constants, pins, timing
 *   - HardwareIO.*      → LCD, Keypad
 *   - ProxyClient.*     → WiFi, HTTP, UDP logging, UART fallback
 *   - LockSafety.*      → Solenoid with retry, reed switch, thermal (EC-21/22/95/96)
 *   - OTPLockout.*      → Attempt counter with lockout (EC-04)
 *   - DisplayHealth.*   → LCD I2C health + LED/buzzer fallback (EC-86)
 *   - AdminOverride.h   → Graceful admin remote-unlock (EC-77)
 *   - KeypadHealth.h    → Stuck key detection (EC-82)
 *
 * Flow:
 *   1. Connects to LilyGO SoftAP for internet access.
 *   2. Fetches live OTP + delivery_id from proxy every 1 s.
 *   3. Stays in STANDBY until an active delivery is available.
 *   4. User enters PIN on keypad → validates against live OTP.
 *   5. If OTP correct → requests face check from proxy GET /face-check.
 *   6. Face detected → fires solenoid (Article 1.2 Photo-First).
 *   7. Reports event to proxy POST /event → written to Firebase.
 *
 * Rules enforced:
 *   - No delay() in loop()       (Article 2.1)
 *   - char[] buffers, no String   (Article 2.2)
 *   - Thermal + retry solenoid    (Article 2.3 / EC-21/96)
 *   - WiFi exponential backoff    (Article 2.4)
 */

#include <WiFi.h>
#include <soc/soc.h>              // WRITE_PERI_REG macro
#include <soc/rtc_cntl_reg.h>    // RTC_CNTL_BROWN_OUT_REG address
#include <Preferences.h>
#include "Config.h"
#include "HardwareIO.h"
#include "ProxyClient.h"
#include "LockSafety.h"
#include "OTPLockout.h"
#include "DisplayHealth.h"
#include "AdminOverride.h"
#include "KeypadHealth.h"

// ==================== STATE MACHINE ====================
enum TesterState {
  STATE_BOOT_SELFTEST,
  STATE_BOOT_AUTH,
  STATE_CONNECTING_WIFI,
  STATE_STANDBY,
  STATE_IDLE,
  STATE_ENTERING_PIN,
  STATE_PERSONAL_PIN_ENTRY,
  STATE_VERIFYING_OTP,
  STATE_REQUESTING_FACE,
  STATE_UNLOCKING,
  STATE_OWNER_UNLOCKED,
  STATE_AWAITING_CLOSE,
  STATE_RELOCKING,
  STATE_SHOW_MESSAGE
};

static TesterState currentState = STATE_BOOT_SELFTEST;

// ── Input buffers ──
static char    inputCode[8];
static uint8_t inputLen = 0;
static char    personalPinCode[8];
static uint8_t personalPinLen = 0;

// ── Timing trackers ──
static unsigned long lastDeliveryContextFetch = 0;
static unsigned long messageStartAt           = 0;
static unsigned long lastDisplayCheck         = 0;
static unsigned long personalPinExpiresAt     = 0;
static const unsigned long KEYPAD_GEO_CHECK_FIRST_DELAY_MS = 800;
static const unsigned long KEYPAD_GEO_CHECK_RETRY_DELAY_MS = 900;
static const uint8_t KEYPAD_GEO_CHECK_ATTEMPTS = 5;
static unsigned long keypadGeoCheckAt = 0;
static uint8_t keypadGeoChecksRemaining = 0;

// ── Face check ──
static bool faceCheckPending = false;
static uint8_t faceAttemptCount = 0;
static unsigned long nextFaceAttemptAt = 0;

// ── Safety modules ──
static AdminOverride adminOverride;
static KeypadHealth  keypadHealth;
enum ControllerPowerPolicy {
  POWER_POLICY_ACTIVE,
  POWER_POLICY_IDLE
};

static ControllerPowerPolicy powerPolicy = POWER_POLICY_ACTIVE;
static unsigned long lastCamPowerCmdAt = 0;
static bool cameraSleepExpected = false;

enum PeerHealthState {
  PEER_HEALTHY,
  PEER_DEGRADED,
  PEER_DOWN
};

static PeerHealthState proxyHealthState = PEER_HEALTHY;
static PeerHealthState camHealthState = PEER_HEALTHY;
static uint8_t proxyMissCount = 0;
static uint8_t proxyRecoveryCount = 0;
static uint8_t camMissCount = 0;
static uint8_t camRecoveryCount = 0;
static bool camDownNotified = false;
static bool offlineModeAnnounced = false;

static ControllerDiagData diagCache = {
  0,
  0.0f,
  -999,
  -1,
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  0,
  0,
  0,
  0,
  0,
  0,
  0
};
static bool diagCacheValid = false;
static unsigned long diagLastSuccessAt = 0;
static unsigned long diagNextPollAt = 0;
static unsigned long diagRetryBackoffMs = CONTROLLER_DIAG_RETRY_BASE_MS;
static uint32_t diagSuccessCount = 0;
static uint32_t diagFailCount = 0;
static uint32_t diagStaleDisplayCount = 0;

static bool utilityModeActive = false;
static char utilityModeKey = '\0';
static unsigned long utilityModeExpiresAt = 0;
static bool ownerSessionActive = false;
static bool deferredContextTransition = false;
static unsigned long closeAssistUntil = 0;
static unsigned long awaitingCloseSince = 0;
static uint8_t closeAssistAttempts = 0;
static bool awaitingCloseWarnSent = false;
static unsigned long connectStateEnteredAt = 0;
static unsigned long otpVerifyStartedAt = 0;
static unsigned long lastUnlockLatencyMs = 0;

// ── Boot self-test ──
static bool selfTestWarned = false;
// (selfTestStarted removed — self-test always starts immediately in reed-only mode)

// ── Boot rider authentication ──
static bool    bootAuthDone = false;      // Rider passed PIN (gates key 6 unjam)
static bool    bootPhaseComplete = false; // Boot sequence finished (selftest+auth done)
static char    bootAuthCode[8];
static uint8_t bootAuthLen = 0;
static unsigned long bootAuthLastInputAt = 0;

// ── LCD auto-recovery ──
static bool          prevSolenoidState     = false; // Edge detection for solenoid de-energise
static unsigned long lcdRecoveryScheduledAt = 0;    // millis() when post-solenoid recovery was scheduled
static unsigned long lastLcdPeriodicResync  = 0;    // millis() of last periodic watchdog re-sync

// ── Open-door safety ──
static unsigned long lastDoorOpenWarnAt = 0;
static bool doorOpenCriticalSent = false;

// ── Unjam auto-reset tracking ──
static unsigned long lastUnjamKeyAt = 0;

// ── Offline OTP recovery context (reboot-safe fallback) ──
static Preferences offlineCtxPrefs;
static bool offlineCtxStoreReady = false;
static bool offlineContextFromBoot = false;
static uint8_t offlineRecoveryTokenRemaining = 0;
static unsigned long lastOnlineOtpSyncAt = 0;

static const char *OFFLINE_NS = "ctrlOff";
static const char *OFFLINE_KEY_ACTIVE = "active";
static const char *OFFLINE_KEY_OTP = "otp";
static const char *OFFLINE_KEY_DELIVERY = "del";
static const char *OFFLINE_KEY_TOKEN = "tok";

// ── Handoff checkpoint persistence (brownout-safe replay) ──
static Preferences handoffPrefs;
static bool handoffStoreReady = false;
static bool pendingRecoveryReport = false;
static char recoveredCheckpointStage[24] = "";
static char recoveredCheckpointDeliveryId[64] = "";

static const char *CP_NS = "ctrlHndf";
static const char *CP_KEY_ACTIVE = "active";
static const char *CP_KEY_DELIVERY = "delId";
static const char *CP_KEY_STAGE = "stage";
static const char *CP_KEY_RETRY = "retry";
static const char *CP_KEY_UPDATED = "upd";
static const char *CP_KEY_CRC = "crc";

// ── Forward declarations ──
void enterState(TesterState newState);
void handleStateMachine(unsigned long now);
void resetFaceSession();
void applyIdlePowerPolicy(unsigned long now);
void applyActivePowerPolicy(unsigned long now);
void pollDiagnosticsIfDue(unsigned long now);
void startUtilityMode(char utilityKey, unsigned long now);
void renderUtilityMode(unsigned long now);
void stopUtilityMode(unsigned long now);
void checkpointHandoffState(const char *stage, uint8_t retryCounter);
void clearHandoffCheckpoint();
void updateHandoffCheckpointForState(TesterState state);
void initOfflineContextStore();
void persistOfflineContextSnapshot();
void clearOfflineContextSnapshot();
void noteOnlineContextSync(unsigned long now);
void consumeOfflineRecoveryToken(const char *reason);
bool isOfflineOtpEligible(unsigned long now);
void updatePeerHealthOnDiagSuccess(const ControllerDiagData &diag);
void updatePeerHealthOnDiagFailure();

static bool hasLoadedOtp() {
  return currentOtp[0] != '\0';
}

static bool isPinEligibleNow() {
  // PIN entry is allowed only when inside the geofence with an active OTP.
  return hasActiveDelivery && hasLoadedOtp() && geoInsideFence;
}

// Format distance for LCD: >9999m → "10km", >999m → "1.2km", else "450m"
static void formatDistance(int16_t meters, char *buf, size_t bufLen) {
  if (meters < 0) {
    snprintf(buf, bufLen, "Check");
  } else if (meters >= 10000) {
    snprintf(buf, bufLen, "%dkm", meters / 1000);
  } else if (meters >= 1000) {
    snprintf(buf, bufLen, "%d.%dkm", meters / 1000, (meters % 1000) / 100);
  } else {
    snprintf(buf, bufLen, "%dm", meters);
  }
}

static uint32_t computeCheckpointCrc(const char *deliveryId,
                                     const char *stage,
                                     uint8_t retryCounter,
                                     unsigned long updatedAtMs) {
  uint32_t crc = 2166136261UL;
  const char *del = deliveryId ? deliveryId : "";
  const char *stg = stage ? stage : "";

  while (*del) {
    crc ^= (uint8_t)*del++;
    crc *= 16777619UL;
  }
  while (*stg) {
    crc ^= (uint8_t)*stg++;
    crc *= 16777619UL;
  }
  crc ^= retryCounter;
  crc *= 16777619UL;
  crc ^= (uint32_t)updatedAtMs;
  crc *= 16777619UL;
  return crc;
}

static void initHandoffCheckpointStore() {
  handoffStoreReady = handoffPrefs.begin(CP_NS, false);
  if (!handoffStoreReady) {
    netLog("[CHECKPOINT] NVS unavailable\n");
    return;
  }

  if (!handoffPrefs.getBool(CP_KEY_ACTIVE, false)) {
    return;
  }

  String del = handoffPrefs.getString(CP_KEY_DELIVERY, "");
  String stg = handoffPrefs.getString(CP_KEY_STAGE, "");
  uint8_t retryCounter = (uint8_t)handoffPrefs.getUChar(CP_KEY_RETRY, 0);
  unsigned long updatedAtMs = handoffPrefs.getULong(CP_KEY_UPDATED, 0);
  uint32_t storedCrc = handoffPrefs.getULong(CP_KEY_CRC, 0);

  uint32_t computed = computeCheckpointCrc(del.c_str(), stg.c_str(), retryCounter,
                                           updatedAtMs);
  if (storedCrc == 0 || storedCrc != computed) {
    netLog("[CHECKPOINT] CRC mismatch; clearing stale checkpoint\n");
    clearHandoffCheckpoint();
    return;
  }

  strncpy(recoveredCheckpointStage, stg.c_str(), sizeof(recoveredCheckpointStage) - 1);
  recoveredCheckpointStage[sizeof(recoveredCheckpointStage) - 1] = '\0';
  strncpy(recoveredCheckpointDeliveryId, del.c_str(), sizeof(recoveredCheckpointDeliveryId) - 1);
  recoveredCheckpointDeliveryId[sizeof(recoveredCheckpointDeliveryId) - 1] = '\0';
  pendingRecoveryReport = true;

  netLog("[CHECKPOINT] Recovered stage=%s delivery=%s retry=%u\n",
         recoveredCheckpointStage,
         recoveredCheckpointDeliveryId,
         (unsigned int)retryCounter);
}

void checkpointHandoffState(const char *stage, uint8_t retryCounter) {
  if (!handoffStoreReady || !hasActiveDelivery) return;

  const char *deliveryId = activeDeliveryId[0] != '\0' ? activeDeliveryId : "UNKNOWN";
  unsigned long now = millis();
  uint32_t crc = computeCheckpointCrc(deliveryId, stage, retryCounter, now);

  handoffPrefs.putBool(CP_KEY_ACTIVE, true);
  handoffPrefs.putString(CP_KEY_DELIVERY, deliveryId);
  handoffPrefs.putString(CP_KEY_STAGE, stage ? stage : "unknown");
  handoffPrefs.putUChar(CP_KEY_RETRY, retryCounter);
  handoffPrefs.putULong(CP_KEY_UPDATED, now);
  handoffPrefs.putULong(CP_KEY_CRC, crc);
}

void clearHandoffCheckpoint() {
  if (!handoffStoreReady) return;
  handoffPrefs.putBool(CP_KEY_ACTIVE, false);
  handoffPrefs.remove(CP_KEY_DELIVERY);
  handoffPrefs.remove(CP_KEY_STAGE);
  handoffPrefs.remove(CP_KEY_RETRY);
  handoffPrefs.remove(CP_KEY_UPDATED);
  handoffPrefs.remove(CP_KEY_CRC);
}

void updateHandoffCheckpointForState(TesterState state) {
  if (!handoffStoreReady) return;

  switch (state) {
  case STATE_STANDBY:
  case STATE_RELOCKING:
    clearHandoffCheckpoint();
    break;
  case STATE_IDLE:
    if (hasActiveDelivery && isPinEligibleNow()) {
      checkpointHandoffState("handoff_started", (uint8_t)getFailedAttemptCount());
    }
    break;
  case STATE_VERIFYING_OTP:
    checkpointHandoffState("unlock_attempted", (uint8_t)getFailedAttemptCount());
    break;
  case STATE_REQUESTING_FACE:
    checkpointHandoffState("face_requested", faceAttemptCount);
    break;
  case STATE_UNLOCKING:
    checkpointHandoffState("unlock_confirmed", faceAttemptCount);
    break;
  case STATE_AWAITING_CLOSE:
    checkpointHandoffState("awaiting_close", closeAssistAttempts);
    break;
  default:
    break;
  }
}

static const char *peerHealthStr(PeerHealthState state) {
  switch (state) {
  case PEER_HEALTHY:
    return "HEALTHY";
  case PEER_DEGRADED:
    return "DEGRADED";
  case PEER_DOWN:
    return "DOWN";
  default:
    return "UNKNOWN";
  }
}

static char peerHealthCode(PeerHealthState state) {
  switch (state) {
  case PEER_HEALTHY:
    return 'H';
  case PEER_DEGRADED:
    return 'D';
  case PEER_DOWN:
    return 'X';
  default:
    return '?';
  }
}

void initOfflineContextStore() {
  offlineCtxStoreReady = offlineCtxPrefs.begin(OFFLINE_NS, false);
  if (!offlineCtxStoreReady) {
    netLog("[OFFLINE] NVS unavailable; reboot fallback disabled\n");
    return;
  }

  if (!offlineCtxPrefs.getBool(OFFLINE_KEY_ACTIVE, false)) {
    return;
  }

  String otp = offlineCtxPrefs.getString(OFFLINE_KEY_OTP, "");
  String delivery = offlineCtxPrefs.getString(OFFLINE_KEY_DELIVERY, "");
  uint8_t token =
      (uint8_t)offlineCtxPrefs.getUChar(OFFLINE_KEY_TOKEN,
                                        OFFLINE_RECOVERY_TOKEN_ATTEMPTS);
  if (token > OFFLINE_RECOVERY_TOKEN_ATTEMPTS) {
    token = OFFLINE_RECOVERY_TOKEN_ATTEMPTS;
  }

  if (delivery.length() == 0 || delivery.length() >= sizeof(activeDeliveryId) || otp.length() > 6) {
    clearOfflineContextSnapshot();
    return;
  }

  strncpy(activeDeliveryId, delivery.c_str(), sizeof(activeDeliveryId) - 1);
  activeDeliveryId[sizeof(activeDeliveryId) - 1] = '\0';
  hasActiveDelivery = true;

  if (otp.length() > 0) {
    strncpy(currentOtp, otp.c_str(), sizeof(currentOtp) - 1);
    currentOtp[sizeof(currentOtp) - 1] = '\0';
  } else {
    currentOtp[0] = '\0';
  }

  offlineContextFromBoot = true;
  offlineRecoveryTokenRemaining = token;
  netLog("[OFFLINE] Restored context for delivery %s (token=%u)\n",
         activeDeliveryId, (unsigned int)offlineRecoveryTokenRemaining);
}

void persistOfflineContextSnapshot() {
  if (!offlineCtxStoreReady || !hasActiveDelivery || !hasLoadedOtp()) {
    return;
  }

  offlineCtxPrefs.putBool(OFFLINE_KEY_ACTIVE, true);
  offlineCtxPrefs.putString(OFFLINE_KEY_OTP, currentOtp);
  offlineCtxPrefs.putString(OFFLINE_KEY_DELIVERY, activeDeliveryId);
  offlineCtxPrefs.putUChar(OFFLINE_KEY_TOKEN, OFFLINE_RECOVERY_TOKEN_ATTEMPTS);
}

void clearOfflineContextSnapshot() {
  if (offlineCtxStoreReady) {
    offlineCtxPrefs.putBool(OFFLINE_KEY_ACTIVE, false);
    offlineCtxPrefs.remove(OFFLINE_KEY_OTP);
    offlineCtxPrefs.remove(OFFLINE_KEY_DELIVERY);
    offlineCtxPrefs.remove(OFFLINE_KEY_TOKEN);
  }

  offlineContextFromBoot = false;
  offlineRecoveryTokenRemaining = 0;
  lastOnlineOtpSyncAt = 0;
}

void noteOnlineContextSync(unsigned long now) {
  lastOnlineOtpSyncAt = now;
  offlineContextFromBoot = false;
  offlineRecoveryTokenRemaining = OFFLINE_RECOVERY_TOKEN_ATTEMPTS;
  persistOfflineContextSnapshot();
}

void noteOnlineDeliveryOnly(unsigned long now) {
  lastOnlineOtpSyncAt = now;
  offlineContextFromBoot = false;
  offlineRecoveryTokenRemaining = OFFLINE_RECOVERY_TOKEN_ATTEMPTS;
  if (!offlineCtxStoreReady || !hasActiveDelivery) return;
  offlineCtxPrefs.putBool(OFFLINE_KEY_ACTIVE, true);
  offlineCtxPrefs.putString(OFFLINE_KEY_OTP, "");
  offlineCtxPrefs.putString(OFFLINE_KEY_DELIVERY, activeDeliveryId);
  offlineCtxPrefs.putUChar(OFFLINE_KEY_TOKEN, OFFLINE_RECOVERY_TOKEN_ATTEMPTS);
}

void consumeOfflineRecoveryToken(const char *reason) {
  if (!offlineContextFromBoot || offlineRecoveryTokenRemaining == 0) {
    return;
  }

  offlineRecoveryTokenRemaining--;
  if (offlineCtxStoreReady) {
    offlineCtxPrefs.putUChar(OFFLINE_KEY_TOKEN, offlineRecoveryTokenRemaining);
  }

  netLog("[OFFLINE] Consumed reboot token (%s), remaining=%u\n",
         reason ? reason : "unknown",
         (unsigned int)offlineRecoveryTokenRemaining);
}

bool isOfflineOtpEligible(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  if (!hasActiveDelivery || !hasLoadedOtp()) {
    return false;
  }

  if (offlineContextFromBoot) {
    return offlineRecoveryTokenRemaining > 0;
  }

  if (lastOnlineOtpSyncAt == 0) {
    return false;
  }

  return (now - lastOnlineOtpSyncAt) <= OFFLINE_OTP_UPTIME_TTL_MS;
}

void updatePeerHealthOnDiagSuccess(const ControllerDiagData &diag) {
  proxyMissCount = 0;
  if (proxyHealthState != PEER_HEALTHY) {
    proxyRecoveryCount++;
    if (proxyRecoveryCount >= LIVENESS_RECOVERY_SUCCESSES) {
      proxyHealthState = PEER_HEALTHY;
      proxyRecoveryCount = 0;
      netLog("[LIVENESS] Proxy -> %s\n", peerHealthStr(proxyHealthState));
    }
  }

  bool camNowUp = diag.camUp;
  if (camNowUp) {
    camMissCount = 0;
    if (camHealthState != PEER_HEALTHY) {
      camRecoveryCount++;
      if (camRecoveryCount >= LIVENESS_RECOVERY_SUCCESSES) {
        camHealthState = PEER_HEALTHY;
        camRecoveryCount = 0;
        camDownNotified = false;
        netLog("[LIVENESS] Camera -> %s\n", peerHealthStr(camHealthState));
      }
    }
  } else {
    camRecoveryCount = 0;
    if (camMissCount < 255) {
      camMissCount++;
    }

    PeerHealthState nextState = camHealthState;
    if (camMissCount >= LIVENESS_DOWN_MISSES) {
      nextState = PEER_DOWN;
    } else if (camMissCount >= LIVENESS_DEGRADED_MISSES) {
      nextState = PEER_DEGRADED;
    }

    if (nextState != camHealthState) {
      camHealthState = nextState;
      netLog("[LIVENESS] Camera -> %s (age=%lums)\n",
             peerHealthStr(camHealthState), diag.camAgeMs);

      if (camHealthState == PEER_DOWN && !camDownNotified) {
        camDownNotified = true;
        reportAlertToProxy("CAM_DOWN", "diag_cam_stale");
      }
    }
  }
}

void updatePeerHealthOnDiagFailure() {
  camRecoveryCount = 0;
  proxyRecoveryCount = 0;

  if (proxyMissCount < 255) {
    proxyMissCount++;
  }

  PeerHealthState nextState = proxyHealthState;
  if (proxyMissCount >= LIVENESS_DOWN_MISSES) {
    nextState = PEER_DOWN;
  } else if (proxyMissCount >= LIVENESS_DEGRADED_MISSES) {
    nextState = PEER_DEGRADED;
  }

  if (nextState != proxyHealthState) {
    proxyHealthState = nextState;
    netLog("[LIVENESS] Proxy -> %s\n", peerHealthStr(proxyHealthState));
  }
}

static void renderIdleJourneyStatus() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (isOfflineOtpEligible(now)) {
      if (offlineContextFromBoot) {
        char line1[17] = "";
        snprintf(line1, sizeof(line1), "Token:%u", (unsigned int)offlineRecoveryTokenRemaining);
        updateDisplay("Offline recovery", line1);
      } else {
        updateDisplay("Offline cached", "PIN window active");
      }
    } else {
      updateDisplay("Offline", "Reconnecting...");
    }
    return;
  }

  if (proxyHealthState == PEER_DOWN) {
    if (isOfflineOtpEligible(now)) {
      updateDisplay("Proxy offline", "Cached OTP mode");
    } else {
      updateDisplay("Proxy offline", "Waiting recover");
    }
    return;
  }

  if (!hasActiveDelivery) {
    updateDisplay("Box Ready", "No Delivery");
    return;
  }

  // PIN-ready: inside geofence with loaded OTP.
  if (isPinEligibleNow()) {
    if (isReturning) {
      updateDisplay("Return PIN:", "PIN: ");
    } else {
      updateDisplay("Enter PIN:", "PIN: ");
    }
    return;
  }

  // Outside geofence: show the next action instead of stale distance.
  if (isReturning) {
    updateDisplay("Pickup locked", "Hold 9 check");
  } else {
    updateDisplay("Dropoff locked", "Hold 9 check");
  }
}

static void clearKeypadGeoCheck() {
  keypadGeoCheckAt = 0;
  keypadGeoChecksRemaining = 0;
}

static void scheduleKeypadGeoCheck(unsigned long now) {
  keypadGeoCheckAt = now + KEYPAD_GEO_CHECK_FIRST_DELAY_MS;
  keypadGeoChecksRemaining = KEYPAD_GEO_CHECK_ATTEMPTS;
}

static void showKeypadGeoCheckFailure(unsigned long now) {
  clearKeypadGeoCheck();
  inputLen = 0;
  inputCode[0] = '\0';

  if (!isDisplayFailed()) {
    if (!hasActiveDelivery) {
      updateDisplay("No delivery", "Check app");
    } else if (!hasLoadedOtp()) {
      updateDisplay("OTP syncing", "Hold 9 again");
    } else if (geoDistMeters >= 0) {
      updateDisplay(isReturning ? "Outside pickup" : "Outside dropoff",
                    "PIN still locked");
    } else {
      updateDisplay("Location stale", "Use app refresh");
    }
  } else {
    fallbackError();
  }

  messageStartAt = now;
  currentState = STATE_SHOW_MESSAGE;
}

static void processKeypadGeoCheck(unsigned long now) {
  if (keypadGeoChecksRemaining == 0 || now < keypadGeoCheckAt) {
    return;
  }

  if (!bootPhaseComplete ||
      !(currentState == STATE_STANDBY || currentState == STATE_IDLE ||
        currentState == STATE_ENTERING_PIN || currentState == STATE_SHOW_MESSAGE)) {
    clearKeypadGeoCheck();
    return;
  }

  if (WiFi.status() != WL_CONNECTED || currentState == STATE_CONNECTING_WIFI) {
    clearKeypadGeoCheck();
    if (!isDisplayFailed()) {
      updateDisplay("Offline", "Cannot check");
    } else {
      fallbackError();
    }
    messageStartAt = now;
    currentState = STATE_SHOW_MESSAGE;
    return;
  }

  fetchDeliveryContext();
  lastDeliveryContextFetch = now;

  if (isPinEligibleNow()) {
    clearKeypadGeoCheck();
    inputLen = 0;
    inputCode[0] = '\0';
    enterState(STATE_IDLE);
    return;
  }

  keypadGeoChecksRemaining--;
  if (keypadGeoChecksRemaining > 0) {
    keypadGeoCheckAt = now + KEYPAD_GEO_CHECK_RETRY_DELAY_MS;
    if (!isDisplayFailed()) {
      updateDisplay("Checking area", "Wait...");
    } else {
      fallbackKeyFeedback();
    }
    messageStartAt = now;
    currentState = STATE_SHOW_MESSAGE;
    return;
  }

  showKeypadGeoCheckFailure(now);
}

static bool isUtilityKey(char key) {
  return key == '1' || key == '2' || key == '3';
}

static bool isDiagnosticsFresh(unsigned long now) {
  return diagCacheValid && (now - diagLastSuccessAt <= CONTROLLER_DIAG_STALE_MS);
}

static bool isCameraLikelyUp(unsigned long now) {
  // Do not block unlock flow on unknown diagnostics; only trust fresh CAM_DOWN.
  if (!isDiagnosticsFresh(now)) {
    return true;
  }
  return camHealthState != PEER_DOWN;
}

static unsigned long getDiagRefreshInterval() {
  if (utilityModeActive) {
    return CONTROLLER_DIAG_UTILITY_REFRESH_MS;
  }
  if (currentState == STATE_STANDBY) {
    return CONTROLLER_DIAG_STANDBY_REFRESH_MS;
  }
  if (currentState == STATE_IDLE || currentState == STATE_ENTERING_PIN ||
      currentState == STATE_PERSONAL_PIN_ENTRY) {
    return CONTROLLER_DIAG_IDLE_REFRESH_MS;
  }
  return CONTROLLER_DIAG_STANDBY_REFRESH_MS;
}

void renderUtilityMode(unsigned long now) {
  if (isDisplayFailed()) {
    fallbackKeyFeedback();
    return;
  }

  char line0[17] = "";
  char line1[17] = "";
  bool fresh = isDiagnosticsFresh(now);

  if (utilityModeKey == '1') {
    snprintf(line0, sizeof(line0), "Battery %3d%%", diagCache.battPct);
    snprintf(line1, sizeof(line1), "%0.2fV %s", diagCache.battVoltage,
             fresh ? "LIVE" : (diagCacheValid ? "STALE" : "NO DATA"));
  } else if (utilityModeKey == '2') {
    if (diagCache.rssi <= -998) {
      snprintf(line0, sizeof(line0), "LTE signal N/A");
    } else {
      snprintf(line0, sizeof(line0), "LTE %d dBm", diagCache.rssi);
    }
    snprintf(line1, sizeof(line1), "GPS:%s %s", diagCache.gpsFix ? "LOCK" : "WAIT",
             fresh ? "LIVE" : (diagCacheValid ? "STALE" : "NO DATA"));
  } else {
    if (diagCache.lteConnected && diagCache.modemOk) {
      snprintf(line0, sizeof(line0), "Net: CONNECTED");
    } else {
      snprintf(line0, sizeof(line0), "Net: OFFLINE");
    }

    if (diagCache.timeSynced && diagCache.firebaseFailures == 0) {
      snprintf(line1, sizeof(line1), "Sync: OK");
    } else {
      snprintf(line1, sizeof(line1), "HTTP/Sync Err:%d", diagCache.firebaseFailures);
    }
  }

  updateDisplay(line0, line1);
  if (!fresh && diagCacheValid) {
    diagStaleDisplayCount++;
  }
}

void startUtilityMode(char utilityKey, unsigned long now) {
  utilityModeActive = true;
  utilityModeKey = utilityKey;
  utilityModeExpiresAt = now + CONTROLLER_DIAG_UTILITY_TIMEOUT_MS;
  displayBacklightOn();

  // Fetch fresh diagnostics immediately so first render shows LIVE data.
  if (WiFi.status() == WL_CONNECTED) {
    ControllerDiagData data = diagCache;
    if (fetchDiagnostics(data)) {
      diagCache = data;
      diagCacheValid = true;
      diagLastSuccessAt = now;
      diagRetryBackoffMs = CONTROLLER_DIAG_RETRY_BASE_MS;
      diagSuccessCount++;
    }
  }

  renderUtilityMode(now);
  // Schedule next poll at the utility refresh rate (1 s).
  diagNextPollAt = now + CONTROLLER_DIAG_UTILITY_REFRESH_MS;
}

void stopUtilityMode(unsigned long now) {
  if (!utilityModeActive) {
    return;
  }

  utilityModeActive = false;
  utilityModeKey = '\0';
  utilityModeExpiresAt = 0;

  if (currentState == STATE_STANDBY) {
    displayBacklightOff();
  } else if (currentState == STATE_IDLE || currentState == STATE_ENTERING_PIN) {
    renderIdleJourneyStatus();
  }

  diagNextPollAt = now + getDiagRefreshInterval();
}

void pollDiagnosticsIfDue(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED || currentState == STATE_CONNECTING_WIFI) {
    return;
  }

    if (!(currentState == STATE_STANDBY || currentState == STATE_IDLE ||
      currentState == STATE_ENTERING_PIN ||
      currentState == STATE_PERSONAL_PIN_ENTRY)) {
    return;
  }

  if (now < diagNextPollAt) {
    return;
  }

  unsigned long interval = getDiagRefreshInterval();
  ControllerDiagData data = diagCache;
  bool ok = fetchDiagnostics(data);
  if (ok) {
    diagCache = data;
    diagCacheValid = true;
    diagLastSuccessAt = now;
    diagRetryBackoffMs = CONTROLLER_DIAG_RETRY_BASE_MS;
    diagNextPollAt = now + interval;
    diagSuccessCount++;
    updatePeerHealthOnDiagSuccess(data);

    if (utilityModeActive) {
      renderUtilityMode(now);
    }
  } else {
    diagFailCount++;
    updatePeerHealthOnDiagFailure();
    if (diagRetryBackoffMs < CONTROLLER_DIAG_RETRY_MAX_MS) {
      diagRetryBackoffMs *= 2;
      if (diagRetryBackoffMs > CONTROLLER_DIAG_RETRY_MAX_MS) {
        diagRetryBackoffMs = CONTROLLER_DIAG_RETRY_MAX_MS;
      }
    }
    unsigned long retryDelay = diagRetryBackoffMs;
    if (retryDelay > interval) {
      retryDelay = interval;
    }
    diagNextPollAt = now + retryDelay;
  }

  if ((diagSuccessCount + diagFailCount) % 20 == 0) {
    netLog("[DIAG] success=%lu fail=%lu stale=%lu\n",
           (unsigned long)diagSuccessCount,
           (unsigned long)diagFailCount,
           (unsigned long)diagStaleDisplayCount);
  }
}

// ==================== SETUP ====================
void setup() {
  // ── Brownout detector disable (ESP32 hardware workaround) ──
  // The BOD fires during peripheral inrush + WiFi radio init when
  // powered via USB (thin cable / weak source). Disabling the detector
  // prevents a reset loop while the power rail stabilises. This is safe
  // because the ESP32 operates correctly at brief dips; only sustained
  // under-voltage would cause real corruption — which would manifest as
  // a watchdog reset instead.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n=== Parcel-Safe Controller v3 (Modular) ==="));
  Serial.println(F("[BOOT] Brownout detector disabled (USB power workaround)"));

  initHardwareIO();
  initLock();
  initDisplayHealth();
  initOtpLockoutPersistence();
  initOfflineContextStore();
  initHandoffCheckpointStore();

  // Boot self-test runs concurrently with WiFi — both are non-blocking.
  // SECURITY: Always skip solenoid pulses at boot to prevent:
  //   1. Brownout from solenoid inrush + WiFi drawing simultaneously
  //   2. Momentary unauthorized unlock windows (Article 1.2)
  // Reed-only verification confirms sensor integrity without energising.
  initSelfTest(true);   // Reed-only mode — NO solenoid pulses
  startWiFiConnection();
  currentState = STATE_BOOT_SELFTEST;
  if (!isDisplayFailed()) {
    updateDisplay("Self-Test...", "Reed check only");
  }

  Serial.printf("  Proxy: %s:%d\n", PROXY_HOST, PROXY_PORT);
  Serial.println(F("  Modules: LockSafety, OTPLockout, DisplayHealth, AdminOverride, KeypadHealth"));
  Serial.println(F("  Boot: SelfTest -> WiFi -> RiderAuth -> Operational"));
}

// ==================== MAIN LOOP (non-blocking) ====================
void loop() {
  unsigned long now = millis();

  // ── 1. WiFi monitoring (runs in all states except boot self-test) ──
  if (WiFi.status() != WL_CONNECTED && currentState != STATE_CONNECTING_WIFI &&
      currentState != STATE_BOOT_SELFTEST) {
    if (isOfflineOtpEligible(now)) {
      // Keep keypad flow alive in degraded mode while reconnecting in background.
      if (!offlineModeAnnounced) {
        netLog("[WIFI] Offline degraded mode active (cached OTP policy)\n");
        offlineModeAnnounced = true;
      }
    } else {
      offlineModeAnnounced = false;
      netLog("[WIFI] Connection lost, reconnecting...\n");
      enterState(STATE_CONNECTING_WIFI);
      WiFi.disconnect();
      WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
    }
  } else if (WiFi.status() == WL_CONNECTED) {
    offlineModeAnnounced = false;
  }
  maintainWiFiConnection(now);

  // ── 2. LockSafety tick (thermal model, reed debounce, cutoff) ──
  bool lockTimedOut = maintainLockSafety(now);
  if (lockTimedOut && currentState == STATE_UNLOCKING) {
    netLog("[LOCK] Solenoid CUT OFF (Thermal Safety or Auto-Relock Timeout)\n");
    if (!isDisplayFailed()) {
      updateDisplay("Unlock timeout", "#=Close assist");
    } else {
      fallbackError();
    }
    reportEventToProxy(false, false, false, true);
    reportAlertToProxy("THERMAL_CUTOFF", "10s_safety_limit");

    // Solenoid hold ended; do not report LOCKED until reed confirms CLOSED.
    reportCommandAckToProxy("UNLOCKING", "timeout_waiting_close",
                            "awaiting_physical_close");

    enterState(STATE_AWAITING_CLOSE);
  }

  if (lockTimedOut && currentState == STATE_AWAITING_CLOSE) {
    if (!isDisplayFailed()) {
      updateDisplay("Assist timeout", "# to retry");
    }
  }

  // ── 2b. Reed switch tamper detection ──
  if (isTamperDetected()) {
    netLog("[TAMPER] Unauthorized lid-open detected! Reporting...\n");
    if (!isDisplayFailed()) {
      updateDisplay("TAMPER ALERT!", "Box breached!");
    } else {
      fallbackError();
    }
    reportTamperToProxy();
    clearTamper();
  }

  // ── 3. Periodic delivery context fetch ──
  if (WiFi.status() == WL_CONNECTED && currentState != STATE_CONNECTING_WIFI) {
    if (now - lastDeliveryContextFetch >= DELIVERY_CONTEXT_FETCH_MS) {
      bool wasActive = hasActiveDelivery;
      String oldStatus = lastStatusCommand;
      fetchDeliveryContext();
      lastDeliveryContextFetch = now;

      if (hasActiveDelivery) {
        if (hasLoadedOtp()) {
          noteOnlineContextSync(now);
        } else {
          noteOnlineDeliveryOnly(now);
        }
      } else {
        clearOfflineContextSnapshot();
      }

      if (currentState == STATE_OWNER_UNLOCKED) {
        if ((hasActiveDelivery != wasActive) || (oldStatus != lastStatusCommand)) {
          deferredContextTransition = true;
          netLog("[OWNER] Deferred delivery transition while owner session active\n");
        }
      } else {
        // Delivery became active
        if (hasActiveDelivery && !wasActive) {
          netLog("[CONTEXT] Delivery active! OTP: %s | ID: %s\n", currentOtp, activeDeliveryId);
          resetOtpAttempts();
          if (currentState == STATE_STANDBY || currentState == STATE_SHOW_MESSAGE) {
            enterState(STATE_IDLE);
          }
        }
        // Delivery cleared
        else if (!hasActiveDelivery && wasActive) {
          netLog("[CONTEXT] Delivery cleared — returning to standby\n");
          resetOtpAttempts();
          clearHandoffCheckpoint();
          if (currentState == STATE_IDLE || currentState == STATE_ENTERING_PIN ||
              currentState == STATE_PERSONAL_PIN_ENTRY) {
            enterState(STATE_STANDBY);
          }
        }
        // Status update while already active (e.g. geofence transit/pickup updates)
        else if (hasActiveDelivery && wasActive && oldStatus != lastStatusCommand) {
          if (currentState == STATE_IDLE || currentState == STATE_ENTERING_PIN ||
              currentState == STATE_PERSONAL_PIN_ENTRY) {
             enterState(STATE_IDLE); // Re-render LCD
          }
        }

        // EC-77: Handle remote lock/unlock commands
        if (lastStatusCommand == "REBOOT_ALL") {
          netLog("[REMOTE] REBOOT_ALL received. Controller restarting...\n");
          lastStatusCommand = "";
          reportCommandAckToProxy("REBOOT_ALL", "accepted", "controller_restarting");
          delay(150);
          ESP.restart();
        } else if (lastStatusCommand == "UNLOCKING" && currentState != STATE_UNLOCKING) {
          netLog("[EC-77] Remote UNLOCK (admin override)\n");
          lastStatusCommand = ""; // Clear immediately to prevent loop on next timeout
          adminOverride.trigger(now);
          reportCommandAckToProxy("UNLOCKING", "accepted", "state_transition_unlocking");
          // Clear any in-progress keypad input
          inputLen = 0;
          inputCode[0] = '\0';
          enterState(STATE_UNLOCKING);
          reportAlertToProxy("ADMIN_OVERRIDE", "remote_unlock");
        } else if (lastStatusCommand == "LOCKED") {
          netLog("[CONTEXT] Remote LOCK commanded\n");
          lastStatusCommand = ""; // Clear immediately
          if (currentState == STATE_UNLOCKING) {
            // Normal remote relock flow
            LockStatus ls = tryLock();
            if (ls == LOCK_OK) {
              reportCommandAckToProxy("LOCKED", "accepted", "state_transition_relocking");
              enterState(STATE_RELOCKING);
            } else if (ls == LOCK_STUCK_OPEN) {
              reportCommandAckToProxy("LOCKED", "waiting_close", "reed_open");
              enterState(STATE_AWAITING_CLOSE);
            } else {
              reportCommandAckToProxy("LOCKED", "failed_lock", lockStatusStr(ls));
              reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
            }
          } else {
            // Already locked or in another state — just actuate and stay in current state or go to relocking
            LockStatus ls = tryLock();
            if (ls == LOCK_OK) {
              reportCommandAckToProxy("LOCKED", "executed", "already_locked");
            } else if (ls == LOCK_STUCK_OPEN) {
              reportCommandAckToProxy("LOCKED", "waiting_close", "reed_open");
              enterState(STATE_AWAITING_CLOSE);
            } else {
              reportCommandAckToProxy("LOCKED", "failed_lock", lockStatusStr(ls));
              reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
            }
          }
        }
      }
    }
  }

  // ── 4. EC-86: Display health check ──
  processKeypadGeoCheck(now);

  if (now - lastDisplayCheck >= DISPLAY_HEALTH_CHECK_MS) {
    if (!checkDisplayHealth()) {
      reportAlertToProxy("DISPLAY_FAILED",
                         isDisplayFailed() ? "lcd_dead" : "lcd_degraded");
    }
    lastDisplayCheck = now;
  }

  // ── 4a. LCD auto-recovery: post-solenoid EMI + periodic watchdog ──
  {
    // Detect solenoid de-energise edge → schedule LCD re-init after EMI settles.
    bool solNow = isSolenoidActive();
    if (prevSolenoidState && !solNow) {
      lcdRecoveryScheduledAt = now + LCD_RECOVERY_POST_SOLENOID_MS;
    }
    prevSolenoidState = solNow;

    // Execute scheduled post-solenoid recovery.
    if (lcdRecoveryScheduledAt > 0 && now >= lcdRecoveryScheduledAt) {
      lcdRecoveryScheduledAt = 0;
      if (!isDisplayFailed()) {
        recoverDisplay();
        lastLcdPeriodicResync = now; // Reset periodic timer too
        Serial.println(F("[LCD] Auto-recovery after solenoid EMI"));
      }
    }

    // Periodic watchdog: re-sync LCD every 60s to prevent accumulated drift.
    if (lastLcdPeriodicResync == 0) {
      lastLcdPeriodicResync = now; // Init on first loop
    } else if (now - lastLcdPeriodicResync >= LCD_PERIODIC_RESYNC_MS) {
      if (!isDisplayFailed()) {
        recoverDisplay();
        Serial.println(F("[LCD] Periodic watchdog re-sync"));
      }
      lastLcdPeriodicResync = now;
    }
  }

  // ── 4b. Utility diagnostics refresh and timeout handling ──
  pollDiagnosticsIfDue(now);
  if (utilityModeActive && now >= utilityModeExpiresAt) {
    stopUtilityMode(now);
  }
  if (currentState == STATE_PERSONAL_PIN_ENTRY && personalPinExpiresAt > 0 &&
      now >= personalPinExpiresAt) {
    netLog("[PIN] Personal PIN mode timeout\n");
    inputLen = 0;
    inputCode[0] = '\0';
    personalPinLen = 0;
    personalPinCode[0] = '\0';
    enterState(hasActiveDelivery ? STATE_IDLE : STATE_STANDBY);
  }

  // ── 5. EC-82: Keypad stuck detection ──
  {
    char heldKey = getHeldKey(); // Thread-safe: written by keypadTask on Core 1
    keypadHealth.update(heldKey, now);

    // Long-press '0' to recover LCD after relay/EMI events.
    static unsigned long zeroHoldStart = 0;
    static bool zeroHoldTriggered = false;
    if (heldKey == '0') {
      if (zeroHoldStart == 0) zeroHoldStart = now;
      if (!zeroHoldTriggered && (now - zeroHoldStart) >= 1200) {
        recoverDisplay();
        checkDisplayHealth();
        if (!isDisplayFailed()) {
          updateDisplay("Display reset", "Continue");
        }
        zeroHoldTriggered = true;
      }
    } else {
      zeroHoldStart = 0;
      zeroHoldTriggered = false;
    }

    // Long-press '9' to force a delivery context refresh from Firebase.
    static unsigned long nineHoldStart = 0;
    static bool nineHoldTriggered = false;
    if (heldKey == '9') {
      if (nineHoldStart == 0) nineHoldStart = now;
      if (!nineHoldTriggered && (now - nineHoldStart) >= 1200) {
        if (!bootPhaseComplete) {
          clearKeypadGeoCheck();
          if (!isDisplayFailed()) {
            updateDisplay("Auth required", "Enter PIN");
          }
          messageStartAt = now;
          currentState = STATE_SHOW_MESSAGE;
        } else {
          bool ok = requestContextRefresh();
          if (ok) {
            scheduleKeypadGeoCheck(now);
          } else {
            clearKeypadGeoCheck();
            fetchDeliveryContext();
            lastDeliveryContextFetch = now;
          }
          if (!isDisplayFailed()) {
            updateDisplay(ok ? "Checking area" : "Refresh failed",
                          ok ? "Wait..." : "Check network");
          }
        }
        nineHoldTriggered = true;
      }
    } else {
      nineHoldStart = 0;
      nineHoldTriggered = false;
    }

    static bool stuckReported = false;
    if (keypadHealth.isStuck && !stuckReported) {
      char msg[32];
      snprintf(msg, sizeof(msg), "key_%c_held_30s", keypadHealth.stuckKey);
      reportAlertToProxy("KEYPAD_STUCK", msg);
      stuckReported = true;
    } else if (!keypadHealth.isStuck) {
      stuckReported = false;
    }
  }

  // ── 6. Unjam counter auto-reset ──
  if (lastUnjamKeyAt > 0 && now - lastUnjamKeyAt >= UNJAM_COUNTER_RESET_MS) {
    resetUnjamCounter();
    lastUnjamKeyAt = 0;
  }

  // ── 7. State machine ──
  handleStateMachine(now);
}

void resetFaceSession() {
  faceCheckPending = false;
  faceAttemptCount = 0;
  nextFaceAttemptAt = 0;
}

void applyIdlePowerPolicy(unsigned long now) {
  if (powerPolicy == POWER_POLICY_IDLE) {
    displayBacklightOff();
    return;
  }

  powerPolicy = POWER_POLICY_IDLE;
  displayBacklightOff();

  if (lastCamPowerCmdAt == 0 ||
      now - lastCamPowerCmdAt >= CONTROLLER_CAM_POWER_CMD_COOLDOWN_MS) {
    requestCameraPowerMode(false);
    lastCamPowerCmdAt = now;
  }

  cameraSleepExpected = true;
  netLog("[POWER] Idle policy active: LCD off, CAM sleep requested\n");
}

void applyActivePowerPolicy(unsigned long now) {
  if (powerPolicy != POWER_POLICY_ACTIVE) {
    powerPolicy = POWER_POLICY_ACTIVE;
    displayBacklightOn();
  }

  if (cameraSleepExpected) {
    if (requestCameraPowerMode(true)) {
      cameraSleepExpected = false;
    }
    lastCamPowerCmdAt = now;
  }
}

// ==================== STATE MACHINE ====================
void handleStateMachine(unsigned long now) {
  switch (currentState) {

  // ── BOOT SELF-TEST (POST) — reed-only, no solenoid pulses ──
  case STATE_BOOT_SELFTEST: {
    SelfTestResult stResult = tickSelfTest(now);
    if (stResult == SELFTEST_RUNNING) {
      if (!isDisplayFailed()) {
        updateDisplay("Self-Test...", "Reed check...");
      }
    } else if (stResult == SELFTEST_PASS) {
      netLog("[SELFTEST] PASS — solenoid and reed nominal\n");
      selfTestWarned = false;
      if (!isDisplayFailed()) {
        updateDisplay("Self-Test OK", "Connecting...");
      }
      messageStartAt = now;
      // WiFi already started in setup() — just transition to wait for it.
      enterState(STATE_CONNECTING_WIFI);
    } else { // SELFTEST_WARN
      netLog("[SELFTEST] WARN — reed unstable, proceeding with caution\n");
      selfTestWarned = true;
      if (!isDisplayFailed()) {
        updateDisplay("Lock Warning!", "Connecting...");
      }
      messageStartAt = now;
      // WiFi already started in setup() — just transition to wait for it.
      enterState(STATE_CONNECTING_WIFI);
    }
    break;
  }

  // ── BOOT RIDER AUTH ──
  case STATE_BOOT_AUTH: {
    // Clear temporary messages (e.g. "Wrong PIN") after LCD_MESSAGE_DURATION
    if (messageStartAt > 0 && now - messageStartAt >= LCD_MESSAGE_DURATION) {
      messageStartAt = 0;
      if (!isDisplayFailed()) {
        char masked[17] = "";
        uint8_t i;
        for (i = 0; i < bootAuthLen && i < 15; i++) {
          masked[i] = (i == bootAuthLen - 1) ? bootAuthCode[i] : '*';
        }
        masked[i] = '\0';
        updateDisplay("Rider PIN:", masked);
      }
    }

    // Idle timeout — dim LCD after 5 minutes of no input.
    if (BOOT_AUTH_IDLE_TIMEOUT_MS > 0 &&
        bootAuthLastInputAt > 0 &&
        now - bootAuthLastInputAt >= BOOT_AUTH_IDLE_TIMEOUT_MS) {
      displayBacklightOff();
    }

    char key = readKeypad();
    if (!key) break;

    // Any keypress resets idle timer and wakes display.
    bootAuthLastInputAt = now;
    displayBacklightOn();

    if (key == '*') {
      bootAuthLen = 0;
      bootAuthCode[0] = '\0';
      if (bootPhaseComplete) {
        // Post-boot voluntary auth: allow exiting
        enterState(hasActiveDelivery ? STATE_IDLE : STATE_STANDBY);
        break;
      } else {
        // Clear input but cannot exit — must authenticate.
        if (!isDisplayFailed()) {
          updateDisplay("Rider PIN:", "");
        }
      }
    } else if (key == '#') {
      if (bootAuthLen == 0) break;

      // EC-04: Check lockout before attempting validation.
      if (isLockedOut(now)) {
        unsigned long secs = getLockoutSecondsLeft(now);
        if (!isDisplayFailed()) {
          char line1[17];
          snprintf(line1, sizeof(line1), "Wait %lus", secs);
          updateDisplay("Too many tries!", line1);
        }
        bootAuthLen = 0;
        bootAuthCode[0] = '\0';
        break;
      }

      // Validate via proxy (reuses personal PIN toggle endpoint).
      // Guard: don't waste a lockout attempt if WiFi is down.
      if (WiFi.status() != WL_CONNECTED) {
        if (!isDisplayFailed()) {
          updateDisplay("No connection", "Wait for WiFi");
        }
        messageStartAt = now;
        break;
      }
      int decision = requestPersonalPinToggle(bootAuthCode, true);
      bootAuthLen = 0;
      bootAuthCode[0] = '\0';

      if (decision == 1) {
        // Authenticated!
        bootAuthDone = true;
        bootPhaseComplete = true;
        netLog("[BOOT-AUTH] Rider authenticated successfully\n");

        // Report self-test warning if it occurred.
        if (selfTestWarned) {
          reportAlertToProxy("BOOT_SELFTEST_WARN", "reed_unstable_at_boot");
        }

        if (!isDisplayFailed()) {
          updateDisplay("Auth OK!", "Ready");
        }
        messageStartAt = now;
        currentState = STATE_SHOW_MESSAGE;
      } else if (decision < 0) {
        if (!isDisplayFailed()) {
          updateDisplay("PIN not ready", "Syncing...");
        }
        messageStartAt = now;
      } else {
        // Invalid PIN.
        recordFailedAttempt(now);
        netLog("[BOOT-AUTH] Wrong PIN, attempts remaining: %d\n", getAttemptsRemaining());
        if (!isDisplayFailed()) {
          char line1[17];
          snprintf(line1, sizeof(line1), "%d tries left", getAttemptsRemaining());
          updateDisplay("Wrong PIN!", line1);
        }
        messageStartAt = now;
        // Stay in BOOT_AUTH after showing message briefly.
        // We'll handle the return via STATE_SHOW_MESSAGE -> BOOT_AUTH.
      }
    } else if (key >= '0' && key <= '9' && bootAuthLen < BOOT_AUTH_MAX_LEN) {
      bootAuthCode[bootAuthLen++] = key;
      bootAuthCode[bootAuthLen] = '\0';
      if (!isDisplayFailed()) {
        char masked[17] = "";
        uint8_t i;
        for (i = 0; i < bootAuthLen && i < 15; i++) {
          if (i == bootAuthLen - 1) {
            masked[i] = bootAuthCode[i];
          } else {
            masked[i] = '*';
          }
        }
        masked[i] = '\0';
        updateDisplay("Rider PIN:", masked);
      }
    }
    break;
  }

  case STATE_CONNECTING_WIFI:
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      fetchDeliveryContext();
      lastDeliveryContextFetch = now;

      if (hasActiveDelivery) {
        if (hasLoadedOtp()) {
          noteOnlineContextSync(now);
        } else {
          noteOnlineDeliveryOnly(now);
        }
      } else {
        clearOfflineContextSnapshot();
      }

      if (pendingRecoveryReport) {
        if (hasActiveDelivery) {
          char detail[96];
          snprintf(detail, sizeof(detail), "stage=%s delivery=%s",
                   recoveredCheckpointStage,
                   recoveredCheckpointDeliveryId);
          reportAlertToProxy("BOOT_RECOVERY", detail);
        }
        pendingRecoveryReport = false;
      }

      // Route to boot auth, skip, or bypass based on context.
      if (offlineContextFromBoot && offlineRecoveryTokenRemaining > 0 &&
          hasActiveDelivery) {
        // Mid-delivery reboot — trust the session, skip auth.
        bootAuthDone = true;
        bootPhaseComplete = true;
        netLog("[BOOT-AUTH] Skipped (mid-delivery reboot recovery)\n");
        enterState(STATE_IDLE);
      } else if (!hasActiveDelivery) {
        // No active delivery = box is unpaired or idle. No rider PIN to
        // validate against, so requiring auth would be redundant.
        bootAuthDone = false; // Key 6 still gated until a rider authenticates.
        bootPhaseComplete = true; // Boot sequence done — don't loop back to auth.
        netLog("[BOOT-AUTH] Skipped (no active delivery / unpaired box)\n");
        enterState(STATE_STANDBY);
      } else {
        // Active delivery on fresh boot — require rider authentication.
        enterState(STATE_BOOT_AUTH);
      }
    } else if (offlineContextFromBoot && offlineRecoveryTokenRemaining > 0 &&
               hasActiveDelivery && hasLoadedOtp() &&
               (now - connectStateEnteredAt) >= OFFLINE_BOOT_FALLBACK_ENTER_MS) {
      if (!isDisplayFailed()) {
        updateDisplay("Offline recovery", "Use PIN once");
      }
      bootAuthDone = true; // Trust offline recovery context.
      bootPhaseComplete = true;
      netLog("[OFFLINE] Entering degraded mode from CONNECTING (token=%u)\n",
             (unsigned int)offlineRecoveryTokenRemaining);
      enterState(STATE_IDLE);
    }
    break;

  case STATE_STANDBY: {
    if (hasActiveDelivery) {
      enterState(STATE_IDLE);
      break;
    }
    char standbyKey = readKeypad();
    if (standbyKey == '5') {
      bool isOn = toggleBacklightOverride();
      displayBacklightOn();
      if (!isDisplayFailed()) {
        updateDisplay(isOn ? "Backlight: ON" : "Backlight: AUTO", 
                      isOn ? "Stay Awake" : "Power Save");
      }
      messageStartAt = now;
      currentState = STATE_SHOW_MESSAGE;
      break;
    }
    if (standbyKey == '4') {
      personalPinLen = 0;
      personalPinCode[0] = '\0';
      personalPinExpiresAt = now + PERSONAL_PIN_TIMEOUT_MS;
      enterState(STATE_PERSONAL_PIN_ENTRY);
      break;
    }
    if (standbyKey && isUtilityKey(standbyKey)) {
      startUtilityMode(standbyKey, now);
      break;
    }
    // Key '6' — Unjam utility (guarded: auth + not-in-transit)
    if (standbyKey == '6') {
      displayBacklightOn();
      if (!bootAuthDone) {
        enterState(STATE_BOOT_AUTH);
        break;
      }
      stopUtilityMode(now);
      lastUnjamKeyAt = now;
      UnjamResult ur = fireUnjamPulse(now);
      if (!isDisplayFailed()) {
        char line0[17];
        switch (ur) {
        case UNJAM_OK:
          snprintf(line0, sizeof(line0), "Unjam %u/%u",
                   (unsigned int)(UNJAM_MAX_PULSES - getUnjamPulsesRemaining()),
                   (unsigned int)UNJAM_MAX_PULSES);
          updateDisplay(line0, "Click! Press 6");
          break;
        case UNJAM_COOLDOWN:
          updateDisplay("Wait...", "Cooling down");
          break;
        case UNJAM_LIMIT:
          updateDisplay("Limit reached", "Wait 10s reset");
          break;
        case UNJAM_OVERHEATED:
          updateDisplay("Overheated!", "Wait to cool");
          break;
        }
      }
      messageStartAt = now;
      currentState = STATE_SHOW_MESSAGE;
      break;
    }
    if (standbyKey && standbyKey != '*') {
      displayBacklightOn();
      if (!isDisplayFailed()) {
        updateDisplay("No active order", "Please wait...");
      } else {
        fallbackError();
      }
      messageStartAt = now;
      currentState = STATE_SHOW_MESSAGE;
    }
    break;
  }

  case STATE_IDLE:
  case STATE_ENTERING_PIN: {
    while(true) {
      char key = readKeypad();
      if (!key) break;

      // Backlight toggle shortcut (key '5') — only from IDLE when no PIN running
      if (currentState == STATE_IDLE && inputLen == 0 && key == '5' && !isPinEligibleNow()) {
        stopUtilityMode(now);
        bool isOn = toggleBacklightOverride();
        displayBacklightOn();
        if (!isDisplayFailed()) {
          updateDisplay(isOn ? "Backlight: ON" : "Backlight: AUTO", 
                        isOn ? "Stay Awake" : "Power Save");
        }
        messageStartAt = now;
        currentState = STATE_SHOW_MESSAGE;
        break;
      }

      // Personal PIN shortcut (key '4') — only when no PIN-eligible delivery
      if (currentState == STATE_IDLE && inputLen == 0 && key == '4' && !isPinEligibleNow()) {
        stopUtilityMode(now);
        personalPinLen = 0;
        personalPinCode[0] = '\0';
        personalPinExpiresAt = now + PERSONAL_PIN_TIMEOUT_MS;
        enterState(STATE_PERSONAL_PIN_ENTRY);
        break;
      }

      // EC-86: Fallback feedback per keypress if LCD dead
      if (isDisplayFailed()) {
        fallbackKeyFeedback();
      }

      if (key == '*') {
        stopUtilityMode(now);
        inputLen = 0;
        inputCode[0] = '\0';
        enterState(STATE_IDLE);
      } else if (key == '#') {
        stopUtilityMode(now);
        if (inputLen > 0) enterState(STATE_VERIFYING_OTP);
      } else if (isUtilityKey(key) && !isPinEligibleNow() &&
                 ((currentState == STATE_IDLE && inputLen == 0) || utilityModeActive)) {
        // Utility mode (1-3) only accessible when delivery is NOT PIN-eligible
        // (i.e., in transit/pickup phase, or no delivery at all)
        startUtilityMode(key, now);
      } else if (key == '6' && currentState == STATE_IDLE && inputLen == 0 && !isPinEligibleNow()) {
        // Key '6' — Unjam utility (guarded: auth + geofence)
        if (!bootAuthDone) {
          enterState(STATE_BOOT_AUTH);
          break;
        }
        if (!geoInsideFence && hasActiveDelivery) {
          if (!isDisplayFailed()) {
            updateDisplay("Not available", "In transit");
          }
          messageStartAt = now;
          currentState = STATE_SHOW_MESSAGE;
          break;
        }
        stopUtilityMode(now);
        lastUnjamKeyAt = now;
        UnjamResult ur = fireUnjamPulse(now);
        if (!isDisplayFailed()) {
          char line0[17];
          switch (ur) {
          case UNJAM_OK:
            snprintf(line0, sizeof(line0), "Unjam %u/%u",
                     (unsigned int)(UNJAM_MAX_PULSES - getUnjamPulsesRemaining()),
                     (unsigned int)UNJAM_MAX_PULSES);
            updateDisplay(line0, "Click! Press 6");
            break;
          case UNJAM_COOLDOWN:
            updateDisplay("Wait...", "Cooling down");
            break;
          case UNJAM_LIMIT:
            updateDisplay("Limit reached", "Wait 10s reset");
            break;
          case UNJAM_OVERHEATED:
            updateDisplay("Overheated!", "Wait to cool");
            break;
          }
        }
        messageStartAt = now;
        currentState = STATE_SHOW_MESSAGE;
        break;
      } else if (inputLen < 6) {
        // Enforce geofence lock: no PIN entry unless journey phase is PIN-eligible.
        if (isPinEligibleNow()) {
          if (!isCameraLikelyUp(now)) {
            netLog("[FACE] Camera marked down at PIN entry; attempting anyway\n");
          }

          stopUtilityMode(now);
          inputCode[inputLen++] = key;
          inputCode[inputLen] = '\0';
          if (currentState == STATE_IDLE) currentState = STATE_ENTERING_PIN;

          if (!isDisplayFailed()) {
            char masked[17] = "";
            uint8_t i;
            for (i = 0; i < inputLen && i < 15; i++) {
              if (i == inputLen - 1) {
                masked[i] = inputCode[i];
              } else {
                masked[i] = '*';
              }
            }
            masked[i] = '\0';

            char display[17];
            snprintf(display, sizeof(display), "PIN: %s", masked);
            if (isReturning) {
              updateDisplay("Return PIN:", display);
            } else {
              updateDisplay("Enter PIN:", display);
            }
          }
        } else {
          // Keypad pressed but we are in transit/pickup so no OTP is loaded
          if (!isDisplayFailed()) {
            updateDisplay("Not at destination", "PIN disabled");
            messageStartAt = millis();
            currentState = STATE_SHOW_MESSAGE;
          } else {
            fallbackError();
          }
          break; // break the while(true) to handle state change
        }
      }
    }
    break;
  }

  case STATE_PERSONAL_PIN_ENTRY: {
    while (true) {
      char key = readKeypad();
      if (!key) break;

      personalPinExpiresAt = now + PERSONAL_PIN_TIMEOUT_MS;

      if (key == '*') {
        personalPinLen = 0;
        personalPinCode[0] = '\0';
        enterState(hasActiveDelivery ? STATE_IDLE : STATE_STANDBY);
        break;
      }

      if (key == '#') {
        if (personalPinLen == 0) {
          break;
        }

        int decision = requestPersonalPinToggle(personalPinCode, isBoxLocked());
        personalPinLen = 0;
        personalPinCode[0] = '\0';

        if (decision == 1) {
          ownerSessionActive = true;
          enterState(STATE_UNLOCKING);
        } else if (decision == 2) {
          LockStatus ls = tryLock();
          ownerSessionActive = false;
          if (ls == LOCK_OK) {
            reportEventToProxy(false, false, false, false);
            enterState(STATE_RELOCKING);
          } else if (ls == LOCK_STUCK_OPEN) {
            if (!isDisplayFailed()) {
              updateDisplay("Close lid first", "#=Close assist");
            }
            enterState(STATE_AWAITING_CLOSE);
          } else {
            reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
            if (!isDisplayFailed()) {
              updateDisplay("Lock jammed!", "Contact support");
            }
            messageStartAt = now;
            currentState = STATE_SHOW_MESSAGE;
          }
        } else if (decision < 0) {
          if (!isDisplayFailed()) {
            updateDisplay("PIN not ready", "Syncing...");
          }
          messageStartAt = now;
          currentState = STATE_SHOW_MESSAGE;
        } else {
          recordFailedAttempt(now);
          if (!isDisplayFailed()) {
            char line1[17];
            snprintf(line1, sizeof(line1), "%d tries left", getAttemptsRemaining());
            updateDisplay("Wrong PIN!", line1);
          }
          messageStartAt = now;
          currentState = STATE_SHOW_MESSAGE;
        }
        break;
      }

      if (key >= '0' && key <= '9' && personalPinLen < PERSONAL_PIN_MAX_LEN) {
        personalPinCode[personalPinLen++] = key;
        personalPinCode[personalPinLen] = '\0';
        if (!isDisplayFailed()) {
          char masked[17] = "";
          uint8_t i;
          for (i = 0; i < personalPinLen && i < 15; i++) {
            if (i == personalPinLen - 1) {
              masked[i] = personalPinCode[i];
            } else {
              masked[i] = '*';
            }
          }
          masked[i] = '\0';
          updateDisplay("Personal PIN:", masked);
        }
      }
    }
    break;
  }

  case STATE_VERIFYING_OTP: {
    // EC-04: Check lockout first
    if (isLockedOut(now)) {
      unsigned long secs = getLockoutSecondsLeft(now);
      netLog("[EC-04] LOCKED OUT (%lus remaining)\n", secs);

      if (!isDisplayFailed()) {
        char line1[17];
        snprintf(line1, sizeof(line1), "Wait %lus", secs);
        updateDisplay("Too many tries!", line1);
      } else {
        fallbackLockout();
      }
      messageStartAt = now;
      inputLen = 0;
      inputCode[0] = '\0';
      currentState = STATE_SHOW_MESSAGE;
      break;
    }

    // EC-04: Check cooldown
    if (isOnCooldown(now)) {
      netLog("[EC-04] Too fast, cooldown active\n");
      inputLen = 0;
      inputCode[0] = '\0';
      currentState = STATE_IDLE;
      break;
    }

    bool proxyRouteDown = (proxyHealthState == PEER_DOWN);
    bool offlineAttempt = (WiFi.status() != WL_CONNECTED) || proxyRouteDown;
    if (proxyRouteDown && WiFi.status() == WL_CONNECTED) {
      netLog("[OFFLINE] Proxy path down; using cached OTP policy\n");
    }
    if (offlineAttempt && !isOfflineOtpEligible(now)) {
      netLog("[OFFLINE] OTP rejected: cached window/token unavailable\n");
      if (!isDisplayFailed()) {
        updateDisplay("Offline expired", proxyRouteDown ? "Proxy recovering" : "Reconnect first");
      } else {
        fallbackError();
      }
      inputLen = 0;
      inputCode[0] = '\0';
      messageStartAt = now;
      currentState = STATE_SHOW_MESSAGE;
      break;
    }

    if (offlineAttempt && offlineContextFromBoot &&
        offlineRecoveryTokenRemaining > 0) {
      consumeOfflineRecoveryToken("otp_verify");
    }

    // Validate OTP
    if (strcmp(inputCode, currentOtp) == 0) {
      netLog("[OTP] CORRECT! Requesting face check...\n");
      resetOtpAttempts();
      resetFaceSession();

      if (!isDisplayFailed()) {
        updateDisplay("PIN Correct!", "Face check...");
      } else {
        fallbackSuccess();
      }
      enterState(STATE_REQUESTING_FACE);
    } else {
      netLog("[OTP] WRONG: entered '%s' vs expected '%s'\n", inputCode, currentOtp);
      recordFailedAttempt(now);

      int remaining = getAttemptsRemaining();
      if (!isDisplayFailed()) {
        char line1[17];
        snprintf(line1, sizeof(line1), "%d tries left", remaining);
        updateDisplay("Wrong PIN!", line1);
      } else {
        fallbackError();
      }

      reportEventToProxy(false, false, false, false);

      // EC-04: Report lockout event if just triggered
      if (isLockedOut(now)) {
        reportAlertToProxy("LOCKOUT", "max_attempts_reached");
      }

      messageStartAt = now;
      currentState = STATE_SHOW_MESSAGE;
    }
    inputLen = 0;
    inputCode[0] = '\0';
    break;
  }

  case STATE_REQUESTING_FACE:
    if (!faceCheckPending) {
      if (faceAttemptCount > 0 && now < nextFaceAttemptAt) {
        break;
      }

      faceCheckPending = true;
      faceAttemptCount++;
      netLog("[FACE] Attempt %u/%u\n", faceAttemptCount,
             (unsigned int)FACE_CHECK_MAX_ATTEMPTS);
      if (!isDisplayFailed()) {
        char attemptLine[17];
        snprintf(attemptLine, sizeof(attemptLine), "Attempt %u/%u", faceAttemptCount,
                 (unsigned int)FACE_CHECK_MAX_ATTEMPTS);
        updateDisplay("Face scanning...", attemptLine);
      }

      int result = requestFaceCheck();
      if (result < 0 && !isCameraLikelyUp(now)) {
        result = -2;
        netLog("[FACE] Camera marked down after request (age=%lums)\n",
               (unsigned long)diagCache.camAgeMs);
      }
      faceCheckPending = false;

      if (result == 1) {
        netLog("[FACE] DETECTED! Unlocking solenoid...\n");
        if (otpVerifyStartedAt > 0) {
          lastUnlockLatencyMs = now - otpVerifyStartedAt;
        } else {
          lastUnlockLatencyMs = 0;
        }
        enterState(STATE_UNLOCKING);
        reportEventToProxy(true, true, true, false, faceAttemptCount, false,
                           false, "", lastUnlockLatencyMs);
      } else {
        bool noFace = (result == 0);
        const char *reason = noFace ? "NO_FACE" :
                (result == -2 ? "CAM_OFFLINE" : "CAMERA_ERROR");
        bool exhausted = (faceAttemptCount >= FACE_CHECK_MAX_ATTEMPTS);

        if (!exhausted) {
          nextFaceAttemptAt = millis() + FACE_RETRY_DELAY_MS;
          netLog("[FACE] %s — retrying in %ums (%u/%u)\n", reason,
                 (unsigned int)FACE_RETRY_DELAY_MS, faceAttemptCount,
                 (unsigned int)FACE_CHECK_MAX_ATTEMPTS);
          if (!isDisplayFailed()) {
            char retryLine[17];
            snprintf(retryLine, sizeof(retryLine), "Retry %u/%u", faceAttemptCount + 1,
                     (unsigned int)FACE_CHECK_MAX_ATTEMPTS);
            updateDisplay(noFace ? "No face found" :
                         (result == -2 ? "Camera offline" : "Camera error"),
                         retryLine);
          } else {
            fallbackError();
          }
          reportEventToProxy(true, false, false, false, faceAttemptCount,
                             false, false, reason);
          break;
        }

        netLog("[FACE] %s after %u attempts — fallback required\n", reason,
               faceAttemptCount);
        if (!isDisplayFailed()) {
          updateDisplay("Face check failed", "Use rider fallback");
        } else {
          fallbackError();
        }
        reportEventToProxy(true, false, false, false, faceAttemptCount, true,
                           true, reason);
        inputLen = 0;
        inputCode[0] = '\0';
        messageStartAt = millis();
        currentState = STATE_SHOW_MESSAGE;
      }
    }
    break;

  case STATE_UNLOCKING: {
    if (!isSolenoidActive()) {
      bool overrideAttempt = adminOverride.pending;
      suppressTamper(); // Authorized unlock — don't flag lid-open as tamper
      // EC-21: Try unlock with retry logic + EC-96 thermal check
      LockStatus ls = tryUnlock(overrideAttempt);

      if (ls == LOCK_OK) {
        netLog("[LOCK] Solenoid ON (unlock OK)\n");
        recoverDisplay(); // Re-sync LCD after relay/solenoid EMI
        if (overrideAttempt) {
          reportCommandAckToProxy("UNLOCKING", "executed", "lock_ok");
          adminOverride.clear();
          // We intentionally do NOT call enterState(STATE_RELOCKING) yet.
          // It will stay retracted/unlocked until App sends "LOCKED" or '*' is pressed.
        }
        if (!isDisplayFailed()) {
          if (isReturning) {
            updateDisplay("Return Complete", "Close lid");
            // EC-32: Signal proxy to mark return as completed.
            reportCommandAckToProxy("RETURN_COMPLETE", "executed", "return_unlock_ok");
          } else {
            updateDisplay("Box Unlocked!", "* = Relock");
          }
        } else {
          fallbackSuccess();
        }

        if (ownerSessionActive) {
          enterState(STATE_OWNER_UNLOCKED);
          break;
        }
      } else {
        // EC-21/22/96: Solenoid failure
        netLog("[LOCK] Unlock FAILED: %s\n", lockStatusStr(ls));

        char detail[48];
        snprintf(detail, sizeof(detail), "%s_retries_%u",
                 lockStatusStr(ls), getLastRetryCount());
        reportAlertToProxy("SOLENOID_STUCK", detail);
        if (overrideAttempt) {
          reportCommandAckToProxy("UNLOCKING", "failed_unlock", lockStatusStr(ls));
          adminOverride.clear();
        }

        if (!isDisplayFailed()) {
          updateDisplay("Lock jammed!", "Contact support");
        } else {
          fallbackError();
        }
        messageStartAt = now;
        currentState = STATE_SHOW_MESSAGE;
        break;
      }
    }

    // Manual relock via '*'
    if (readKeypad() == '*') {
      LockStatus ls = tryLock();
      netLog("[LOCK] Manual relock: %s\n", lockStatusStr(ls));
      reportEventToProxy(false, false, false, false);

      if (ls == LOCK_OK) {
        enterState(STATE_RELOCKING);
      } else {
        reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
        if (ls == LOCK_STUCK_OPEN) {
          if (!isDisplayFailed()) {
            updateDisplay("Close lid first", "#=Close assist");
          }
          enterState(STATE_AWAITING_CLOSE);
        } else {
          enterState(STATE_RELOCKING);
        }
      }
    }
    break;
  }

  case STATE_OWNER_UNLOCKED: {
    char key = readKeypad();
    if (key == '*') {
      LockStatus ls = tryLock();
      netLog("[OWNER] Relock from owner session: %s\n", lockStatusStr(ls));
      if (ls == LOCK_OK) {
        ownerSessionActive = false;
        enterState(STATE_RELOCKING);
      } else {
        reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
        if (ls == LOCK_STUCK_OPEN) {
          if (!isDisplayFailed()) {
            updateDisplay("Close lid first", "#=Close assist");
          }
          enterState(STATE_AWAITING_CLOSE);
        }
      }
    }
    break;
  }

  case STATE_AWAITING_CLOSE: {
    // Finalize lock only when physical close is confirmed.
    if (!isSolenoidActive() && isBoxLocked()) {
      reportCommandAckToProxy("LOCKED", "executed", "reed_closed_confirmed");
      enterState(STATE_RELOCKING);
      break;
    }

    if (!awaitingCloseWarnSent && awaitingCloseSince > 0 &&
        now - awaitingCloseSince >= LOCK_AWAIT_CLOSE_WARN_MS) {
      awaitingCloseWarnSent = true;
      reportAlertToProxy("SOLENOID_STUCK", "awaiting_close_timeout");
    }

    // ── Open-door safety: periodic LCD warning every 30 s ──
    if (awaitingCloseSince > 0 && now - lastDoorOpenWarnAt >= DOOR_OPEN_LCD_WARN_INTERVAL_MS) {
      lastDoorOpenWarnAt = now;
      if (!isDisplayFailed()) {
        updateDisplay("! DOOR OPEN !", "Close lid now");
      }
    }

    // ── Open-door safety: escalated critical alert after 5 min ──
    if (!doorOpenCriticalSent && awaitingCloseSince > 0 &&
        now - awaitingCloseSince >= DOOR_OPEN_CRITICAL_MS) {
      doorOpenCriticalSent = true;
      reportAlertToProxy("DOOR_OPEN_CRITICAL", "open_5min_exceeded");
      netLog("[DOOR] Critical: door open > 5 minutes\n");
    }

    char key = readKeypad();
    if (key == '#' && !isSolenoidActive()) {
      if (closeAssistAttempts >= LOCK_CLOSE_ASSIST_MAX_ATTEMPTS) {
        if (!isDisplayFailed()) {
          updateDisplay("Assist limit hit", "Check alignment");
        }
        reportAlertToProxy("SOLENOID_STUCK", "close_assist_limit");
        break;
      }

      LockStatus ls = tryUnlock(true);
      if (ls == LOCK_OK) {
        closeAssistAttempts++;
        closeAssistUntil = now + LOCK_CLOSE_ASSIST_HOLD_MS;
        if (!isDisplayFailed()) {
          updateDisplay("Push lid closed", "Assist active");
        }
      } else {
        reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
      }
    }

    if (isSolenoidActive()) {
      // If lid closes during assist, lock immediately.
      if (isBoxLocked()) {
        LockStatus ls = tryLock();
        if (ls == LOCK_OK) {
          reportCommandAckToProxy("LOCKED", "executed", "reed_closed_confirmed");
          enterState(STATE_RELOCKING);
          break;
        }
      }

      // End assist window and return to waiting state if still open.
      if (closeAssistUntil > 0 && now >= closeAssistUntil) {
        LockStatus ls = tryLock();
        closeAssistUntil = 0;
        if (ls != LOCK_OK && !isDisplayFailed()) {
          updateDisplay("Still open", "# to retry");
        }
      }
    }
    break;
  }

  case STATE_RELOCKING:
    ownerSessionActive = false;
    deferredContextTransition = false;
    armTamper(); // Re-enable tamper detection after authorized unlock completes
    if (!isDisplayFailed()) {
      updateDisplay("Box Locked", "Ready");
    }
    messageStartAt = now;
    currentState = STATE_SHOW_MESSAGE;
    netLog("[LOCK] Relocked\n");
    break;

  case STATE_SHOW_MESSAGE:
    if (now - messageStartAt >= LCD_MESSAGE_DURATION) {
      // If boot phase is not complete, route back to BOOT_AUTH.
      // (bootPhaseComplete is separate from bootAuthDone — an unpaired box
      //  completes boot phase but doesn't authenticate, avoiding a loop.)
      if (!bootPhaseComplete) {
        enterState(STATE_BOOT_AUTH);
      } else {
        enterState(hasActiveDelivery ? STATE_IDLE : STATE_STANDBY);
      }
    }
    break;
  }
}


// ==================== STATE HELPER ====================
void enterState(TesterState newState) {
  if (utilityModeActive && newState != STATE_STANDBY && newState != STATE_IDLE &&
      newState != STATE_ENTERING_PIN && newState != STATE_PERSONAL_PIN_ENTRY) {
    stopUtilityMode(millis());
  }

  currentState = newState;
  updateHandoffCheckpointForState(newState);

  if (newState == STATE_STANDBY) {
    applyIdlePowerPolicy(millis());
  } else {
    applyActivePowerPolicy(millis());
  }

  switch (newState) {
  case STATE_STANDBY:
    if (!isDisplayFailed()) {
      updateDisplay("Box Ready", "No Delivery");
    }
    inputLen = 0;
    inputCode[0] = '\0';
    resetFaceSession();
    adminOverride.clear();
    break;
  case STATE_IDLE:
    if (!isDisplayFailed()) {
      renderIdleJourneyStatus();
    }
    inputLen = 0;
    inputCode[0] = '\0';
    resetFaceSession();
    break;
  case STATE_PERSONAL_PIN_ENTRY:
    personalPinLen = 0;
    personalPinCode[0] = '\0';
    personalPinExpiresAt = millis() + PERSONAL_PIN_TIMEOUT_MS;
    if (!isDisplayFailed()) {
      updateDisplay("Personal PIN:", "");
    }
    break;
  case STATE_VERIFYING_OTP:
    otpVerifyStartedAt = millis();
    break;
  case STATE_OWNER_UNLOCKED:
    if (!isDisplayFailed()) {
      updateDisplay("Owner Access", "* = Relock");
    }
    break;
  case STATE_AWAITING_CLOSE:
    suppressTamper();
    closeAssistUntil = 0;
    awaitingCloseSince = millis();
    closeAssistAttempts = 0;
    awaitingCloseWarnSent = false;
    lastDoorOpenWarnAt = millis();
    doorOpenCriticalSent = false;
    if (!isDisplayFailed()) {
      updateDisplay("Close lid first", "#=Close assist");
    }
    break;
  case STATE_BOOT_SELFTEST:
    initSelfTest(true);   // Always reed-only — no solenoid pulses at boot
    if (!isDisplayFailed()) {
      updateDisplay("Self-Test...", "Reed check only");
    }
    break;
  case STATE_BOOT_AUTH:
    bootAuthLen = 0;
    bootAuthCode[0] = '\0';
    bootAuthLastInputAt = millis();
    if (!isDisplayFailed()) {
      updateDisplay("Rider PIN:", "");
    }
    displayBacklightOn();
    break;
  case STATE_CONNECTING_WIFI:
    connectStateEnteredAt = millis();
    if (!isDisplayFailed()) {
      updateDisplay("Parcel-Safe v3", "Connecting WiFi");
    }
    break;
  default:
    break;
  }
}
