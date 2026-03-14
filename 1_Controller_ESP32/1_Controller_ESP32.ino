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
  STATE_CONNECTING_WIFI,
  STATE_STANDBY,
  STATE_IDLE,
  STATE_ENTERING_PIN,
  STATE_VERIFYING_OTP,
  STATE_REQUESTING_FACE,
  STATE_UNLOCKING,
  STATE_RELOCKING,
  STATE_SHOW_MESSAGE
};

static TesterState currentState = STATE_CONNECTING_WIFI;

// ── Input buffers ──
static char    inputCode[8];
static uint8_t inputLen = 0;

// ── Timing trackers ──
static unsigned long lastDeliveryContextFetch = 0;
static unsigned long messageStartAt           = 0;
static unsigned long lastDisplayCheck         = 0;

// ── Face check ──
static bool faceCheckPending = false;

// ── Safety modules ──
static AdminOverride adminOverride;
static KeypadHealth  keypadHealth;

// ── Forward declarations ──
void enterState(TesterState newState);
void handleStateMachine(unsigned long now);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n=== Parcel-Safe Controller v3 (Modular) ==="));

  initHardwareIO();
  initLock();
  initDisplayHealth();
  startWiFiConnection();

  currentState = STATE_CONNECTING_WIFI;

  Serial.printf("  Proxy: %s:%d\n", PROXY_HOST, PROXY_PORT);
  Serial.println(F("  Modules: LockSafety, OTPLockout, DisplayHealth, AdminOverride, KeypadHealth"));
}

// ==================== MAIN LOOP (non-blocking) ====================
void loop() {
  unsigned long now = millis();

  // ── 1. WiFi monitoring (runs in all states) ──
  if (WiFi.status() != WL_CONNECTED && currentState != STATE_CONNECTING_WIFI) {
    netLog("[WIFI] Connection lost, reconnecting...\n");
    enterState(STATE_CONNECTING_WIFI);
    WiFi.disconnect();
    WiFi.begin((const char *)WIFI_SSID, WIFI_PASSWORD);
  }
  maintainWiFiConnection(now);

  // ── 2. LockSafety tick (thermal model, reed debounce, cutoff) ──
  if (maintainLockSafety(now) && currentState == STATE_UNLOCKING) {
    netLog("[LOCK] Solenoid CUT OFF (Thermal Safety or Auto-Relock Timeout)\n");
    if (!isDisplayFailed()) {
      updateDisplay("Lock timeout!", "Relocking...");
    } else {
      fallbackError();
    }
    reportEventToProxy(false, false, false, true);
    reportAlertToProxy("THERMAL_CUTOFF", "10s_safety_limit");
    
    // Auto-update the mobile app UI back to 'Lock Box' since the mechanical 
    // hold has ended to prevent burnout.
    reportCommandAckToProxy("LOCKED", "executed", "auto_relock_timeout");
    
    enterState(STATE_RELOCKING);
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
        if (currentState == STATE_IDLE || currentState == STATE_ENTERING_PIN) {
          enterState(STATE_STANDBY);
        }
      }
      // Status update while already active (e.g. geofence transit/pickup updates)
      else if (hasActiveDelivery && wasActive && oldStatus != lastStatusCommand) {
        if (currentState == STATE_IDLE || currentState == STATE_ENTERING_PIN) {
           enterState(STATE_IDLE); // Re-render LCD
        }
      }

      // EC-77: Handle remote lock/unlock commands
      if (lastStatusCommand == "UNLOCKING" && currentState != STATE_UNLOCKING) {
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
          LockStatus ls = tryLock(true); // ignore reed for remote lock
          reportCommandAckToProxy("LOCKED", "accepted", "state_transition_relocking");
          if (ls != LOCK_OK) {
             reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
          }
          enterState(STATE_RELOCKING);
        } else {
          // Already locked or in another state — just actuate and stay in current state or go to relocking
          LockStatus ls = tryLock(true); // ignore reed for remote lock
          reportCommandAckToProxy("LOCKED", "executed", "already_locked");
          // Optionally go to relocking state to show message, or stay in current state
        }
      } else if (lastStatusCommand.length() > 0) {
        netLog("[EC-77] Remote command '%s' received but not actionable in state=%d\n",
               lastStatusCommand.c_str(), (int)currentState);
        lastStatusCommand = ""; // Clear invalid command
        char stateDetail[32];
        snprintf(stateDetail, sizeof(stateDetail), "state_%d", (int)currentState);
        reportCommandAckToProxy(lastStatusCommand.c_str(), "rejected_state", stateDetail);
      }
    }
  }

  // ── 4. EC-86: Display health check ──
  if (now - lastDisplayCheck >= DISPLAY_HEALTH_CHECK_MS) {
    if (!checkDisplayHealth()) {
      reportAlertToProxy("DISPLAY_FAILED",
                         isDisplayFailed() ? "lcd_dead" : "lcd_degraded");
    }
    lastDisplayCheck = now;
  }

  // ── 5. EC-82: Keypad stuck detection ──
  {
    Keypad& kp = getKeypad();
    char heldKey = (kp.getState() == HOLD) ? kp.key[0].kchar : 0;
    keypadHealth.update(heldKey, now);

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

  // ── 6. State machine ──
  handleStateMachine(now);
}

// ==================== STATE MACHINE ====================
void handleStateMachine(unsigned long now) {
  switch (currentState) {

  case STATE_CONNECTING_WIFI:
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
      fetchDeliveryContext();
      lastDeliveryContextFetch = now;
      enterState(hasActiveDelivery ? STATE_IDLE : STATE_STANDBY);
    }
    break;

  case STATE_STANDBY: {
    if (hasActiveDelivery) {
      enterState(STATE_IDLE);
      break;
    }
    char standbyKey = readKeypad();
    if (standbyKey && standbyKey != '*') {
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

      // EC-86: Fallback feedback per keypress if LCD dead
      if (isDisplayFailed()) {
        fallbackKeyFeedback();
      }

      if (key == '*') {
        inputLen = 0;
        inputCode[0] = '\0';
        enterState(STATE_IDLE);
      } else if (key == '#') {
        if (inputLen > 0) enterState(STATE_VERIFYING_OTP);
      } else if (inputLen < 6) {
        // Enforce geofence lock: no PIN entry unless OTP exists
        if (currentOtp[0] != '\0') {
          inputCode[inputLen++] = key;
          inputCode[inputLen] = '\0';
          if (currentState == STATE_IDLE) currentState = STATE_ENTERING_PIN;

          if (!isDisplayFailed()) {
            char display[17];
            snprintf(display, sizeof(display), "PIN: %s", inputCode);
            if (lastStatusCommand == "RETURNING") {
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

    // Validate OTP
    if (strcmp(inputCode, currentOtp) == 0) {
      netLog("[OTP] CORRECT! Requesting face check...\n");
      resetOtpAttempts();

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
      faceCheckPending = true;

      int result = requestFaceCheck();
      faceCheckPending = false;

      if (result == 1) {
        netLog("[FACE] DETECTED! Unlocking solenoid...\n");
        enterState(STATE_UNLOCKING);
        reportEventToProxy(true, true, true, false);
      } else if (result == 0) {
        netLog("[FACE] NOT DETECTED! Box stays locked.\n");
        if (!isDisplayFailed()) {
          updateDisplay("No face found!", "Box stays locked");
        } else {
          fallbackError();
        }
        reportEventToProxy(true, false, false, false);
        messageStartAt = millis();
        currentState = STATE_SHOW_MESSAGE;
      } else {
        netLog("[FACE] Check failed (camera offline?)\n");
        if (!isDisplayFailed()) {
          updateDisplay("Camera offline!", "Box stays locked");
        } else {
          fallbackError();
        }
        reportEventToProxy(true, false, false, false);
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
        if (overrideAttempt) {
          reportCommandAckToProxy("UNLOCKING", "executed", "lock_ok");
          adminOverride.clear();
          // We intentionally do NOT call enterState(STATE_RELOCKING) yet.
          // It will stay retracted/unlocked until App sends "LOCKED" or '*' is pressed.
        }
        if (!isDisplayFailed()) {
          updateDisplay("Box Unlocked!", "* = Relock");
        } else {
          fallbackSuccess();
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

      if (ls != LOCK_OK) {
        reportAlertToProxy("SOLENOID_STUCK", lockStatusStr(ls));
      }
      enterState(STATE_RELOCKING);
    }
    break;
  }

  case STATE_RELOCKING:
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
      enterState(hasActiveDelivery ? STATE_IDLE : STATE_STANDBY);
    }
    break;
  }
}

// ==================== STATE HELPER ====================
void enterState(TesterState newState) {
  currentState = newState;

  switch (newState) {
  case STATE_STANDBY:
    if (!isDisplayFailed()) {
      updateDisplay("Box Ready", "No Delivery");
    }
    inputLen = 0;
    inputCode[0] = '\0';
    faceCheckPending = false;
    adminOverride.clear();
    break;
  case STATE_IDLE:
    if (!isDisplayFailed()) {
      if (lastStatusCommand == "RETURNING") {
        updateDisplay("Return PIN:", "PIN: ");
      } else if (lastStatusCommand == "GEO_TRANSIT_DROP") {
        updateDisplay("In Transit", "Head to dropoff");
      } else if (lastStatusCommand == "GEO_TRANSIT_PICK") {
        updateDisplay("In Transit", "Head to pickup");
      } else if (lastStatusCommand == "GEO_PICKUP") {
        updateDisplay("Ongoing Pickup", "Please wait...");
      } else {
        updateDisplay("Enter PIN:", "PIN: ");
      }
    }
    inputLen = 0;
    inputCode[0] = '\0';
    faceCheckPending = false;
    break;
  case STATE_CONNECTING_WIFI:
    if (!isDisplayFailed()) {
      updateDisplay("Parcel-Safe v3", "Connecting WiFi");
    }
    break;
  default:
    break;
  }
}
