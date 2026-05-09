/**
 * LockSafety.cpp — Solenoid lock control with safety features
 */

#include "LockSafety.h"

// ── Solenoid state ──
static unsigned long solenoidOnAt    = 0;    // millis() when energised (0 = off)
static uint8_t       lastRetryCount  = 0;

// ── EC-96: Thermal model ──
static float         coilTempC       = 25.0f; // Ambient start
static unsigned long lastCoolTick    = 0;

// ── EC-95: Reed switch debounce ──
static bool          reedRaw         = true;  // true = closed/locked
static bool          reedStable      = true;
static unsigned long reedLastChange  = 0;

// ── Tamper detection ──
static bool          tamperLatch     = false; // latched until cleared
static bool          tamperSuppressed = false; // true during authorized unlock
static bool          prevReedStable  = true;  // for edge detection
static unsigned long tamperLastAt    = 0;     // cooldown: millis() of last trigger
static unsigned long lockInitAt      = 0;     // millis() at initLock()
static unsigned long reedOpenSince   = 0;     // millis() when stable OPEN began
static bool          prevQualifiedOpen = false;
static volatile bool reedEdgeSeen = false;

// ── Unjam utility state (used by maintainLockSafety + fireUnjamPulse) ──
static uint8_t       unjamPulseCount    = 0;
static unsigned long unjamLastPulseAt   = 0;
static unsigned long unjamDeEnergiseAt  = 0;

static void IRAM_ATTR reedSwitchISR() {
  reedEdgeSeen = true;
}

// ── Internal helpers ──
static bool readReedRaw() {
  int raw = digitalRead(REED_SWITCH_PIN);
  return REED_CLOSED_IS_HIGH ? (raw == HIGH) : (raw == LOW);
}

static void energise(unsigned long now) {
  digitalWrite(LOCK_PIN, LOCK_PIN_ON);
  solenoidOnAt = now;
  coilTempC += LOCK_HEAT_PER_ACTUATION;
  unjamDeEnergiseAt = 0; // Clear any pending unjam timer (defense in depth).
}

static void deEnergise() {
  digitalWrite(LOCK_PIN, LOCK_PIN_OFF);
  solenoidOnAt = 0;
}

// ==================== PUBLIC API ====================

void initLock() {
  digitalWrite(LOCK_PIN, LOCK_PIN_OFF);
  pinMode(LOCK_PIN, OUTPUT);

#if REED_USE_INTERNAL_PULLUP
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
#else
  pinMode(REED_SWITCH_PIN, INPUT);
#endif

  reedRaw    = readReedRaw();
  reedStable = reedRaw;
  prevReedStable = reedRaw;
  lastCoolTick = millis();
  tamperLastAt = 0;
  lockInitAt = millis();
  reedOpenSince = reedStable ? 0 : lockInitAt;
  prevQualifiedOpen = false;

  // Keep reed interrupt active in every firmware state.
  attachInterrupt(digitalPinToInterrupt(REED_SWITCH_PIN), reedSwitchISR,
                  CHANGE);

  Serial.printf("[REED] boot raw=%d interpreted=%s (REED_CLOSED_IS_HIGH=%d)\n",
                digitalRead(REED_SWITCH_PIN),
                reedStable ? "CLOSED" : "OPEN", REED_CLOSED_IS_HIGH);
}

LockStatus tryUnlock(bool ignoreReed) {
  // EC-96: Refuse if overheated
  if (coilTempC >= LOCK_THERMAL_MAX_TEMP) {
    Serial.println(F("[LOCK] Blocked — coil overheated"));
    return LOCK_OVERHEATED;
  }

  unsigned long now = millis();
  lastRetryCount = 0;

  // Hold energized for user to open
  energise(now);
  
  if (ignoreReed) {
    Serial.println(F("[LOCK] Unlocked forcefully (admin override, reed ignored, holding)"));
  } else {
    Serial.println(F("[LOCK] Unlocked, holding for user to lift lid (10s max)"));
  }
  
  return LOCK_OK;
}

LockStatus tryLock(bool ignoreReed) {
  deEnergise();
  delay(LOCK_DEBOUNCE_MS);

  if (ignoreReed) {
    reedStable = true;
    Serial.println(F("[LOCK] Locked forcefully (admin override, reed ignored)"));
    return LOCK_OK;
  }

  // EC-22: Verify reed switch shows closed
  bool closed = readReedRaw();
  if (!closed) {
    Serial.println(F("[LOCK] STUCK OPEN — reed still reads open"));
    return LOCK_STUCK_OPEN;
  }

  reedStable = true;
  Serial.println(F("[LOCK] Locked OK"));
  return LOCK_OK;
}

bool maintainLockSafety(unsigned long now) {
  if (reedEdgeSeen) {
    // ISR only marks that an edge occurred; filtering remains here.
    reedEdgeSeen = false;
  }

  // ── EC-95: Debounce reed switch ──
  bool raw = readReedRaw();
  if (raw != reedRaw) {
    reedRaw = raw;
    reedLastChange = now;
  }
  if ((now - reedLastChange) >= LOCK_DEBOUNCE_MS) {
    reedStable = reedRaw;
  }

  // Track sustained OPEN duration from debounced reed state.
  if (!reedStable) {
    if (reedOpenSince == 0) {
      reedOpenSince = now;
    }
  } else {
    reedOpenSince = 0;
  }

  // ── Tamper detection: require sustained OPEN + post-boot grace ──
  bool cooldownActive =
      (tamperLastAt > 0) && (now - tamperLastAt < TAMPER_COOLDOWN_MS);
  bool bootGraceActive = (now - lockInitAt < TAMPER_BOOT_GRACE_MS);
  bool openConfirmed =
      (reedOpenSince > 0) && (now - reedOpenSince >= TAMPER_OPEN_CONFIRM_MS);
  bool qualifiedOpen = openConfirmed && !isSolenoidActive() &&
                       !tamperSuppressed && !bootGraceActive;

  // Trigger once when entering a qualified-open state.
  if (!prevQualifiedOpen && qualifiedOpen && !cooldownActive) {
    tamperLatch = true;
    tamperLastAt = now;
    Serial.println(F("[TAMPER] Reed opened without unlock — TAMPER DETECTED"));
  }

  prevQualifiedOpen = qualifiedOpen;
  prevReedStable = reedStable;

  // ── EC-96: Passive cooling ──
  if (now > lastCoolTick) {
    unsigned long elapsed = now - lastCoolTick;
    coilTempC -= elapsed * LOCK_COOLING_RATE;
    if (coilTempC < 25.0f) coilTempC = 25.0f; // Floor at ambient
  }
  lastCoolTick = now;

  // ── Unjam pulse auto-deenergise (non-blocking) ──
  if (unjamDeEnergiseAt > 0 && now >= unjamDeEnergiseAt) {
    deEnergise();
    unjamDeEnergiseAt = 0;
    Serial.println(F("[UNJAM] Pulse de-energised (300ms elapsed)"));
  }

  // ── Hard thermal cutoff ──
  if (solenoidOnAt > 0 && (now - solenoidOnAt >= LOCK_MAX_ACTIVE_MS)) {
    deEnergise();
    unjamDeEnergiseAt = 0; // Clear any pending unjam de-energise
    Serial.println(F("[LOCK] THERMAL CUTOFF (timeout limit)"));
    return true;
  }

  return false;
}

bool isBoxLocked()       { return reedStable; }
bool isOverheated()      { return coilTempC >= LOCK_THERMAL_MAX_TEMP; }
bool isSolenoidActive()  { return solenoidOnAt > 0; }
uint8_t getLastRetryCount() { return lastRetryCount; }
bool isTamperDetected()  { return tamperLatch; }
void clearTamper()       { tamperLatch = false; }
void suppressTamper()    { tamperSuppressed = true; }
void armTamper()         { tamperSuppressed = false; }

const char* lockStatusStr(LockStatus s) {
  switch (s) {
    case LOCK_OK:             return "OK";
    case LOCK_STUCK_CLOSED:   return "STUCK_CLOSED";
    case LOCK_STUCK_OPEN:     return "STUCK_OPEN";
    case LOCK_OVERHEATED:     return "OVERHEATED";
    case LOCK_THERMAL_CUTOFF: return "THERMAL_CUTOFF";
    default:                  return "UNKNOWN";
  }
}

// ==================== BOOT SELF-TEST (POST) ====================

// Sub-stages: even = energise, odd = settle, final = verify
static uint8_t  stStage       = 0;
static uint8_t  stPulsesDone  = 0;
static bool     stBaselineReed = true;
static bool     stReedFault   = false;
static unsigned long stStageStartAt = 0;
static bool     stSkipSolenoid = false; // Reed-only mode (no pulses)

void initSelfTest(bool skipSolenoid) {
  stStage        = 0;
  stPulsesDone   = 0;
  stBaselineReed = readReedRaw();
  stReedFault    = false;
  stStageStartAt = 0;
  stSkipSolenoid = skipSolenoid;
  Serial.printf("[SELFTEST] Init — baseline reed=%s, solenoid=%s\n",
                stBaselineReed ? "CLOSED" : "OPEN",
                stSkipSolenoid ? "SKIP (mid-delivery)" : "ACTIVE");
}

uint8_t getSelfTestPulseNumber() {
  if (stSkipSolenoid) return 0; // No pulses in reed-only mode
  return stPulsesDone + (stStage % 2 == 0 && stStage > 0 ? 0 : 1);
}

SelfTestResult tickSelfTest(unsigned long now) {
  // Stage 0: Record baseline. If mid-delivery (stSkipSolenoid), skip
  // solenoid pulses entirely to prevent unauthorized unlocking.
  // On a fresh boot (no delivery), run the full diagnostic pulse check.
  if (stStage == 0) {
    stBaselineReed = readReedRaw();
    stStageStartAt = now;

    if (stSkipSolenoid) {
      // ── Reed-only mode: active delivery in progress ──
      stStage = 254; // Jump straight to final reed verify
      Serial.println(F("[SELFTEST] Reed-only mode — solenoid pulses SKIPPED (active delivery)"));
      return SELFTEST_RUNNING;
    }

    // ── Normal boot: run full solenoid diagnostic ──
    energise(now);
    stStage = 1;
    Serial.printf("[SELFTEST] Pulse %u/%u — energise\n",
                  stPulsesDone + 1, (unsigned int)SELFTEST_PULSE_COUNT);
    return SELFTEST_RUNNING;
  }


  // From here on, only reachable during normal (non-skip) boot.


  // Odd stages (1, 3, 5): solenoid is ON — wait for pulse duration.
  if (stStage % 2 == 1) {
    if (now - stStageStartAt >= SELFTEST_PULSE_DURATION_MS) {
      deEnergise();
      stPulsesDone++;
      Serial.printf("[SELFTEST] Pulse %u/%u — de-energise\n",
                    stPulsesDone, (unsigned int)SELFTEST_PULSE_COUNT);

      // Check if we've done all pulses.
      if (stPulsesDone >= SELFTEST_PULSE_COUNT) {
        stStage = 254; // Final verify stage
        stStageStartAt = now;
        return SELFTEST_RUNNING;
      }

      // Move to settle stage.
      stStage++;
      stStageStartAt = now;
    }
    return SELFTEST_RUNNING;
  }

  // Even stages (2, 4): settle/cooling gap between pulses.
  if (stStage < 254) {
    if (now - stStageStartAt >= SELFTEST_INTERVAL_MS) {
      // Check reed during settle — should still match baseline.
      bool currentReed = readReedRaw();
      if (currentReed != stBaselineReed) {
        Serial.printf("[SELFTEST] Reed drift during settle: was=%d now=%d\n",
                      stBaselineReed, currentReed);
        stReedFault = true;
      }

      // Start next pulse.
      stStageStartAt = now;
      energise(now);
      stStage++;
      Serial.printf("[SELFTEST] Pulse %u/%u — energise\n",
                    stPulsesDone + 1, (unsigned int)SELFTEST_PULSE_COUNT);
    }
    return SELFTEST_RUNNING;
  }

  // Stage 254: Final settle after last pulse, then verify.
  if (stStage == 254) {
    // In reed-only mode, use a short settle (500ms) instead of full interval.
    unsigned long settleTime = stSkipSolenoid ? 500UL : SELFTEST_INTERVAL_MS;
    if (now - stStageStartAt >= settleTime) {
      bool finalReed = readReedRaw();
      if (finalReed != stBaselineReed) {
        Serial.printf("[SELFTEST] Reed mismatch after test: baseline=%d final=%d\n",
                      stBaselineReed, finalReed);
        stReedFault = true;
      }

      if (stReedFault) {
        Serial.println(F("[SELFTEST] WARN — reed was unstable during test"));
        return SELFTEST_WARN;
      }

      Serial.println(F("[SELFTEST] PASS — solenoid and reed nominal"));
      return SELFTEST_PASS;
    }
    return SELFTEST_RUNNING;
  }

  // Should never reach here.
  return SELFTEST_WARN;
}

// ==================== UNJAM UTILITY (KEY 6) ====================

UnjamResult fireUnjamPulse(unsigned long now) {
  // Auto-reset counter if idle long enough.
  if (unjamPulseCount > 0 && unjamLastPulseAt > 0 &&
      now - unjamLastPulseAt >= UNJAM_COUNTER_RESET_MS) {
    resetUnjamCounter();
  }

  // Guard: thermal
  if (coilTempC >= LOCK_THERMAL_MAX_TEMP) {
    Serial.println(F("[UNJAM] Blocked — coil overheated"));
    return UNJAM_OVERHEATED;
  }

  // Guard: cooldown
  if (unjamLastPulseAt > 0 && now - unjamLastPulseAt < UNJAM_COOLDOWN_MS) {
    Serial.println(F("[UNJAM] Blocked — cooldown active"));
    return UNJAM_COOLDOWN;
  }

  // Guard: pulse limit
  if (unjamPulseCount >= UNJAM_MAX_PULSES) {
    Serial.println(F("[UNJAM] Blocked — max pulses reached, wait for reset"));
    return UNJAM_LIMIT;
  }

  // Fire pulse — energise now, schedule de-energise.
  energise(now);
  unjamDeEnergiseAt = now + UNJAM_PULSE_DURATION_MS;
  unjamPulseCount++;
  unjamLastPulseAt = now;

  Serial.printf("[UNJAM] Pulse %u/%u fired (300ms)\n",
                unjamPulseCount, (unsigned int)UNJAM_MAX_PULSES);
  return UNJAM_OK;
}

void resetUnjamCounter() {
  if (unjamPulseCount > 0) {
    Serial.println(F("[UNJAM] Pulse counter reset"));
  }
  unjamPulseCount = 0;
  unjamLastPulseAt = 0;
  unjamDeEnergiseAt = 0;
}

uint8_t getUnjamPulsesRemaining() {
  uint8_t used = unjamPulseCount;
  return used < UNJAM_MAX_PULSES ? (UNJAM_MAX_PULSES - used) : 0;
}

