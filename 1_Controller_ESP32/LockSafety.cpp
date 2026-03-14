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

// ── Internal helpers ──
static bool readReedRaw() {
    return digitalRead(REED_SWITCH_PIN) == HIGH; // HIGH = magnet present = locked
}

static void energise(unsigned long now) {
  digitalWrite(LOCK_PIN, HIGH);
  solenoidOnAt = now;
  coilTempC += LOCK_HEAT_PER_ACTUATION;
}

static void deEnergise() {
  digitalWrite(LOCK_PIN, LOW);
  solenoidOnAt = 0;
}

// ==================== PUBLIC API ====================

void initLock() {
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW);

  pinMode(REED_SWITCH_PIN, INPUT);

  reedRaw    = readReedRaw();
  reedStable = reedRaw;
  lastCoolTick = millis();
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
  // ── EC-95: Debounce reed switch ──
  bool raw = readReedRaw();
  if (raw != reedRaw) {
    reedRaw = raw;
    reedLastChange = now;
  }
  if ((now - reedLastChange) >= LOCK_DEBOUNCE_MS) {
    reedStable = reedRaw;
  }

  // ── Tamper detection: lid opened while solenoid is off and not suppressed ──
  if (prevReedStable && !reedStable && !isSolenoidActive() && !tamperSuppressed) {
    tamperLatch = true;
    Serial.println(F("[TAMPER] Reed opened without unlock — TAMPER DETECTED"));
  }
  prevReedStable = reedStable;

  // ── EC-96: Passive cooling ──
  if (now > lastCoolTick) {
    unsigned long elapsed = now - lastCoolTick;
    coilTempC -= elapsed * LOCK_COOLING_RATE;
    if (coilTempC < 25.0f) coilTempC = 25.0f; // Floor at ambient
  }
  lastCoolTick = now;

  // ── Hard thermal cutoff ──
  if (solenoidOnAt > 0 && (now - solenoidOnAt >= LOCK_MAX_ACTIVE_MS)) {
    deEnergise();
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
