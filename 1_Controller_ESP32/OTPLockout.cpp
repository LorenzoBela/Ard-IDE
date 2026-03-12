/**
 * OTPLockout.cpp — OTP attempt tracking with lockout (EC-04)
 */

#include "OTPLockout.h"

static int           attemptCount   = 0;
static bool          lockedOut      = false;
static unsigned long lockoutStartAt = 0;
static unsigned long lastAttemptAt  = 0;

void resetOtpAttempts() {
  if (attemptCount > 0) {
    Serial.println(F("[EC-04] OTP attempts reset"));
  }
  attemptCount   = 0;
  lockedOut      = false;
  lockoutStartAt = 0;
}

void recordFailedAttempt(unsigned long now) {
  attemptCount++;
  lastAttemptAt = now;
  Serial.printf("[EC-04] Failed attempt %d/%d\n", attemptCount, MAX_OTP_ATTEMPTS);

  if (attemptCount >= MAX_OTP_ATTEMPTS) {
    lockedOut      = true;
    lockoutStartAt = now;
    Serial.println(F("[EC-04] MAX ATTEMPTS — LOCKOUT ACTIVE"));
  }
}

bool isLockedOut(unsigned long now) {
  if (!lockedOut) return false;

  if (now - lockoutStartAt >= OTP_LOCKOUT_MS) {
    lockedOut    = false;
    attemptCount = 0;
    Serial.println(F("[EC-04] Lockout expired"));
    return false;
  }
  return true;
}

bool isOnCooldown(unsigned long now) {
  if (lastAttemptAt == 0) return false;
  return (now - lastAttemptAt) < OTP_ATTEMPT_COOLDOWN_MS;
}

int getAttemptsRemaining() {
  int remaining = MAX_OTP_ATTEMPTS - attemptCount;
  return remaining > 0 ? remaining : 0;
}

unsigned long getLockoutSecondsLeft(unsigned long now) {
  if (!lockedOut) return 0;
  unsigned long elapsed = now - lockoutStartAt;
  if (elapsed >= OTP_LOCKOUT_MS) return 0;
  return (OTP_LOCKOUT_MS - elapsed) / 1000;
}

int getFailedAttemptCount() {
  return attemptCount;
}
