/**
 * OTPLockout.cpp — OTP attempt tracking with lockout (EC-04)
 */

#include "OTPLockout.h"
#include <Preferences.h>

static int           attemptCount   = 0;
static bool          lockedOut      = false;
static unsigned long lockoutStartAt = 0;
static unsigned long lastAttemptAt  = 0;

static Preferences otpPrefs;
static bool otpPrefsReady = false;

static const char *OTP_NS = "otpLock";
static const char *OTP_KEY_ATTEMPTS = "att";
static const char *OTP_KEY_LOCKED = "lock";
static const char *OTP_KEY_COOLDOWN = "cool";

static void persistOtpState() {
  if (!otpPrefsReady) return;
  otpPrefs.putInt(OTP_KEY_ATTEMPTS, attemptCount);
  otpPrefs.putBool(OTP_KEY_LOCKED, lockedOut);
  otpPrefs.putBool(OTP_KEY_COOLDOWN, lastAttemptAt != 0);
}

void initOtpLockoutPersistence() {
  otpPrefsReady = otpPrefs.begin(OTP_NS, false);
  if (!otpPrefsReady) {
    Serial.println(F("[EC-04] OTP lockout NVS unavailable"));
    return;
  }

  attemptCount = otpPrefs.getInt(OTP_KEY_ATTEMPTS, 0);
  if (attemptCount < 0) attemptCount = 0;
  if (attemptCount > MAX_OTP_ATTEMPTS) attemptCount = MAX_OTP_ATTEMPTS;

  lockedOut = otpPrefs.getBool(OTP_KEY_LOCKED, false);
  if (lockedOut) {
    // Conservative behavior: re-apply full lockout window after reboot.
    lockoutStartAt = millis();
  } else {
    lockoutStartAt = 0;
  }

  if (otpPrefs.getBool(OTP_KEY_COOLDOWN, false)) {
    lastAttemptAt = millis();
  } else {
    lastAttemptAt = 0;
  }

  Serial.printf("[EC-04] Lockout restore: attempts=%d locked=%d\n",
                attemptCount, lockedOut ? 1 : 0);
}

void resetOtpAttempts() {
  if (attemptCount > 0) {
    Serial.println(F("[EC-04] OTP attempts reset"));
  }
  attemptCount   = 0;
  lockedOut      = false;
  lockoutStartAt = 0;
  lastAttemptAt  = 0;
  persistOtpState();
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

  persistOtpState();
}

bool isLockedOut(unsigned long now) {
  if (!lockedOut) return false;

  if (now - lockoutStartAt >= OTP_LOCKOUT_MS) {
    lockedOut    = false;
    attemptCount = 0;
    lockoutStartAt = 0;
    lastAttemptAt = 0;
    persistOtpState();
    Serial.println(F("[EC-04] Lockout expired"));
    return false;
  }
  return true;
}

bool isOnCooldown(unsigned long now) {
  if (lastAttemptAt == 0) return false;
  if ((now - lastAttemptAt) < OTP_ATTEMPT_COOLDOWN_MS) {
    return true;
  }
  lastAttemptAt = 0;
  persistOtpState();
  return false;
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
