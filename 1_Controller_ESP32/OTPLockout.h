/**
 * OTPLockout.h — OTP attempt tracking with lockout (EC-04)
 *
 * Ported from PlatformIO hardware/src/main.cpp EC-04 logic.
 *
 * Features:
 *   - Tracks failed OTP attempts (max 5)
 *   - 5-minute lockout after max attempts reached
 *   - 1-second cooldown between consecutive attempts
 *   - Resets on correct OTP or proxy command
 */

#ifndef OTP_LOCKOUT_H
#define OTP_LOCKOUT_H

#include <Arduino.h>
#include "Config.h"

/** Call once a correct OTP is entered or on admin/proxy reset. */
void resetOtpAttempts();

/** Record a failed attempt. May trigger lockout. */
void recordFailedAttempt(unsigned long now);

/** true if currently in lockout period. Also clears lockout if expired. */
bool isLockedOut(unsigned long now);

/** true if cooldown has not yet elapsed since the last attempt. */
bool isOnCooldown(unsigned long now);

/** Remaining attempts before lockout (0 when locked out). */
int getAttemptsRemaining();

/** Seconds remaining in lockout (0 if not locked out). */
unsigned long getLockoutSecondsLeft(unsigned long now);

/** Total failed attempts in current cycle. */
int getFailedAttemptCount();

#endif // OTP_LOCKOUT_H
