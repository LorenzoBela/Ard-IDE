/**
 * AdminOverride.h — Graceful admin remote-unlock handling (EC-77)
 *
 * Ported from PlatformIO hardware/lib/AdminOverride.
 *
 * Handles the edge case where an admin triggers remote unlock while
 * the customer is mid-OTP entry on the keypad. The override is
 * immediate: clears the keypad buffer, skips face check, and unlocks.
 *
 * Header-only — lightweight struct, no .cpp needed.
 */

#ifndef ADMIN_OVERRIDE_H
#define ADMIN_OVERRIDE_H

#include <Arduino.h>
#include "Config.h"

struct AdminOverride {
  bool         pending;
  unsigned long triggeredAt;

  AdminOverride() : pending(false), triggeredAt(0) {}

  /** Call when proxy sends "UNLOCKING" status command. */
  void trigger(unsigned long now) {
    pending     = true;
    triggeredAt = now;
    Serial.println(F("[EC-77] Admin override triggered"));
  }

  /** Clear after processing. */
  void clear() {
    pending     = false;
    triggeredAt = 0;
  }

  /**
   * Returns true if the override is active and has not timed out.
   * Auto-clears on timeout so stale overrides don't fire later.
   */
  bool isActive(unsigned long now) {
    if (!pending) return false;
    if (now - triggeredAt > ADMIN_OVERRIDE_TIMEOUT_MS) {
      Serial.println(F("[EC-77] Admin override expired (timeout)"));
      clear();
      return false;
    }
    return true;
  }
};

#endif // ADMIN_OVERRIDE_H
