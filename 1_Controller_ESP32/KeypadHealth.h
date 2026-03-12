/**
 * KeypadHealth.h — Stuck key detection (EC-82)
 *
 * Ported from PlatformIO hardware/src/main.cpp EC-82 logic.
 *
 * Detects a key held continuously for > 30 seconds, which indicates
 * physical damage or debris jamming a button.
 *
 * Header-only — lightweight struct, no .cpp needed.
 */

#ifndef KEYPAD_HEALTH_H
#define KEYPAD_HEALTH_H

#include <Arduino.h>
#include "Config.h"

struct KeypadHealth {
  char          stuckKey;
  unsigned long heldSince;
  bool          isStuck;

  KeypadHealth() : stuckKey(0), heldSince(0), isStuck(false) {}

  /**
   * Call every loop() tick with the currently-held key (or 0 if none).
   * The Keypad library's getState()==HOLD and getKey() can feed this.
   */
  void update(char currentKey, unsigned long now) {
    if (currentKey != 0) {
      if (stuckKey != currentKey) {
        // New key being held — start timing
        stuckKey  = currentKey;
        heldSince = now;
        isStuck   = false;
      } else if (!isStuck && (now - heldSince >= KEYPAD_STUCK_THRESHOLD_MS)) {
        isStuck = true;
        Serial.printf("[EC-82] Key '%c' STUCK (held >%lus)\n",
                       stuckKey, KEYPAD_STUCK_THRESHOLD_MS / 1000);
      }
    } else {
      // No key held — clear
      if (isStuck) {
        Serial.println(F("[EC-82] Stuck key released"));
      }
      stuckKey  = 0;
      heldSince = 0;
      isStuck   = false;
    }
  }

  void reset() {
    stuckKey  = 0;
    heldSince = 0;
    isStuck   = false;
  }
};

#endif // KEYPAD_HEALTH_H
