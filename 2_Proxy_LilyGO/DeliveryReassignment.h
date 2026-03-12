/**
 * DeliveryReassignment.h — EC-78 Rider Swap Handling
 *
 * Adapted from PlatformIO hardware/lib/DeliveryReassignment.
 * Header-only: lightweight struct + inline logic.
 *
 * Behaviour:
 *   - Firebase writes `reassignment_pending: true` with new rider + OTP
 *   - Auto-acknowledge after 30 s timeout
 *   - Clears old OTP cache, accepts new OTP immediately
 */

#ifndef DELIVERY_REASSIGNMENT_H
#define DELIVERY_REASSIGNMENT_H

#include <Arduino.h>

#define REASSIGN_AUTO_ACK_MS   30000UL
#define REASSIGN_CLEAR_DELAY   1000UL

struct DeliveryReassignment {

  bool  pending;
  bool  acknowledged;
  char  oldRiderId[64];
  char  newRiderId[64];
  char  deliveryId[64];
  char  newOtp[8];
  bool  newOtpReady;
  unsigned long triggeredAt;
  unsigned long acknowledgedAt;

  void clear() {
    pending = false;
    acknowledged = false;
    oldRiderId[0] = '\0';
    newRiderId[0] = '\0';
    deliveryId[0] = '\0';
    newOtp[0] = '\0';
    newOtpReady = false;
    triggeredAt = 0;
    acknowledgedAt = 0;
  }

  void trigger(const char *oldR, const char *newR,
               const char *delId, unsigned long now) {
    pending = true;
    acknowledged = false;
    newOtpReady = false;
    triggeredAt = now;

    strncpy(oldRiderId, oldR, sizeof(oldRiderId) - 1); oldRiderId[63] = '\0';
    strncpy(newRiderId, newR, sizeof(newRiderId) - 1); newRiderId[63] = '\0';
    strncpy(deliveryId, delId, sizeof(deliveryId) - 1); deliveryId[63] = '\0';

    Serial.printf("[EC-78] Reassign: %s → %s\n", oldR, newR);
  }

  void setNewOtp(const char *otp) {
    strncpy(newOtp, otp, 7);
    newOtp[7] = '\0';
    newOtpReady = true;
  }

  bool isPending() const { return pending && !acknowledged; }

  /** Call from loop(). Returns true the instant auto-ack fires. */
  bool processAutoAck(unsigned long now) {
    if (!isPending()) return false;
    if ((now - triggeredAt) < REASSIGN_AUTO_ACK_MS) return false;
    acknowledge(now);
    return true;
  }

  void acknowledge(unsigned long now) {
    acknowledged = true;
    pending = false;
    acknowledgedAt = now;
    Serial.println("[EC-78] Reassignment acknowledged");
  }

  /**
   * After acknowledge, call this to get the replacement OTP.
   * Returns empty string if not ready.
   */
  const char *consumeNewOtp() {
    if (!acknowledged || !newOtpReady) return "";
    newOtpReady = false;
    return newOtp;
  }
};

#endif // DELIVERY_REASSIGNMENT_H
