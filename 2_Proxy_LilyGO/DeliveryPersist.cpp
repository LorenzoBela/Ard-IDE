/**
 * DeliveryPersist.cpp — NVS persistence implementation
 */

#include "DeliveryPersist.h"
#include <Preferences.h>

static void getAuditSlotKey(int idx, char *out, size_t outLen) {
  snprintf(out, outLen, "aq%u", (unsigned int)idx);
}

static Preferences prefs;

void dpBegin() {
  prefs.begin(DP_NAMESPACE, false);
  Serial.println("[DP] NVS delivery namespace opened");
}

void dpSaveOtp(const char *otp) {
  prefs.putString(DP_KEY_OTP, otp);
}

void dpSaveDeliveryId(const char *id) {
  prefs.putString(DP_KEY_DELID, id);
}

bool dpLoadOtp(char *buf, size_t len) {
  String val = prefs.getString(DP_KEY_OTP, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool dpLoadDeliveryId(char *buf, size_t len) {
  String val = prefs.getString(DP_KEY_DELID, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

void dpSavePersonalPinHash(const char *hashHex) {
  prefs.putString(DP_KEY_PIN_HASH, hashHex ? hashHex : "");
}

void dpSavePersonalPinSalt(const char *salt) {
  prefs.putString(DP_KEY_PIN_SALT, salt ? salt : "");
}

void dpSavePersonalPinEnabled(bool enabled) {
  prefs.putBool(DP_KEY_PIN_EN, enabled);
}

void dpSavePersonalPinRiderId(const char *riderId) {
  prefs.putString(DP_KEY_PIN_RIDER, riderId ? riderId : "");
}

bool dpLoadPersonalPinHash(char *buf, size_t len) {
  String val = prefs.getString(DP_KEY_PIN_HASH, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool dpLoadPersonalPinSalt(char *buf, size_t len) {
  String val = prefs.getString(DP_KEY_PIN_SALT, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool dpLoadPersonalPinEnabled() {
  return prefs.getBool(DP_KEY_PIN_EN, false);
}

bool dpLoadPersonalPinRiderId(char *buf, size_t len) {
  String val = prefs.getString(DP_KEY_PIN_RIDER, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

static int dpGetAuditHead() {
  int head = prefs.getInt(DP_KEY_AUDIT_HEAD, 0);
  if (head < 0 || head >= DP_AUDIT_QUEUE_CAPACITY) return 0;
  return head;
}

static int dpGetAuditTail() {
  int tail = prefs.getInt(DP_KEY_AUDIT_TAIL, 0);
  if (tail < 0 || tail >= DP_AUDIT_QUEUE_CAPACITY) return 0;
  return tail;
}

static void dpSetAuditHead(int head) {
  prefs.putInt(DP_KEY_AUDIT_HEAD, head % DP_AUDIT_QUEUE_CAPACITY);
}

static void dpSetAuditTail(int tail) {
  prefs.putInt(DP_KEY_AUDIT_TAIL, tail % DP_AUDIT_QUEUE_CAPACITY);
}

int dpAuditQueueCount() {
  int head = dpGetAuditHead();
  int tail = dpGetAuditTail();
  if (tail >= head) {
    return tail - head;
  }
  return (DP_AUDIT_QUEUE_CAPACITY - head) + tail;
}

bool dpEnqueueAuditEvent(const char *eventJson) {
  if (!eventJson || eventJson[0] == '\0') return false;

  int head = dpGetAuditHead();
  int tail = dpGetAuditTail();
  int nextTail = (tail + 1) % DP_AUDIT_QUEUE_CAPACITY;

  // Queue full: drop oldest entry.
  if (nextTail == head) {
    head = (head + 1) % DP_AUDIT_QUEUE_CAPACITY;
    dpSetAuditHead(head);
  }

  char slotKey[8];
  getAuditSlotKey(tail, slotKey, sizeof(slotKey));
  prefs.putString(slotKey, eventJson);
  dpSetAuditTail(nextTail);
  return true;
}

bool dpDequeueAuditEvent(char *buf, size_t len) {
  if (!buf || len == 0) return false;
  buf[0] = '\0';

  int head = dpGetAuditHead();
  int tail = dpGetAuditTail();
  if (head == tail) return false;

  char slotKey[8];
  getAuditSlotKey(head, slotKey, sizeof(slotKey));
  String val = prefs.getString(slotKey, "");
  prefs.remove(slotKey);
  dpSetAuditHead((head + 1) % DP_AUDIT_QUEUE_CAPACITY);

  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

void dpClear() {
  prefs.remove(DP_KEY_OTP);
  prefs.remove(DP_KEY_DELID);
  prefs.remove(DP_KEY_PIN_HASH);
  prefs.remove(DP_KEY_PIN_SALT);
  prefs.remove(DP_KEY_PIN_EN);
  prefs.remove(DP_KEY_PIN_RIDER);
  prefs.remove(DP_KEY_AUDIT_HEAD);
  prefs.remove(DP_KEY_AUDIT_TAIL);
  for (int i = 0; i < DP_AUDIT_QUEUE_CAPACITY; i++) {
    char slotKey[8];
    getAuditSlotKey(i, slotKey, sizeof(slotKey));
    prefs.remove(slotKey);
  }
  Serial.println("[DP] NVS delivery context cleared");
}
