/**
 * DeliveryPersist.cpp — NVS persistence implementation
 */

#include "DeliveryPersist.h"
#include <Preferences.h>
#include <stdlib.h>

static void getAuditSlotKey(int idx, char *out, size_t outLen) {
  snprintf(out, outLen, "aq%u", (unsigned int)idx);
}

static Preferences prefs;
static DpWriteMetrics gWriteMetrics = {0, 0, 0, 0};

static void bumpWriteCounter(uint32_t *counter) {
  if (!counter) return;
  if (*counter < 0xFFFFFFFFUL) {
    (*counter)++;
  }
}

static uint32_t fnv1a32Update(uint32_t hash, const char *text) {
  const char *src = text ? text : "";
  while (*src) {
    hash ^= (uint8_t)*src++;
    hash *= 16777619u;
  }
  hash ^= 0xFF;
  hash *= 16777619u;
  return hash;
}

static uint32_t deliveryContextHash(const char *otp, const char *deliveryId) {
  uint32_t hash = 2166136261u;
  hash = fnv1a32Update(hash, otp);
  hash = fnv1a32Update(hash, deliveryId);
  return hash;
}

static uint32_t commandStateHash(const char *command, uint8_t stage,
                                 uint8_t serves, const char *ackStatus,
                                 const char *ackDetails) {
  uint32_t hash = 2166136261u;
  hash = fnv1a32Update(hash, command);
  char stageBuf[8];
  snprintf(stageBuf, sizeof(stageBuf), "%u", (unsigned int)stage);
  hash = fnv1a32Update(hash, stageBuf);
  char servesBuf[8];
  snprintf(servesBuf, sizeof(servesBuf), "%u", (unsigned int)serves);
  hash = fnv1a32Update(hash, servesBuf);
  hash = fnv1a32Update(hash, ackStatus);
  hash = fnv1a32Update(hash, ackDetails);
  return hash;
}

static bool parseDeliveryContextBlob(const String &blob, char *otpBuf,
                                     size_t otpLen, char *deliveryIdBuf,
                                     size_t deliveryIdLen) {
  int p1 = blob.indexOf('|');
  int p2 = (p1 >= 0) ? blob.indexOf('|', p1 + 1) : -1;
  int p3 = (p2 >= 0) ? blob.indexOf('|', p2 + 1) : -1;
  if (p1 < 0 || p2 < 0 || p3 < 0) {
    return false;
  }

  String version = blob.substring(0, p1);
  if (version != "1") {
    return false;
  }

  String otp = blob.substring(p1 + 1, p2);
  String deliveryId = blob.substring(p2 + 1, p3);
  String hashHex = blob.substring(p3 + 1);
  if (hashHex.length() == 0) {
    return false;
  }

  uint32_t parsedHash = strtoul(hashHex.c_str(), nullptr, 16);
  uint32_t expectedHash = deliveryContextHash(otp.c_str(), deliveryId.c_str());
  if (parsedHash != expectedHash) {
    return false;
  }

  if (otpBuf && otpLen > 0) {
    strncpy(otpBuf, otp.c_str(), otpLen - 1);
    otpBuf[otpLen - 1] = '\0';
  }
  if (deliveryIdBuf && deliveryIdLen > 0) {
    strncpy(deliveryIdBuf, deliveryId.c_str(), deliveryIdLen - 1);
    deliveryIdBuf[deliveryIdLen - 1] = '\0';
  }
  return true;
}

static void readLegacyContextStrings(String &otp, String &deliveryId) {
  otp = prefs.getString(DP_KEY_OTP, "");
  deliveryId = prefs.getString(DP_KEY_DELID, "");
}

void dpBegin() {
  prefs.begin(DP_NAMESPACE, false);
  Serial.println("[DP] NVS delivery namespace opened");
}

bool dpSaveDeliveryContext(const char *otp, const char *deliveryId) {
  const char *safeOtp = otp ? otp : "";
  const char *safeDeliveryId = deliveryId ? deliveryId : "";
  uint32_t hash = deliveryContextHash(safeOtp, safeDeliveryId);

  char hashHex[9];
  snprintf(hashHex, sizeof(hashHex), "%08lX", (unsigned long)hash);

  String blob = "1|";
  blob += safeOtp;
  blob += "|";
  blob += safeDeliveryId;
  blob += "|";
  blob += hashHex;

  size_t written = prefs.putString(DP_KEY_CTX_BLOB, blob);

  // Keep legacy keys in sync so older builds can still recover context.
  prefs.putString(DP_KEY_OTP, safeOtp);
  prefs.putString(DP_KEY_DELID, safeDeliveryId);

  if (written > 0) {
    bumpWriteCounter(&gWriteMetrics.deliveryWrites);
  }

  return written > 0;
}

bool dpLoadDeliveryContext(char *otpBuf, size_t otpLen, char *deliveryIdBuf,
                           size_t deliveryIdLen) {
  if (otpBuf && otpLen > 0) {
    otpBuf[0] = '\0';
  }
  if (deliveryIdBuf && deliveryIdLen > 0) {
    deliveryIdBuf[0] = '\0';
  }

  String blob = prefs.getString(DP_KEY_CTX_BLOB, "");
  if (blob.length() > 0 &&
      parseDeliveryContextBlob(blob, otpBuf, otpLen, deliveryIdBuf,
                               deliveryIdLen)) {
    return true;
  }

  // Fallback for old persisted format, then migrate to atomic blob format.
  String legacyOtp, legacyDeliveryId;
  readLegacyContextStrings(legacyOtp, legacyDeliveryId);
  if (legacyOtp.length() == 0 && legacyDeliveryId.length() == 0) {
    return false;
  }

  if (otpBuf && otpLen > 0) {
    strncpy(otpBuf, legacyOtp.c_str(), otpLen - 1);
    otpBuf[otpLen - 1] = '\0';
  }
  if (deliveryIdBuf && deliveryIdLen > 0) {
    strncpy(deliveryIdBuf, legacyDeliveryId.c_str(), deliveryIdLen - 1);
    deliveryIdBuf[deliveryIdLen - 1] = '\0';
  }

  dpSaveDeliveryContext(legacyOtp.c_str(), legacyDeliveryId.c_str());
  return true;
}

void dpClearDeliveryContext() {
  prefs.remove(DP_KEY_CTX_BLOB);
  prefs.remove(DP_KEY_OTP);
  prefs.remove(DP_KEY_DELID);
}

void dpSaveOtp(const char *otp) {
  char currentOtp[16] = "";
  char currentDeliveryId[96] = "";
  dpLoadDeliveryContext(currentOtp, sizeof(currentOtp), currentDeliveryId,
                        sizeof(currentDeliveryId));
  dpSaveDeliveryContext(otp ? otp : "", currentDeliveryId);
}

void dpSaveDeliveryId(const char *id) {
  char currentOtp[16] = "";
  char currentDeliveryId[96] = "";
  dpLoadDeliveryContext(currentOtp, sizeof(currentOtp), currentDeliveryId,
                        sizeof(currentDeliveryId));
  dpSaveDeliveryContext(currentOtp, id ? id : "");
}

bool dpLoadOtp(char *buf, size_t len) {
  char contextOtp[16] = "";
  char contextDeliveryId[96] = "";
  if (dpLoadDeliveryContext(contextOtp, sizeof(contextOtp), contextDeliveryId,
                            sizeof(contextDeliveryId)) &&
      contextOtp[0] != '\0') {
    strncpy(buf, contextOtp, len - 1);
    buf[len - 1] = '\0';
    return true;
  }

  String val = prefs.getString(DP_KEY_OTP, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool dpLoadDeliveryId(char *buf, size_t len) {
  char contextOtp[16] = "";
  char contextDeliveryId[96] = "";
  if (dpLoadDeliveryContext(contextOtp, sizeof(contextOtp), contextDeliveryId,
                            sizeof(contextDeliveryId)) &&
      contextDeliveryId[0] != '\0') {
    strncpy(buf, contextDeliveryId, len - 1);
    buf[len - 1] = '\0';
    return true;
  }

  String val = prefs.getString(DP_KEY_DELID, "");
  if (val.length() == 0) return false;
  strncpy(buf, val.c_str(), len - 1);
  buf[len - 1] = '\0';
  return true;
}

bool dpSaveCommandState(const char *command, uint8_t stage, uint8_t serves,
                        const char *ackStatus, const char *ackDetails) {
  const char *safeCommand = command ? command : "";
  const char *safeAckStatus = ackStatus ? ackStatus : "";
  const char *safeAckDetails = ackDetails ? ackDetails : "";

  prefs.putString(DP_KEY_CMD, safeCommand);
  prefs.putUChar(DP_KEY_CMD_STAGE, stage);
  prefs.putUChar(DP_KEY_CMD_SERVES, serves);
  prefs.putString(DP_KEY_CMD_ACK_STATUS, safeAckStatus);
  prefs.putString(DP_KEY_CMD_ACK_DETAILS, safeAckDetails);

  uint32_t hash = commandStateHash(safeCommand, stage, serves, safeAckStatus,
                                   safeAckDetails);
  prefs.putUInt(DP_KEY_CMD_HASH, hash);
  bumpWriteCounter(&gWriteMetrics.commandWrites);
  return true;
}

bool dpLoadCommandState(char *commandBuf, size_t commandLen, uint8_t *stage,
                        uint8_t *serves, char *ackStatusBuf,
                        size_t ackStatusLen, char *ackDetailsBuf,
                        size_t ackDetailsLen) {
  if (!commandBuf || commandLen == 0 || !stage || !serves || !ackStatusBuf ||
      ackStatusLen == 0 || !ackDetailsBuf || ackDetailsLen == 0) {
    return false;
  }

  commandBuf[0] = '\0';
  ackStatusBuf[0] = '\0';
  ackDetailsBuf[0] = '\0';
  *stage = DP_CMD_STAGE_NONE;
  *serves = 0;

  String command = prefs.getString(DP_KEY_CMD, "");
  uint8_t persistedStage = prefs.getUChar(DP_KEY_CMD_STAGE, DP_CMD_STAGE_NONE);
  uint8_t persistedServes = prefs.getUChar(DP_KEY_CMD_SERVES, 0);
  String persistedAckStatus = prefs.getString(DP_KEY_CMD_ACK_STATUS, "");
  String persistedAckDetails = prefs.getString(DP_KEY_CMD_ACK_DETAILS, "");
  uint32_t persistedHash = prefs.getUInt(DP_KEY_CMD_HASH, 0);

  if (command.length() == 0 && persistedStage == DP_CMD_STAGE_NONE &&
      persistedAckStatus.length() == 0 && persistedAckDetails.length() == 0) {
    return false;
  }

  uint32_t expectedHash =
      commandStateHash(command.c_str(), persistedStage, persistedServes,
                       persistedAckStatus.c_str(), persistedAckDetails.c_str());
  if (persistedHash == 0 || persistedHash != expectedHash) {
    return false;
  }

  strncpy(commandBuf, command.c_str(), commandLen - 1);
  commandBuf[commandLen - 1] = '\0';
  strncpy(ackStatusBuf, persistedAckStatus.c_str(), ackStatusLen - 1);
  ackStatusBuf[ackStatusLen - 1] = '\0';
  strncpy(ackDetailsBuf, persistedAckDetails.c_str(), ackDetailsLen - 1);
  ackDetailsBuf[ackDetailsLen - 1] = '\0';
  *stage = persistedStage;
  *serves = persistedServes;
  return true;
}

void dpClearCommandState() {
  prefs.remove(DP_KEY_CMD);
  prefs.remove(DP_KEY_CMD_STAGE);
  prefs.remove(DP_KEY_CMD_SERVES);
  prefs.remove(DP_KEY_CMD_ACK_STATUS);
  prefs.remove(DP_KEY_CMD_ACK_DETAILS);
  prefs.remove(DP_KEY_CMD_HASH);
}

void dpSavePersonalPinHash(const char *hashHex) {
  prefs.putString(DP_KEY_PIN_HASH, hashHex ? hashHex : "");
  bumpWriteCounter(&gWriteMetrics.personalPinWrites);
}

void dpSavePersonalPinSalt(const char *salt) {
  prefs.putString(DP_KEY_PIN_SALT, salt ? salt : "");
  bumpWriteCounter(&gWriteMetrics.personalPinWrites);
}

void dpSavePersonalPinEnabled(bool enabled) {
  prefs.putBool(DP_KEY_PIN_EN, enabled);
  bumpWriteCounter(&gWriteMetrics.personalPinWrites);
}

void dpSavePersonalPinRiderId(const char *riderId) {
  prefs.putString(DP_KEY_PIN_RIDER, riderId ? riderId : "");
  bumpWriteCounter(&gWriteMetrics.personalPinWrites);
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
  bumpWriteCounter(&gWriteMetrics.auditWrites);
  return true;
}

void dpGetWriteMetrics(DpWriteMetrics *out) {
  if (!out) return;
  *out = gWriteMetrics;
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
  dpClearDeliveryContext();
  dpClearCommandState();
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
