/**
 * DeliveryPersist.cpp — NVS persistence implementation
 */

#include "DeliveryPersist.h"
#include <Preferences.h>

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

void dpClear() {
  prefs.remove(DP_KEY_OTP);
  prefs.remove(DP_KEY_DELID);
  Serial.println("[DP] NVS delivery context cleared");
}
