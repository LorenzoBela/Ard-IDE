/**
 * DeliveryPersist.h — NVS Persistence for Delivery Context
 *
 * Uses ESP32 Preferences (NVS) to persist cachedOtp + cachedDeliveryId
 * across reboots / brown-outs so the Controller can still validate OTP
 * even if the LTE link was lost at boot.
 */

#ifndef DELIVERY_PERSIST_H
#define DELIVERY_PERSIST_H

#include <Arduino.h>

#define DP_NAMESPACE  "delivery"
#define DP_KEY_OTP    "otp"
#define DP_KEY_DELID  "delId"

void  dpBegin();
void  dpSaveOtp(const char *otp);
void  dpSaveDeliveryId(const char *id);
bool  dpLoadOtp(char *buf, size_t len);
bool  dpLoadDeliveryId(char *buf, size_t len);
void  dpClear();

#endif // DELIVERY_PERSIST_H
