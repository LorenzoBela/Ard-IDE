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
#define DP_KEY_PIN_HASH "ppHash"
#define DP_KEY_PIN_SALT "ppSalt"
#define DP_KEY_PIN_EN   "ppEn"
#define DP_KEY_PIN_RIDER "ppRid"
#define DP_KEY_AUDIT_HEAD "aqHead"
#define DP_KEY_AUDIT_TAIL "aqTail"

#define DP_AUDIT_QUEUE_CAPACITY 8

void  dpBegin();
void  dpSaveOtp(const char *otp);
void  dpSaveDeliveryId(const char *id);
bool  dpLoadOtp(char *buf, size_t len);
bool  dpLoadDeliveryId(char *buf, size_t len);
void  dpSavePersonalPinHash(const char *hashHex);
void  dpSavePersonalPinSalt(const char *salt);
void  dpSavePersonalPinEnabled(bool enabled);
void  dpSavePersonalPinRiderId(const char *riderId);
bool  dpLoadPersonalPinHash(char *buf, size_t len);
bool  dpLoadPersonalPinSalt(char *buf, size_t len);
bool  dpLoadPersonalPinEnabled();
bool  dpLoadPersonalPinRiderId(char *buf, size_t len);
bool  dpEnqueueAuditEvent(const char *eventJson);
bool  dpDequeueAuditEvent(char *buf, size_t len);
int   dpAuditQueueCount();
void  dpClear();

#endif // DELIVERY_PERSIST_H
