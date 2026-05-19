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
#define DP_KEY_CTX_BLOB "ctxBlob"
#define DP_KEY_PIN_HASH "ppHash"
#define DP_KEY_PIN_SALT "ppSalt"
#define DP_KEY_PIN_EN   "ppEn"
#define DP_KEY_PIN_RIDER "ppRid"
#define DP_KEY_AUDIT_HEAD "aqHead"
#define DP_KEY_AUDIT_TAIL "aqTail"
#define DP_KEY_CMD "cmd"
#define DP_KEY_CMD_STAGE "cmdSt"
#define DP_KEY_CMD_SERVES "cmdSv"
#define DP_KEY_CMD_ACK_STATUS "cmdAckSt"
#define DP_KEY_CMD_ACK_DETAILS "cmdAckDt"
#define DP_KEY_CMD_HASH "cmdHash"

#define DP_CMD_STAGE_NONE 0
#define DP_CMD_STAGE_PENDING 1
#define DP_CMD_STAGE_ACK_SENT 2
#define DP_CMD_STAGE_DONE 3

#define DP_AUDIT_QUEUE_CAPACITY 8

typedef struct {
	uint32_t deliveryWrites;
	uint32_t commandWrites;
	uint32_t personalPinWrites;
	uint32_t auditWrites;
} DpWriteMetrics;

void  dpBegin();
bool  dpSaveDeliveryContext(const char *otp, const char *deliveryId);
bool  dpLoadDeliveryContext(char *otpBuf, size_t otpLen,
							char *deliveryIdBuf, size_t deliveryIdLen);
void  dpClearDeliveryContext();
void  dpSaveOtp(const char *otp);
void  dpSaveDeliveryId(const char *id);
bool  dpLoadOtp(char *buf, size_t len);
bool  dpLoadDeliveryId(char *buf, size_t len);
bool  dpSaveCommandState(const char *command, uint8_t stage, uint8_t serves,
						 const char *ackStatus, const char *ackDetails);
bool  dpLoadCommandState(char *commandBuf, size_t commandLen,
						 uint8_t *stage, uint8_t *serves,
						 char *ackStatusBuf, size_t ackStatusLen,
						 char *ackDetailsBuf, size_t ackDetailsLen);
void  dpClearCommandState();
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
void  dpGetWriteMetrics(DpWriteMetrics *out);
void  dpClear();

#endif // DELIVERY_PERSIST_H
