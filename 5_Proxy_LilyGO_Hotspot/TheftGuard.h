/**
 * TheftGuard.h — EC-81 Theft Detection State Machine
 *
 * Refactored from PlatformIO hardware/lib/TheftDetection.
 * Static globals replaced with a proper struct to avoid ODR issues
 * when included from both .ino and .cpp files.
 *
 * Features:
 *   - Motion without ignition → SUSPICIOUS
 *   - Geofence breach → STOLEN
 *   - Suspicious timeout escalation (60 s)
 *   - Admin lockdown / recovery
 *   - Location history ring buffer (288 entries ≈ 24 h @ 5 min)
 */

#ifndef THEFT_GUARD_H
#define THEFT_GUARD_H

#include <Arduino.h>
#include <math.h>

#define TG_GEOFENCE_RADIUS_KM       50.0f
#define TG_MOTION_THRESHOLD_MPS     2.0f
#define TG_LOCATION_HISTORY_MAX     288
#define TG_SUSPICIOUS_TIMEOUT_MS    60000UL

enum TheftState : uint8_t {
  TG_NORMAL,
  TG_SUSPICIOUS,
  TG_STOLEN,
  TG_LOCKDOWN,
  TG_RECOVERED
};

struct LocEntry { float lat; float lng; unsigned long ts; };

void          theftGuardInit();
void          theftGuardUpdate(float lat, float lng, float speedMps,
                               bool ignitionOn, unsigned long now);
void          theftGuardSetGeofence(float cLat, float cLng, float rKm);
void          theftGuardActivateLockdown(const char *adminUid, unsigned long now);
void          theftGuardDeactivateLockdown();
void          theftGuardReportTheft(const char *uid, unsigned long now, const char *notes);
void          theftGuardReset();

TheftState    theftGuardGetState();
bool          theftGuardIsLockdown();
bool          theftGuardIsStolen();
bool          theftGuardShouldBlockOtp();
const char   *theftGuardStateStr();

float         theftGuardLastLat();
float         theftGuardLastLng();
float         theftGuardLastSpeed();

#endif // THEFT_GUARD_H
