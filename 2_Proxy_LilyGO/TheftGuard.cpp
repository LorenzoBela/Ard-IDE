/**
 * TheftGuard.cpp — EC-81 Theft Detection Implementation
 */

#include "TheftGuard.h"

static struct {
  TheftState state;
  bool       isStolen;
  char       reportedBy[64];
  unsigned long reportedAt;
  float      lastLat, lastLng, lastSpeed;
  bool       lockdownActive;
  unsigned long lockdownAt;
  unsigned long geofenceBreachAt;
  unsigned long suspiciousStartTime;
  char       notes[64];
} ts;

static struct {
  float  centerLat, centerLng, radiusKm;
  bool   configured;
} gf;

static LocEntry locHist[TG_LOCATION_HISTORY_MAX];
static int locHistCount = 0;
static int locHistIdx   = 0;

static float haversineKm(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371.0f;
  float dLat = (lat2 - lat1) * M_PI / 180.0f;
  float dLng = (lng2 - lng1) * M_PI / 180.0f;
  float a = sinf(dLat / 2) * sinf(dLat / 2) +
            cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) *
            sinf(dLng / 2) * sinf(dLng / 2);
  return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

void theftGuardInit() {
  ts.state             = TG_NORMAL;
  ts.isStolen          = false;
  ts.reportedBy[0]     = '\0';
  ts.reportedAt        = 0;
  ts.lastLat = ts.lastLng = ts.lastSpeed = 0;
  ts.lockdownActive    = false;
  ts.lockdownAt        = 0;
  ts.geofenceBreachAt  = 0;
  ts.suspiciousStartTime = 0;
  ts.notes[0]          = '\0';

  gf.configured = false;
  gf.centerLat = gf.centerLng = 0;
  gf.radiusKm = TG_GEOFENCE_RADIUS_KM;

  locHistCount = 0;
  locHistIdx   = 0;
}

void theftGuardSetGeofence(float cLat, float cLng, float rKm) {
  gf.centerLat = cLat;
  gf.centerLng = cLng;
  gf.radiusKm  = rKm;
  gf.configured = true;
}

static bool withinGeofence(float lat, float lng) {
  if (!gf.configured) return true;
  return haversineKm(gf.centerLat, gf.centerLng, lat, lng) <= gf.radiusKm;
}

static void addLocationHistory(float lat, float lng, unsigned long now) {
  locHist[locHistIdx] = {lat, lng, now};
  locHistIdx = (locHistIdx + 1) % TG_LOCATION_HISTORY_MAX;
  if (locHistCount < TG_LOCATION_HISTORY_MAX) locHistCount++;
}

void theftGuardUpdate(float lat, float lng, float speedMps,
                      bool ignitionOn, unsigned long now) {
  if (ts.state == TG_LOCKDOWN || ts.state == TG_RECOVERED) return;

  ts.lastLat   = lat;
  ts.lastLng   = lng;
  ts.lastSpeed = speedMps;
  addLocationHistory(lat, lng, now);

  // Motion without ignition
  if (!ignitionOn && speedMps >= TG_MOTION_THRESHOLD_MPS) {
    if (ts.state == TG_NORMAL) {
      ts.state = TG_SUSPICIOUS;
      ts.suspiciousStartTime = now;
      Serial.println("[EC-81] Suspicious motion detected");
    }
  } else if (ignitionOn && ts.state == TG_SUSPICIOUS) {
    ts.state = TG_NORMAL;
    ts.suspiciousStartTime = 0;
  }

  // Geofence breach
  if (gf.configured && !withinGeofence(lat, lng)) {
    if (ts.state != TG_STOLEN && ts.state != TG_LOCKDOWN) {
      ts.state = TG_STOLEN;
      ts.isStolen = true;
      ts.geofenceBreachAt = now;
      Serial.println("[EC-81] Geofence breach → STOLEN");
    }
  }

  // Escalate suspicious → stolen after timeout
  if (ts.state == TG_SUSPICIOUS &&
      (now - ts.suspiciousStartTime) >= TG_SUSPICIOUS_TIMEOUT_MS) {
    ts.state = TG_STOLEN;
    ts.isStolen = true;
    Serial.println("[EC-81] Suspicious timeout → STOLEN");
  }
}

void theftGuardActivateLockdown(const char *adminUid, unsigned long now) {
  ts.state = TG_LOCKDOWN;
  ts.lockdownActive = true;
  ts.lockdownAt = now;
  ts.isStolen = true;
  if (adminUid) {
    strncpy(ts.reportedBy, adminUid, sizeof(ts.reportedBy) - 1);
    ts.reportedBy[sizeof(ts.reportedBy) - 1] = '\0';
  }
  Serial.println("[EC-81] Lockdown activated");
}

void theftGuardDeactivateLockdown() {
  ts.state = TG_RECOVERED;
  ts.lockdownActive = false;
  ts.isStolen = false;
  Serial.println("[EC-81] Lockdown deactivated → RECOVERED");
}

void theftGuardReportTheft(const char *uid, unsigned long now, const char *notes) {
  ts.isStolen = true;
  ts.reportedAt = now;
  if (uid) { strncpy(ts.reportedBy, uid, 63); ts.reportedBy[63] = '\0'; }
  if (notes) { strncpy(ts.notes, notes, 63); ts.notes[63] = '\0'; }
  if (ts.state == TG_NORMAL || ts.state == TG_SUSPICIOUS)
    ts.state = TG_STOLEN;
}

void theftGuardReset() { theftGuardInit(); }

TheftState  theftGuardGetState()      { return ts.state; }
bool        theftGuardIsLockdown()    { return ts.lockdownActive; }
bool        theftGuardIsStolen()      { return ts.isStolen; }
bool        theftGuardShouldBlockOtp(){ return ts.lockdownActive || ts.state == TG_LOCKDOWN || ts.state == TG_STOLEN; }

const char *theftGuardStateStr() {
  switch (ts.state) {
    case TG_NORMAL:     return "NORMAL";
    case TG_SUSPICIOUS: return "SUSPICIOUS";
    case TG_STOLEN:     return "STOLEN";
    case TG_LOCKDOWN:   return "LOCKDOWN";
    case TG_RECOVERED:  return "RECOVERED";
    default:            return "UNKNOWN";
  }
}

float theftGuardLastLat()   { return ts.lastLat; }
float theftGuardLastLng()   { return ts.lastLng; }
float theftGuardLastSpeed() { return ts.lastSpeed; }
