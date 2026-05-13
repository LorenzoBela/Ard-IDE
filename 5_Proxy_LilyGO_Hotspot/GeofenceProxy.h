/**
 * GeofenceProxy.h — EC-92/93/94 Geofence Stability
 *
 * Adapted from PlatformIO hardware/lib/GeofenceStability.
 * Header-only: all logic inlined for Arduino IDE compatibility.
 *
 * EC-92: Urban canyon HDOP gating (freeze state on degraded signal)
 * EC-93: Warehouse return detection (5-min timer)
 * EC-94: Inner/outer radius hysteresis with consecutive-sample confirmation
 */

#ifndef GEOFENCE_PROXY_H
#define GEOFENCE_PROXY_H

#include <Arduino.h>
#include <math.h>

#define GEO_INNER_RADIUS_M       40.0
#define GEO_OUTER_RADIUS_M       60.0
#define GEO_DEFAULT_RADIUS_M     50.0
#define GEO_EXPANDED_RADIUS_M    100.0
#define GEO_HYSTERESIS_SAMPLES   3
#define GPS_HDOP_DEGRADED        5.0
#define GPS_MIN_SATELLITES       4
#define WAREHOUSE_RETURN_MS      300000UL   // 5 min

enum GeoState {
  GEO_OUTSIDE,
  GEO_ENTERING,
  GEO_INSIDE,
  GEO_EXITING,
  GEO_DEAD_ZONE
};

struct GeoSnapshot {
  GeoState stableState;
  GeoState rawState;
  double   distanceM;
  double   hdop;
  int      satellites;
  bool     urbanCanyon;
  bool     warehouseReturn;
  int      hysteresisCount;
};

struct GeofenceProxy {

  double targetLat, targetLon;       // dropoff (primary target for state machine)
  double pickupLat, pickupLon;       // pickup point
  bool   pickupSet;
  double warehouseLat, warehouseLon;
  bool   warehouseSet;

  GeoSnapshot snap;
  GeoState    prevConfirmed;
  GeoState    consecState;
  int         consecCount;
  unsigned long warehouseEntryMs;

  void reset() {
    targetLat = targetLon = 0;
    pickupLat = pickupLon = 0;
    pickupSet = false;
    warehouseLat = warehouseLon = 0;
    warehouseSet = false;
    snap.stableState   = GEO_OUTSIDE;
    snap.rawState      = GEO_OUTSIDE;
    snap.distanceM     = 0;
    snap.hdop          = 0;
    snap.satellites    = 0;
    snap.urbanCanyon   = false;
    snap.warehouseReturn = false;
    snap.hysteresisCount = 0;
    prevConfirmed      = GEO_OUTSIDE;
    consecState        = GEO_OUTSIDE;
    consecCount        = 0;
    warehouseEntryMs   = 0;
  }

  void setTarget(double lat, double lon) { targetLat = lat; targetLon = lon; }

  void setPickup(double lat, double lon) {
    pickupLat = lat; pickupLon = lon;
    pickupSet = (lat != 0.0 || lon != 0.0);
  }

  void setWarehouse(double lat, double lon) {
    warehouseLat = lat;
    warehouseLon = lon;
    warehouseSet = true;
  }

  GeoSnapshot update(double lat, double lon, double hdop, int sats) {
    unsigned long now = millis();
    double dist = haversineM(lat, lon, targetLat, targetLon);

    snap.distanceM  = dist;
    snap.hdop       = hdop;
    snap.satellites = sats;
    snap.urbanCanyon = (hdop > GPS_HDOP_DEGRADED) || (sats < GPS_MIN_SATELLITES);

    if (snap.urbanCanyon) return snap;

    GeoState raw;
    if      (dist < GEO_INNER_RADIUS_M) raw = GEO_INSIDE;
    else if (dist > GEO_OUTER_RADIUS_M) raw = GEO_OUTSIDE;
    else                                  raw = GEO_DEAD_ZONE;
    snap.rawState = raw;

    if (raw == GEO_DEAD_ZONE) { consecCount = 0; return snap; }

    if (raw == consecState) consecCount++;
    else { consecState = raw; consecCount = 1; }
    snap.hysteresisCount = consecCount;

    if (consecCount >= GEO_HYSTERESIS_SAMPLES && snap.stableState != raw) {
      snap.stableState = raw;
      prevConfirmed    = raw;
    }

    if (warehouseSet) {
      double wDist = haversineM(lat, lon, warehouseLat, warehouseLon);
      if (wDist < GEO_DEFAULT_RADIUS_M) {
        if (warehouseEntryMs == 0) warehouseEntryMs = now;
        if ((now - warehouseEntryMs) >= WAREHOUSE_RETURN_MS)
          snap.warehouseReturn = true;
      } else {
        warehouseEntryMs = 0;
        snap.warehouseReturn = false;
      }
    }
    return snap;
  }

  bool isArrived()          const { return snap.stableState == GEO_INSIDE; }
  double effectiveRadius()  const { return snap.urbanCanyon ? GEO_EXPANDED_RADIUS_M : GEO_DEFAULT_RADIUS_M; }

  bool isNearDropoff(double lat, double lon) const {
    return haversineM(lat, lon, targetLat, targetLon) <= GEO_OUTER_RADIUS_M;
  }

  bool isNearPickup(double lat, double lon) const {
    if (!pickupSet) return false;
    return haversineM(lat, lon, pickupLat, pickupLon) <= GEO_OUTER_RADIUS_M;
  }

  /** True if current position is within the outer radius of EITHER pickup or dropoff */
  bool isNearAnyTarget(double lat, double lon) const {
    double dropDist = haversineM(lat, lon, targetLat, targetLon);
    if (dropDist <= GEO_OUTER_RADIUS_M) return true;
    if (pickupSet) {
      double pickDist = haversineM(lat, lon, pickupLat, pickupLon);
      if (pickDist <= GEO_OUTER_RADIUS_M) return true;
    }
    return false;
  }

  const char* stateStr(GeoState s) const {
    switch (s) {
      case GEO_OUTSIDE:   return "OUTSIDE";
      case GEO_ENTERING:  return "ENTERING";
      case GEO_INSIDE:    return "INSIDE";
      case GEO_EXITING:   return "EXITING";
      case GEO_DEAD_ZONE: return "DEAD_ZONE";
      default:            return "UNKNOWN";
    }
  }

private:
  static double toRad(double d) { return d * PI / 180.0; }

  static double haversineM(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = toRad(lat2 - lat1);
    double dLon = toRad(lon2 - lon1);
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(toRad(lat1)) * cos(toRad(lat2)) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
    return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  }
};

#endif // GEOFENCE_PROXY_H
