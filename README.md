# Ard IDE Firmware (Production Reference)

## 1) Production Scope

This document defines the production firmware set for Smart Top Box in this folder.

Production folders:
- 1_Controller_ESP32
- 2_Proxy_LilyGO
- 3_Eye_ESP32CAM

Non-production folders in this directory are test/prototype/legacy only and are not release firmware.

Current non-production folders:
- ESP32CAM_OV3660_Supabase_R3_Test
- GPS_LTE_Firebase_Test
- Tester
- .git

## 2) System Overview

Smart Top Box production firmware runs on 3 boards with clear separation of concerns:

- Controller (ESP32 DevKit): user interaction, OTP validation, lock actuation, local safety enforcement.
- Proxy (LilyGO T-SIM A7670E): LTE/GPS/backend bridge, local AP host, local HTTP routing.
- Eye (AI-Thinker ESP32-CAM): face detection service and capture/upload producer.

High-level unlock flow:
1. Controller pulls delivery context and OTP from Proxy.
2. Rider enters OTP on Controller keypad.
3. Controller asks Proxy for face verification.
4. Proxy forwards to Eye and returns FACE_OK or NO_FACE.
5. Controller unlocks with lock safety logic.
6. Controller posts event data to Proxy.
7. Proxy writes backend records and relays photo uploads.

## 3) Firmware Responsibilities by Board

### 3.1) 1_Controller_ESP32

Primary role:
- Main state machine for lock-box experience.

Responsibilities:
- Connect to Proxy SoftAP (SmartTopBox_AP_*).
- Poll delivery context (OTP, delivery_id, status).
- Support utility diagnostics mode on keypad keys 1/2/3 while in standby and idle states.
- Gate keypad flow based on delivery state/status.
- Enforce OTP lockout policy and cooldown.
- Request face check through Proxy.
- Control solenoid and reed-verified lock/relock.
- Detect tamper, keypad issues, and display health failures.
- Report event, alert, tamper, and command-ack payloads to Proxy.

Key states implemented:
- STATE_CONNECTING_WIFI
- STATE_STANDBY
- STATE_IDLE
- STATE_ENTERING_PIN
- STATE_VERIFYING_OTP
- STATE_REQUESTING_FACE
- STATE_UNLOCKING
- STATE_RELOCKING
- STATE_SHOW_MESSAGE

### 3.2) 2_Proxy_LilyGO

Primary role:
- Communication and integration backbone.

Responsibilities:
- Own LTE modem and data path to backend.
- Host local AP used by Controller and Eye.
- Serve board-to-board endpoints for Controller and Eye.
- Cache delivery context for fast local reads.
- Serve cached diagnostics endpoint for Controller utility mode.
- Forward face-check requests to Eye.
- Relay camera JPEG uploads to Supabase via LTE.
- Maintain command token/ack workflows.
- Run geofence/theft/battery/persistence support modules.

### 3.3) 3_Eye_ESP32CAM

Primary role:
- Face-gate unlock authorization and capture evidence.

Responsibilities:
- Connect to Proxy AP.
- Run person/face detection window on request.
- Return face status quickly to avoid blocking unlock flow.
- Queue high-resolution capture and POST JPEG bytes to Proxy.
- Offer UART fallback path for face query.

## 4) Production Source Map

Only production files present in these folders are listed.

### 4.1) 1_Controller_ESP32

- 1_Controller_ESP32.ino
  Main non-blocking state machine and orchestration layer.

- Config.h
  Central constants for pins, timing, retries, thresholds, and feature tuning.

- HardwareIO.h / HardwareIO.cpp
  LCD and keypad implementation, display update behavior, input handling.

- ProxyClient.h / ProxyClient.cpp
  WiFi scan/reconnect, endpoint calls, context parsing, UDP logging, face-check fallback.

- LockSafety.h / LockSafety.cpp
  Solenoid control, retry logic, reed debounce, tamper detection, thermal model/cutoff.

- OTPLockout.h / OTPLockout.cpp
  OTP attempt counting, lockout timing, cooldown handling.

- DisplayHealth.h / DisplayHealth.cpp
  LCD I2C health checks and fallback signaling behaviors.

- AdminOverride.h
  Remote unlock override handling with timeout-safe semantics.

- KeypadHealth.h
  Held-key/stuck-key detection logic.

### 4.2) 2_Proxy_LilyGO

- 2_Proxy_LilyGO.ino
  LTE/AP/main integration sketch for routing, caching, relay, and backend writes.

- BatteryMonitor.h / BatteryMonitor.cpp
  Battery voltage sampling, smoothing, threshold checks.

- DeliveryPersist.h / DeliveryPersist.cpp
  NVS persistence for OTP and delivery ID context.

- DeliveryReassignment.h
  Rider reassignment handling and auto-ack mechanics.

- GeofenceProxy.h
  Header-only geofence stability model and hysteresis policy.

- TheftGuard.h / TheftGuard.cpp
  Theft state machine, lockdown/recovery helpers.

### 4.3) 3_Eye_ESP32CAM

- 3_Eye_ESP32CAM.ino
  Face detection service endpoint, deferred capture queue, upload-to-proxy logic.

## 5) Build and Toolchain Requirements

### 5.1) Arduino and core

- Arduino IDE 2.x recommended.
- ESP32 Arduino core 2.0.x baseline across production boards.
- Eye sketch enforces 2.0.x and explicitly references 2.0.17 compatibility.

### 5.2) Board-level dependencies from includes

Controller:
- WiFi, HTTPClient, WiFiUdp, Wire
- Keypad
- LiquidCrystal_I2C

Proxy:
- TinyGsmClient (A7672X profile)
- Preferences
- WiFi, WiFiUdp

Eye:
- esp_camera, img_converters
- HTTPClient, WiFi, WiFiUdp
- human_face_detect_msr01.hpp

### 5.3) Hardware assumptions

- Controller: ESP32 DevKit + keypad + I2C LCD + lock driver + reed sensor.
- Proxy: LilyGO T-SIM A7670E with valid SIM/data and LTE/GPS antennas.
- Eye: AI-Thinker ESP32-CAM with PSRAM and stable camera module.

## 6) Arduino IDE Board Settings

### 6.1) Controller (1_Controller_ESP32)

- Board: ESP32 Dev Module
- PSRAM: Not required
- Partition: Any stable option with enough app space

### 6.2) Proxy (2_Proxy_LilyGO)

- Board: ESP32 Dev Module (LilyGO-compatible target workflow)
- PSRAM: Not required
- Partition: Default is typically sufficient

### 6.3) Eye (3_Eye_ESP32CAM)

- Board: AI Thinker ESP32-CAM
- ESP32 Core: 2.0.x
- PSRAM: Enabled
- Partition Scheme: Huge APP (3MB No OTA / 1MB SPIFFS)

## 7) Flashing Order and Bring-Up

Recommended flashing order:
1. 2_Proxy_LilyGO
2. 3_Eye_ESP32CAM
3. 1_Controller_ESP32

Why this order:
- Proxy must be online first because it provides AP and routing endpoints.
- Eye depends on Proxy AP and upload target.
- Controller depends on /otp and /face-check provided by Proxy.

Bring-up checklist:
1. Boot Proxy and verify AP SmartTopBox_AP_* is visible.
2. Verify Proxy serial logs show modem init and network progress.
3. Boot Eye and verify AP join and face service readiness.
4. Boot Controller and verify recurring context fetch.
5. Run controlled OTP+face unlock cycle.
6. Verify event and command-ack traffic reaches Proxy.
7. Verify image upload relay path through Proxy.

## 8) Runtime Contract and Data Flow

### 8.1) Controller loop model

- Non-blocking loop for WiFi maintenance, state transitions, and safety ticks.
- Periodic context refresh from Proxy.
- Utility diagnostics mode wakes LCD on keys 1/2/3 and auto-closes after 5 seconds.
- Utility screens render immediately from cached diagnostics (no blocking keypress path).
- Diagnostics polling is dynamic by power state:
  - Standby (backlight off): slow polling to save battery.
  - Idle (active delivery): medium polling.
  - Utility active window: burst polling for live updates.
- OTP + face verification gates unlock attempt.
- LockSafety keeps thermal/reed/tamper protections active continuously.
- Idle policy keeps command listening active while forcing LCD backlight off and requesting camera sleep.

### 8.2) Proxy loop model

- Maintains AP and local HTTP serving.
- Manages LTE modem health and backend writes.
- Keeps context cache refreshed for low-latency /otp responses.
- Keeps battery/network/backend diagnostics cached for low-latency /diag responses.
- Routes face-check and upload traffic between Controller/Eye and backend.
- Applies idle-safe polling throttles while keeping LTE command-serving path online.

### 8.3) Eye loop model

- Maintains AP connection.
- Serves face-check status endpoint.
- Decouples detection response from heavier image upload work.
- Supports software camera power states via proxy-triggered sleep/wake commands.

## 9) Production Endpoint Interface

### 9.1) GET /otp (Controller -> Proxy)

Purpose:
- Retrieve OTP and delivery context for local keypad validation.

Typical response shape:
- otp_or_NO_OTP,delivery_id_or_NO_DELIVERY[,status]

Status tokens observed in runtime logic:
- UNLOCKING
- LOCKED
- GEO_PICKUP
- GEO_TRANSIT_DROP
- GEO_TRANSIT_PICK
- RETURNING

### 9.2) GET /face-check (Controller -> Proxy)

Purpose:
- Trigger face verification through Proxy forwarding to Eye.

Typical response values:
- FACE_OK
- NO_FACE
- error/timeout text if camera route fails

### 9.3) GET /diag (Controller -> Proxy)

Purpose:
- Retrieve fast cached diagnostics for utility mode without triggering expensive modem/GPS operations in request path.

Typical response shape (comma-separated key/value):
- batt_pct=<0..100>
- batt_v=<smoothed_voltage>
- rssi=<dBm_or_-999>
- csq=<0..31_or_-1>
- gps_fix=<0|1>
- lte=<0|1>
- modem=<0|1>
- time=<0|1>
- fb_fail=<count>
- uptime=<proxy_millis>

### 9.4) POST /event (Controller -> Proxy)

Purpose:
- Report unlock outcome, safety flags, failure reasons, and alert classes.

Representative payload fields:
- otp_valid
- face_detected
- unlocked
- box_id
- delivery_id
- thermal_cutoff
- face_attempts
- face_retry_exhausted
- fallback_required
- failure_reason
- alert_type
- details
- tamper

### 9.5) POST /command-ack (Controller -> Proxy)

Purpose:
- Acknowledge remote lock/unlock command handling status.

Representative fields:
- command
- status
- details
- box_id
- delivery_id

### 9.6) POST /upload (Eye -> Proxy)

Purpose:
- Send JPEG image bytes from Eye to Proxy for LTE relay.

Expected behavior:
- Content-Type image/jpeg
- Header X-Object-Path used as destination key/path
- Proxy validates payload and performs backend relay/writeback

### 9.7) GET /cam-power?mode=sleep|wake (Controller -> Proxy -> Eye)

Purpose:
- Apply software camera power policy from Controller state transitions.

Typical response values:
- CAM_SLEEP_OK
- CAM_WAKE_OK
- ERROR:cam_unreachable / ERROR:cam_timeout

## 10) Safety and Edge-Case Feature Coverage

The production code explicitly covers the following IDs:

- EC-04: OTP lockout after failed attempts with timed lockout window.
- EC-21: Unlock retry policy when lock appears stuck closed.
- EC-22: Relock verification and stuck-open detection.
- EC-77: Admin remote unlock override with timeout controls.
- EC-82: Keypad stuck-key detection.
- EC-86: LCD health checks and degraded operation fallback.
- EC-92: Geofence behavior under degraded GPS quality (urban canyon).
- EC-93: Warehouse return timing logic.
- EC-94: Geofence hysteresis with confirmation samples.
- EC-95: Reed switch debounce reliability.
- EC-96: Thermal model and hard solenoid cutoff protection.

Utility diagnostics behavior:
- Keys 1/2/3 in standby and idle states temporarily wake LCD to show diagnostics.
- Display auto-times out to low-power behavior after 5 seconds.
- Proxy battery values are smoothed with EMA before display/export to reduce LTE sag spikes.

## 11) Operations Checklist (Per Deployment)

1. Confirm all 3 boards are flashed from production folders only.
2. Verify Proxy AP naming and identity are correct.
3. Verify Eye face service is reachable.
4. Verify Controller receives context from /otp.
5. Validate OTP + face + unlock + relock end-to-end.
6. Validate /event payload writes are visible in backend traces.
7. Validate remote command path and /command-ack lifecycle.
8. Validate /upload relay and resulting photo URL/data updates.
9. Capture serial logs from all boards for run evidence.

## 12) Troubleshooting Matrix

| Symptom | Likely side | Most likely cause | First checks |
|---|---|---|---|
| Controller keeps reconnecting | Controller/Proxy | Proxy AP unavailable or unstable | AP broadcast, credentials, distance/interference |
| No OTP context | Proxy/backend | Cache not populated or backend read issue | Proxy context refresh logs, delivery node values |
| OTP accepted but no unlock | Controller/Eye | Face check failed/timeouts | Face endpoint response, Eye connectivity, retries |
| Remote command not reflected | Proxy/Controller | Command token/ack mismatch | Cached status token, /command-ack request logs |
| Frequent tamper alerts | Controller | Reed noise or wiring instability | Reed debounce config, wiring, magnetic alignment |
| Lock cuts off unexpectedly | Controller | Thermal/timeout safety path triggered | LOCK_MAX_ACTIVE_MS, thermal logs, repeated attempts |
| Camera upload fails | Eye/Proxy/LTE | Relay rejection or LTE degradation | /upload diagnostics, payload headers, modem status |
| Missing backend events | Proxy | LTE transient/write failure | LTE attach/APN health, retry counters, HTTP status |

## 13) Release Checklist

1. Verify release includes only 1_Controller_ESP32, 2_Proxy_LilyGO, 3_Eye_ESP32CAM.
2. Freeze toolchain versions used for build.
3. Build and flash all 3 boards with documented settings.
4. Run full integration test for all local endpoints.
5. Run targeted edge-case validation for EC-04/21/22/77/82/86/92/93/94/95/96.
6. Record firmware revision identifiers for each board.
7. Archive test logs and release notes.
8. Mark deployment date, operator, and environment.

## 14) Important Boundary Reminder

For production operations, always treat the three folders below as the single source of truth:

- 1_Controller_ESP32
- 2_Proxy_LilyGO
- 3_Eye_ESP32CAM

All other Ard IDE sketches are out of scope for production unless explicitly promoted in a future controlled release.
