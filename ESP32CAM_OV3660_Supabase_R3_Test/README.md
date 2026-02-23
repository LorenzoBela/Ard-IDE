# ESP32-CAM OV3660 -> Supabase R3 Upload Test

Arduino IDE sketch to validate OV3660 camera capture and upload every captured JPEG to a Supabase Storage bucket (`r3`).

## Files
- `ESP32CAM_OV3660_Supabase_R3_Test.ino`

## Board setup (Arduino IDE)
- **Board**: `AI Thinker ESP32-CAM`
- **Partition Scheme**: `Huge APP (3MB No OTA/1MB SPIFFS)` or default 4MB
- **PSRAM**: Enabled (if available)
- **Upload speed**: 115200 (safe)

## Libraries / Core
- Install **ESP32 by Espressif Systems** board package (v2.0.x or newer).
- `esp_camera.h`, `WiFi.h`, and `HTTPClient.h` are included via ESP32 core.

## Configure before upload
Edit values at the top of `ESP32CAM_OV3660_Supabase_R3_Test.ino`:
- `WIFI_SSID`
- `WIFI_PASSWORD`
- `SUPABASE_URL` (example: `https://abcxyz.supabase.co`)
- `SUPABASE_BUCKET` (`r3` by default)
- `SUPABASE_API_KEY`
- `SUPABASE_SERVICE_ROLE_KEY`

## Supabase notes
- Endpoint used: `/storage/v1/object/{bucket}/{path}`
- Upload uses JPEG binary and sets `x-upsert: true`
- Bucket `r3` should exist in Supabase Storage.
- Auth header uses `SUPABASE_SERVICE_ROLE_KEY` when provided, otherwise falls back to `SUPABASE_API_KEY`.
- If using anon key, add Storage policy that allows `insert` on bucket `r3` for your use case.

## Flashing ESP32-CAM
1. Connect `GPIO0` to `GND`.
2. Press reset, upload sketch.
3. Disconnect `GPIO0` from `GND`.
4. Press reset to run.
5. Open Serial Monitor at `115200`.

## Expected output
- `OV3660 detected.`
- `Captured JPEG bytes: ...`
- `Upload success: esp32cam/OV3660_CAM_001/capture_...jpg`

Each successful capture is uploaded as a new file path in bucket `r3`.
