# GPS & LTE Signal to Firebase Test

Simple Arduino IDE test sketch for LILYGO T-SIM A7670E that reads GPS and LTE signal strength and sends to Firebase.

## 📋 Features

- ✅ Reads GPS data from A7670E internal modem
- ✅ Connects via LTE/4G (no WiFi needed)
- ✅ Measures cellular signal strength (RSSI & CSQ)
- ✅ Sends data to Firebase Realtime Database
- ✅ Updates every 5 seconds
- ✅ Serial monitor output for debugging

## 🔧 Required Hardware

- LILYGO T-SIM A7670E (GPS Version)
- Nano SIM card with data plan (4G/LTE enabled)
- USB-C cable for programming
- GPS antenna (usually included with board)
- LTE antenna (usually included with board)

## 📚 Required Libraries

Install these libraries via Arduino IDE Library Manager (`Tools > Manage Libraries...`):

1. **TinyGSM** by Volodymyr Shymanskyy
   - Search: "TinyGSM"
   - Version: 0.11.7 or higher

2. **TinyGPSPlus** by Mikal Hart
   - Search: "TinyGPSPlus"
   - Version: 1.0.3 or higher

3. **Firebase ESP Client** by Mobizt
   - Search: "Firebase ESP Client"
   - Version: 4.3.0 or higher

## ⚙️ Setup Instructions
Insert SIM Card

Insert your nano SIM card with data plan into the SIM slot on the board.

### 2. Configure APN Settings

Open `GPS_WiFi_Firebase_Test.ino` and update your carrier's APN:

```cpp
#define GPRS_APN "internet"     // Common APNs: "internet", "web.globe.com.ph"
#define GPRS_USER ""            // Usually empty
#define GPRS_PASS ""            // Usually empty
```

**Common Philippine APNs:**
- Globe: `internet.globe.com.ph` or `http.globe.com.ph`
- Smart: `internet` or `smartlte`
- DITO: `internet`

### 3. Firebase Already Configured ✅

Firebase credentials are already set:
- API Key: Already configured
- Database URL: Already configured
### 3. Set Hardware ID

Optionally change the device identifier:

```cpp
#define HARDWARE_ID "TEST_BOX_001"
```

### 4. Configure Arduino IDE

### 4. Set Hardware ID

Optionally change the device identifier:

```cpp
#define HARDWARE_ID "TEST_BOX_001"
```

### 5artition Scheme**: Default 4MB with spiffs
- **Core Debug Level**: None
- **Port**: Select your COM port

### 5. Upload

1. Connect LILYGO T-SIM A7670E via USB-C
2. Click **Upload** button
3. Wait for upload to complete

### 6. Upload

1. Connect LILYGO T-SIM A7670E via USB-C
2. Click **Upload** button
3. WaLTE network connection
   - GPS status
### 7GPS status
   - WiFi connection
   - Firebase uploads

## 📊 Firebase Data Structure

Data is stored at: `/test_devices/{HARDWARE_ID}/`

```json
{
  "testlte_rssi": -71,
      "lte_csq": 21,
      "lte_quality": "Good",
      "network_operator": "Globe Telecom",
      "gps_latitude": 14.5995,
      "gps_longitude": 120.9842,
      "gps_status": "Fixed",
      "last_update": 1707494400000
    }
  }
}
```

### Fields:
- `lte_rssi`: LTE signal strength in dBm (-999 = no signal)
- `lte_csq`: Signal quality indicator (0-31, higher is better)
- `lte_quality`: Human-readable quality (Excellent/Good/Fair/Weak/Very Weak)
- `network_operator`: Carrier name (e.g., "Globe Telecom", "Smart Communications")
- `gps_latitude`: GPS latitude (only when GPS has fix)
- `gps_longitude`: GPS longitude (only when GPS has fix)
- `gps_status`: "Fixed" or "No Fix"
- `last_update`: Firebase server timestamp

## 📡 Signal Quality Reference

### LTE Signal (RSSI in dBm):
- **-65 or better**: Excellent signal
- **-65 to -75**: Good signal
- **-75 to -85**: Fair signal
- **-85 to -95**: Weak signal
- **Below -95**: Very weak signal

### SIM Card Not Detected:
- Ensure SIM card is properly inserted
- Check if SIM has a PIN - disable it or add to `GSM_PIN`
- Try reinserting the SIM card

### LTE Connection Failed:
- Verify APN settings for your carrier
- Check if SIM has data plan enabled
- Ensure good cellular coverage
- Try moving to area with better signal

### GPS Not Getting Fix:
- Ensure GPS antenna is connected
- Place device near window or outdoors
- Wait 1-2 minutes for initial fix (cold start)
- Check Serial Monitor for GPS messages

### Firebase Upload Failed:
- Verify LTE connection is active
- Check Firebase credentials
- Place device near window or outdoors
- Wait 1-2 minutes for initial fix (cold start)
- Check Serial Monitor for GPS messages

### WiFi Connection Failed:
- Double-check SSID and password
- Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- Move closer to router

### Firebase Upload Failed:
- Verify API key and database URL
- Check Firebase Realtime Database rules:
  ```json
  {LTE Signal to Firebase Test
LILYGO T-SIM A7670E
========================================

Powering on A7670E modem...
Modem power sequence complete
Testing modem communication...
Modem Info: SIMCOM_A7670E
Initializing GPS...
GPS initialized successfully
Connecting to cellular network...
Waiting for network... registered!
SIM card OK
Connecting to APN: internet
LTE Connected!
SIM CCID: 8963...
Operator: Globe Telecom
Local IP: 10.123.45.67
Initializing Firebase...
Firebase initialized with LTE

========================================
Setup complete - starting main loop
========================================

--- Reading GPS ---
GPS: No fix yet

--- Sending to Firebase ---
LTE Signal: -71 dBm (CSQ: 21, Good)
Sending data to Firebase...
✓ LTE RSSI sent
✓ LTE CSQ sent
✓ LTE quality sent
✓ Network operator sent
✓ Timestamp sent
✓ GPS status sent (No Fix)
Data upload complete!

--- Reading GPS ---
GPS Fix - Lat: 14.5995, Lon: 120.9842

--- Sending to Firebase ---
LTE Signal: -69 dBm (CSQ: 22, Good)
Sending data to Firebase...
✓ LTE RSSI sent
✓ LTE CSQ sent
✓ LTE quality sent
✓ Network operatort

--- Sending to Firebase ---
WiFi Signal: -45 dBm (Excellent)
Sending data to Firebase...
- **Signal Check**: Every 10 seconds

You can adjust in the code:
```cpp
#define GPS_UPDATE_INTERVAL 2000      // milliseconds
#define FIREBASE_UPDATE_INTERVAL 5000 // milliseconds
#define SIGNAL_CHECK_INTERVAL 10000   // milliseconds
```

## 📞 Support

For issues or questions about this test sketch, check:
- Serial Monitor output for error messages
- Firebase Console for received data
- SIM card has active data plan
- Cellular coverage in your area
- LILYGO documentation for hardware issues

## ⚡ Next Steps

Once this test works:
1. ✅ GPS and LTE are working
2. ✅ Firebase connection is established
3. 🚀 Ready to integrate into full Smart Top Box firmware

---

**Hardware**: LILYGO T-SIM A7670E (GPS Version)  
**Connectivity**: LTE/4G (No WiFi needed)  
**Project**: Parcel-Safe Smart Top Box  
**Test Version**: 2.0 (LTE)
You can adjust in the code:
```cpp
#define GPS_UPDATE_INTERVAL 2000      // milliseconds
#define FIREBASE_UPDATE_INTERVAL 5000 // milliseconds
```

## 📞 Support

For issues or questions about this test sketch, check:
- Serial Monitor output for error messages
- Firebase Console for received data
- LILYGO documentation for hardware issues

## ⚡ Next Steps

Once this test works:
1. ✅ GPS and WiFi are working
2. ✅ Firebase connection is established
3. 🚀 Ready to integrate into full Smart Top Box firmware

---

**Hardware**: LILYGO T-SIM A7670E (GPS Version)  
**Project**: Parcel-Safe Smart Top Box  
**Test Version**: 1.0
