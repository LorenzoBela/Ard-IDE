/**
 * Configuration Template
 * Copy these values and paste into GPS_WiFi_Firebase_Test.ino
 */

// ==================== STEP 1: SIM Card APN Settings ====================
// Replace with your carrier's APN settings
#define GPRS_APN "internet"     // Your carrier's APN
#define GPRS_USER ""            // Usually empty
#define GPRS_PASS ""            // Usually empty
#define GSM_PIN ""              // Leave empty if SIM has no PIN

// Common Philippine APNs:
// Globe: "internet.globe.com.ph" or "http.globe.com.ph"
// Smart: "internet" or "smartlte"
// DITO: "internet"
// TM (Touch Mobile): "internet"
//
// Common International APNs:
// T-Mobile US: "fast.t-mobile.com"
// AT&T US: "phone"
// Vodafone: "internet"
// Orange: "internet"


// ==================== STEP 2: Firebase Configuration ====================

// ✅ Already configured - no changes needed!
#define FIREBASE_API_KEY "AIzaSyA7DETBpsdPN6icfWi7PijCbpmLNWEZyTQ"
#define FIREBASE_DATABASE_URL "https://smart-top-box-default-rtdb.asia-southeast1.firebasedatabase.app"


// ==================== STEP 3: Hardware Identity ====================
// Optional: Change the device identifier (useful for multiple boxes)
#define HARDWARE_ID "TEST_BOX_001"

// Example:
// #define HARDWARE_ID "TEST_BOX_LORENZO"


// ==================== HOW TO FIND YOUR APN ====================

/*
✅ FIREBASE IS ALREADY CONFIGURED!

Your Firebase credentials are already set in the sketch:
- API Key: AIzaSyA7DETBpsdPN6icfWi7PijCbpmLNWEZyTQ
- Database: smart-top-box-default-rtdb.asia-southeast1.firebasedatabase.app

No need to change anything unless you want to use a different Firebase project.

If you want to view your data:
1. Go to: https://console.firebase.google.com/
2. Select project: smart-top-box
3. Click "Realtime Database" in the left menu
4. Navigate to: test_devices/TEST_BOX_001/
5. You'll see live updates every 5 seconds!
*/


// ==================== FIREBASE SECURITY RULES
☐ SIM card is inserted correctly (gold contacts facing down)
☐ SIM has active data plan
☐ SIM PIN is disabled OR entered in GSM_PIN above
☐ You are in area with cellular coverage
☐ APN is correct for your carrier
☐ LTE antenna is connected to board

Common Issues:
- "SIM not detected": Reinsert SIM, check orientation
- "Network registration failed": Check coverage, try different location
- "GPRS connection failed": Wrong APN, check with carrier
- "No data": SIM has no active data plan
*/


// ==================== HOW TO GET FIREBASE CREDENTIALS ====================

/*
1. GET API KEY:
   - Go to: https://console.firebase.google.com/
   - Select your project
   - Click the gear icon (⚙️) > Project settings
   - Scroll down to "Your apps" section
   - Under "Web API Key", copy the key
   - Paste it in FIREBASE_API_KEY above

2. GET DATABASE URL:
   - In Firebase Console, click "Realtime Database" in left menu
   - If you don't have a database, click "Create Database"
   - Choose a location and start in test mode
   - Copy the URL from the top of the page (looks like: https://xxxxx.firebaseio.com/)
   - Paste it in FIREBASE_DATABASE_URL above

3. SET FIREBASE RULES (IMPORTANT FOR TESTING):
   - In Realtime Database, go to "Rules" tab
   - Replace with this (FOR TESTING ONLY):
   
   {
     "rules": {
       ".read": true,
       ".write": true
     }
   }
   
   - Click "Publish"
   - ⚠️ WARNING: This allows anyone to read/write your database!
   - For production, use proper security rules!

4. SAVE YOUR CREDENTIALS:
   - After configuring, save this file for reference
   - Don't share your API key publicly!
*/


// ==================== FIREBASE SECURITY RULES (PRODUCTION) ====================

/*
For production use, replace the test rules with these secure rules:

{
  "rules": {
    "test_devices": {
      "$deviceId": {
        ".read": "auth != null",
        ".write": "auth != null && $deviceId == auth.uid"
      }
    }
  }
}

This ensures:
- Only authenticated users can read/write
- Each device can only write to its own data
*/
OR TESTING - Current rules (allows anyone to read/write):
{
  "rules": {
    ".read": true,
    ".write": true
  }
}

⚠️ WARNING: These open rules are OK for testing but NOT for production!

For production use, replace with these secure rules:
{
  "rules": {
    "test_devices": {
      "$deviceId": {
        ".read": "auth != null",
        ".write": "auth != null"
      }
    }
  }
}

This ensures:
- Only authenticated users can read/write
- Each device can only access