/**
 * HardwareIO.h — LCD and Keypad control
 *
 * Encapsulates display and input behind clean function calls.
 * Solenoid control has moved to LockSafety.h/.cpp.
 *
 * LCD update uses space-padded overwrite (no flicker from clear()).
 */

#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include "Config.h"

/** Initialise LCD, keypad debounce, and UART to ESP32-CAM. */
void initHardwareIO();

/** Update LCD without flicker (space-padded overwrite). */
void updateDisplay(const char *line0, const char *line1);

/** Enable LCD backlight if it is currently off. */
void displayBacklightOn();

/** Disable LCD backlight to reduce idle power draw. */
void displayBacklightOff();

/** Read one keypad character (non-blocking). Returns '\0' if nothing pressed. */
char readKeypad();

/** Access the global Keypad instance (for getState() etc.). */
Keypad& getKeypad();

#endif // HARDWARE_IO_H
