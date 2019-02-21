/*********************************************************************************
An open source system with controls for relative humidity and fan speed for
adjusting samples to a desired water activity.

Copyright (C) 2019 Soon Kiat Lau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*********************************************************************************/

#include "HumidOSH.h"
#include <EMC2301.h>
#include <serLCD_cI2C.h>
#include <SPI.h>
#include <SHT3x.h>
#include <PID_modified.h>
#include <Keypad.h>
#include <Key.h>
#include <I2C.h>

// Pins
const uint8_t PIN_VALVE_DRY     = 8;
const uint8_t PIN_VALVE_WET     = 9;
const uint8_t PIN_FAN_PWMDRAIN  = 6;
const uint8_t PIN_KEY_ROW[KEY_ROWS] = { 5, 4, 3, 2 }; // connect to the row pinouts of the keypad
const uint8_t PIN_KEY_COL[KEY_COLS] = { 8, 7, 6, 1 }; // connect to the column pinouts of the keypad
const uint8_t PIN_LED_RH        = 2;
const uint8_t PIN_LED_FAN       = 2;

// Keypad setup
// Note that we could have went with a 5x3 config to have exactly the number of keys needed,
// but the 4x4 config uses same pins and gives an extra key that can be used in the future, if needed.
const uint8_t KEY_ROWS = 4; // four rows
const uint8_t KEY_COLS = 4; // four cols
char keys[KEY_ROWS][KEY_COLS] = { { '1','2','3','s' },    // s for changing control 's'ettings.
                                  { '4','5','6','r' },    // r for starting 'r'elative humidity control.
                                  { '7','8','9','f' },    // f for starting 'f'an speed control.
                                  { 'b','0','.','x' } };  // b is 'b'ackspace, . is to insert decimal point, x is an unused key, reserved for future purposes.
Keypad keypad = Keypad(makeKeymap(keys), PIN_KEY_ROW, PIN_KEY_COL, KEY_ROWS, KEY_COLS);

void setup()
{


  // Set up I2C
  I2c.setTimeOut(30);   // Timeout after 30 ms.
  I2c.begin(false);     // True to use internal pull-up resistors; false for external pull-ups.
  I2c.setSpeed(false);  // true for 400 kHz (fast-mode); false for 100 kHz (standard mode). Need standard mode to ensure SMBus compatibility on some of the components.

}

// the loop function runs over and over again until power down or reset
void loop()
{
  
}
