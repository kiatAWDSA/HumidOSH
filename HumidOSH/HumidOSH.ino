/*********************************************************************************
An open source system with controls for relative humidity and fan speed for
adjusting samples to a desired water activity.

I2C is used as the main comm standard, and there are three devices that use it:
- Microchip EMC2301 (Fan speed tachometer and controller)
- Sensirion SHT35-DIS-F (Relative humidity sensor)
- SparkFun LCD-14074 (LCD screen)

The EMC2301 uses SMBus, while the other two uses I2C. SMBus is, for the most part,
compatible with I2C comm. However, the clock speed must be 100 kHz, and clock stretching
is NOT allowed (https://www.maximintegrated.com/en/app-notes/index.mvp/id/476). Therefore,
the SHT35-DIS-F should not be placed in the clock stretching mode. However, p12 of the
datasheet of EMC2301 states that this device actually supports clock stretching
(https://ww1.microchip.com/downloads/en/DeviceDoc/2301.pdf). The user can test this out
if desired.

The value of pull-up resistors on the SCL and SDA lines should also be considered.
P33 of the NXP I2C manual (https://www.nxp.com/docs/en/user-guide/UM10204.pdf) states
that the max current for SMBus high power and I2C is 4 mA and 3 mA, respectively. The SMBus 2.0
specs states that in "high-power" mode, a max of 4 mA can be sunk (http://smbus.org/specs/smbus20.pdf).
Since the EMC2301 sinks 4 mA (p10 of datasheet) and is SMBus 2.0, it is thus SMBus 2.0 high power.
The SHT35 uses I2C at 3.3 V; similarly, the LCD screen uses I2C at 3.3 V.
The pull-up resistors should thus be sized accordingly to these values, preferably close
to the max current to produce sharper signal edges while being below the max current
for both standards (I2C being the lowest). Thus, for the 5 V side (EMC2301 and Arduino), 2.2 kOhm
pull-up resistor is recommended, while for the 3.3 V side (SHT35 and LCD), 1.5 kOhm is recommended.
This config sinks ~2.2 mA to both sides.

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
#include <EEPROM.h>
#include <EMC2301.h>
#include <serLCD_cI2C.h>
#include <SPI.h>
#include <SHT3x.h>
#include <PID_modified.h>
#include <Keypad.h>
#include <Key.h>
#include <I2C.h>

// The RH sensor actually measures temperature too.
// By default, this functionality is not utilized because the equipment is expected to be operated at room temperature.
// Setting MEASURE_TEMPERATURE to 1 will enable temperature measurement. The temperature reading
// will be displayed on the second line of the screen displaying the readings.
#define MEASURE_TEMPERATURE 0


// Keypad config
// Note that we could have went with a 5x3 config to have exactly the number of keys needed,
// but the 4x4 config uses same pins and gives an extra key that can be used in the future, if needed.
const uint8_t KEY_ROWS = 4; // four rows
const uint8_t KEY_COLS = 4; // four cols
const uint16_t KEY_HOLD_DELAY     = 400; // Duration (ms) for a key to be pressed before it is considered as being held.
const uint16_t KEY_HOLD_DURATION  = 3000; // Duration (ms) that the key must be held to stop humidity or fan speed control.

// Pins
const uint8_t PIN_PUMP          = 9; // This must be pin 9 or 10 because they are affected by Timer 1 which will be configured to 25 kHz for PWM.
const uint8_t PIN_VALVE_DRY     = A2;
const uint8_t PIN_VALVE_WET     = A3;
const uint8_t PIN_FAN_PWMDRAIN  = A1;
uint8_t PIN_KEY_ROW[KEY_ROWS] = { 3, 8, 7, 5 }; // connect to the row pinouts of the keypad
uint8_t PIN_KEY_COL[KEY_COLS] = { 4, 2, 6, 11 }; // connect to the column pinouts of the keypad
const uint8_t PIN_LED_RH        = 12;
const uint8_t PIN_LED_FAN       = A0;

// Limits for controlling humidity and fan speed. This would depend on the system characteristics.
// For example, all fans have a maximum speed, and some fans have a minimum RPM, below which the fan just stops working.
// These limits are important to prevent the user from inadvertently setting a target which is unachievable.
const double RH_MIN         = 0;    // %
const double RH_MAX         = 100;  // %
const uint8_t PUMP_MIN      = 60;   // Duty cycle expressed in byte format: min is 0, max is 255. Convert the duty cycle percentage accordingly.
const uint8_t PUMP_MAX      = 255;  // See above. Note that the pump doesn't operate below a certain duty cycle.
const double FANSPEED_USER_MIN  = 1200;   // The minimum fan speed (RPM) allowed for user input.
const double FANSPEED_USER_MAX  = 7500;   // The maximum fan speed (RPM) allowed for user input.
const double FANSPEED_ABS_MIN   = 500;    // The absolute minimum fan speed (RPM) allowed for the fan. If the fan speed dips below this value,
                                          // the fan will restart itself, so this value should be quite a bit below FANSPEED_USER_MIN.
const uint8_t  FAN_DRIVE_MIN    = 30;  // The minimum drive/duty cycle (0 to 255) for the fan. This is essentially the minimum PWM duty cycle for the fan.
                                          // Setting this value too high prevents the fan from reaching lower speeds.
                                          // Setting it too low may cause the fan to restart due to violation of FANSPEED_ABS_MIN.
                                          // This value should be adjusted such that the fan speed can dip to FANSPEED_USER_MIN without violating FANSPEED_ABS_MIN.

// Humidity PID settings
const double PID_RH_KP = 300;
const double PID_RH_KI = 0.001;
const double PID_RH_KD = 0;

// Init keypad
char keys[KEY_ROWS][KEY_COLS] = { { '1','2','3','s' },    // s for changing control 's'ettings.
                                  { '4','5','6','h' },    // r for starting 'h'umidity control.
                                  { '7','8','9','f' },    // f for starting 'f'an speed control.
                                  { '.','0','d','x' } };  // d is to 'd'elete a character, . is to insert decimal point, x is an unused key, reserved for future purposes.
Keypad keypad = Keypad(makeKeymap(keys), PIN_KEY_ROW, PIN_KEY_COL, KEY_ROWS, KEY_COLS);

// Main class
HumidOSH chamber = HumidOSH(&I2c, &keypad,
                            PIN_PUMP, PIN_VALVE_DRY, PIN_VALVE_WET, PIN_FAN_PWMDRAIN, PIN_LED_RH, PIN_LED_FAN,
                            RH_MIN, RH_MAX, PUMP_MIN, PUMP_MAX, FANSPEED_USER_MIN, FANSPEED_USER_MAX, FANSPEED_ABS_MIN, FAN_DRIVE_MIN,
                            PID_RH_KP, PID_RH_KI, PID_RH_KD,
                            KEY_HOLD_DURATION);

void setup()
{
  // Configure Timer 1 for PWM @ 25 kHz to reduce noise from controlling pump with PWM.
  // This affects both pin 9 and 10
  // From https://arduino.stackexchange.com/a/25623
  // My explanation: Due to 16-bit nature of Timer 1, it counts from 0 to 320.
  // Setting to phase-correct PWM makes it count back down, so a total of 640 ticks.
  // Since prescaler is 1, the final timer frequency is 16 MHz / 640 / 1 = 25 kHz.
  TCCR1A = 0;           // undo the configuration done by...
  TCCR1B = 0;           // ...the Arduino core library
  TCNT1 = 0;            // reset timer
  TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
    | _BV(COM1B1)       // same on ch; B
    | _BV(WGM11);       // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13)   // ditto
    | _BV(CS10);        // prescaler = 1
  ICR1 = 320;           // TOP = 320

  // Ensure that the PCA9615 chips have enough time to go through the power-up routine.
  delay(100);

  keypad.setHoldTime(KEY_HOLD_DELAY);
  keypad.addEventListener(keypadEvent); //add an event listener for this keypad

  chamber.init();

  // TODELETE
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again until power down or reset
void loop()
{
  keypad.getKey();
  chamber.run();
}

void keypadEvent(KeypadEvent key)
{
  chamber.handleKeyPress(key);
}
