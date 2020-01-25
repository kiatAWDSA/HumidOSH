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

#ifndef _HUMIDOSH_h
#define _HUMIDOSH_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

// The RH sensor actually measures temperature too.
// By default, this functionality is not utilized because the equipment is expected to be operated at room temperature.
// Uncommenting MEASURE_TEMPERATURE will enable temperature measurement. The temperature reading
// will be displayed on the second line of the screen displaying the readings.
//#define DISPLAY_TEMPERATURE 1

#include "SerialCommunication.h"
#include <EMC2301.h>
#include <serLCD_cI2C.h>
#include <SPI.h>
#include <SHT3x.h>
#include <PID_modified.h>
#include <Keypad.h>
#include <Key.h>
#include <I2C.h>


/*
const uint8_t PIN_VALVE_DRY     = 8;
const uint8_t PIN_VALVE_WET     = 9;
const uint8_t PIN_FAN_PWMDRAIN  = 6;
uint8_t PIN_KEY_ROW[KEY_ROWS] = { 5, 4, 3, 2 }; // connect to the row pinouts of the keypad
uint8_t PIN_KEY_COL[KEY_COLS] = { 8, 7, 6, 1 }; // connect to the column pinouts of the keypad
const uint8_t PIN_LED_RH        = 2;
const uint8_t PIN_LED_FAN       = 2;
*/


class HumidOSH
{
public:
  HumidOSH( SerialCommunication* communicator, I2C* i2cWire, Keypad* keypad, // class ref
            uint8_t pinPump, uint8_t pinValveDry, uint8_t pinValveWet, uint8_t pinFanPWMDrain, uint8_t pinLEDRH, uint8_t pinLEDFan, // pins
            double humidityMin, double humidityMax, uint8_t pumpDutyCycleMin, uint8_t pumpDutyCycleMax, double fanSpeedMin, double fanSpeedMax, double fanSpeedAbsMin, double fanMinDrive,  // Limits for the controls
            double humidityKp, double humidityKi, double humidityKd, // PID params
            uint16_t keyHoldDuration
          );
  ~HumidOSH();
  void init();
  void run();
  void handleKeyPress(KeypadEvent key);
  void startSendData();
  void stopSendData();

private:
  SerialCommunication* communicator_;
  I2C* i2cWire_;
  Keypad* keypad_;
  SHT3x humiditySensor_;
  EMC2301 fan_;
  SerLCD screen_;
  PID humidityPID_;

  // Execute functions with retries
  const uint8_t RETRIES_MAX = 10; // Maximum number of times to retry a command if it fails.
  bool retryFunc(bool (HumidOSH::*func)());
  bool retryFunc(bool (HumidOSH::*func)(bool), bool param);
  bool retryFunc(bool (HumidOSH::*func)(char), char param);
  bool retryFunc(bool (HumidOSH::*func)(char[]), char param[]);
  bool retryFunc(bool (HumidOSH::*func)(uint8_t), uint8_t param);
  bool retryFunc(bool (HumidOSH::*func)(uint16_t), uint16_t param);
  bool retryFunc(bool (HumidOSH::*func)(double), double param);
  bool retryFunc(bool (HumidOSH::*func)(double, uint8_t), double paramDouble, uint8_t paramByte);

  // Screen
  typedef enum
  {
    SCREEN_PAGE_READINGS    = 1,
    SCREEN_PAGE_HUMIDITYADJ = 2,
    SCREEN_PAGE_FANSPEEDADJ = 3,
    SCREEN_PAGE_CAL         = 4,
    SCREEN_PAGE_CAL_POINT   = 5,
    SCREEN_PAGE_CAL_RESET   = 6,
    SCREEN_PAGE_HOLD        = 7,
    SCREEN_PAGE_MINVAL      = 8,
    SCREEN_PAGE_MAXVAL      = 9
  } SCREEN_PAGE;
  const char CHAR_DECIMAL = '.';
  const char CHAR_EMPTY   = ' ';
  const char CHAR_RUN     = '>';
  const char CHAR_NULL    = '\0';
  const uint16_t keyHoldDuration_;
  const unsigned long SCREEN_BACKGROUND_DEFAULT = 0x00FFFFFF; // Bright white
  const unsigned long SCREEN_BACKGROUND_IDLE    = 0x00000000; // Turn off when not in use
  const unsigned long SCREEN_ACTIVE_DURATION    = 10000;      // The backlight is only left one for certain time, then is turned off until user presses a button
  SCREEN_PAGE screenPage_;
  bool backlightOn_;
  bool screenPageChanged_;
  bool holdHumidityButton_;
  bool holdFanSpeedButton_;
  bool humidityControlRecentlyStopped_;
  bool fanSpeedControlRecentlyStopped_;
  bool errorInputHumidity_;
  unsigned long holdTimeStart_;
  unsigned long screenActiveTimerStart_;
  uint8_t keyHoldSecondsRemaining_;
  void changeScreenPage(SCREEN_PAGE newScreenPage);
  void updateScreen();
  bool resetScreen();
  bool clearValueRightAligned(const uint8_t & rightmostColNumber, const uint8_t & rowNumber, const uint8_t & charCount);
  bool resetScreenInput(uint8_t charMax, uint8_t charOffset);
  bool idleScreenInput();
  bool printToDisplay(char charToPrint);
  bool printToDisplay(const char stringToPrint[]);
  bool printToDisplay(double doubleToPrint, uint8_t decimalsToPrint);

  // Readings screen
  bool controlActiveIndicatorLeft_;
  unsigned long controlIndicatorTimerStart_;
  const uint8_t MAXCHAR_READINGS            = 7;
  const uint16_t PERIOD_SCREEN_CONTROLINDICATOR = 500; // Period (ms) between each update of the "running" symbol to indicate control is active.
  const char CONTROLINDICATOR_RUN_LEFT[5]   = { CHAR_RUN, CHAR_RUN, CHAR_EMPTY, CHAR_EMPTY, CHAR_NULL };
  const char CONTROLINDICATOR_RUN_RIGHT[5]  = { CHAR_EMPTY, CHAR_EMPTY, CHAR_RUN, CHAR_RUN, CHAR_NULL };
  const char CONTROLINDICATOR_IDLE[5]       = { 'I', 'D', 'L', 'E', CHAR_NULL };
  const char PRINT_ERROR[6]                 = { 'E', 'R', 'R', 'O', 'R', CHAR_NULL };
  const char PRINT_NOREADING[4]             = { 'N', '/', 'A', CHAR_NULL };
  const uint8_t COL_READING_RIGHTMOST       = 15;
  const uint8_t ROW_READING_HUMIDITY        = 2;
  const uint8_t ROW_READING_FANSPEED        = 3;
  void printValueRightAligned(const double & value, const uint8_t & decimalsMax, const uint8_t & rightmostColNumber, const uint8_t & rowNumber);
  void printValueRightAligned(const char value[], const uint8_t & rightmostColNumber, const uint8_t & rowNumber);
  void printControlIndicators(bool printingLeft);
  void printReadingRightAligned(float reading, uint8_t maxDecimals, uint8_t readingCharMaxCount, uint8_t rightmostCol, uint8_t row);
  void printTextRightAligned(const char text[], uint8_t textCharMaxCount, uint8_t rightmostCol, uint8_t row);
  void printNoReading();

  // Hold screen
  void printSecondsRemaining();

  // Calibration-related screens
  const uint8_t MAXCHAR_RHRAW = 5;
  const uint16_t PERIOD_SCREEN_CALRESET = 2000; // Duration (ms) for the reset calibration screen to be shown before the screen is reverted back to calibration menu.
  bool calibratingPoint1;
  unsigned long calResetSplashTimerStart_; // Keeps track of when the splash screen for confirming calibration reset was shown.

  // Min/max error screen
  // const unsigned long SCREEN_BACKGROUND_ERROR = 0x00FF6161; // Light red
  const char CHAR_FLASHER_LEFT  = '>';
  const char CHAR_FLASHER_RIGHT = '<';
  const char ERROR_FLASHER_LEFT[6]  = { CHAR_FLASHER_LEFT, CHAR_FLASHER_LEFT, CHAR_FLASHER_LEFT, CHAR_FLASHER_LEFT, CHAR_FLASHER_LEFT, CHAR_NULL };
  const char ERROR_FLASHER_RIGHT[6] = { CHAR_FLASHER_RIGHT, CHAR_FLASHER_RIGHT, CHAR_FLASHER_RIGHT, CHAR_FLASHER_RIGHT, CHAR_FLASHER_RIGHT, CHAR_NULL };
  const char ERROR_FLASHER_CLEAR[6] = { CHAR_EMPTY, CHAR_EMPTY, CHAR_EMPTY, CHAR_EMPTY, CHAR_EMPTY, CHAR_NULL };
  const uint16_t PERIOD_ERROR_INPUT_FLASH = 700; // Duration between each flash
  const uint8_t ERROR_INPUT_FLASH_COUNT = 6;       // Number of times to flash. This, multiplied with errorInputTimerFlashDuration_, gives the total duration for the error screen.
  bool errorInputFlashOn_;
  uint8_t errorInputTimerFlashCounter_;
  unsigned long errorInputTimerStart_;
  void printValueLimit(double limit, uint8_t decimalsMax);


  // Keypad or button presses
  const uint8_t INPUT_HUMIDITY_MAXCHAR  = 4; // Max characters for inputting humidity target (100.0 is 5 chars)
  const uint8_t INPUT_HUMIDITY_DECIMALS = 1; // Number of decimal places allowed for humidity input.
  const uint8_t INPUT_FANSPEED_MAXCHAR  = 4; // Max characters for inputting fan speed target (9800 is 4 chars)
  const uint8_t INPUT_FANSPEED_DECIMALS = 0; // Number of decimal places allowed for fan speed input.
  bool decimalUsed_;
  uint8_t inputCharCount_;
  uint8_t inputIntCount_;
  uint8_t inputDecimalCount_;
  double inputValue_;
  void handleButtonControl(bool humidity, const bool & controlActiveFlag, bool & recentlyStoppedFlag);
  void handleInputNumber(char inputKey, const uint8_t & charMax, const uint8_t & decimalsMax);
  void handleInputDelete(const uint8_t & charMax);
  void handleInputDot(const uint8_t & charMax, const uint8_t & decimalsMax);
  bool saveInput(bool humidity, double & targetBuffer, const double & min, const double & max);
  void resetInputVars();
  
  // Pins
  const uint8_t pinPump_;
  const uint8_t pinValveDry_;
  const uint8_t pinValveWet_;
  const uint8_t pinFanPWMDrain_;
  const uint8_t pinLEDRH_;
  const uint8_t pinLEDFan_;

  // Acquiring measurements
  unsigned long DAQTimerStart_;
  const uint16_t PERIOD_DAQ_HUMIDITY_TRIGGER = SHT3x::DURATION_HIGREP + 300; // Wait time (ms) between triggering a measurement and attempting to grab it. Add more to be conservative.
  const uint16_t PERIOD_DAQ = 1000; // Period (ms) between each data acquisition.

  // Humidity
  bool humidityOK_;
  bool humidityErrorHandlingActive_;
  bool humidityControlActive_;
  bool newHumidityReadingPrint_;
  bool newHumidityReadingControl_;
  bool humidityTriggered_;
  bool humidityTriggeredOK_;
  const double humidityMin_;
  const double humidityMax_;
  const uint8_t pumpDutyCycleMin_;
  const uint8_t pumpDutyCycleMax_;
  double humidity_;
  double humidityTarget_;
  double humidityControlOutput_;
  uint8_t pumpDutyCycle_;
  bool triggerHumidity();
  bool getHumidity();
  void toggleHumidityControl(bool enable);
  void setHumidityTarget(double targetPercent);
  void setPumpDutyCycle(uint8_t dutyCycle);

  // Temperature
  double temperature_;
#ifdef DISPLAY_TEMPERATURE
  // Temperature
  const uint8_t ROW_READING_TEMPERATURE = 1;
  const uint8_t TEMPERATURE_DECIMALS = 1; // Number of decimal places displayed for temperature.
#endif // DISPLAY_TEMPERATURE

  // Fan speed
  bool fanSpeedOK_;
  bool fanSpeedControlActive_;
  bool newFanSpeedReadingPrint_;
  const double fanSpeedMin_;
  const double fanSpeedMax_;
  const double fanSpeedAbsMin_;
  const uint8_t fanMinDrive_;
  double fanSpeed_;
  double fanSpeedTarget_;
  bool getFanSpeed();
  bool updateFanSpeedTarget(double targetRPM);
  bool toggleFanSpeedControl(bool enable);

  // Serial communication
  bool sendData_ = false;

  // On/off functions
  void togglePump(bool enable);
  void toggleValveDry(bool enable);
  void toggleValveWet(bool enable);
  void toggleFan(bool enable);
  void toggleLEDRH(bool enable);
  void toggleLEDFan(bool enable);

  // Utility functions
  uint8_t getIntegerCount(double value);
};


#endif

