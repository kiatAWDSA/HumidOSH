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

HumidOSH::HumidOSH( I2C* i2cWire, Keypad* keypad, // class ref
                    uint8_t pinPump, uint8_t pinValveDry, uint8_t pinValveWet, uint8_t pinFanPWMDrain, uint8_t pinLEDRH, uint8_t pinLEDFan, // pins
                    double humidityMin, double humidityMax, uint8_t pumpDutyCycleMin, uint8_t pumpDutyCycleMax, double fanSpeedMin, double fanSpeedMax, double fanSpeedAbsMin, double fanMinDrive,  // Limits for the controls
                    double humidityKp, double humidityKi, double humidityKd,  // PID params
                    uint16_t keyHoldDuration
                  )
  :
  i2cWire_(i2cWire),
  keypad_(keypad),
  pinPump_(pinPump),
  pinValveDry_(pinValveDry),
  pinValveWet_(pinValveWet),
  pinFanPWMDrain_(pinFanPWMDrain),
  pinLEDRH_(pinLEDRH),
  pinLEDFan_(pinLEDFan),
  humidityMin_(humidityMin),
  humidityMax_(humidityMax),
  pumpDutyCycleMin_(pumpDutyCycleMin),
  pumpDutyCycleMax_(pumpDutyCycleMax),
  fanSpeedMin_(fanSpeedMin),
  fanSpeedMax_(fanSpeedMax),
  fanSpeedAbsMin_(fanSpeedAbsMin),
  fanMinDrive_(fanMinDrive),
  keyHoldDuration_(keyHoldDuration),
  humiditySensor_(i2cWire_),
  fan_(i2cWire_),
  humidityPID_(&humidity_, &humidityControlOutput_, &humidityTarget_, humidityKp, humidityKi, humidityKd, millis(), P_ON_M, DIRECT)
{
  // Set up pins
  digitalWrite(pinPump_, LOW);
  digitalWrite(pinValveDry_, LOW);
  digitalWrite(pinValveWet_, LOW);
  digitalWrite(pinFanPWMDrain_, HIGH);
  digitalWrite(pinLEDRH_, LOW);
  digitalWrite(pinLEDFan_, LOW);

  // Enable them
  pinMode(pinPump_, OUTPUT);
  pinMode(pinValveDry_, OUTPUT);
  pinMode(pinValveWet_, OUTPUT);
  pinMode(pinFanPWMDrain_, OUTPUT);
  pinMode(pinLEDRH_, OUTPUT);
  pinMode(pinLEDFan_, OUTPUT);

  // Set default state for some flags
  humidityOK_                     = false;
  humidityControlActive_          = false;
  newHumidityReadingPrint_        = false;
  newHumidityReadingControl_      = false;
  humidityErrorHandlingActive_    = false;
  humidityTriggered_              = false;
  humidityTriggeredOK_            = false;
  holdHumidityButton_             = false;
  humidityControlRecentlyStopped_ = false;
  fanSpeedOK_                     = false;
  fanSpeedControlActive_          = false;
  newFanSpeedReadingPrint_        = false;
  holdFanSpeedButton_             = false;
  fanSpeedControlRecentlyStopped_ = false;
}

HumidOSH::~HumidOSH()
{
}

void HumidOSH::init()
{
  // Set up I2C
  I2c.setTimeOut(100);   // Timeout after 20 ms.
  I2c.begin(false);     // True to use internal pull-up resistors; false for external pull-ups.
  I2c.setSpeed(false);  // true for 400 kHz (fast-mode); false for 100 kHz (standard mode). Need standard mode to ensure SMBus compatibility on EMC2301.

  // Init the LCD screen
  screen_.begin(*i2cWire_);
  screen_.setBacklight(SCREEN_BACKGROUND_DEFAULT); //Set backlight to bright white
  screen_.noDisplay();  // Hide the confirmation message from setting contrast
  screen_.setContrast(0); //Set contrast. Lower to 0 for higher contrast.
  screen_.clear(); // clear screen
  screen_.display();  // Turn back on display
  delay(2000);  // Give time for display to turn back on
  // Write splash screen.
  screen_.setCursor(0, 0);
  printToDisplay("********************");
  screen_.setCursor(0, 1);
  printToDisplay("----> HumidOSH <----");
  screen_.setCursor(0, 2);
  printToDisplay(" Soon Kiat Lau 2019 ");
  screen_.setCursor(0, 3);
  printToDisplay("********************");
  delay(2000);

  // Set up the fan
  fan_.toggleControlAlgorithm(true);
  fan_.setFanSpeedMin(fanSpeedAbsMin_);
  fan_.setSpinUpDrive(30);
  fan_.setFanSpeedSpinupMin(fanSpeedAbsMin_);
  fan_.setFanMinDrive(fanMinDrive_);

  // Init PID settings
  humidityPID_.SetOutputLimits(-255, 255);  // Range matches the limits of analogWrite().
  humidityPID_.SetMode(AUTOMATIC);

  changeScreenPage(SCREEN_PAGE_READINGS);
  backlightOn_ = true;
  screenActiveTimerStart_ = millis();

  // Set some default numbers
  humidityTarget_ = humidityMin_ + (humidityMax_ - humidityMin_) / 2;
  fanSpeedTarget_ = fanSpeedMax_;

  // Force acquisition of measurements before the display is updated to the readings screen.
  DAQTimerStart_ = millis() - PERIOD_DAQ;
  retryFunc(&HumidOSH::triggerHumidity) ? humidityTriggeredOK_ = true : humidityTriggeredOK_ = false;
  humidityTriggered_ = true;
  delay(PERIOD_DAQ_HUMIDITY_TRIGGER + 10); // Ensure that when run() is called, it will instantly fetch the sensor measurements.

  // Default values until the measurements are made.
  humidity_ = 0;
  fanSpeed_ = 0;
#ifdef MEASURE_TEMPERATURE
  temperature_ = 0;
#endif // MEASURE_TEMPERATURE
}

// The main function that should be called in loop().
void HumidOSH::run()
{
  // Grab/trigger measurements.
  if (millis() - DAQTimerStart_ >= PERIOD_DAQ - PERIOD_DAQ_HUMIDITY_TRIGGER)
  {
    if (humidityTriggered_ && millis() - DAQTimerStart_ >= PERIOD_DAQ)
    {// Time to grab measurements.
      DAQTimerStart_ = millis();

      // Relative humidity
      if (humidityTriggeredOK_ && retryFunc(&HumidOSH::getHumidity))
      {
        humidityOK_ = true;
        newHumidityReadingPrint_ = true;
        newHumidityReadingControl_ = true;
      }
      else
      {
        humidityOK_ = false;
        newHumidityReadingPrint_ = false;
        newHumidityReadingControl_ = false;
      }

      // Fan speed
      if (retryFunc(&HumidOSH::getFanSpeed))
      {
        fanSpeedOK_ = true;
        newFanSpeedReadingPrint_ = true;
      }
      else
      {
        fanSpeedOK_ = false;
        newFanSpeedReadingPrint_ = false;
      }

      humidityTriggered_ = false;
    }
    else if (!humidityTriggered_)
    {// Trigger the SHT3x sensor to perform a measurement.
      retryFunc(&HumidOSH::triggerHumidity) ? humidityTriggeredOK_ = true : humidityTriggeredOK_ = false;
      // Set triggered flag to true even if the above fails after retries, otherwise the program will be retrying again in the next loop.
      humidityTriggered_ = true;
    }
  }

  // Perform controls on relative humidity, if necessary.
  if (humidityControlActive_)
  {
    if (humidityOK_)
    { // No errors
      if (newHumidityReadingControl_)
      { // Got a new reading
        newHumidityReadingControl_ = false;

        if (humidityErrorHandlingActive_)
        { // Just recovered from an error but don't start control just yet; instead restart the control PID
          // and assign the current reading as the "last" value that will be used in the next control loop.
          humidityErrorHandlingActive_ = false;
          humidityPID_.Reset();
          humidityPID_.setLastInput(humidity_);
          humidityPID_.setLastTime(millis());
        }
        else
        { // Everything is fine and dandy; proceed to perform control on RH.
          humidityPID_.Compute(millis());

          if (humidityControlOutput_ >= pumpDutyCycleMin_)
          { // Humidifying
            toggleValveWet(true);
            toggleValveDry(false);

            // If duty cycle is at least the upper limit, then run pump at max duty cycle
            if (humidityControlOutput_ >= pumpDutyCycleMax_)
            {
              setPumpDutyCycle(255);
            }
            else
            {
              setPumpDutyCycle(humidityControlOutput_);
            }
          }
          else if (humidityControlOutput_  < 0 && -humidityControlOutput_ >= pumpDutyCycleMin_)
          { // Drying
            toggleValveDry(true);
            toggleValveWet(false);

            // If duty cycle is at least the upper limit, then run pump at max duty cycle
            if (-humidityControlOutput_ >= pumpDutyCycleMax_)
            {
              setPumpDutyCycle(255);
            }
            else
            {
              setPumpDutyCycle(-humidityControlOutput_);
            }
          }
          else
          { // The pump duty cycle is within the minimum range; turn the pump and valves off.
            setPumpDutyCycle(0);
            toggleValveDry(false);
            toggleValveWet(false);
          }
        }
      }
    }
    else if (!humidityErrorHandlingActive_)
    { // Encountered an error while trying to get a measurement, and haven't taken steps to handle it.
      humidityErrorHandlingActive_ = true;

      // Turn off all the actuators for humidity control until the problem is resolved.
      togglePump(false);
      toggleValveDry(false);
      toggleValveWet(false);
    }
  }

  /* TODO
  // This code block darkens the screen when it is idle after a while.
  // Commented out because some Sparkfun LCD screens use an older firmware
  // that does not have setFastBacklight functionality, thus messing up
  // the screen.
  // Turn off the backlight if no buttons were pressed in a while
  if (backlightOn_)
  {
    if (millis() - screenActiveTimerStart_ >= SCREEN_ACTIVE_DURATION)
    {
      backlightOn_ = false;
      screen_.setFastBacklight(SCREEN_BACKGROUND_IDLE);
      screen_.setBacklight(SCREEN_BACKGROUND_IDLE);
      changeScreenPage(screenPage_);  // Refresh the screen
    }
  }
  */

  // Update the screen as necessary.
  updateScreen();
}

void HumidOSH::handleKeyPress(KeypadEvent key)
{
  /* TODO
  // This code block lights up the screen indicating returning to active state from idle state.
  // Commented out because some Sparkfun LCD screens use an older firmware
  // that does not have setFastBacklight functionality, thus messing up
  // the screen.
  if (backlightOn_)
  { // Screen is already on; refresh active timer
    screenActiveTimerStart_ = millis();
  }
  else
  { // Screen was off; turn the backlight on
    backlightOn_ = true;
    screen_.setFastBacklight(SCREEN_BACKGROUND_DEFAULT);
    //screen_.setBacklight(SCREEN_BACKGROUND_DEFAULT);
    //changeScreenPage(screenPage_);  // Refresh the screen
    screenActiveTimerStart_ = millis();
    //return; // TODELETE Ignore the user input; this is only done because the time taken for backlight to change messes things up. If Sparkfun fixes the setFastBacklight bug, then maybe we can remove this.
  }
  */

  switch (screenPage_)
  {
  case SCREEN_PAGE_READINGS:
    switch (key)
    {
    case 's':
      // Begin adjusting target for RH
      if (keypad_->getState() == PRESSED)
      {
        resetInputVars();
        changeScreenPage(SCREEN_PAGE_HUMIDITYADJ);
      }
    break;
    case 'h':
      // Starting/stopping humidity control.
      // Only turn on/off control when screen is displaying readings (i.e. not in settings mode)
      handleButtonControl(true, humidityControlActive_, humidityControlRecentlyStopped_);
      break;
    case 'f':
      // Starting/stopping fan control.
      // Only turn on/off control when screen is displaying readings (i.e. not in settings mode)
      handleButtonControl(false, fanSpeedControlActive_, fanSpeedControlRecentlyStopped_);
      break;
    }
    break;
  case SCREEN_PAGE_HUMIDITYADJ:
    if (keypad_->getState() == PRESSED)
    {
      switch (key)
      {
      case 's':
        if (inputCharCount_ > 0)
        { // Only analyze/save the data if there were entered characters, otherwise don't change the setpoint.
          if (saveInput(true, humidityTarget_, humidityMin_, humidityMax_))
          { // User input is valid.
            // Begin adjusting target for fan speed.
            resetInputVars();
            changeScreenPage(SCREEN_PAGE_FANSPEEDADJ);
          }
        }
        else
        { // There is no user input. Assume no changes made to the target.
          resetInputVars();

          // Begin adjusting target for fan speed
          changeScreenPage(SCREEN_PAGE_FANSPEEDADJ);
        }
        break;
      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': case '0':
        // Adding a character.
        handleInputNumber(key, INPUT_HUMIDITY_MAXCHAR, INPUT_HUMIDITY_DECIMALS);
        break;
      case 'd':
        // Deleting a character.
        handleInputDelete(INPUT_HUMIDITY_MAXCHAR);
        break;
      case '.':
        // Adding a decimal place.
        handleInputDot(INPUT_HUMIDITY_MAXCHAR, INPUT_HUMIDITY_DECIMALS);
        break;
      }
    }
    break;
  case SCREEN_PAGE_FANSPEEDADJ:
    if (keypad_->getState() == PRESSED)
    {
      switch (key)
      {
      case 's':
        if (inputCharCount_ > 0)
        { // Only analyze/save the data if there were entered characters, otherwise don't change the setpoint.
          if (saveInput(false, fanSpeedTarget_, fanSpeedMin_, fanSpeedMax_))
          { // User input is valid.
            // Begin adjusting target for fan speed.
            resetInputVars();
            changeScreenPage(SCREEN_PAGE_CAL);
          }
        }
        else
        { // There is no user input. Assume no changes made to the target.
          resetInputVars();

          // Proceed to calibration screen
          changeScreenPage(SCREEN_PAGE_CAL);
        }
        break;
      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': case '0':
        // Adding a character.
        handleInputNumber(key, INPUT_FANSPEED_MAXCHAR, INPUT_FANSPEED_DECIMALS);
        break;
      case 'd':
        // Deleting a character.
        handleInputDelete(INPUT_FANSPEED_MAXCHAR);
        break;
      case '.':
        // Adding a decimal place.
        handleInputDot(INPUT_FANSPEED_MAXCHAR, INPUT_FANSPEED_DECIMALS);
        break;
      }
    }
    break;
  case SCREEN_PAGE_CAL:
    if (keypad_->getState() == PRESSED)
    {
      switch (key)
      {
      case 's':
        // Back to readings screen.
        resetInputVars();
        changeScreenPage(SCREEN_PAGE_READINGS);
        break;
      case '1':
        // Move to screen to calibrate point 1.
        calibratingPoint1 = true;
        changeScreenPage(SCREEN_PAGE_CAL_POINT);
        break;
      case '2':
        // Move to screen to calibrate point 2.
        calibratingPoint1 = false;
        changeScreenPage(SCREEN_PAGE_CAL_POINT);
        break;
      case '3':
        // Move to confirmation screen for resetting calibration data.
        changeScreenPage(SCREEN_PAGE_CAL_RESET);
        break;
      }
    }
    break;
  case SCREEN_PAGE_CAL_POINT:
    if (keypad_->getState() == PRESSED)
    {
      switch (key)
      {
      case 's':
        if (inputCharCount_ > 0)
        { // Only save calibration data if there were entered characters.
          humiditySensor_.saveAndApplyCalibration(calibratingPoint1, inputValue_, humiditySensor_.getRHRaw());
          resetInputVars();
          changeScreenPage(SCREEN_PAGE_CAL);
        }
        else
        { // There is no user input. Assume no changes made to the target.
          resetInputVars();

          // Return to calibration screen.
          changeScreenPage(SCREEN_PAGE_CAL);
        }
        break;
      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': case '0':
        // Adding a character.
        handleInputNumber(key, INPUT_HUMIDITY_MAXCHAR, INPUT_HUMIDITY_DECIMALS);
        break;
      case 'd':
        // Deleting a character.
        handleInputDelete(INPUT_HUMIDITY_MAXCHAR);
        break;
      case '.':
        // Adding a decimal place.
        handleInputDot(INPUT_HUMIDITY_MAXCHAR, INPUT_HUMIDITY_DECIMALS);
        break;
      }
    }
    break;
  case SCREEN_PAGE_CAL_RESET:
    // Only reset calibration data if user confirms by pressing key '5'.
    if (keypad_->getState() == PRESSED && key == '5')
    {
      humiditySensor_.resetCalibration();
      changeScreenPage(SCREEN_PAGE_CAL);
    }
    break;
  }
}

bool HumidOSH::retryFunc(bool(HumidOSH::* func)())
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)()) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool (HumidOSH::*func)(bool), bool param)
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(param)) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool(HumidOSH::* func)(char), char param)
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(param)) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool(HumidOSH::* func)(char[]), char param[])
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(param)) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool (HumidOSH::*func)(uint8_t), uint8_t param)
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(param)) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool (HumidOSH::*func)(uint16_t), uint16_t param)
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(param)) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool (HumidOSH::*func)(double), double param)
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(param)) { return true; }
    tries++;
  }
  return false;
}

bool HumidOSH::retryFunc(bool(HumidOSH::* func)(double, uint8_t), double paramDouble, uint8_t paramByte)
{
  uint8_t tries = 0;

  while (tries < RETRIES_MAX)
  {
    if ((this->*func)(paramDouble, paramByte)) { return true; }
    tries++;
  }
  return false;
}

// Change the page that should be displayed on the screen.
// The actual update of the screen would happen on the next
// call of the function updateScreen().
void HumidOSH::changeScreenPage(SCREEN_PAGE newScreenPage)
{
  screenPageChanged_ = true;
  screenPage_ = newScreenPage;
}

// Update the currently displayed screen page as necessary.
void HumidOSH::updateScreen()
{
  switch (screenPage_)
  {
  case SCREEN_PAGE_READINGS:
    // Screen displaying readings and status of environmental controls.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(6, 0);
      screen_.print("Readings");
    #ifdef MEASURE_TEMPERATURE
      screen_.setCursor(7, ROW_READING_TEMPERATURE);
      screen_.print("T:        C");
    #else
      screen_.setCursor(0, 1);
      screen_.print("--------------------");
    #endif // MEASURE_TEMPERATURE
      screen_.setCursor(6, ROW_READING_HUMIDITY);
      screen_.print("RH:        %");
      screen_.setCursor(5, ROW_READING_FANSPEED);
      screen_.print("Fan:        RPM");

      /* Alternative display
      screen_.print("Relative humidity(%)");
      screen_.print("Set:100.0 Read:100.0");
      screen_.print("Fan speed (RPM)     ");
      screen_.print("Set:10000 Read:10000");*/

      // Print out the sensor readings
      if (humidityOK_)
      {
        printReadingRightAligned(humidity_, INPUT_HUMIDITY_DECIMALS, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_HUMIDITY);

      #ifdef MEASURE_TEMPERATURE
        printReadingRightAligned(temperature_, TEMPERATURE_DECIMALS, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_TEMPERATURE);
      #endif // MEASURE_TEMPERATURE
      }
      else
      {
        printTextRightAligned(PRINT_ERROR, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_HUMIDITY);
      }

      // For fan speed, the tachometer only gives out correct readings when control is active.
      if (fanSpeedControlActive_)
      {
        if (fanSpeedOK_)
        {
          printReadingRightAligned(fanSpeed_, INPUT_FANSPEED_DECIMALS, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_FANSPEED);
        }
        else
        {
          printTextRightAligned(PRINT_ERROR, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_FANSPEED);
        }
      }
      else
      {
        printNoReading();
      }

      // Indicate if control is running
      controlActiveIndicatorLeft_ = true;
      printControlIndicators(controlActiveIndicatorLeft_);
      controlIndicatorTimerStart_ = millis();
    }
    else
    {
      if (humidityOK_)
      {
        if (newHumidityReadingPrint_)
        {
          printReadingRightAligned(humidity_, INPUT_HUMIDITY_DECIMALS, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_HUMIDITY);
          newHumidityReadingPrint_ = false;
      
      #ifdef MEASURE_TEMPERATURE
        printReadingRightAligned(temperature_, TEMPERATURE_DECIMALS, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_TEMPERATURE);
      #endif // MEASURE_TEMPERATURE
        }
      }
      else
      {
        printTextRightAligned(PRINT_ERROR, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_HUMIDITY);
      }

      // For fan speed, the tachometer only gives out correct readings when control is active.
      if (fanSpeedControlActive_)
      {
        if (fanSpeedOK_)
        {
          if (newFanSpeedReadingPrint_)
          {
            printReadingRightAligned(fanSpeed_, INPUT_FANSPEED_DECIMALS, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_FANSPEED);
            newFanSpeedReadingPrint_ = false;
          }
        }
        else
        {
          printTextRightAligned(PRINT_ERROR, MAXCHAR_READINGS, COL_READING_RIGHTMOST, ROW_READING_FANSPEED);
        }
      }
      else
      {
        printNoReading();
      }

      if (millis() - controlIndicatorTimerStart_ >= PERIOD_SCREEN_CONTROLINDICATOR)
      {
        controlIndicatorTimerStart_ = millis();
        controlActiveIndicatorLeft_ ? controlActiveIndicatorLeft_ = false : controlActiveIndicatorLeft_ = true;
        printControlIndicators(controlActiveIndicatorLeft_);
      }
    }
    break;
  case SCREEN_PAGE_HUMIDITYADJ:
    // Screen displaying current target humidity and ability to adjust target humidity.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(0, 0);
      screen_.print("Relative humidity(%)");
      screen_.setCursor(0, 1);
      screen_.print("--------------------");
      screen_.setCursor(0, 2);
      screen_.print("Old target:");
      screen_.setCursor(0, 3);
      screen_.print("New target:");

      // Display current setpoint.
      printValueRightAligned(humidityTarget_, INPUT_HUMIDITY_DECIMALS, MAX_COLUMNS - 1, 2);

      // Prompt user for input.
      screen_.setCursor(MAX_COLUMNS - 1, 3);
      screen_.blink();
    }
    break;
  case SCREEN_PAGE_FANSPEEDADJ:
    // Screen displaying current target fan speed and ability to adjust target fan speed.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(2, 0);
      screen_.print("Fan speed (RPM)");
      screen_.setCursor(0, 1);
      screen_.print("--------------------");
      screen_.setCursor(0, 2);
      screen_.print("Old target:");
      screen_.setCursor(0, 3);
      screen_.print("New target:");

      // Display current setpoint.
      printValueRightAligned(fanSpeedTarget_, INPUT_FANSPEED_DECIMALS, MAX_COLUMNS - 1, 2);

      // Prompt user for input.
      screen_.setCursor(MAX_COLUMNS - 1, 3);
      screen_.blink();
    }
    break;
  case SCREEN_PAGE_CAL:
    // Screen displaying options for relative humidity two-point calibration.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(0, 0);
      screen_.print("---RH calibration---");
      screen_.setCursor(0, 1);
      screen_.print("Press 1 for point 1");
      screen_.setCursor(0, 2);
      screen_.print("Press 2 for point 2");
      screen_.setCursor(0, 3);
      screen_.print("Press 3 to reset all");
    }
    break;
  case SCREEN_PAGE_CAL_POINT:
    // Screen displaying stored (if any) calibration data and input for new calibration data for the chosen point.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(0, 0);
      screen_.print("------Point  -------");
      screen_.setCursor(12, 0);
      if (calibratingPoint1)
      {
        screen_.print('1');
      }
      else
      {
        screen_.print('2');
      }
      screen_.setCursor(0, 1);
      screen_.print("raw:      ref.:");
      screen_.setCursor(3, 2);
      screen_.print("New raw RH:");
      screen_.setCursor(2, 3);
      screen_.print("New ref. RH:");

      // Print out stored calibration data.
      float storedRHRef;
      float storedRHRaw;
      if (humiditySensor_.getSavedCalibration(calibratingPoint1, &storedRHRef, &storedRHRaw))
      {
        printValueRightAligned(storedRHRaw, INPUT_HUMIDITY_DECIMALS, 8, 1);
        printValueRightAligned(storedRHRef, INPUT_HUMIDITY_DECIMALS, MAX_COLUMNS - 1, 1);
      }
      else
      { // No saved calibration data or corrupt data.
        screen_.setCursor(5, 1);
        screen_.print("N/A");
        screen_.setCursor(16, 1);
        screen_.print("N/A");
      }

      // Print out current raw humidity reading.
      printValueRightAligned(humiditySensor_.getRHRaw(), INPUT_HUMIDITY_DECIMALS, MAX_COLUMNS - 1, 2);

      // Prompt user for input.
      screen_.setCursor(MAX_COLUMNS - 1, 3);
      screen_.blink();
    }
    else
    {
      if (humidityOK_)
      {
        if (newHumidityReadingPrint_)
        { // Since we are calibrating, print out the RAW reading.
          screen_.noBlink();
          printReadingRightAligned(humiditySensor_.getRHRaw(), INPUT_HUMIDITY_DECIMALS, MAXCHAR_RHRAW, MAX_COLUMNS - 1, 2);
          newHumidityReadingPrint_ = false;

          // Prompt user for input.
          screen_.setCursor(MAX_COLUMNS - 1, 3);
          screen_.blink();
        }
      }
      else
      {
        screen_.noBlink();
        printTextRightAligned(PRINT_ERROR, MAXCHAR_RHRAW, MAX_COLUMNS - 1, 2);

        // Prompt user for input.
        screen_.setCursor(MAX_COLUMNS - 1, 3);
        screen_.blink();
      }
    }
    break;
  case SCREEN_PAGE_CAL_RESET:
    // Confirmation screen to clear stored (if any) calibration data.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(1, 0);
      screen_.print("Reset calibration?");
      screen_.setCursor(0, 1);
      screen_.print("This will delete all");
      screen_.setCursor(1, 2);
      screen_.print("calibration data!!");
      screen_.setCursor(0, 3);
      screen_.print("--Press 5 to reset--");

      // Start the timer to return to calibration options screen.
      calResetSplashTimerStart_ = millis();
    }
    else if (millis() - calResetSplashTimerStart_ >= PERIOD_SCREEN_CALRESET)
    {
      changeScreenPage(SCREEN_PAGE_CAL);
    }
    break;
  case SCREEN_PAGE_HOLD:
    // Screen prompting user to keep holding the control start/stop button to stop control.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(0, 0);
      screen_.print("********************");
      screen_.setCursor(2, 1);
      screen_.print("Hold button for");
      screen_.setCursor(6, 2);
      screen_.print("second(s)");
      screen_.setCursor(0, 3);
      screen_.print("********************");

      // Print out seconds remaining, rounded towards the lesser integer
      printSecondsRemaining();
    }
    else
    {
      // Check how long the button has been held. Perform comparison in milliseconds to avoid floats/doubles
      if ((millis() - holdTimeStart_) + (keyHoldSecondsRemaining_ - 1) * 1000 >= keyHoldDuration_) 
      { // One second has passed, update the countdown.
        keyHoldSecondsRemaining_--;

        if (keyHoldSecondsRemaining_ < 1)
        { // Satisfied minimum hold time; stop control
          if (holdHumidityButton_)
          {
            humidityControlRecentlyStopped_ = true;
            toggleHumidityControl(false);
            changeScreenPage(SCREEN_PAGE_READINGS); // return screen to readings screen
          }
          else if (holdFanSpeedButton_)
          {
            fanSpeedControlRecentlyStopped_ = true;
            retryFunc(&HumidOSH::toggleFanSpeedControl, false);
            changeScreenPage(SCREEN_PAGE_READINGS); // return screen to readings screen
          }
        }
        else
        { // Still need to hold the button; update the countdown on the screen.
          // Print out seconds remaining, rounded towards the lesser integer.
          printSecondsRemaining();
        }
      }

      if (keypad_->getState() == RELEASED)
      { // User stopped holding the button before the minimum hold time was satisfied.
        // Return back to readings screen and continue on as usual.
        changeScreenPage(SCREEN_PAGE_READINGS); // return screen to readings screen
      }
    }
    break;
  case SCREEN_PAGE_MINVAL:
  case SCREEN_PAGE_MAXVAL:
    // Screen warning user that their input value is below the minimum or above the maximum.
    if (screenPageChanged_)
    {
      screenPageChanged_ = false;
      resetScreen();
      screen_.setCursor(0, 0);
      screen_.print("********************");
      screen_.setCursor(2, 1);

      if (screenPage_ == SCREEN_PAGE_MINVAL)
      {
        screen_.print("Minimum value is");
      }
      else
      {
        screen_.print("Maximum value is");
      }

      // Print the flasher
      screen_.setCursor(0, 2);
      screen_.print(ERROR_FLASHER_LEFT);
      screen_.setCursor(MAX_COLUMNS - strlen(ERROR_FLASHER_RIGHT), 2);
      screen_.print(ERROR_FLASHER_RIGHT);

      // Print the limit value, center-aligned
      if (errorInputHumidity_)
      {
        if (screenPage_ == SCREEN_PAGE_MINVAL)
        {
          printValueLimit(humidityMin_, INPUT_HUMIDITY_DECIMALS);
        }
        else
        {
          printValueLimit(humidityMax_, INPUT_HUMIDITY_DECIMALS);
        }
      }
      else
      {
        if (screenPage_ == SCREEN_PAGE_MINVAL)
        {
          printValueLimit(fanSpeedMin_, INPUT_FANSPEED_DECIMALS);
        }
        else
        {
          printValueLimit(fanSpeedMax_, INPUT_FANSPEED_DECIMALS);
        }
      }

      screen_.setCursor(0, 3);
      screen_.print("********************");
    }
    else if (millis() - errorInputTimerStart_ >= (errorInputTimerFlashCounter_ + 1) * PERIOD_ERROR_INPUT_FLASH)
    {
      errorInputTimerFlashCounter_++;

      if (errorInputTimerFlashCounter_ < ERROR_INPUT_FLASH_COUNT)
      {// Keep flashing
        if (errorInputFlashOn_)
        {
          errorInputFlashOn_ = false;

          // Erase the flasher
          screen_.setCursor(0, 2);
          screen_.print(ERROR_FLASHER_CLEAR);
          screen_.setCursor(MAX_COLUMNS - strlen(ERROR_FLASHER_CLEAR), 2);
          screen_.print(ERROR_FLASHER_CLEAR);
          //screen_.setFastBacklight(SCREEN_BACKGROUND_DEFAULT);
        }
        else
        {
          errorInputFlashOn_ = true;

          // Print the flasher
          screen_.setCursor(0, 2);
          screen_.print(ERROR_FLASHER_LEFT);
          screen_.setCursor(MAX_COLUMNS - strlen(ERROR_FLASHER_RIGHT), 2);
          screen_.print(ERROR_FLASHER_RIGHT);
          //screen_.setFastBacklight(SCREEN_BACKGROUND_ERROR);
        }
      }
      else
      {// Return to the settings screen
        resetInputVars();

        if (errorInputHumidity_)
        {
          changeScreenPage(SCREEN_PAGE_HUMIDITYADJ);
        }
        else
        {
          changeScreenPage(SCREEN_PAGE_FANSPEEDADJ);
        }
      }
    }
    break;
  }
}

bool HumidOSH::resetScreen()
{
  screen_.noBlink();
  return screen_.clear();
}

bool HumidOSH::clearValueRightAligned(const uint8_t & rightmostColNumber, const uint8_t & rowNumber, const uint8_t & charCount)
{
  if (charCount > 0)
  {
    // Move cursor to the leftmost column of the value
    if (screen_.setCursor(rightmostColNumber + 1 - charCount, rowNumber))
    {
      // Clear up the input space by overwriting with spaces
      for (uint8_t i = 0; i < charCount; i++)
      {
        if (!printToDisplay(CHAR_EMPTY))
        {
          return false;
        }
      }
      return true;
    }
    else { return false; }
  }
  else { return true; }
}

// The user input will always be at the bottom right of the screen.
// Calling this function clears out a number of spaces according to charCount
// at the user input field.
bool HumidOSH::resetScreenInput(uint8_t charMax, uint8_t charOffset)
{
  // Clear out old input
  if (clearValueRightAligned(MAX_COLUMNS - 1, MAX_ROWS - 1, charMax))
  {
    // Now set the cursor at the appropriate offset for printing new input
    // If nothing is to be printed, set the cursor to the right.
    if (charOffset < 1)
    {
      charOffset = 1;
    }

    if (screen_.setCursor(MAX_COLUMNS - charOffset, MAX_ROWS - 1))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else { return false; }
}

// Called after printing out user input. Places the cursor at
// bottom right and blink to prompt user to enter numbers.
bool HumidOSH::idleScreenInput()
{
  // Move cursor back to bottom right of screen
  if (screen_.setCursor(MAX_COLUMNS - 1, MAX_ROWS - 1))
  {
    if (screen_.blink())
    {
      return true;
    }
    else { return false; }
  }
  else { return false; }
}

bool HumidOSH::printToDisplay(char charToPrint)
{
  return screen_.print(charToPrint);
}

bool HumidOSH::printToDisplay(const char stringToPrint[])
{
  return screen_.print(stringToPrint);
}

bool HumidOSH::printToDisplay(double doubleToPrint, uint8_t decimalsToPrint)
{
  return screen_.print(doubleToPrint, decimalsToPrint);
}

// Print out values on the screen, right aligned.
void HumidOSH::printValueRightAligned(const double & value, const uint8_t & decimalsMax, const uint8_t & rightmostColNumber, const uint8_t & rowNumber)
{ // First, calculate the number of integers
  uint8_t integerCount = getIntegerCount(value);
  // Move cursor, accounting for integer, decimal point , and decimals.
  if (decimalsMax > 0)
  {
    screen_.setCursor(rightmostColNumber - decimalsMax - integerCount, rowNumber);
  }
  else
  {
    screen_.setCursor(rightmostColNumber - integerCount + 1, rowNumber);
  }

  screen_.print(value, decimalsMax);
}

// Print out text on the screen, right aligned.
void HumidOSH::printValueRightAligned(const char value[], const uint8_t & rightmostColNumber, const uint8_t & rowNumber)
{
  screen_.setCursor(rightmostColNumber - strlen(value) + 1, rowNumber);
  screen_.print(value);
}

// Prints symbols to indicate if control is running or not.
void HumidOSH::printControlIndicators(bool printingLeft)
{
  // Print for humidity
  screen_.setCursor(0, 2);
  if (humidityControlActive_)
  {
    if (printingLeft)
    {
      screen_.print(CONTROLINDICATOR_RUN_LEFT);
    }
    else
    {
      screen_.print(CONTROLINDICATOR_RUN_RIGHT);
    }
  }
  else
  {
    screen_.print(CONTROLINDICATOR_IDLE);
  }

  // Print for fan speed
  screen_.setCursor(0, 3);
  if (fanSpeedControlActive_)
  {
    if (printingLeft)
    {
      screen_.print(CONTROLINDICATOR_RUN_LEFT);
    }
    else
    {
      screen_.print(CONTROLINDICATOR_RUN_RIGHT);
    }
  }
  else
  {
    screen_.print(CONTROLINDICATOR_IDLE);
  }
}

// Prints out the given reading right-aligned at the given position on screen (rightmostCol and row).
// Clears out the area (size depends on readingCharMaxCount) before printing out the reading.
void HumidOSH::printReadingRightAligned(float reading, uint8_t maxDecimals, uint8_t readingCharMaxCount, uint8_t rightmostCol, uint8_t row)
{
  // Truncate the reading to satisfy maxDecimals.
  // This is especially important in cases where we get numbers like 9.999999999999.
  // Assume maxDecimals was 2. If we do not truncate, readingCharCount will be 4,
  // but upon printing the float, it will be rounded into 10.00, leaving us with
  // an actual readingCharCount of 5.
  float truncatedReading = round(reading * pow(10, maxDecimals)) / pow(10, maxDecimals);

  // Calculate the number of shown characters of the printed reading
  uint8_t readingCharCount = getIntegerCount(truncatedReading);
  if (maxDecimals > 0) { readingCharCount = readingCharCount + 1 + maxDecimals; }
  
  // Clear only the spots where the number is not going to be printed. If we keep clearing
  // the entire reading spot every update, there will be a noticeable flicker of the readings
  // due to that short period where the entire reading spot is empty.
  clearValueRightAligned(rightmostCol - readingCharCount, row, readingCharMaxCount - readingCharCount);
  printValueRightAligned(truncatedReading, maxDecimals, rightmostCol, row);
}

// Prints out the given reading right-aligned at the given position on screen (rightmostCol and row).
// Clears out the area (size depends on textCharMaxCount) before printing out the text.
void HumidOSH::printTextRightAligned(const char text[], uint8_t textCharMaxCount, uint8_t rightmostCol, uint8_t row)
{
  clearValueRightAligned(rightmostCol - strlen(text), row, textCharMaxCount - strlen(text));
  printValueRightAligned(text, rightmostCol, row);
}

// Prints out an indicator that no readings are available.
// Clears out the area before printing out the text.
void HumidOSH::printNoReading()
{
  clearValueRightAligned(COL_READING_RIGHTMOST - strlen(PRINT_NOREADING), ROW_READING_FANSPEED, MAXCHAR_READINGS - strlen(PRINT_NOREADING));
  printValueRightAligned(PRINT_NOREADING, COL_READING_RIGHTMOST, ROW_READING_FANSPEED);
}

// Print out seconds remaining, rounded towards the lesser integer
void HumidOSH::printSecondsRemaining()
{
  uint8_t integerCount = getIntegerCount(keyHoldSecondsRemaining_);
  screen_.setCursor(4 - integerCount, 2);
  screen_.print(keyHoldSecondsRemaining_);
}

// Prints the limit that is applied onto the user input. This is
// called when a min/max input error screen is displayed.
void HumidOSH::printValueLimit(double limit, uint8_t decimalsMax)
{
  // Print the limit center-aligned
  uint8_t charCount = getIntegerCount(limit);
  if (decimalsMax > 0)
  {
    charCount += 1 + decimalsMax;   // Account for decimal point and decimal chars.
  }
  screen_.setCursor(MAX_COLUMNS / 2 - 1 - charCount / 2, 2);
  screen_.print(limit, decimalsMax);
}

// Handle button presses for starting/stopping control
void HumidOSH::handleButtonControl(bool humidity, const bool & controlActiveFlag, bool & recentlyStoppedFlag)
{
  if (controlActiveFlag)
  {// User is trying to stop humidity control
    if (keypad_->getState() == HOLD)
    { // User begun to hold the button to stop control.
      // Begin countdown for minimum hold time.
      holdTimeStart_ = millis();
      holdHumidityButton_ = humidity;
      holdFanSpeedButton_ = !humidity;
      keyHoldSecondsRemaining_ = keyHoldDuration_ / 1000; // Convert to seconds
      changeScreenPage(SCREEN_PAGE_HOLD); // change screen to prompt user to keep holding
    }
  }
  else
  {// User is trying to start humidity control
    if (keypad_->getState() == RELEASED)
    {
      if (!recentlyStoppedFlag)
      {// Control has not begun; begin control.
        if (humidity)
        {
          toggleHumidityControl(true);
        }
        else
        {
          retryFunc(&HumidOSH::toggleFanSpeedControl, true);
        }
      }
      else
      { // The button was being held to stop the control and control has already been stopped.
        // After the control was stopped, the screen was changed back to readings screen,
        // which is how the code ended up in this location.
        recentlyStoppedFlag = false;
      }
    }
  }
}

// Handle number key presses.
void HumidOSH::handleInputNumber(char inputKey, const uint8_t & charMax, const uint8_t & decimalsMax)
{
  if (inputCharCount_ + 1 <= charMax)
  { // Still within character limit. Ignore the input if we are already at maximum char limit.
    if (decimalUsed_)
    {
      if (inputDecimalCount_ + 1 <= decimalsMax)
      { // Still within the max allowable decimal places. Ignore the input if we are already at max decimal places.
        inputCharCount_++;
        inputDecimalCount_++;
        inputValue_ += atoi(&inputKey) / pow(10, inputDecimalCount_);
        resetScreenInput(charMax, inputCharCount_);
        printToDisplay(inputValue_, inputDecimalCount_);
        idleScreenInput();
      }
    }
    else
    {
      inputCharCount_++;
      inputIntCount_++;
      inputValue_ *= 10;  // Move one power up
      inputValue_ += atoi(&inputKey);
      resetScreenInput(charMax, inputCharCount_);
      printToDisplay(inputValue_, 0);  // No decimal places.
      idleScreenInput();
    }
  }
}

void HumidOSH::handleInputDelete(const uint8_t & charMax)
{
  if (inputCharCount_ > 0)
  { // Only delete if there are any chars present.
    if (decimalUsed_)
    { // There are decimal number(s) present
      if (inputDecimalCount_ > 0)
      { // Remove decimal number
        inputDecimalCount_--;
        inputCharCount_--;

        if (inputDecimalCount_ > 0)
        { // There is still decimal number(s) remaining to be printed.
          inputValue_ = trunc(inputValue_ * pow(10, inputDecimalCount_)) / pow(10, inputDecimalCount_);
          resetScreenInput(charMax, inputCharCount_);
          printToDisplay(inputValue_, inputDecimalCount_);
          idleScreenInput();
        }
        else
        { // No remaining decimal places, make sure to still print the decimal dot.
          inputValue_ = trunc(inputValue_);
          resetScreenInput(charMax, inputCharCount_);
          printToDisplay(inputValue_, 0);  // Print out the integers first.
          printToDisplay(CHAR_DECIMAL);    // Add on the decimal point.
          idleScreenInput();
        }
      }
      else
      { // Remove the decimal point char
        inputCharCount_--;
        decimalUsed_ = false;
        resetScreenInput(charMax, inputCharCount_);
        printToDisplay(inputValue_, 0);  // No decimal char, so just print the integers.
        idleScreenInput();
      }
    }
    else
    { // Only integers are present.
      inputIntCount_--;
      inputCharCount_--;
      inputValue_ = trunc(inputValue_ / 10);
      resetScreenInput(charMax, inputCharCount_);

      if (inputCharCount_ > 0)
      { // Only print stuff out if there are any digits to print
        printToDisplay(inputValue_, 0);
      }

      idleScreenInput();
    }
  }
}

// Handle the decimal symbol key press
void HumidOSH::handleInputDot(const uint8_t & charMax, const uint8_t & decimalsMax)
{
  if (decimalsMax > 0 &&            // Only insert the decimal symbol if decimals are allowed
      inputCharCount_ < charMax &&  // Adding a decimal point will not exceed max char limit
      !decimalUsed_ &&              // We cannot have more than one decimal symbol
      inputIntCount_ > 0)           // An integer (including 0) must be already present
  {
    inputCharCount_++;
    decimalUsed_ = true;
    resetScreenInput(charMax, inputCharCount_);
    printToDisplay(inputValue_, 0);  // Print out the integers first.
    printToDisplay(CHAR_DECIMAL);    // Add on the decimal point.
    idleScreenInput();
  }
}

// Check user input and display error message if necessary, otherwise save it.
bool HumidOSH::saveInput(bool humidity, double & targetBuffer, const double & min, const double & max)
{
  if (inputValue_ > max)
  { // Warn user if input value exceeds max setting.
    errorInputHumidity_ = humidity;
    changeScreenPage(SCREEN_PAGE_MAXVAL);
    errorInputTimerStart_ = millis();
    errorInputFlashOn_ = true;
    errorInputTimerFlashCounter_ = 0;
    return false;
  }
  else if (inputValue_ < min)
  { // Warn user if input value is below min setting.
    errorInputHumidity_ = humidity;
    changeScreenPage(SCREEN_PAGE_MINVAL);
    errorInputTimerStart_ = millis();
    errorInputFlashOn_ = true;
    errorInputTimerFlashCounter_ = 0;
    return false;
  }
  else
  {
    targetBuffer = inputValue_;

    // Update tachometer target.
    if (!humidity)
    {
      retryFunc(&HumidOSH::updateFanSpeedTarget, targetBuffer);
    }
    return true;
  }
}

// Reset input vars.
void HumidOSH::resetInputVars()
{
  inputValue_         = 0;
  inputCharCount_     = 0;
  inputIntCount_      = 0;
  inputDecimalCount_  = 0;
  decimalUsed_        = false;
}

// The SHT3x-DIS sensor needs to be triggered to grab a measurement.
bool HumidOSH::triggerHumidity()
{
  if (this->humiditySensor_.triggerOneMeasurement(false, SHT3x::REP_HIG) == SHT3X_STATUS_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Grab the data saved 
bool HumidOSH::getHumidity()
{
  if (this->humiditySensor_.fetchMeasurement() == SHT3X_STATUS_OK)
  {
    humidity_ = this->humiditySensor_.getRH();
#ifdef MEASURE_TEMPERATURE
    temperature_ = this->humiditySensor_.getTemperature();
#endif // MEASURE_TEMPERATURE

    return true;
  }
  else
  {
    return false;
  }
}

// Toggle the humidity control on or off. Resets PID params upon toggling on.
void HumidOSH::toggleHumidityControl(bool enable)
{
  if (enable)
  {
    digitalWrite(pinLEDRH_, HIGH);
    humidityControlActive_ = true;
    humidityPID_.Reset();
  }
  else
  {
    digitalWrite(pinLEDRH_, LOW);
    humidityControlActive_ = false;
    setPumpDutyCycle(0);
    toggleValveDry(false);
    toggleValveWet(false);
  }
}

// Update RH target, ensuring it's within the limits.
void HumidOSH::setHumidityTarget(double targetPercent)
{
  humidityTarget_ = constrain(targetPercent, humidityMin_, humidityMax_);
}

// Update the pump duty cycle, ensuring it's within the limits.
void HumidOSH::setPumpDutyCycle(uint8_t dutyCycle)
{
  pumpDutyCycle_ = dutyCycle;
  analogWrite(pinPump_, pumpDutyCycle_);
}

// Get fan speed in RPM.
bool HumidOSH::getFanSpeed()
{
  if (this->fan_.fetchFanSpeed() == EMC2301_STATUS_OK)
  {
    fanSpeed_ = this->fan_.getFanSpeed();
    return true;
  }
  else
  {
    return false;
  }
}

// Update fan speed target, ensuring it's within the limits.
bool HumidOSH::updateFanSpeedTarget(double targetRPM)
{
  // Only change fan speed if control is already active.
  // If it's not, the fan speed will be set when toggling the control on.
  if (fanSpeedControlActive_)
  {
    if (this->fan_.setFanSpeedTarget(fanSpeedTarget_) == EMC2301_STATUS_OK)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return true;
  }
}

// Toggle the fan speed control on or off. Note that the EMC2301 will perform
// a spin-up routine upon turning on the control.
bool HumidOSH::toggleFanSpeedControl(bool enable)
{
  if (enable)
  {
    if (this->fan_.setFanSpeedTarget(fanSpeedTarget_) == EMC2301_STATUS_OK)
    {
      toggleFan(true);
      digitalWrite(pinLEDFan_, HIGH);
      fanSpeedControlActive_ = true;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    // Set the target to 0 so the EMC2301 stops trying to achieve a target fan speed.
    if (this->fan_.setFanSpeedTarget(0) == EMC2301_STATUS_OK)
    {
      toggleFan(false);
      digitalWrite(pinLEDFan_, LOW);
      fanSpeedControlActive_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }
}


/***************************************************
*                                                  *
* Functions for turning on/off parts of the system *
*                                                  *
***************************************************/
void HumidOSH::togglePump(bool enable)
{
  if (enable)
  {
    analogWrite(pinPump_, pumpDutyCycle_);
  }
  else
  {
    analogWrite(pinPump_, 0);
  }
}

void HumidOSH::toggleValveDry(bool enable)
{
  if (enable)
  {
    digitalWrite(pinValveDry_, HIGH);
  }
  else
  {
    digitalWrite(pinValveDry_, LOW);
  }
}

void HumidOSH::toggleValveWet(bool enable)
{
  if (enable)
  {
    digitalWrite(pinValveWet_, HIGH);
  }
  else
  {
    digitalWrite(pinValveWet_, LOW);
  }
}

void HumidOSH::toggleFan(bool enable)
{
  if (enable)
  {
    digitalWrite(pinFanPWMDrain_, LOW);
  }
  else
  {
    digitalWrite(pinFanPWMDrain_, HIGH);
  }
}

void HumidOSH::toggleLEDRH(bool enable)
{
  if (enable)
  {
    digitalWrite(pinLEDRH_, HIGH);
  }
  else
  {
    digitalWrite(pinLEDRH_, LOW);
  }
}

void HumidOSH::toggleLEDFan(bool enable)
{
  if (enable)
  {
    digitalWrite(pinLEDFan_, HIGH);
  }
  else
  {
    digitalWrite(pinLEDFan_, LOW);
  }
}

uint8_t HumidOSH::getIntegerCount(double value)
{
  if (value <= 0)
  { // log10 is undefined for <= 0. Consider the 0 as a single digit.
    // We won't be displaying negative values in this program, so don't have to deal with that case.
    return 1;
  }
  else
  {
    return ((uint8_t)log10(value)) + 1;
  }
}
