// Prints a number of characters depending on charLimit, then
// clears all of them and start writing again.
// This sketch shows how to use SerLCD to clear characters.
#include <serLCD_cI2C.h>
#include <SPI.h>
#include <I2C.h>
const uint8_t charLimit = 5;
uint8_t charCounter;

SerLCD screen_;

void setup() {
  // Set up I2C
  I2c.setTimeOut(30);   // Timeout after 30 ms.
  I2c.begin(false);     // True to use internal pull-up resistors; false for external pull-ups.
  I2c.setSpeed(true);  // true for 400 kHz (fast-mode); false for 100 kHz (standard mode). Need standard mode to ensure SMBus compatibility on some of the components.

  // Init the LCD screen
  screen_.begin(I2c);
  screen_.setBacklight(255, 255, 255); //Set backlight to bright white
  screen_.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
  screen_.setCursor(0, 0);
  screen_.print("Printing test nums");
  screen_.setCursor(0, 1);
  randomSeed(2); //Feed the random number generator
}

void loop() {
  if (charCounter < charLimit)
  {
    charCounter++;
    screen_.print(random(0, 9));
    delay(200);
  }
  else
  {
    charCounter = 0;
    screen_.setCursor(0, 1);
    for (uint8_t i = 0; i < charLimit; i++)
    {
      screen_.print(' ');
    }
    screen_.setCursor(0, 1);
    delay(1000);
  }
}
