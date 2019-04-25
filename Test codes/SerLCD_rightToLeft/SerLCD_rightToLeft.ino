// Shows how to print from right to left.
// It is highly recommended to use leftToRight() while tracking the character count.
// Using the rightToLeft() causes buggy behavior
#include <serLCD_cI2C.h>
#include <SPI.h>
#include <I2C.h>

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

  screen_.clear(); // clear screen
  delay(3000);
  
  // First row: Print a string of right-aligned text, but using left-to-right
  uint8_t charCount = 9; // 1234.5678
  screen_.setCursor(MAX_COLUMNS - charCount, 0);
  screen_.leftToRight();
  screen_.print(1234);
  screen_.print('.');
  screen_.print(5678);

  // Second row: Print a string of text right to left
  screen_.setCursor(MAX_COLUMNS - charCount, 1);
  screen_.rightToLeft();
  screen_.print(1234);
  screen_.print('.');
  screen_.print(5678);

  /*
   * This code block is how we would expect to use rightToLeft to print a string of text right to left, but it behaves erratically by printing text on the first line.
  screen_.setCursor(MAX_COLUMNS-1, MAX_ROWS-1);
  screen_.rightToLeft();
  screen_.print(1234);
  screen_.print('.');
  screen_.print(5678);
   */
}

void loop()
{
}
