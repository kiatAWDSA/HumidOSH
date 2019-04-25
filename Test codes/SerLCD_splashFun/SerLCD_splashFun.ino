// Change the splash screen shown when starting up the screen.
// After that, do the same thing as the other test code SerLCD_clearBehavior.
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

  screen_.clear(); // clear screen
  
  // Send our new content to display - this will soon become our new splash screen.
  screen_.setCursor(0, 0);
  screen_.print("********************");
  screen_.setCursor(0, 1);
  screen_.print("----> HumidOSH <----");
  screen_.setCursor(0, 2);
  screen_.print(" Soon Kiat Lau 2019 ");
  screen_.setCursor(0, 3);
  screen_.print("********************");

  // The following two lines can actually be wrapped into a function. The clear() functions calls '|' before calling '-'. Here, it is '|' before 10.
  screen_.write('|'); //Put LCD into setting mode
  screen_.write(10); //Set current contents to splash screen memory (this is also a "ctrl-j", if you are doing it manually)
  
  delay(3000);  // Without this delay, the next few lines of code will be ignored for some reason.
  screen_.setCursor(0, 1);
  screen_.print(">     HumidOSH     <");
  delay(200);
  screen_.setCursor(0, 1);
  screen_.print(">     HumidOSH     <");
  delay(200);
  screen_.setCursor(0, 1);
  screen_.print("->    HumidOSH    <-");
  delay(200);
  screen_.setCursor(0, 1);
  screen_.print("-->   HumidOSH   <--");
  delay(200);
  screen_.setCursor(0, 1);
  screen_.print("--->  HumidOSH  <---");
  delay(200);
  screen_.setCursor(0, 1);
  screen_.print("----> HumidOSH <----");
  delay(2000);
  
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
