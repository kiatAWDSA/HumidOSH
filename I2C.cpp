/*
I2C.cpp - I2C library
Copyright (c) 2011-2012 Wayne Truchsess.  All right reserved.
Rev 5.0 - January 24th, 2012
- Removed the use of interrupts completely from the library
so TWI state changes are now polled.
- Added calls to lockup() function in most functions
to combat arbitration problems
- Fixed scan() procedure which left timeouts enabled
and set to 80msec after exiting procedure
- Changed scan() address range back to 0 - 0x7F
- Removed all Wire legacy functions from library
- A big thanks to Richard Baldwin for all the testing
and feedback with debugging bus lockups!
Rev 4.0 - January 14th, 2012
- Updated to make compatible with 8MHz clock frequency
Rev 3.0 - January 9th, 2012
- Modified library to be compatible with Arduino 1.0
- Changed argument type from boolean to uint8_t in pullUp(),
setSpeed() and receiveByte() functions for 1.0 compatability
- Modified return values for timeout feature to report
back where in the transmission the timeout occured.
- added function scan() to perform a bus scan to find devices
attached to the I2C bus.  Similar to work done by Todbot
and Nick Gammon
Rev 2.0 - September 19th, 2011
- Added support for timeout function to prevent
and recover from bus lockup (thanks to PaulS
and CrossRoads on the Arduino forum)
- Changed return type for stop() from void to
uint8_t to handle timeOut function
Rev 1.0 - August 8th, 2011

This is a modified version of the Arduino Wire/TWI
library.  Functions were rewritten to provide more functionality
and also the use of Repeated Start.  Some I2C devices will not
function correctly without the use of a Repeated Start.  The
initial version of this library only supports the Master.


This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Modified by Soon Kiat Lau (2019) for:
- ping function
- error codes
- allow user to enable/disable internal pull-up resistors in begin()
*/

#include "I2C.h"



uint8_t I2C::bytesAvailable_ = 0;
uint8_t I2C::bufferIndex_ = 0;
uint8_t I2C::totalBytes_ = 0;

I2C::I2C()
{
}


////////////// Public Methods ////////////////////////////////////////


// Choose to use internal pull-up resistors or not. The choice depends
// on the speed of I2C and number of devices on the I2C bus.
// NOTE: It is important to disable internal pull-ups if there are
// external pull-ups connected, otherwise the I2C lines will never be
// pulled fully to GND due to the internal pull-ups.
void I2C::begin(bool enableInternalPullUps)
{
  enableInternalPullUps_ = enableInternalPullUps;
  pullup(enableInternalPullUps);

  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);

  // Defaults to 100 kHz
  TWBR = ((F_CPU / 100000) - 16) / 2;
  // enable twi module and acks
  TWCR = _BV(TWEN) | _BV(TWEA);
}

void I2C::end()
{
  TWCR = 0;
}

void I2C::setTimeOut(uint16_t timeOut)
{
  timeOut_ = timeOut;
}

// Call setSpeed AFTER begin(), because begin() sets speed to 100 kHz
void I2C::setSpeed(bool useFastMode)
{
  useFastMode_ = useFastMode;

  if (!useFastMode)
  {
    // 100 kHz
    TWBR = ((F_CPU / 100000) - 16) / 2;
  }
  else
  {
    // 400 kHz
    TWBR = ((F_CPU / 400000) - 16) / 2;
  }
}

void I2C::pullup(bool activate)
{
  if (activate)
  {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // activate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    sbi(PORTC, 4);
    sbi(PORTC, 5);
#else
    // activate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    sbi(PORTD, 0);
    sbi(PORTD, 1);
#endif
  }
  else
  {
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
#else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
#endif
  }
}

void I2C::scan()
{
  uint16_t tempTime = timeOut_;
  setTimeOut(80);
  uint8_t totalDevicesFound = 0;
  Serial.println("Scanning for devices...please wait");
  Serial.println();
  for (uint8_t s = 0; s <= 0x7F; s++)
  {
    // Begin transmission
    returnStatus_ = beginTransmission(s, true, false);

    if (returnStatus_ != I2C_STATUS_OK)
    {
      if (returnStatus_ != I2C_STATUS_BEGIN_NACK)
      {// Will receive a NACK if a device has the address, but is unable to communicate now. Therefore, that is not a problem with the I2C bus.
        // Other errors indicate there is a problem with the bus.
        Serial.println("There is a problem with the bus, could not complete scan");
        timeOut_ = tempTime;
        return;
      }
    }
    else
    {
      Serial.print("Found device at address - ");
      Serial.print(" 0x");
      Serial.println(s, HEX);
      totalDevicesFound++;
    }
    stop();
  }
  if (!totalDevicesFound) { Serial.println("No devices found"); }
  timeOut_ = tempTime;
}


uint8_t I2C::available()
{
  return(bytesAvailable_);
}

uint8_t I2C::getByte()
{
  bufferIndex_ = totalBytes_ - bytesAvailable_;
  if (!bytesAvailable_)
  {
    bufferIndex_ = 0;
    return(0);
  }
  bytesAvailable_--;
  return(data_[bufferIndex_]);
}


/*return values for new functions that use the timeOut feature
will now return at what point in the transmission the timeout
occurred. Looking at a full communication sequence between a
master and slave (transmit data and then readback data) there
a total of 7 points in the sequence where a timeout can occur.
These are listed below and correspond to the returned value:
1 - Waiting for successful completion of a Start bit
2 - Waiting for ACK/NACK while addressing slave in transmit mode (MT)
3 - Waiting for ACK/NACK while sending data to the slave
4 - Waiting for successful completion of a Repeated Start
5 - Waiting for ACK/NACK while addressing slave in receiver mode (MR)
6 - Waiting for ACK/NACK while receiving data from the slave
7 - Waiting for successful completion of the Stop bit

All possible return values:
0           Function executed with no errors
1 - 7       Timeout occurred, see above list
8 - 0xFF    Unknown or yet to be defined error */


/////////////////////////////////////////////////////


// Sends a write request to the device without any additional bytes.
// This is sometimes used to trigger a device.
I2C_STATUS I2C::ping(uint8_t address)
{
  // Begin transmission
  returnStatus_ = beginTransmission(address, true, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Send the register address of interest to the target device.
I2C_STATUS I2C::write(uint8_t address, uint8_t registerAddress)
{
  // Begin transmission
  returnStatus_ = beginTransmission(address, true, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Send register address byte
  returnStatus_ = transmit(registerAddress);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Same as above, but accounts for int arguments.
I2C_STATUS I2C::write(int address, int registerAddress)
{
  return(write((uint8_t)address, (uint8_t)registerAddress));
}

// Write a single data byte to a given register in the target device.
I2C_STATUS I2C::write(uint8_t address, uint8_t registerAddress, uint8_t data)
{
  // Begin transmission
  returnStatus_ = beginTransmission(address, true, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Send register address byte
  returnStatus_ = transmit(registerAddress);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Send a byte, with expectation for a NACK bit
  returnStatus_ = transmit(data);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Same as above, but accounts for int arguments
I2C_STATUS I2C::write(int address, int registerAddress, int data)
{
  return write((uint8_t)address, (uint8_t)registerAddress, (uint8_t)data);
}

// Write data to a given register in the target device.
// User is responsible for defining the number of bytes of the given data buffer to be sent.
// User is also responsible for ensuring that the given data buffer has sufficient bytes to be sent.
I2C_STATUS I2C::write(uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes)
{
  // Begin transmission
  returnStatus_ = beginTransmission(address, true, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Send register address byte
  returnStatus_ = transmit(registerAddress);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Send data
  for (uint8_t i = 0; i < numberBytes; i++)
  {
    // Send a byte, with expectation for a NACK bit
    returnStatus_ = transmit(data[i]);
    if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }
  }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Same as above, but accounts for a char buffer.
I2C_STATUS I2C::write(uint8_t address, uint8_t registerAddress, char *data, uint8_t bufferLength)
{
  return write(address, registerAddress, (uint8_t*)data, bufferLength);
}

// Writes data bytes to the target device, ignoring the concept of register address.
I2C_STATUS I2C::write(uint8_t address, uint8_t * data, uint8_t numberBytes)
{
  // Begin transmission
  returnStatus_ = beginTransmission(address, true, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Send data
  for (uint8_t i = 0; i < numberBytes; i++)
  {
    // Send a byte, with expectation for a NACK bit
    returnStatus_ = transmit(data[i]);
    if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }
  }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Same as above, but for a char data buffer.
I2C_STATUS I2C::write(int address, char * data, uint8_t numberBytes)
{
  return write(address, (uint8_t*) data, numberBytes);
}

// Read data from the target device into the given data buffer.
// User is responsible for ensuring the buffer has sufficient size for the incoming data.
I2C_STATUS I2C::read(uint8_t address, uint8_t numberBytes, uint8_t *dataBuffer)
{
  bytesAvailable_ = 0;
  bufferIndex_ = 0;
  if (numberBytes == 0) { numberBytes++; }
  nack_ = numberBytes - 1;

  // Begin transmission
  returnStatus_ = beginTransmission(address, false, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Start grabbing data
  for (uint8_t i = 0; i < numberBytes; i++)
  {
    if (i == nack_)
    {
      // Receive byte, with expectation for a NACK bit
      returnStatus_ = receive(false);
      if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }
    }
    else
    {
      // Receive byte, with expectation for an ACK bit
      returnStatus_ = receive(true);
      if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }
    }

    dataBuffer[i] = TWDR;
    bytesAvailable_ = i + 1;
    totalBytes_ = i + 1;
  }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Read data from the target device into the internal data buffer.
I2C_STATUS I2C::read(uint8_t address, uint8_t numberBytes)
{
  return read(address, numberBytes, data_);
}

// Same as above, but accounts for int arguments
I2C_STATUS I2C::read(int address, int numberBytes)
{
  return(read((uint8_t)address, (uint8_t)numberBytes));
}

// Utilize repeated start to read data from a register address into the given data buffer. User is responsible to ensure the buffer is of sufficient size.
// Note that the internal counter for number of bytes received will increase for each byte received,
// regardless of whether the buffer is an internal or external one. The user is thus responsible for not calling
// available() or getData() if an external buffer is used here.
I2C_STATUS I2C::read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes, uint8_t *dataBuffer)
{
  bytesAvailable_ = 0;
  bufferIndex_ = 0;
  if (numberBytes == 0) { numberBytes++; }
  nack_ = numberBytes - 1;

  // Begin transmission
  returnStatus_ = beginTransmission(address, true, false);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Ask for this register address
  returnStatus_ = transmit(registerAddress);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  // Repeated start to start acquiring data
  returnStatus_ = beginTransmission(address, false, true);
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  for (uint8_t i = 0; i < numberBytes; i++)
  {
    if (i == nack_)
    {
      // Receive byte, with expectation for a NACK bit
      returnStatus_ = receive(false);
      if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }
    }
    else
    {
      // Receive byte, with expectation for an ACK bit
      returnStatus_ = receive(true);
      if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }
    }

    dataBuffer[i] = TWDR;
    bytesAvailable_ = i + 1;
    totalBytes_ = i + 1;
  }

  // End transmission
  returnStatus_ = endTransmission();
  if (returnStatus_ != I2C_STATUS_OK) { return returnStatus_; }

  return I2C_STATUS_OK;
}

// Utilize repeated start to read data from a register address into the internal buffer
I2C_STATUS I2C::read(uint8_t address, uint8_t registerAddress, uint8_t numberBytes)
{
  return read(address, registerAddress, numberBytes, data_);
}

// Same as above, but accounts for int arguments
I2C_STATUS I2C::read(int address, int registerAddress, int numberBytes)
{
  return(read((uint8_t)address, (uint8_t)registerAddress, (uint8_t)numberBytes));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////// Abstractions of some of the private functions to return the appropriate I2C_STATUS /////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Send out start bit, the address, and read/write byte (depending on second argument)
I2C_STATUS I2C::beginTransmission(uint8_t address, bool write, bool repeatedStart)
{
  // Start bit
  TWSRStatus_ = start();
  if (TWSRStatus_ != TWSR_STATUS_STARTED)
  {
    if (repeatedStart)
    {
      return(I2C_STATUS_REPSTART_TIMEOUT);
    }
    else
    {
      return(I2C_STATUS_START_TIMEOUT);
    }
  }

  // Address
  TWSRStatus_ = sendAddress(write ? SLA_W(address) : SLA_R(address));
  if (TWSRStatus_ != TWSR_STATUS_ACK)
  {
    switch (TWSRStatus_)
    {
    case TWSR_STATUS_TIMEOUT:
      return I2C_STATUS_BEGIN_TIMEOUT;
      break;
    case TWSR_STATUS_NACK:
      return I2C_STATUS_BEGIN_NACK;
      break;
    case TWSR_STATUS_LOSTARB:
      return I2C_STATUS_BEGIN_LOSTARB;
      break;
    default:
      return I2C_STATUS_UNKNOWN;
      break;
    }
  }
  else
  {
    return I2C_STATUS_OK;
  }
}

I2C_STATUS I2C::transmit(uint8_t dataByte)
{
  TWSRStatus_ = sendByte(dataByte);
  if (TWSRStatus_ != TWSR_STATUS_ACK)
  {
    switch (TWSRStatus_)
    {
    case TWSR_STATUS_TIMEOUT:
      return I2C_STATUS_TRS_TIMEOUT;
      break;
    case TWSR_STATUS_NACK:
      return I2C_STATUS_TRS_NACK;
      break;
    default:
      return I2C_STATUS_UNKNOWN;
      break;
    }
  }
  else
  {
    return I2C_STATUS_OK;
  }
}

I2C_STATUS I2C::receive(bool sendACK)
{
  if (sendACK)
  {
    TWSRStatus_ = receiveByte(true);

    if (TWSRStatus_ != TWSR_STATUS_ACK)
    {
      switch (TWSRStatus_)
      {
      case TWSR_STATUS_TIMEOUT:
        return I2C_STATUS_TRS_TIMEOUT;
        break;
      case TWSR_STATUS_NACK:
        return I2C_STATUS_REC_ACKBUTNACK;
        break;
      case TWSR_STATUS_LOSTARB:
        return I2C_STATUS_REC_LOSTARB;
      default:
        return I2C_STATUS_UNKNOWN;
        break;
      }
    }
    else
    {
      return I2C_STATUS_OK;
    }
  }
  else
  {
    TWSRStatus_ = receiveByte(false);

    if (TWSRStatus_ != TWSR_STATUS_NACK)
    {
      switch (TWSRStatus_)
      {
      case TWSR_STATUS_TIMEOUT:
        return I2C_STATUS_TRS_TIMEOUT;
        break;
      case TWSR_STATUS_ACK:
        return I2C_STATUS_REC_NACKBUTACK;
        break;
      case TWSR_STATUS_LOSTARB:
        return I2C_STATUS_REC_LOSTARB;
      default:
        return I2C_STATUS_UNKNOWN;
        break;
      }
    }
    else
    {
      return I2C_STATUS_OK;
    }
  }
}

I2C_STATUS I2C::endTransmission()
{
  TWSRStatus_ = stop();
  if (TWSRStatus_ != TWSR_STATUS_STOPPED)
  {
    if (TWSRStatus_ == TWSR_STATUS_TIMEOUT) { return(I2C_STATUS_STOP_TIMEOUT); }
    else { return(I2C_STATUS_UNKNOWN); }
  }
  else
  {
    return I2C_STATUS_OK;
  }
}

///////////////////////////////////////////////////////////////
/////////////// Private Methods ///////////////////////////////
///////////////////////////////////////////////////////////////

TWSR_STATUS I2C::start()
{
  unsigned long startingTime = millis();
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if (!timeOut_) { continue; }
    if ((millis() - startingTime) >= timeOut_)
    {
      resetI2CBus();
      return(TWSR_STATUS_TIMEOUT);
    }

  }
  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))
  {
    return(TWSR_STATUS_STARTED);
  }
  if (TWI_STATUS == LOST_ARBTRTN)
  {
    resetI2CBus();
    return(TWSR_STATUS_LOSTARB);
  }
  return(TWSR_STATUS_UNKNOWN);
}

TWSR_STATUS I2C::sendAddress(uint8_t i2cAddress)
{
  TWDR = i2cAddress;
  unsigned long startingTime = millis();
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if (!timeOut_) { continue; }
    if ((millis() - startingTime) >= timeOut_)
    {
      // Time out error
      resetI2CBus();
      return(TWSR_STATUS_TIMEOUT);
    }
  }
  if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK))
  {
    // I2C comm completed successfully with ACK bit received
    return(TWSR_STATUS_ACK);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK))
  {
    // I2C comm completed, but with a NACK bit
    stop();
    return(TWSR_STATUS_NACK);
  }
  else
  {
    // Unknown error
    resetI2CBus();
    return(TWSR_STATUS_UNKNOWN);
  }
}

TWSR_STATUS I2C::sendByte(uint8_t i2cData)
{
  TWDR = i2cData;
  unsigned long startingTime = millis();
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)))
  {
    if (!timeOut_) { continue; }
    if ((millis() - startingTime) >= timeOut_)
    {
      resetI2CBus();
      return(TWSR_STATUS_TIMEOUT);
    }
  }
  if (TWI_STATUS == MT_DATA_ACK)
  {
    return(TWSR_STATUS_ACK);
  }
  uint8_t bufferedStatus = TWI_STATUS;
  if (TWI_STATUS == MT_DATA_NACK)
  {
    stop();
    return(TWSR_STATUS_NACK);
  }
  else
  {
    resetI2CBus();
    return(TWSR_STATUS_UNKNOWN);
  }
}

TWSR_STATUS I2C::receiveByte(bool sendACK)
{
  unsigned long startingTime = millis();
  if (sendACK)
  {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

  }
  else
  {
    TWCR = (1 << TWINT) | (1 << TWEN);
  }
  while (!(TWCR & (1 << TWINT)))
  {
    if (!timeOut_) { continue; }
    if ((millis() - startingTime) >= timeOut_)
    {
      resetI2CBus();
      return(TWSR_STATUS_TIMEOUT);
    }
  }
  if (TWI_STATUS == MR_DATA_ACK)
  {
    return(TWSR_STATUS_ACK);
  }
  if (TWI_STATUS == MR_DATA_NACK)
  {
    return(TWSR_STATUS_NACK);
  }
  if (TWI_STATUS == LOST_ARBTRTN)
  {
    resetI2CBus();
    return(TWSR_STATUS_LOSTARB);
  }
  return(TWSR_STATUS_UNKNOWN);
}

TWSR_STATUS I2C::stop()
{
  unsigned long startingTime = millis();
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  while ((TWCR & (1 << TWSTO)))
  {
    if (!timeOut_) { continue; }
    if ((millis() - startingTime) >= timeOut_)
    {
      resetI2CBus();
      return(TWSR_STATUS_TIMEOUT);
    }
  }
  return(TWSR_STATUS_STOPPED);
}

// Resets the I2C bus. This helps to solve a "stuck" I2C bus due to failure of arbitration.
void I2C::resetI2CBus()
{
  TWCR = 0; //releases SDA and SCL lines to high impedance

  // Re-initialize the I2C bus.
  begin(enableInternalPullUps_);
  setSpeed(useFastMode_);
}

I2C I2c = I2C();

