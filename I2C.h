/*
I2C.h   - I2C library
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

Modified by Soon Kiat Lau (2019):
- massive revamp to incorporate error codes, improve reuseability of functions and enhance human readability
- ping function
- added additional read/write functions to ignore the read/write register
- allow user to enable/disable internal pull-up resistors in begin()
*/

#ifndef _I2C_h
#define _I2C_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <inttypes.h>

#ifndef I2C_h
#define I2C_h

#define START           0x08
#define REPEATED_START  0x10

// See https://www.microchip.com/webdoc/AVRLibcReferenceManual/ch20s33s01.html for status code definitions.
// The "T" in MT stands for transmit. The "R" in MR stands for receive.
// SLA indicates we are sending/receiving the address byte. So MT_SLA_*** are statuses returned when sending out addresses to find a slave device.
// DATA indicates we are sending/receiving the data byte(s). So MT_DATA_*** are statuses returned when sending out data to a slave device.
#define MT_SLA_ACK	    0x18  // SLA+W transmitted, ACK received
#define MT_SLA_NACK	    0x20  // SLA+W transmitted, NACK received
#define MT_DATA_ACK     0x28  // data transmitted, ACK received
#define MT_DATA_NACK    0x30  // data transmitted, NACK received
#define MR_SLA_ACK	    0x40  // SLA+R transmitted, ACK received
#define MR_SLA_NACK	    0x48  // SLA+R transmitted, NACK received
#define MR_DATA_ACK     0x50  // data received, ACK returned
#define MR_DATA_NACK    0x58  // data received, NACK returned
#define LOST_ARBTRTN    0x38  // arbitration lost in (SLA+W or data), or (SLA+R or NACK)
#define TWI_STATUS      (TWSR & 0xF8)
#define SLA_W(address)  (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)
#define cbi(sfr, bit)   (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MAX_BUFFER_SIZE 32

// Status of I2C communication.
typedef enum
{
  I2C_STATUS_OK               = 0,  // No problemo
  I2C_STATUS_START_TIMEOUT    = 1,  // I2C timeout while attempting to start comm on the I2C bus.
  I2C_STATUS_REPSTART_TIMEOUT = 2,  // I2C timeout while attempting to perform a repeated start comm on the I2C bus.
  I2C_STATUS_BEGIN_TIMEOUT    = 3,  // I2C timeout while waiting for an ACK/NACK bit after sending out the byte to begin transmission. This could indicate no devices have the address, or the targeted device somehow did not interpret the signals correctly.
  I2C_STATUS_BEGIN_NACK       = 4,  // A NACK bit was received after sending out the byte to begin transmission. Some devices do this to indicate they are busy or not ready for communication.
  I2C_STATUS_BEGIN_LOSTARB    = 5,  // Lost arbitration of the I2C bus while attempting to begin transmission.
  I2C_STATUS_TRS_TIMEOUT      = 6,  // I2C timeout while waiting for ACK/NACK while sending data to the target device.
  I2C_STATUS_TRS_NACK         = 7,  // Received a NACK after sending a data byte to the target device.
  I2C_STATUS_REC_TIMEOUT      = 8,  // I2C timeout while waiting for ACK/NACK while receiving data from the target device.
  I2C_STATUS_REC_LOSTARB      = 9,  // Lost arbitration of the I2C bus while attempting to receive a data byte.
  I2C_STATUS_REC_ACKBUTNACK   = 10, // A NACK bit was received even though we were expecting an ACK bit while receiving data. This could happen when the target device wants to end the communication.
  I2C_STATUS_REC_NACKBUTACK   = 11, // An ACK bit was received even though we were expecting a NACK bit while receiving data. This could happen when we requested too few bytes and the target device still wants to send more data.
  I2C_STATUS_STOP_TIMEOUT     = 12, // I2C timeout while attempting to stop comm on the I2C bus.
  I2C_STATUS_UNKNOWN          = 99  // Unknown or yet to be defined error.
} I2C_STATUS;


// Status of the I2C communication, as read from the TWSR register.
typedef enum
{
  TWSR_STATUS_STARTED     = 0,  // I2C comm successfully started
  TWSR_STATUS_STOPPED     = 1,  // I2C comm successfully stopped
  TWSR_STATUS_ACK         = 2,  // I2C comm completed with an ACK bit
  TWSR_STATUS_TIMEOUT     = 3,  // I2C comm failed because it took too long to receive a complete response
  TWSR_STATUS_NACK        = 4,  // I2C comm completed with a NACK bit
  TWSR_STATUS_LOSTARB     = 5,  // The arbitration of the I2C bus has failed, and the bus will be reset. See https://www.i2c-bus.org/i2c-primer/analysing-obscure-problems/master-reports-arbitration-lost/ or https://community.nxp.com/thread/348665
  TWSR_STATUS_UNKNOWN     = 99  // An unknown condition was seen on the TSWR register
} TWSR_STATUS;

class I2C
{
public:
  I2C();
  void begin(bool enableInternalPullUps);
  void end();
  void setTimeOut(uint16_t timeOut);
  void setSpeed(bool useFastMode);
  void pullup(bool activate);
  void scan();

  // Functions for obtaining the received data.
  uint8_t available();
  uint8_t getByte();

  // This function simply sends a write request to the addressed device without any additional bytes. This is sometimes used to trigger a device.
  I2C_STATUS ping(uint8_t address);

  // The following are the write and read functions. The "registerAddress" is there to follow the expected I2C standard, whereby immediately after a write/read
  // command byte, the master is expected to specify which register should be written on/read from. However, there are many devices that do not follow this
  // standard strictly, and sometimes a "command" is sent instead of the address of the register. Still, the "registerAddress" variable is simply a byte
  // that is sent out right after the write/read byte, therefore it can be replaced with any byte such as a command byte or whatever your device needs to send
  // immediately after the write/read byte. The "data" is simply the third byte in the whole command train. Any additional bytes can be stuffed into an array
  // and sent out with the function using *data.
  //
  // This is how the entire command train looks like:
  // [address+read/write byte] -> ["registerAddress" byte] -> [data (if available) OR *data (sends out one byte for each array entry)]
  //
  // Alternatively, the write functions without registerAddress can be used. They are functionally similar, but maybe less confusing.
  //
  I2C_STATUS write(uint8_t  address,  uint8_t registerAddress);
  I2C_STATUS write(int      address,  int     registerAddress);
  I2C_STATUS write(uint8_t  address,  uint8_t registerAddress,  uint8_t data);
  I2C_STATUS write(int      address,  int     registerAddress,  int     data);
  I2C_STATUS write(uint8_t  address,  uint8_t registerAddress,  uint8_t *data,        uint8_t numberBytes);
  I2C_STATUS write(uint8_t  address,  uint8_t registerAddress,  char    *data,        uint8_t numberBytes);
  I2C_STATUS write(uint8_t  address,  uint8_t *data,            uint8_t numberBytes);
  I2C_STATUS write(int      address,  char    *data,            uint8_t numberBytes);

  // These read functions send out the address byte with a read bit, and get the incoming data.
  I2C_STATUS read(uint8_t   address,  uint8_t numberBytes,      uint8_t *dataBuffer);
  I2C_STATUS read(uint8_t   address,  uint8_t numberBytes);
  I2C_STATUS read(int       address,  int     numberBytes);

  // These read functions first write onto a register on the target device, then utilize repeated start to begin obtaining incoming data.
  I2C_STATUS read(uint8_t   address,  uint8_t registerAddress,  uint8_t numberBytes,  uint8_t *dataBuffer);
  I2C_STATUS read(uint8_t   address,  uint8_t registerAddress,  uint8_t numberBytes);
  I2C_STATUS read(int       address,  int     registerAddress,  int     numberBytes);

  // Abstractions of some of the private functions to return the appropriate I2C_STATUS.
  // Use the write/read functions instead of these functions unless you really have to.
  I2C_STATUS beginTransmission(uint8_t address, bool write, bool repeatedStart);
  I2C_STATUS transmit(uint8_t dataByte);
  I2C_STATUS receive(bool sendACK);
  I2C_STATUS endTransmission();

private:
  TWSR_STATUS start();
  TWSR_STATUS sendAddress(uint8_t address);
  TWSR_STATUS sendByte(uint8_t byte);
  TWSR_STATUS receiveByte(bool sendACK);
  TWSR_STATUS stop();
  void resetI2CBus();

  TWSR_STATUS TWSRStatus_;
  I2C_STATUS returnStatus_;
  uint8_t nack_;
  uint8_t data_[MAX_BUFFER_SIZE];
  static uint8_t bytesAvailable_;
  static uint8_t bufferIndex_;
  static uint8_t totalBytes_;

  // Configuration of the I2C bus.
  bool enableInternalPullUps_;
  bool useFastMode_;
  uint16_t timeOut_;
};

extern I2C I2c;

#endif

#endif

