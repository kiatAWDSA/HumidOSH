/**********************************************************************
Copyright 2018 Soon Kiat Lau

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http ://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef _SERIALCOMMUNICATION_h
#define _SERIALCOMMUNICATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class SerialCommunication
{
  public:
    // Communication standards
    static const char SERIAL_CMD_START            = '^';
    static const char SERIAL_CMD_DAQ_START        = 'd';
    static const char SERIAL_CMD_DAQ_STOP         = 's';
    const char SERIAL_CMD_SEPARATOR               = '|';
    static const char SERIAL_CMD_END              = '@';
    static const char SERIAL_CMD_EOL              = '\n';

    static const char SERIAL_SEND_START                     = '^';
    static const char SERIAL_SEND_DATA                      = 'd';
    static const char SERIAL_SEND_DATA_ERROR                = 'e';
    static const char SERIAL_SEND_DATA_CONTROLINACTIVE      = 'i';
    static const char SERIAL_SEND_CMDRESPONSE               = 'r';  // Used to indicate execution status of a received command
      static const char SERIAL_SEND_CMDRESPONSE_SUCC        = 'y';  // Success
      static const char SERIAL_SEND_CMDRESPONSE_FAIL        = 'n';  // Failed
    static const char SERIAL_SEND_SEPARATOR               = '|';
    static const char SERIAL_SEND_END                     = '@';
    static const char SERIAL_SEND_EOL                     = '\n';


    SerialCommunication();
    void init(unsigned long baudRate);

    // Enable/disable sending to computer
    void enableSending();
    void disableSending();

    // Process incoming strings into fragments
    bool processIncoming();

    // Functions for obtaining the extracted fragments from an incoming string
    int           getFragmentInt(uint8_t fragmentIndex);
    char          getFragmentChar(uint8_t fragmentIndex);
    double        getFragmentDouble(uint8_t fragmentIndex);
    unsigned long getFragmentULong(uint8_t fragmentIndex);

    // Functions for sending strings to computer
    void sendData(bool humidityOK, double humidity, double temperature, bool fanSpeedOK, double fanSpeed, bool humidityControlActive, double humidityTarget, bool fanSpeedControlActive, double fanSpeedTarget);
    void sendCommandResponse(char commandType, bool success);


  private:
    // Number of parameters in every command sent by computer
    const uint8_t MAXPARAM_DAQ_START    = 0;
    const uint8_t MAXPARAM_DAQ_STOP     = 0;

    // Decimal places for data sent to computer
    const uint8_t DECIMALS_HUMIDITY     = 1; // Number of decimal places allowed for humidity.
    const uint8_t DECIMALS_TEMPERATURE  = 1; // Number of decimal places allowed for temperature.
    const uint8_t DECIMALS_FANSPEED     = 0; // Number of decimal places allowed for fan speed.

    bool serialActive_;
    // Serial communication and buffers
    // Must manually define and add fragmentBufferElement to the char pointer array fragmentBuffer, because the arrays points
    // to char arrays which must be defined.
    static const uint8_t serialBufferLength   = 128;  // Should be always sufficient for command strings sent by the C# program
    static const uint8_t fragmentMaxCount     = 1;    // Max possible of fragments in a command sent by computer.
    static const uint8_t fragmentBufferLength = 20;   // Should be always sufficient for fragments in command strings sent by the C# program
    char serialBuffer[serialBufferLength];
    char fragmentBufferElement0[fragmentBufferLength];
    // This array of char array pointers will be temp storage for fragments extracted from incoming command lines
    // Only parts of the buffer are used for each command; see the constants defined above for MAXPARAM
    char* fragmentBuffer[fragmentMaxCount] = {fragmentBufferElement0};
    char trimmedSerialBuffer[serialBufferLength]; // For holding the unextracted fragments portion of the command string
};

#endif