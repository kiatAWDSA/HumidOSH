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

#include "SerialCommunication.h"

SerialCommunication::SerialCommunication() {}

void SerialCommunication::init(unsigned long baudRate)
{
  Serial.begin(baudRate);
  serialActive_ = true;
}

void SerialCommunication::enableSending()
{
  serialActive_ = true;
}

void SerialCommunication::disableSending()
{
  serialActive_ = false;
}

bool SerialCommunication::processIncoming()
{
  // All communication must start with SERIAL_CMD_START
  if (Serial.peek() == SERIAL_CMD_START)
  {
    // Extract this command string
    memset(serialBuffer, '\0', serialBufferLength);
    Serial.readBytesUntil(SERIAL_CMD_EOL, serialBuffer, (serialBufferLength - 1));

    // Get the pointer to SERIAL_CMD_END 
    char* endFlag = strchr(serialBuffer, SERIAL_CMD_END);

    // All communication must end with SERIAL_CMD_END 
    // Check if it isn't missing and that there's nothing coming after it
    if (endFlag && serialBuffer[endFlag - serialBuffer + 1] == '\0')
    {
      uint8_t endPos = endFlag - serialBuffer;// Index of SERIAL_CMD_END

      // Remove the start and end flags, so we are only left with the inner string
      // First, clear the processing buffer
      memset(trimmedSerialBuffer, '\0', serialBufferLength);

      // inputBuffer + 1 to start after SERIAL_CMD_START
      // endPos - 1 because array is indexed 0, so only minus 1 for one of the start/end flags
      strncpy(trimmedSerialBuffer, serialBuffer + 1, endPos - 1);

      
      // Extract first fragment
      uint8_t fragmentsExtracted = 0;
      char *tokenPtr = strtok(trimmedSerialBuffer, &SERIAL_CMD_SEPARATOR);

      if (tokenPtr)
      {// There was at least one command fragment
        // The first (or maybe only)  is always the command itself. Based on the command,
        // we can expect the number of parameters that is associated with it
        uint8_t paramsCount;

        switch (tokenPtr[0])
        {
          case SERIAL_CMD_DAQ_START:
            paramsCount = MAXPARAM_DAQ_START;
            break;
          case SERIAL_CMD_DAQ_STOP:
            paramsCount = MAXPARAM_DAQ_STOP;
            break;
          default:
            // Unknown command, stop processing
            return(false);
        }

        do
        {// Process the extracted fragment. Notice this is a do-while loop.
          fragmentsExtracted++;

          // Clear the output buffer
          memset(fragmentBuffer[fragmentsExtracted - 1], '\0', fragmentBufferLength);
          strcpy(fragmentBuffer[fragmentsExtracted - 1], tokenPtr); // IMPORTANT: Make sure that fragments[fragmentsExtracted - 1] is a char array with at least as many elements as tokenPtr
          tokenPtr = strtok(NULL, &SERIAL_CMD_SEPARATOR);
          // Repeat the loop while we haven't exceeded the max number of params for this command and while there are more params available
        } while (fragmentsExtracted - 1 < paramsCount && tokenPtr);

        // Check if the number of extracted command params are as expected
        // (because we might exit the while loop above due to tokenPtr being null)
        // Minus one to account for the command itself being one of the fragments
        if (fragmentsExtracted - 1 < paramsCount)
        {
          // Not enough params were extracted
          return false;
        }

        // Check if there are still params remaining
        // (because we might exit the while loop above due to fragmentsExtracted >= paramsCount)
        if (tokenPtr)
        {
          // Too many params command params were received, though not all were extracted
          return false;
        }

        // Extracted command and associated params, all good to go.
        return true;
      }
      else
      {
        // Not a single fragment is available
        return false;
      }  
    }
    else
    {// SERIAL_CMD_END wasn't seen or in wrong location, so assume this is garbage
      return false;
    }
  }
  else
  {// SERIAL_CMD_START wasn't seen, so assume this is garbage
    // Grab the buffer, but don't store it anywhere. This helps to clear the buffer
    Serial.read();
    return false;
  }
}

int SerialCommunication::getFragmentInt(uint8_t fragmentIndex)
{
  return atoi(fragmentBuffer[fragmentIndex]);
}

char SerialCommunication::getFragmentChar(uint8_t fragmentIndex)
{
  char* fragmentEntry = fragmentBuffer[fragmentIndex];

  // Return only the first character
  return fragmentEntry[0];
}

double SerialCommunication::getFragmentDouble(uint8_t fragmentIndex)
{
  return atof(fragmentBuffer[fragmentIndex]);
}

unsigned long SerialCommunication::getFragmentULong(uint8_t fragmentIndex)
{
  // This relies on the fragment not being larger than max of long and not being negative.
  return (unsigned long) atol(fragmentBuffer[fragmentIndex]);
}

void SerialCommunication::sendData(bool humidityOK, double humidity, double temperature, bool fanSpeedOK, double fanSpeed, bool humidityControlActive, double humidityTarget, bool fanSpeedControlActive, double fanSpeedTarget)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_DATA);
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (humidityOK)
  {
    Serial.print(humidity, DECIMALS_HUMIDITY);  // Relative humidity (%)
  }
  else
  {
    Serial.print(SERIAL_SEND_DATA_ERROR);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (humidityOK)
  {
    Serial.print(temperature, DECIMALS_TEMPERATURE);  // Temperature (°C)
  }
  else
  {
    Serial.print(SERIAL_SEND_DATA_ERROR);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (fanSpeedOK)
  {
    Serial.print(fanSpeed, DECIMALS_FANSPEED);  // Fan speed (RPM)
  }
  else
  {
    Serial.print(SERIAL_SEND_DATA_ERROR);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (humidityControlActive)
  {
    Serial.print(humidityTarget, DECIMALS_HUMIDITY);  // Relative humidity target (%)
  }
  else
  {
    Serial.print(SERIAL_SEND_DATA_CONTROLINACTIVE);
  }
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (fanSpeedControlActive)
  {
    Serial.print(fanSpeedTarget, DECIMALS_FANSPEED);  // Fan speed target (%)
  }
  else
  {
    Serial.print(SERIAL_SEND_DATA_CONTROLINACTIVE);
  }
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}

// Inform C# program on the status of a command for a specific chamber
void SerialCommunication::sendCommandResponse(char commandType, bool success)
{
  Serial.print(SERIAL_SEND_START);
  Serial.print(SERIAL_SEND_CMDRESPONSE);
  Serial.print(SERIAL_SEND_SEPARATOR);
  Serial.print(commandType);
  Serial.print(SERIAL_SEND_SEPARATOR);
  if (success)
  {
    Serial.print(SERIAL_SEND_CMDRESPONSE_SUCC);
  }
  else
  {
    Serial.print(SERIAL_SEND_CMDRESPONSE_FAIL);
  }
  Serial.print(SERIAL_SEND_END);
  Serial.print(SERIAL_SEND_EOL);
}