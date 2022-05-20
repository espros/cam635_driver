/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup serial_connection Serial Connection
 * @brief Specialized serial port
 * @ingroup communication
 *
 * @{
 */
#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H


#include <vector>
#include <string>
#include <list>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>

#include "communication_constants.h"
#include "cam_signal.h"

namespace com_lib
{

  class Message
  {
    public:
      Message(const uint8_t *data, const uint32_t type)
      {
        memcpy(this->data, data, CommunicationConstants::Command::SIZE_TOTAL);
        this->type = static_cast<uint8_t>(type);
      }

      uint8_t *getData()
      {
        return data;
      }

      uint8_t getType()
      {
        return type;
      }

    private:
      uint8_t data[CommunicationConstants::Command::SIZE_TOTAL];
      uint8_t type;
  };


//! Specialized implementation of serial port
/*!
 * This class implements some specific functionality for the communication into the
 * base of serial port
 */
class SerialConnection
{    

public:
    SerialConnection();
    ~SerialConnection();

    bool openPort(std::string &portName);
    void closePort();

    std::vector<std::string> availableDevices();
    ssize_t sendData(uint8_t *data);
    bool sendCommand(uint8_t *data, uint8_t expectedType, bool blocking);
    void addGeneralAnswerType(const uint8_t type);
    int readRxData(int size); //slot    
    bool processData(std::vector<uint8_t> array);
    std::vector<uint8_t> rxArray;

    //signals:
    Gallant::Signal2<std::vector<uint8_t>&, uint8_t> sigReceivedData;

  private:
    ssize_t sendCommandInternal(uint8_t *data, uint8_t expectedType_);
    uint32_t calculateChecksum(const uint8_t *data, const uint32_t size);

    int getExpextedSize(const std::vector<uint8_t> &array);
    bool checksumIsCorrect(const std::vector<uint8_t> &array, const unsigned int expectedSize);

    uint8_t getType(const std::vector<uint8_t> &array);
    void setBlocking (int should_block);
    int setInterfaceAttribs (int speed);

    std::vector<std::string> deviceListString;   
    int expectedSize;    
    int fileDescription;

    bool waitForSpecificDataType;
    uint8_t expectedType;
    std::queue<Message> queue;
    std::list<uint8_t> generalAnswerTypes;

};
}

#endif // SERIAL_CONNECTION_H

/** @} */
