#pragma once

#include <memory>
#include <string>

#include <serial/serial.h>

#include "CommMsgs.pb.h"

namespace snd_serial
{

class SerialComm
{
public:
  SerialComm(const std::string& _serialPort, uint32_t _baudrate,
             uint32_t _timeout);
  ~SerialComm();
  bool isOpen();
  int sendMsg(const snd_proto::SerialRequest& _req);
  snd_proto::SerialResponse readIncomingMsg();
  void readStatus();

private:
  serial::Serial m_serial;
};

} // snd_serial namespace
