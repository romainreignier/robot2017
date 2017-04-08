#pragma once

#include "hal.h"

#include "CommMsgs.pb.h"

class SerialComm
{
public:
  static constexpr size_t TX_MAX_SIZE = 256;
  static constexpr size_t RX_MAX_SIZE = 256;
  struct serialMsg
  {
    uint8_t length;
    uint8_t payload[RX_MAX_SIZE];
  };

  SerialComm(SerialDriver* _driver);
  void start();
  bool handleSerialRequest(const serialMsg* _msg);
  void serialThread();

  unsigned int processSerial();
  int sendMsg(uint8_t* _msg, unsigned int _msgLength);
  unsigned int encodeMsg(const snd_proto_SerialResponse& _resp, uint8_t* _msg);
  int encodeAndSendMsg(const snd_proto_SerialResponse& _resp);

private:
  SerialDriver* m_driver;
};
