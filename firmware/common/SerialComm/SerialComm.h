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
  unsigned int encodeMsg(const snd_msgs_SerialResponse& _resp, uint8_t* _msg);
  int encodeAndSendMsg(const snd_msgs_SerialResponse& _resp);
  /*
  void sendLog(snd_msgs_Log_eLogLevel _level, char* _msg);
  void getEncoders(const snd_msgs_SerialRequest& _req);
  void getPosition(const snd_msgs_SerialRequest& _req);
  void getMotorsSpeed(const snd_msgs_SerialRequest& _req);
  void getPidSpeedLeft(const snd_msgs_SerialRequest& _req);
  void getPidSpeedRight(const snd_msgs_SerialRequest& _req);
  void setMotorsSpeed(const snd_msgs_SerialRequest& _req);
  void setPidSpeedLeft(const snd_msgs_SerialRequest& _req);
  void setPidSpeedRight(const snd_msgs_SerialRequest& _req);
  void setEncoders(const snd_msgs_SerialRequest& _req);
  void sendEncoders(int32_t _left, int32_t _right);
  void sendPosition(float _x, float _y, float _th);
  */
  private:
  SerialDriver* m_driver;
};
