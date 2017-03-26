#pragma once

/**
  \file Communication.h
  \author Romain Reignier
  \date 26/03/2017
  \brief The handling of the messages received through the SerialComm object is
  done inside the board specific source tree in order to share the code among
  different boards.
 */

#include "CommMsgs.pb.h"

class Communication
{
public:
  bool handleSerialRequest(const snd_msgs_SerialRequest& _req);
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
};
