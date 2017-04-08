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
  bool handleSerialRequest(const snd_proto_SerialRequest& _req);
protected:
  void getStatus(const snd_proto_SerialRequest& _req);
  void setMotorsSpeed(const snd_proto_SerialRequest& _req);
  void setPidSpeedLeft(const snd_proto_SerialRequest& _req);
  void setPidSpeedRight(const snd_proto_SerialRequest& _req);
};
