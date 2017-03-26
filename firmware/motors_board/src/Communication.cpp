#include "Board.h"

#include "Communication.h"

#include "chprintf.h"

#define HANDLE_MSG(msg)                                                        \
  case snd_msgs_SerialRequest_##msg##_tag:                                     \
  {                                                                            \
    chprintf(dbg, #msg " request received.\n");                                \
    msg(req);                                                                  \
    break;                                                                     \
  }

bool Communication::handleSerialRequest(const snd_msgs_SerialRequest& _req)
{
    bool ret{true};
  // Process the action
  switch(_req.which_type)
  {
  /*
      HANDLE_MSG(setMotorsSpeed)
      HANDLE_MSG(setPidSpeedLeft)
      HANDLE_MSG(setPidSpeedRight)
      HANDLE_MSG(setEncoders)
  */
  case snd_msgs_SerialRequest_setMotorsSpeed_tag:
  {
    chprintf(dbg, "SetMotorsSpeed request received ");
    chprintf(dbg,
             "left: %f right: %f\n",
             _req.type.setMotorsSpeed.left,
             _req.type.setMotorsSpeed.right);
    const int16_t leftPercent = static_cast<int16_t>(
      _req.type.setMotorsSpeed.left * 0.75 * gBoard.leftMotor.maxPwm / 1.47f);
    const int16_t rightPercent = static_cast<int16_t>(
      _req.type.setMotorsSpeed.right * 0.75 * gBoard.rightMotor.maxPwm / 1.47f);
    gBoard.leftMotor.pwm(leftPercent);
    gBoard.rightMotor.pwm(rightPercent);
    break;
  }
  case snd_msgs_SerialRequest_getStatus_tag:
  {
    snd_msgs_SerialResponse resp = snd_msgs_SerialResponse_init_zero;
    resp.which_type = snd_msgs_SerialResponse_status_tag;
    resp.type.status.pose.x = 1.2f;
    resp.type.status.pose.y = 2.4f;
    resp.type.status.pose.th = 4.8f;
    resp.type.status.speed.left = 8.16f;
    resp.type.status.speed.right = 16.32f;
    resp.type.status.starter = true;
    resp.type.status.estop = false;
    resp.type.status.ir.left = false;
    resp.type.status.ir.center = true;
    resp.type.status.ir.right = true;
    int32_t left, right;
    gBoard.qei.getValues(&left, &right);
    resp.type.status.encoders.left = left;
    resp.type.status.encoders.right = right;
    ret = gBoard.serial.encodeAndSendMsg(resp);
    break;
  }
  default: chprintf(dbg, "message type not handled\n");
  }
  return ret;
}

/*
void SerialComm::sendLog(snd_msgs_Log_eLogLevel _level, char *_msg)
{
  (void)_msg;
  snd_msgs_SerialResponse resp = snd_msgs_SerialResponse_init_zero;
  resp.type.log.level = _level;
  resp.type.log.stamp = osalOsGetSystemTimeX();
  // strcpy _msp in resp resp.type.log.text
  // encodeAndSendMsg(resp);
}

void SerialComm::setMotorsSpeed(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
  float speedLeft = _req.type.setMotorsSpeed.left;
  float speedRight = _req.type.setMotorsSpeed.right;
  chprintf(dbg, "Change motors speed for %f %f\n", speedLeft, speedRight);
}

void SerialComm::setPidSpeedLeft(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
}

void SerialComm::setPidSpeedRight(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
}

void SerialComm::setEncoders(const snd_msgs_SerialRequest& _req)
{
    (void)_req;
}

void SerialComm::sendEncoders(int32_t _left, int32_t _right)
{
  snd_msgs_SerialResponse resp = snd_msgs_SerialResponse_init_zero;
  resp.type.encoders.left = _left;
  resp.type.encoders.right = _right;
  resp.which_type = snd_msgs_SerialResponse_encoders_tag;
  encodeAndSendMsg(resp);
}

void SerialComm::sendPosition(float _x, float _y, float _th)
{
  snd_msgs_SerialResponse resp = snd_msgs_SerialResponse_init_zero;
  resp.type.pose.x = _x;
  resp.type.pose.y = _y;
  resp.type.pose.th = _th;
  resp.which_type = snd_msgs_SerialResponse_pose_tag;
  encodeAndSendMsg(resp);
}
*/
