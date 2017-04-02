#include "Board.h"

#include "Communication.h"

#include "chprintf.h"

bool Communication::handleSerialRequest(const snd_msgs_SerialRequest& _req)
{
  bool ret{true};
  // Process the action
  switch(_req.which_type)
  {
  case snd_msgs_SerialRequest_setMotorsSpeed_tag: setMotorsSpeed(_req); break;
  case snd_msgs_SerialRequest_setPidSpeedLeft_tag: setPidSpeedLeft(_req); break;
  case snd_msgs_SerialRequest_setPidSpeedRight_tag:
    setPidSpeedRight(_req);
    break;
  case snd_msgs_SerialRequest_getStatus_tag: getStatus(_req); break;
  default: chprintf(dbg, "message type not handled\n"); return false;
  }
  return ret;
}

void Communication::getStatus(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
  snd_msgs_SerialResponse resp = snd_msgs_SerialResponse_init_zero;
  resp.which_type = snd_msgs_SerialResponse_status_tag;
  resp.type.status.speed.left = 8.16f;
  resp.type.status.speed.right = 16.32f;
  resp.type.status.starter = gBoard.starter.read();
  resp.type.status.estop = gBoard.starter.read();
  resp.type.status.colorSwitch = gBoard.colorSwitch.read()
                                   ? snd_msgs_eTeamColor_BLUE
                                   : snd_msgs_eTeamColor_YELLOW;
  resp.type.status.ir.left = false;
  resp.type.status.ir.center = true;
  resp.type.status.ir.right = true;
  int32_t left, right;
  gBoard.qei.getValues(&left, &right);
  resp.type.status.encoders.left = left;
  resp.type.status.encoders.right = right;
  gBoard.serial.encodeAndSendMsg(resp);
}

void Communication::setMotorsSpeed(const snd_msgs_SerialRequest& _req)
{
  chprintf(dbg, "SetMotorsSpeed request received ");
  chprintf(dbg,
           "left: %f right: %f\n",
           _req.type.setMotorsSpeed.left,
           _req.type.setMotorsSpeed.right);
  const int16_t leftPercent = static_cast<int16_t>(
    _req.type.setMotorsSpeed.left * 0.75f * gBoard.leftMotor.kPwmPeriod / 1.47f);
  const int16_t rightPercent = static_cast<int16_t>(
    _req.type.setMotorsSpeed.right * 0.75f * gBoard.rightMotor.kPwmPeriod / 1.47f);
  gBoard.motors.pwm(leftPercent, rightPercent);
}

void Communication::setPidSpeedLeft(const snd_msgs_SerialRequest& _req)
{
  chprintf(dbg,
           "SetPidSpeedLeft request received: P: %.4f I: %.4f  D: %.4f\n",
           _req.type.setPidSpeedLeft.p,
           _req.type.setPidSpeedLeft.i,
           _req.type.setPidSpeedLeft.d);
}

void Communication::setPidSpeedRight(const snd_msgs_SerialRequest& _req)
{
  chprintf(dbg,
           "SetPidSpeedRight request received: P: %.4f I: %.4f  D: %.4f\n",
           _req.type.setPidSpeedRight.p,
           _req.type.setPidSpeedRight.i,
           _req.type.setPidSpeedRight.d);
}
