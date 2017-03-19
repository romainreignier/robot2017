#include "SerialComm.h"

#include "ch.h"

#include "Board.h"
#include "chprintf.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define HANDLE_MSG(msg)                                                        \
  case snd_msgs_SerialRequest_##msg##_tag:                                     \
  {                                                                            \
    chprintf(dbg, #msg " request received.\n");                                \
    msg(req);                                                                  \
    break;                                                                     \
  }

// Serial Event Thread
static THD_WORKING_AREA(waThreadSerial, 2048);
static THD_FUNCTION(ThreadSerial, _arg)
{
  ((SerialComm*)_arg)->serialThread();
}

void SerialComm::serialThread()
{
  chRegSetThreadName("SerialListener");

  event_listener_t elSerial;
  eventmask_t flags;
  enum eState
  {
    WAITING_LENGTH,
    WAITING_PAYLOAD,
  };
  eState state = WAITING_LENGTH;
  serialMsg currentMsg;
  uint8_t currentIdx = 0;

  chEvtRegisterMask((event_source_t*)chnGetEventSource(m_driver), &elSerial, 1);
  while(TRUE)
  {
    // Wait for an event from the serial driver
    chEvtWaitOne(1);

    // Clear the event flags
    chSysLock();
    flags = chEvtGetAndClearFlagsI(&elSerial);
    chSysUnlock();

    // if the event means that data is available
    if(flags & CHN_INPUT_AVAILABLE)
    {
      msg_t read;
      // We have cleared the flag so we have to read the full queue
      do
      {
        // Read one byte. We know that there is at least one byte because
        // we get the event
        read = chnGetTimeout(m_driver, TIME_IMMEDIATE);
        // if we managed to get a byte
        if(read != Q_TIMEOUT)
        {
          // chprintf(dbg, "Read byte 0x%x\n", read);
          switch(state)
          {
          case WAITING_LENGTH:
            // Waiting to get the 1 byte long length of the payload
            currentMsg.length = read;
            currentIdx = 0;
            state = WAITING_PAYLOAD;
            break;
          case WAITING_PAYLOAD:
            currentMsg.payload[currentIdx] = read;
            currentIdx++;
            if(currentIdx == currentMsg.length)
            {
              // Send the received buffer to the mailbox
              handleSerialRequest(&currentMsg);
              state = WAITING_LENGTH;
            }
            break;
          }
        }
      } while(read != Q_TIMEOUT);
    }
  }
}

SerialComm::SerialComm(SerialDriver* _driver) : m_driver(_driver)
{
}

void SerialComm::start()
{
  sdStart(m_driver, NULL);
  // Create Serial Thread
  chThdCreateStatic(
      waThreadSerial, sizeof(waThreadSerial), NORMALPRIO, ThreadSerial, this);
}

bool SerialComm::handleSerialRequest(const serialMsg* _msg)
{
  // Initialize a msg struct
  snd_msgs_SerialRequest req = snd_msgs_SerialRequest_init_default;
  // Initialize a stream
  pb_istream_t inStream = pb_istream_from_buffer(_msg->payload, _msg->length);

  // Decode the msg
  if(!pb_decode(&inStream, snd_msgs_SerialRequest_fields, &req))
  {
    chprintf(dbg, "Decoding failed: %s\n", PB_GET_ERROR(&inStream));
    return false;
  }

  // Process the action
  switch(req.which_type)
  {
  /*
      HANDLE_MSG(getEncoders)
      HANDLE_MSG(getPosition)
      HANDLE_MSG(getMotorsSpeed)
      HANDLE_MSG(getPidSpeedLeft)
      HANDLE_MSG(getPidSpeedRight)
      HANDLE_MSG(setMotorsSpeed)
      HANDLE_MSG(setPidSpeedLeft)
      HANDLE_MSG(setPidSpeedRight)
      HANDLE_MSG(setEncoders)
  */
  case snd_msgs_SerialRequest_getEncoders_tag:
    chprintf(dbg, "Get Encoders request received.\n");
    break;
  case snd_msgs_SerialRequest_setMotorsSpeed_tag:
  {
    chprintf(dbg, "SetMotorsSpeed request received ");
    chprintf(dbg,
             "left: %f right: %f\n",
             req.type.setMotorsSpeed.left,
             req.type.setMotorsSpeed.right);
    const int16_t leftPercent = static_cast<int16_t>(
        req.type.setMotorsSpeed.left * gBoard.leftMotor.maxPwm / 1.0f);
    const int16_t rightPercent = static_cast<int16_t>(
        req.type.setMotorsSpeed.right * gBoard.rightMotor.maxPwm / 1.0f);
    gBoard.leftMotor.pwm(leftPercent);
    gBoard.rightMotor.pwm(rightPercent);
    break;
  }
  case snd_msgs_SerialRequest_getStatus_tag:
  {
    // chprintf(dbg, "Get Status request received.\n");
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
    uint8_t txBuf[256];
    // Create the stream to encode into ProtoBuf
    pb_ostream_t outStream =
        pb_ostream_from_buffer((pb_byte_t*)txBuf, sizeof(txBuf));

    // Now we are ready to encode the message!
    pb_encode(&outStream, snd_msgs_SerialResponse_fields, &resp);
    chnWrite(m_driver, reinterpret_cast<uint8_t*>(&outStream.bytes_written), 1);
    chnWrite(m_driver, txBuf, outStream.bytes_written);

    // encodeAndSendMsg(resp);
    break;
  }
  default: chprintf(dbg, "message type not handled\n");
  }
  return true;
}

int SerialComm::sendMsg(uint8_t* _msg, unsigned int _msgLength)
{
  chnWrite(m_driver, reinterpret_cast<uint8_t*>(&_msgLength), 1);
  chnWrite(m_driver, _msg, _msgLength);
  chprintf(dbg, "Send response of %u bytes.\n", _msgLength);
  return 0;
}

unsigned int SerialComm::encodeMsg(const snd_msgs_SerialResponse& _resp,
                                   uint8_t* _outMsg)
{
  // Create the stream to encode into ProtoBuf
  pb_ostream_t stream =
      pb_ostream_from_buffer((pb_byte_t*)_outMsg, sizeof(_outMsg));

  // Now we are ready to encode the message!
  pb_encode(&stream, snd_msgs_SerialResponse_fields, &_resp);

  /*
  chprintf(dbg, "Protobuf of %lu : ", stream.bytes_written);
  for(size_t i = 0; i < stream.bytes_written; ++i)
  {
    chprintf(dbg, "0x%x ", _outMsg[i]);
  }
  chprintf(dbg, "\n");
  */
  return stream.bytes_written;
}

int SerialComm::encodeAndSendMsg(const snd_msgs_SerialResponse& _resp)
{
  uint8_t buffer[TX_MAX_SIZE];
  unsigned int length = encodeMsg(_resp, buffer);
  return sendMsg(buffer, length);
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

void SerialComm::getEncoders(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
  sendEncoders(-1000, 1000);
}

void SerialComm::getPosition(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
  sendPosition(1, 2, 3);
}

void SerialComm::getMotorsSpeed(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
}

void SerialComm::getPidSpeedLeft(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
}

void SerialComm::getPidSpeedRight(const snd_msgs_SerialRequest& _req)
{
  (void)_req;
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
