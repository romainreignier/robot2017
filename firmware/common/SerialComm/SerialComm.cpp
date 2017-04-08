#include "SerialComm.h"

#include "ch.h"

#include "Board.h"
#include "chprintf.h"
#include "pb_decode.h"
#include "pb_encode.h"

// Serial Thread function, outside of the SerialComm class
static THD_WORKING_AREA(waThreadSerial, 2048);
static THD_FUNCTION(ThreadSerial, _arg)
{
  ((SerialComm*)_arg)->serialThread();
}

// SerialComm method called by the SerialThread
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
      // We have cleared the flag so we have to read the full queue before
      // returning
      do
      {
        // Read one byte. We know that there is at least one byte because
        // we get the event
        // Reading with TIME_IMMEDIATE means that it is a non-blocking call so
        // if there
        // is no data available, Q_TIMEOUT is returned
        read = chnGetTimeout(m_driver, TIME_IMMEDIATE);
        // if we managed to get a byte
        if(read != Q_TIMEOUT)
        {
          // chprintf(dbg, "Read byte 0x%x\n", read);
          switch(state)
          {
          case WAITING_LENGTH:
            // Waiting to get the 1 byte long length of the payload
            currentMsg.length = static_cast<uint8_t>(read);
            // reset the message index
            currentIdx = 0;
            state = WAITING_PAYLOAD;
            break;
          case WAITING_PAYLOAD:
            currentMsg.payload[currentIdx] = static_cast<uint8_t>(read);
            currentIdx++;
            if(currentIdx == currentMsg.length)
            {
              // handle the received message
              // Note that the handling is done in the same thread so it is
              // blocking.
              // It is no so important because the SerialDriver fill the input
              // queue
              // for us. But be carefull to have a big enough serial queue.
              handleSerialRequest(&currentMsg);
              // Reset the state to wait the new message length
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
  // Start the serial diver.
  // Note that the pin-muxing is done in the specific board source files
  sdStart(m_driver, NULL);
  // Create Serial Thread
  chThdCreateStatic(
      waThreadSerial, sizeof(waThreadSerial), NORMALPRIO, ThreadSerial, this);
}

bool SerialComm::handleSerialRequest(const serialMsg* _msg)
{
  // Initialize a msg struct
  snd_proto_SerialRequest req = snd_proto_SerialRequest_init_default;
  // Initialize a stream
  pb_istream_t inStream = pb_istream_from_buffer(_msg->payload, _msg->length);

  // Decode the msg
  if(!pb_decode(&inStream, snd_proto_SerialRequest_fields, &req))
  {
    chprintf(dbg, "Decoding failed: %s\n", PB_GET_ERROR(&inStream));
    return false;
  }
  // Let the board handle the message now that is is decode
  return gBoard.comm.handleSerialRequest(req);
}

int SerialComm::encodeAndSendMsg(const snd_proto_SerialResponse& _resp)
{
  uint8_t txBuf[256];
  // Create the stream to encode into ProtoBuf
  pb_ostream_t outStream =
      pb_ostream_from_buffer((pb_byte_t*)txBuf, sizeof(txBuf));

  // Now we are ready to encode the message!
  if(!pb_encode(&outStream, snd_proto_SerialResponse_fields, &_resp))
  {
    chprintf(dbg, "Failed to encode the response into a protobuf\n");
    return false;
  }

  size_t bytesWritten = chnWrite(
      m_driver, reinterpret_cast<uint8_t*>(&outStream.bytes_written), 1);
  bytesWritten += chnWrite(m_driver, txBuf, outStream.bytes_written);
  if(bytesWritten != outStream.bytes_written +1)
  {
      return false;
  }
  else
  {
      return true;
  }
}
