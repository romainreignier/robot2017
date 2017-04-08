#include <iostream>

#include "snd_serial/SerialComm.h"

namespace snd_serial
{

SerialComm::SerialComm(const std::string& _serialPort, uint32_t _baudrate,
                       uint32_t _timeout)
  : m_serial(_serialPort, _baudrate, serial::Timeout::simpleTimeout(_timeout))
{
  // Check Protobuf version
  GOOGLE_PROTOBUF_VERIFY_VERSION;
}

SerialComm::~SerialComm()
{
  google::protobuf::ShutdownProtobufLibrary();
}

bool SerialComm::isOpen()
{
  return m_serial.isOpen();
}

void printBuffer(const uint8_t* _buffer, size_t _length)
{
  for(size_t i = 0; i < _length; ++i)
  {
    std::cout << "0x" << std::hex << static_cast<uint16_t>(_buffer[i]) << " ";
  }
  std::cout << std::dec << std::endl;
}

snd_proto::SerialResponse SerialComm::readIncomingMsg()
{
  snd_proto::SerialResponse resp;
  // Read the first byte to get the length of the message
  uint8_t length;
  size_t readBytes = m_serial.read(reinterpret_cast<uint8_t*>(&length), 1);
  if(readBytes < 1)
  {
    throw std::runtime_error("Timeout while retrieving the message length.");
  }
  // Read the message of length bytes
  std::string msg;
  readBytes = m_serial.read(msg, length);
  if(readBytes < 1)
  {
    throw std::runtime_error("Timeout while retrieving the message payload.");
  }
  if(!resp.ParseFromString(msg))
  {
    throw std::runtime_error("Failed to parse the protobuf message.");
    // std::cout << "Received length: " << static_cast<uint16_t>(length) <<
    // std::endl;
    // std::cout << "Read bytes: " << msg.length() << std::endl;
    // std::cout << "0x" << std::hex << static_cast<uint16_t>(length) << " ";
    // printBuffer(reinterpret_cast<const uint8_t*>(msg.data()), msg.length());
  }
  return resp;
}

int SerialComm::sendMsg(const snd_proto::SerialRequest& _req)
{
  // Serialize the protobuf
  std::string outString;
  if(!_req.SerializeToString(&outString))
  {
    throw std::runtime_error("Failed to serialize the message.");
  }
  // make sure the message length can be encoded on 1 byte
  assert(outString.length() < 0xff);
  uint8_t length = outString.length();
  size_t bytesWritten = m_serial.write(&length, 1);
  bytesWritten += m_serial.write(outString);
  // A bit of waste of time be to be sure to separate each packet
  m_serial.waitByteTimes(bytesWritten);
  return static_cast<int>(bytesWritten);
}

} // snd_serial namespace

// vim: sw=2 ts=2 et
