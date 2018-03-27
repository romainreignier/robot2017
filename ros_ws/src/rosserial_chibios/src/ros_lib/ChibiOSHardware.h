#ifndef ROS_CHIBIOS_HARDWARE_H
#define ROS_CHIBIOS_HARDWARE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hal.h"

class ChibiOSHardware
{
public:
  ChibiOSHardware(BaseChannel* io) { iostream = io; }

  ChibiOSHardware() { iostream = (BaseChannel*)&SD2; }

  void init() {}

  void setDriver(SerialDriver* driver) { iostream = (BaseChannel*)driver; }

  int read() { return chnGetTimeout(iostream, TIME_IMMEDIATE); }

  void write(uint8_t* data, int length) { chnWrite(iostream, data, length); }

  unsigned long time()
  {
#if OSAL_ST_FREQUENCY == 1000
    return osalOsGetSystemTimeX();
#elif(OSAL_ST_FREQUENCY / 1000) >= 1 && (OSAL_ST_FREQUENCY % 1000) == 0
    return (osalOsGetSystemTimeX() - 1) / (OSAL_ST_FREQUENCY / 1000) + 1;
#elif(1000 / OSAL_ST_FREQUENCY) >= 1 && (1000 % OSAL_ST_FREQUENCY) == 0
    return (osalOsGetSystemTimeX() - 1) * (1000 / OSAL_ST_FREQUENCY) + 1;
#else
    return (((static_cast<uint64_t>(osalOsGetSystemTimeX()) - 1) * 1000) /
           OSAL_ST_FREQUENCY) + 1;
#endif
  }

protected:
  BaseChannel* iostream;
};

#ifdef __cplusplus
}
#endif

#endif /* ROS_CHIBIOS_HARDWARE_H */
