#pragma once

#include "hal.h"

#include "vl53l0x_api.h"

class VL53L0X
{
public:
  static constexpr uint8_t VL53L0X_I2C_ADDR = 0x29;
  VL53L0X(const I2CDriver* _driver);
  bool begin(const I2CConfig* _i2cCfg, uint8_t _i2cAddr = VL53L0X_I2C_ADDR);
  void setAddress(uint8_t _addr);
  int16_t longRange();
  int16_t accuracyRange();

private:
  VL53L0X_Error init();
  int16_t measure();

  const I2CDriver* m_driver;
  VL53L0X_Dev_t m_device;
  VL53L0X_Version_t m_version;
  VL53L0X_DeviceInfo_t m_info;
};
