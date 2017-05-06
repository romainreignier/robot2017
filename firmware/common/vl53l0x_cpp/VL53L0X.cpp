/**
 * \date: 02/05/2017
 * \author: Romain Reignier
 * \brief: C++ Wrapper on to of ST's API library to use VL53L0X sensors
 */

#include "VL53L0X.h"

VL53L0X::VL53L0X(const I2CDriver* _driver) : m_driver{_driver}
{
}

bool VL53L0X::begin(const I2CConfig* _i2cCfg, uint8_t _i2cAddr)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  m_device.I2cDevAddr = _i2cAddr;
  m_device.comms_type = 1;
  m_device.comms_speed_khz = 400;

  VL53L0X_i2c_init(m_driver, _i2cCfg);

  status = VL53L0X_DataInit(&m_device);
  if(status != VL53L0X_ERROR_NONE)
  {
    return false;
  }

  status = VL53L0X_GetDeviceInfo(&m_device, &m_info);
  if(status != VL53L0X_ERROR_NONE)
  {
    return false;
  }
  return true;
}

void VL53L0X::setAddress(uint8_t _addr)
{
  VL53L0X_SetDeviceAddress(&m_device, _addr);
  m_device.I2cDevAddr = _addr;
}

VL53L0X_Error VL53L0X::init()
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;

  // Device Initialization
  status = VL53L0X_StaticInit(&m_device);
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_PerformRefCalibration(&m_device, &VhvSettings, &PhaseCal);
  }

  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_PerformRefSpadManagement(
      &m_device, &refSpadCount, &isApertureSpads);
  }

  if(status == VL53L0X_ERROR_NONE)
  {
    status =
      VL53L0X_SetDeviceMode(&m_device, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  }
  return status;
}

int16_t VL53L0X::measure()
{
  VL53L0X_RangingMeasurementData_t measure;
  VL53L0X_Error status = VL53L0X_PerformSingleRangingMeasurement(&m_device, &measure);
  if(measure.RangeStatus != 4)
  {
    return measure.RangeMilliMeter;
  }
  return -1;
}

int16_t VL53L0X::accuracyRange()
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  status = init();
  // Set the device for high accuracy profile
  if(status == VL53L0X_ERROR_NONE)
  {
    status =
      VL53L0X_SetLimitCheckValue(&m_device,
                                 VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                 (FixPoint1616_t)(0.25 * 65536));
  }
  else
    return -1;
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetLimitCheckValue(&m_device,
                                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        (FixPoint1616_t)(18 * 65536));
  }
  else
    return -1;
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_device, 200000);
  }
  else
    return -1;
  return measure();
}

int16_t VL53L0X::longRange()
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  status = init();
  // Set the device for long range profile
  if(status == VL53L0X_ERROR_NONE)
  {
    status =
      VL53L0X_SetLimitCheckValue(&m_device,
                                 VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                 (FixPoint1616_t)(0.1 * 65536));
  }
  else
    return -1;
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetLimitCheckValue(&m_device,
                                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                        (FixPoint1616_t)(60 * 65536));
  }
  else
    return -1;
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&m_device, 33000);
  }
  else
    return -1;
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetVcselPulsePeriod(
      &m_device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  }
  else
    return -1;
  if(status == VL53L0X_ERROR_NONE)
  {
    status = VL53L0X_SetVcselPulsePeriod(
      &m_device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  }
  else
    return -1;
  return measure();
}
