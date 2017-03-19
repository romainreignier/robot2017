/*
 * Supemeca Never Dies 2017
 * \file Qei.h
 * \date 17/03/2017
 * \author Romain Reignier
 */

#pragma once

#include "hal.h"

class Qei
{
  public:
  Qei(QEIDriver* _driverLeft, bool _rightIsInverted, QEIDriver* _driverRight,
      bool _leftIsInverted);
  void begin();
  void getValues(int32_t* _leftCnt, int32_t* rightCnt);

  private:
  QEIDriver* m_leftDriver;
  QEIDriver* m_rightDriver;
  int32_t m_leftCnt;
  int32_t m_rightCnt;
  QEIConfig m_leftCfg;
  QEIConfig m_rightCfg;
};
