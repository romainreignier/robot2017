/*
 * Supemeca Never Dies 2017
 * \file Qei.cpp
 * \date 17/03/2017
 * \author Romain Reignier
 */

#include "Qei.h"

Qei::Qei(QEIDriver* _leftDriver, bool _leftIsInverted, QEIDriver* _rightDriver,
         bool _rightIsInverted)
  : m_leftDriver{_leftDriver}, m_rightDriver{_rightDriver}, m_leftCnt{0},
    m_rightCnt{0},
    m_leftCfg{QEI_MODE_QUADRATURE,
              QEI_BOTH_EDGES,
              _leftIsInverted ? QEI_DIRINV_TRUE : QEI_DIRINV_FALSE,
              QEI_OVERFLOW_WRAP,
              0,
              0,
              NULL,
              NULL},
    m_rightCfg{QEI_MODE_QUADRATURE,
               QEI_BOTH_EDGES,
               _rightIsInverted ? QEI_DIRINV_TRUE : QEI_DIRINV_FALSE,
               QEI_OVERFLOW_WRAP,
               0,
               0,
               NULL,
               NULL}
{
}

void Qei::begin()
{
  qeiStart(m_leftDriver, &m_leftCfg);
  qeiStart(m_rightDriver, &m_rightCfg);
  qeiEnable(m_leftDriver);
  qeiEnable(m_rightDriver);
}

/*
void Qei::getValues(int32_t* _left, int32_t* _right)
{
  osalSysLock();
  getValuesI(_left, _right);
  osalSysUnlock();
}

void Qei::getValuesI(int32_t* _left, int32_t* _right)
{
  m_leftCnt += qeiUpdateI(m_leftDriver);
  m_rightCnt += qeiUpdateI(m_rightDriver);
  *_left = m_leftCnt;
  *_right = m_rightCnt;
}
*/

QEIDriver *Qei::getLeftDriver() const
{
  return m_leftDriver;
}

QEIDriver *Qei::getRightDriver() const
{
  return m_rightDriver;
}
