/*
 * Supemeca Never Dies 2017
 * \file Qei.cpp
 * \date 17/03/2017
 * \author Romain Reignier
 */

#include "Qei.h"

Qei::Qei(QEIDriver* _leftDriver, bool _leftIsInverted, QEIDriver* _rightDriver,
         bool _rightIsInverted)
    : m_leftDriver(_leftDriver), m_rightDriver(_rightDriver),
      m_leftCfg{QEI_MODE_QUADRATURE,
                QEI_BOTH_EDGES,
                _leftIsInverted ? QEI_DIRINV_TRUE : QEI_DIRINV_FALSE},
      m_rightCfg{QEI_MODE_QUADRATURE,
                 QEI_BOTH_EDGES,
                 _rightIsInverted ? QEI_DIRINV_TRUE : QEI_DIRINV_FALSE}
{
}

void Qei::begin()
{
  qeiStart(m_leftDriver, &m_leftCfg);
  qeiStart(m_rightDriver, &m_rightCfg);
  qeiEnable(m_leftDriver);
  qeiEnable(m_rightDriver);
}

void Qei::getValues(int32_t* _left, int32_t* _right)
{
  *_left = qeiGetCount(m_leftDriver);
  *_right = qeiGetCount(m_rightDriver);
}
