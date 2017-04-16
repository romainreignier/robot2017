#include "Board.h"


BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;

Board::Board()
    : leftMotor{&PWMD1, 2, false, GPIOA, 12, GPIOA, 10},
      rightMotor{&PWMD1, 1, false, GPIOA, 6, GPIOA, 5},
      motors(leftMotor, rightMotor),
      qei{&QEID3, false, &QEID2, false},
      starter{GPIOA, 7}, colorSwitch{GPIOA, 3}, eStop{GPIOA, 4},
      starterPub{"starter", &starterMsg}
{
}

void Board::begin()
{
  // Activate Serial Driver for debug
  sdStart(&DEBUG_DRIVER, NULL);
  sdStart(&SERIAL_DRIVER, NULL);

  // Start each component
  qei.begin();
  motors.begin();
  starter.begin();

  // Pin muxing of every the components
  // see p.37, chap 4, table 15 of STM32F303x8 datasheet

  // UART1: PB6 (TX), PB7 (RX)
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

  // UART2: PA2 (TX), PA15 (RX)
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 15, PAL_MODE_ALTERNATE(7));

  // PWM Motor left: PA9 = TIM1_CH2
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(6));
  // PWM Motor right: PA8 = TIM1_CH1
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(6));

  // QEI Left: PB4, PB5 = TIM3
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(2) | PAL_MODE_INPUT_PULLUP);
  // QEI Right: PA0, PA1 = TIM2
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);

  // ROS
  nh.initNode();
  nh.advertise(starterPub);
}

void Board::publishAll()
{
  starterMsg.data = starter.read();
  starterPub.publish(&starterMsg);
}

Board gBoard;
