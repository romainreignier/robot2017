#include "Board.h"

BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;
BaseSequentialStream* ser = (BaseSequentialStream*)&SERIAL_DRIVER;

Board::Board()
    : leftMotor(&PWMD8, 1, true, GPIOA, 6),
      rightMotor(&PWMD2, 2, false, GPIOA, 5), qei(&QEID1, false, &QEID3, true),
      serial(&SERIAL_DRIVER)
{
}

void Board::begin()
{
  // Activate Serial Driver for debug
  sdStart(&DEBUG_DRIVER, NULL);

  // Start each component
  gBoard.serial.start();
  gBoard.qei.begin();
  gBoard.leftMotor.begin();
  gBoard.rightMotor.begin();

  // Pin muxing
  // PC4 : UART
  palSetPadMode(GPIOC, 4, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOC, 5, PAL_MODE_ALTERNATE(7));

  // PWM Motor left: PA7 = TIM8_1N
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(3));
  // PWM Motor right: PB3 = TIM2_2
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(1));

  // QEI Left:
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1));
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));
  // QEI Right:
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2));
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(2));
}

Board gBoard;
