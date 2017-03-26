#include "Board.h"

BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;
BaseSequentialStream* ser = (BaseSequentialStream*)&SERIAL_DRIVER;

Board::Board()
    : leftMotor{&PWMD3, 1, false, GPIOA, 8, GPIOA, 9},
      rightMotor{&PWMD2, 3, false, GPIOB, 5, GPIOC, 7},
      qei{&QEID1, false, &QEID8, true}, serial{&SERIAL_DRIVER},
      starter{GPIOB, 12}, colorSwitch{GPIOA, 12}
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

  // Pin muxing of every the components
  // see p.73, chap 4, table 16 of STM32L476xx datasheet

  // UART3: PC4 (TX), PC5 (RX)
  palSetPadMode(GPIOC, 4, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOC, 5, PAL_MODE_ALTERNATE(7));

  // PWM Motor left: PB10 = TIM2_CH3
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(1));
  // PWM Motor right: PB4 = TIM3_CH1
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2));

  // QEI Left: PA8, PA9
  /*
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  // QEI Right: PC6, PC7
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(3) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(3) | PAL_MODE_INPUT_PULLUP);
  */
}

Board gBoard;
