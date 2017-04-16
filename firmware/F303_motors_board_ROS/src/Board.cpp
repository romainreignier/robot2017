#include "Board.h"


BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;

Board::Board()
    : leftMotor{&PWMD1, 2, false, GPIOA, 12, GPIOA, 10},
      rightMotor{&PWMD1, 1, false, GPIOA, 6, GPIOA, 5},
      motors(leftMotor, rightMotor),
      qei{&QEID3, false, &QEID2, false},
      starter{GPIOA, 7}, colorSwitch{GPIOA, 3}, eStop{GPIOA, 11},
      starterPub{"starter", &starterMsg}, eStopPub{"eStop", &eStopMsg},
      colorSwitchPub{"color_switch", &colorSwitchMsg}
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
  eStop.begin();
  colorSwitch.begin();

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
  nh.advertise(eStopPub);
  nh.advertise(colorSwitchPub);
}

void Board::publishAll()
{
  starterMsg.data = starter.read();
  starterPub.publish(&starterMsg);
  eStopMsg.data = eStop.read();
  eStopPub.publish(&eStopMsg);
  colorSwitchMsg.color = colorSwitch.read() ? snd_msgs::Color::BLUE : snd_msgs::Color::YELLOW;
  colorSwitchPub.publish(&colorSwitchMsg);
}

Board gBoard;
