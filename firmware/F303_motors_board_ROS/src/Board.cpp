#include "Board.h"

BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;

#if defined(USE_ROS_LOG)
char logBuffer[LOG_BUFFER_SIZE];
#endif

// Drivers Callbacks
static void adc1Callback(ADCDriver* _adcd, adcsample_t* _buffer, size_t _n)
{
  gBoard.motorsCurrentChecker.adcCb(_adcd, _buffer, _n);
}

static void gpt7cb(GPTDriver* _gptd)
{
  (void)_gptd;
  // Compute PID for motors
  gBoard.motorsControl();
}

// Drivers configs
static const GPTConfig gpt7cfg = {1000, gpt7cb, 0, 0};

Board::Board()
  : leftMotor{&PWMD1, 2, false, GPIOA, 12, GPIOA, 10, GPIOF, 1},
    rightMotor{&PWMD1, 1, false, GPIOA, 6, GPIOA, 5, GPIOF, 0},
    motors(leftMotor, rightMotor), qei{&QEID3, false, &QEID2, false},
    starter{GPIOA, 7}, colorSwitch{GPIOA, 11},
    eStop{GPIOA, 3, PAL_MODE_INPUT_PULLUP},
    motorsCurrentChecker{
      &ADCD1, &GPTD6, ADC_CHANNEL_IN11, ADC_CHANNEL_IN12, 80000, 1000},
    pidTimerPeriodMs{25},
    leftMotorPid{
      &leftMotorSpeed, &leftMotorPwm, &leftMotorCommand, 1, 0, 0, DIRECT},
    rightMotorPid{
      &rightMotorSpeed, &rightMotorPwm, &rightMotorCommand, 1, 0, 0, DIRECT},
    lastLeftTicks{0}, lastRightTicks{0}, statusPub{"status", &statusMsg},
    encodersPub{"encoders", &encodersMsg},
    motorsSpeedSub("motors_speed", &Board::motorsSpeedCb, this),
    leftMotorPidSub("left_motor_pid", &Board::leftMotorPidCb, this),
    rightMotorPidSub("right_motor_pid", &Board::rightMotorPidCb, this),
    resetStatusSub{"reset_status", &Board::resetStatusCb, this},
    timeStartOverCurrent{0}
{
  leftMotorPid.SetOutputLimits(-10000, 10000);
  rightMotorPid.SetOutputLimits(-10000,
                                10000);
  leftMotorPid.SetSampleTime(pidTimerPeriodMs);
  rightMotorPid.SetSampleTime(pidTimerPeriodMs);
  leftMotorPid.SetMode(AUTOMATIC);
  rightMotorPid.SetMode(AUTOMATIC);
}

void Board::begin()
{
  // Activate Serial Driver for debug
  sdStart(&DEBUG_DRIVER, NULL);
  sdStart(&SERIAL_DRIVER, NULL);

  // Fill the adc conversion group
  auto& adcConversionGroup = motorsCurrentChecker.getAdcConversionGroup();
  adcConversionGroup.end_cb = adc1Callback;
  adcConversionGroup.smpr[0] = 0;
  adcConversionGroup.smpr[1] = ADC_SMPR2_SMP_AN11(ADC_SMPR_SMP_181P5) |
                               ADC_SMPR2_SMP_AN12(ADC_SMPR_SMP_181P5);

  // Start Timer
  gptStart(&GPTD7, &gpt7cfg);
  // Timer at 1 kHz so the period == ms
  gptStartContinuous(&GPTD7, pidTimerPeriodMs);

  // Start each component
  qei.begin();
  motors.begin();
  starter.begin();
  eStop.begin();
  colorSwitch.begin();
  motorsCurrentChecker.begin();

  // Pin muxing of each peripheral
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

  // ADC motor left: PB0 = ADC1_IN11
  palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
  // ADC motor left: PB1 = ADC1_IN12
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

  // QEI Left: PB4, PB5 = TIM3
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(2) | PAL_MODE_INPUT_PULLUP);
  // QEI Right: PA0, PA1 = TIM2
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);

  // ROS
  // nh.getHardware()->setDriver(&SD1);
  nh.initNode();
  // Publishers
  nh.advertise(statusPub);
  nh.advertise(encodersPub);
  // Subscribers
  nh.subscribe(motorsSpeedSub);
  nh.subscribe(leftMotorPidSub);
  nh.subscribe(rightMotorPidSub);
  nh.subscribe(resetStatusSub);

  globalStatus = snd_msgs::Status::STATUS_OK;
}

void Board::publishFeedback()
{
  encodersMsg.header.stamp = nh.now();
  gBoard.qei.getValues(&encodersMsg.left, &encodersMsg.right);
  encodersPub.publish(&encodersMsg);
  DEBUG("pwm l %d r %d\nin l %d r %d",
        static_cast<int16_t>(leftMotorPwm),
        static_cast<int16_t>(rightMotorPwm),
        static_cast<int16_t>(leftMotorSpeed),
        static_cast<int16_t>(rightMotorSpeed));
}

void Board::publishStatus()
{
  statusMsg.header.stamp = nh.now();
  statusMsg.starter = starter.read();
  statusMsg.eStop = eStop.read();
  statusMsg.color_switch.color =
    colorSwitch.read() ? static_cast<uint8_t>(snd_msgs::Color::BLUE)
                       : static_cast<uint8_t>(snd_msgs::Color::YELLOW);
  statusMsg.left_motor_current =
    motorsCurrentChecker.value1() * kAdcToMilliAmps;
  statusMsg.right_motor_current =
    motorsCurrentChecker.value2() * kAdcToMilliAmps;
  statusMsg.status = globalStatus;
  statusPub.publish(&statusMsg);
  // DEBUG("Current Motors pwm left: %d right: %d",
  //       static_cast<int16_t>(leftMotorPwm),
  //       static_cast<int16_t>(rightMotorPwm));
}

void Board::motorsSpeedCb(const snd_msgs::Motors& _msg)
{
  if(globalStatus != snd_msgs::Status::STATUS_MOTORS_OVERCURRENT)
  {
    // DEBUG("Motors Speed received, left: %f right: %f\n", _msg.left,
    // _msg.right);
    leftMotorCommand = _msg.left;
    rightMotorCommand = _msg.right;
  }
  else
  {
    // DEBUG("Motors Speed received but motors stopped because of
    // overcurrent.\n");
    motors.stop();
  }
}

void Board::leftMotorPidCb(const snd_msgs::Pid& _msg)
{
  DEBUG("Left Motor Pid received: P: %.4f I: %.4f  D: %.4f",
        _msg.p,
        _msg.i,
        _msg.d);
  leftMotorPid.SetTunings(_msg.p, _msg.i, _msg.d);
}

void Board::rightMotorPidCb(const snd_msgs::Pid& _msg)
{
  DEBUG("Right Motor Pid received: P: %.4f I: %.4f  D: %.4f",
        _msg.p,
        _msg.i,
        _msg.d);
  rightMotorPid.SetTunings(_msg.p, _msg.i, _msg.d);
}

void Board::resetStatusCb(const std_msgs::Empty& _msg)
{
  (void)_msg;
  DEBUG("Received a service request to reset the status flag");
  globalStatus = snd_msgs::Status::STATUS_OK;
  timeStartOverCurrent = 0;
}

void Board::checkMotorsCurrent()
{
  if(motorsCurrentChecker.value1() * kAdcToMilliAmps > kCurrentThreshold ||
     motorsCurrentChecker.value2() * kAdcToMilliAmps > kCurrentThreshold)
  {
    DEBUG("Overcurrent detected");
    if(timeStartOverCurrent != 0)
    {
      DEBUG("It is not the first time");
      if(chVTGetSystemTimeX() - timeStartOverCurrent >= kMaxTimeOverCurrent)
      {
        DEBUG("Lasts more than 1 second, raise the flag!");
        globalStatus = snd_msgs::Status::STATUS_MOTORS_OVERCURRENT;
      }
    }
    else
    {
      DEBUG("It is the first time, take note of the time");
      timeStartOverCurrent = chVTGetSystemTimeX();
    }
  }
  else
  {
    if(timeStartOverCurrent != 0)
    {
      DEBUG("The flag was previoulsy set, reset it");
      timeStartOverCurrent = 0;
    }
  }
}

void Board::motorsControl()
{
  int32_t leftTicks;
  int32_t rightTicks;
  chSysLockFromISR();
  gBoard.qei.getValuesI(&leftTicks, &rightTicks);
  chSysUnlockFromISR();
  // TODO see if it is better to get system time instead of fixed timer period
  leftMotorSpeed =
    (leftTicks - lastLeftTicks) * 1000.0 / pidTimerPeriodMs; // ticks/s
  rightMotorSpeed =
    (rightTicks - lastRightTicks) * 1000.0 / pidTimerPeriodMs; // ticks/s
  lastLeftTicks = leftTicks;
  lastRightTicks = rightTicks;
  if(leftMotorPid.Compute() && rightMotorPid.Compute())
  {
    chSysLockFromISR();
    motors.pwmI(static_cast<int16_t>(leftMotorPwm),
                static_cast<int16_t>(rightMotorPwm));
    chSysUnlockFromISR();
  }
}

Board gBoard;
