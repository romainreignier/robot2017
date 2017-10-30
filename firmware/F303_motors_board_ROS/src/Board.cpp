#include "Board.h"

// BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;

#if defined(USE_ROS_LOG)
char logBuffer[LOG_BUFFER_SIZE];
#endif

// Drivers Callbacks
static void adc1Callback(ADCDriver* _adcd, adcsample_t* _buffer, size_t _n)
{
  (void)_adcd;
  (void)_buffer;
  (void)_n;
  //  gBoard.motorsCurrentChecker.adcCb(_adcd, _buffer, _n);
}

static void pidGptCb(GPTDriver* _gptd)
{
  (void)_gptd;
  // Compute PID for motors
  gBoard.motorsControl();
}

static uint16_t g_righTimerACount = 0;
// unit : µs between 2 rising edges of the same channel (µs for 4 ticks)
static int32_t g_rightMeasuredSpeedWithTimer = 0;

static void rightEncoderChAExtCb(EXTDriver* _extp, expchannel_t _channel)
{
  (void)_extp;
  (void)_channel;
  const uint16_t secondRisingEdgeCount = (uint16_t)ENCODER_COUNT_TIMER.tim->CNT;
  const int chBLevel = palReadPad(GPIOA, 1);

  if(!chBLevel)
  {
    // positive direction
    g_rightMeasuredSpeedWithTimer = static_cast<int32_t>(
      static_cast<uint16_t>(secondRisingEdgeCount - g_righTimerACount));
  }
  else
  {
    // negative direction
    g_rightMeasuredSpeedWithTimer = -static_cast<int32_t>(
      static_cast<uint16_t>(secondRisingEdgeCount - g_righTimerACount));
  }

  g_righTimerACount = secondRisingEdgeCount;
}

// Drivers configs
static const EXTConfig extcfg = {
  {{EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA,
    rightEncoderChAExtCb}, // 0
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   // {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB,
   // leftEncoderChAExtCb}, // 4
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL}}};

// ADC for motors current checking
static const ADCConversionGroup adcConversionGroup = {
  TRUE,
  ADC_CHANNELS,
  adc1Callback,
  NULL,
  ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(13),
  ADC_TR(0, 4095),
  {0,
   ADC_SMPR2_SMP_AN11(ADC_SMPR_SMP_181P5) |
     ADC_SMPR2_SMP_AN12(ADC_SMPR_SMP_181P5)},
  {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN11) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN12),
   0,
   0,
   0}};

// Timer to trigger ADC motors current checking
// static const GPTConfig gpt6cfg = {80000, NULL, TIM_CR2_MMS_1, 0};

// Timer to trigger PID computation
static const GPTConfig pidGptCfg = {10000, pidGptCb, 0, 0};

static const GPTConfig encodersCountGptCfg = {
  gBoard.kQeiTimerFrequency, NULL, 0, 0};

Board::Board()
  :
#if defined(USE_MONSTER_SHIELD)
    leftMotor{&PWMD1,
              kPwmTimerFrequency,
              kPwmTimerPeriod,
              2,
              false,
              GPIOA,
              10,
              GPIOA,
              12,
              GPIOF,
              1},
    rightMotor{&PWMD1,
               kPwmTimerFrequency,
               kPwmTimerPeriod,
               1,
               false,
               GPIOA,
               5,
               GPIOA,
               6,
               GPIOF,
               0},
#elif defined(USE_L298)
    leftMotor{&PWMD1,
               kPwmTimerFrequency,
               kPwmTimerPeriod,
               1,
               false,
               4,
               false,
               GPIOA,
               8, // ch1
               GPIOA,
               11}, // ch4
    rightMotor{&PWMD1,
              kPwmTimerFrequency,
              kPwmTimerPeriod,
              2,
              false,
              3,
              false,
              GPIOA,
              9, // ch2
              GPIOA,
              10}, // ch3
#endif
    motors(leftMotor, rightMotor), qei{&QEID3, false, &QEID2, false},
    starter{GPIOA, 4}, colorSwitch{GPIOA, 4},
    eStop{GPIOA, 3, PAL_MODE_INPUT_PULLUP},
    // motorsCurrentChecker{&ADCD1, &GPTD6, 1000},
    leftMotorPid{
      &leftMotorSpeed, &leftMotorPwm, &leftMotorCommand, 1, 0, 0, DIRECT},
    rightMotorPid{
      &rightMotorSpeed, &rightMotorPwm, &rightMotorCommand, 1, 0, 0, DIRECT},
    // ROS related
    statusPub{"status", &statusMsg}, encodersPub{"encoders", &encodersMsg},
    motorsCurrentPub{"current", &motorsCurrentMsg},
    motorsSpeedSub{"motors_speed", &Board::motorsSpeedCb, this},
    motorsModeSub{"motors_mode", &Board::motorsModeCb, this},
    leftMotorPwmSub{"left_motor_pwm", &Board::leftMotorPwmCb, this},
    rightMotorPwmSub{"right_motor_pwm", &Board::rightMotorPwmCb, this},
    leftMotorPidSub{"left_motor_pid", &Board::leftMotorPidCb, this},
    rightMotorPidSub{"right_motor_pid", &Board::rightMotorPidCb, this},
    resetStatusSub{"reset_status", &Board::resetStatusCb, this}
{
  leftMotorPid.SetOutputLimits(-10000, 10000);
  rightMotorPid.SetOutputLimits(-10000, 10000);
  leftMotorPid.SetSampleTime(pidTimerPeriodMs);
  rightMotorPid.SetSampleTime(pidTimerPeriodMs);
  leftMotorPid.SetMode(AUTOMATIC);
  rightMotorPid.SetMode(AUTOMATIC);
}

void Board::begin()
{
  // Pin muxing of each peripheral
  // see p.37, chap 4, table 15 of STM32F303x8 datasheet

  // UART2: PA2 (TX), PA15 (RX)
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 15, PAL_MODE_ALTERNATE(7));

  // PWM Motor left: PA9 = TIM1_CH2
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(6));
  // PWM Motor right: PA8 = TIM1_CH1
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(6));
#if defined(USE_L298)
  // PWM Motor left: PA10 = TIM1_CH3
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(6));
  // PWM Motor right: PA11 = TIM1_CH4
  palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(11));
#endif

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

  // Activate Serial Driver for debug
  // sdStart(&DEBUG_DRIVER, NULL);
  sdStart(&SERIAL_DRIVER, NULL);

  // Start Timers
  gptStart(&PID_TIMER, &pidGptCfg);
  gptStartContinuous(&PID_TIMER, pidTimerPeriodMs * 10);

  gptStart(&ENCODER_COUNT_TIMER, &encodersCountGptCfg);
  gptStartContinuous(&ENCODER_COUNT_TIMER, 0xffff);
  extStart(&EXTD1, &extcfg);

  // Start each component
  qei.begin();
  motors.begin();
  starter.begin();
  eStop.begin();
  colorSwitch.begin();
  // motorsCurrentChecker.begin(&gpt6cfg, &adcConversionGroup);

  // ROS
  // nh.getHardware()->setDriver(&SD1);
  nh.initNode();
  // Publishers
  nh.advertise(statusPub);
  nh.advertise(encodersPub);
  // Subscribers
  nh.subscribe(motorsSpeedSub);
  nh.subscribe(leftMotorPwmSub);
  nh.subscribe(rightMotorPwmSub);
  nh.subscribe(leftMotorPidSub);
  nh.subscribe(rightMotorPidSub);
  nh.subscribe(resetStatusSub);
  nh.subscribe(motorsModeSub);

  globalStatus = snd_msgs::Status::STATUS_OK;
  motorsMode.mode = snd_msgs::MotorControlMode::PWM;
}

void Board::publishFeedback()
{
  int32_t rightMeasuredSpeedWithTimer;
  chSysLock();
  {
    rightMeasuredSpeedWithTimer = g_rightMeasuredSpeedWithTimer;
    gBoard.mustPublishFeedback = false;
    encodersMsg.left_pos = leftQeiCnt;
    encodersMsg.right_pos = rightQeiCnt;
    encodersMsg.left_speed = leftMotorSpeed;
    encodersMsg.right_speed = rightMotorSpeed;
  }
  chSysUnlock();

  // If uses the time between 2 ticks to compute the speed
  // encodersMsg.right_speed = (4.0f * kQeiTimerFrequency) /
  // (rightMeasuredSpeedWithTimer);

  encodersMsg.header.stamp = nh.now();
  encodersPub.publish(&encodersMsg);

  // motorsCurrentMsg.left = motorsCurrentChecker.value1();
  // motorsCurrentMsg.right = motorsCurrentChecker.value2();
  // motorsCurrentPub.publish(&motorsCurrentMsg);
}

void Board::publishStatus()
{
  statusMsg.header.stamp = nh.now();
  statusMsg.starter = starter.read();
  statusMsg.eStop = eStop.read();
  statusMsg.color_switch.color =
    colorSwitch.read() ? static_cast<uint8_t>(snd_msgs::Color::BLUE)
                       : static_cast<uint8_t>(snd_msgs::Color::YELLOW);
  // statusMsg.left_motor_current =
  //   static_cast<uint16_t>(motorsCurrentChecker.value1() * kAdcToMilliAmps);
  // statusMsg.right_motor_current =
  //   static_cast<uint16_t>(motorsCurrentChecker.value2() * kAdcToMilliAmps);
  // statusMsg.status = globalStatus;
  // statusPub.publish(&statusMsg);
}

void Board::motorsSpeedCb(const snd_msgs::Motors& _msg)
{
  /*
  if(globalStatus != snd_msgs::Status::STATUS_MOTORS_OVERCURRENT)
  {
    motors.stop();
    return;
  }
  */
  switch(motorsMode.mode)
  {
  case snd_msgs::MotorControlMode::PID:
    // Update PIDs setpoints
    // DEBUG("Motors Speed received, left: %f right: %f\n", _msg.left,
    // _msg.right);
    leftMotorCommand = _msg.left;
    rightMotorCommand = _msg.right;
  // DEBUG("Motors Speed received but motors stopped because of
  // overcurrent.\n");
  case snd_msgs::MotorControlMode::PWM:
    // Set direct PWM to motors
    // DEBUG("Motors Speed received, left: %f right: %f\n", _msg.left,
    // _msg.right);
    motors.pwm(static_cast<int16_t>(_msg.left),
               static_cast<int16_t>(_msg.right));
    break;
  case snd_msgs::MotorControlMode::DISABLED:
  default:
    // Do nothing
    motors.stop();
  }
}

void Board::motorsModeCb(const snd_msgs::MotorControlMode& _msg)
{
  motorsMode = _msg;
}

void Board::leftMotorPwmCb(const std_msgs::Int16& _msg)
{
  if(motorsMode.mode == snd_msgs::MotorControlMode::PWM)
  {
    leftMotor.pwm(static_cast<int16_t>(_msg.data));
  }
}

void Board::rightMotorPwmCb(const std_msgs::Int16& _msg)
{
  if(motorsMode.mode == snd_msgs::MotorControlMode::PWM)
  {
    rightMotor.pwm(static_cast<int16_t>(_msg.data));
  }
}

void Board::leftMotorPidCb(const snd_msgs::Pid& _msg)
{
  // DEBUG("Left Motor Pid received: P: %.4f I: %.4f  D: %.4f", _msg.p, _msg.i,
  // _msg.d);
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
  /*
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
  */
}

void Board::motorsControl()
{
  qeidelta_t dLeft;
  qeidelta_t dRight;
  int32_t rightMeasuredSpeedWithTimer;

  // Retrieve the values from a locked system
  chSysLockFromISR();
  {
    dLeft = qeiUpdateI(qei.getLeftDriver());
    dRight = qeiUpdateI(qei.getRightDriver());
    rightMeasuredSpeedWithTimer = g_rightMeasuredSpeedWithTimer;
  }
  chSysUnlockFromISR();

  // Increment the internal counters
  leftQeiCnt += dLeft;
  rightQeiCnt += dRight;

  // Compute the average
  leftQeiAvg.add(dLeft);
  rightQeiAvg.add(dRight);

  // Speeds in ticks/s
  leftMotorSpeed = (leftQeiAvg.getAverage() * 1000.0f) / (pidTimerPeriodMs);
  rightMotorSpeed = (rightQeiAvg.getAverage() * 1000.0f) / (pidTimerPeriodMs);

  // If uses the time between 2 ticks to compute the speed
  // rightMotorSpeed = (4.0f * kQeiTimerFrequency) /
  // (rightMeasuredSpeedWithTimer);

  // Set the flag
  gBoard.mustPublishFeedback = true;

  // Update the PID if it is used
  if(gBoard.motorsMode.mode == snd_msgs::MotorControlMode::PID)
  {
    if(leftMotorPid.Compute() && rightMotorPid.Compute())
    {
      chSysLockFromISR();
      motors.pwmI(static_cast<int16_t>(leftMotorPwm),
                  static_cast<int16_t>(rightMotorPwm));
      chSysUnlockFromISR();
    }
  }
}

Board gBoard;
