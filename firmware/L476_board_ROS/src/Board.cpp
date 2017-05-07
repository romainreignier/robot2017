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
// ADC for motors current checking
static const ADCConversionGroup adcConversionGroup = {
  TRUE,
  ADC_CHANNELS,
  adc1Callback,
  NULL,
  ADC_CFGR_EXTEN_RISING | ADC_CFGR_EXTSEL_SRC(13),
  ADC_TR(0, 4095),
  {ADC_SMPR1_SMP_AN3(ADC_SMPR_SMP_247P5) |
     ADC_SMPR1_SMP_AN4(ADC_SMPR_SMP_247P5),
   0},
  {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN4), 0, 0, 0}};

// Timer to trigger ADC motors current checking
static const GPTConfig gpt6cfg = {80000, NULL, TIM_CR2_MMS_1, 0};

// Timer to trigger PID computation
static const GPTConfig gpt7cfg = {10000, gpt7cb, 0, 0};

// VL53L0X i2c bus
static const I2CConfig i2c1cfg = {
  //  0x00702991, // Computed with CubeMX, but also equals to:
  STM32_TIMINGR_PRESC(0U) | STM32_TIMINGR_SCLDEL(7U) |
    STM32_TIMINGR_SDADEL(0U) | STM32_TIMINGR_SCLH(41U) |
    STM32_TIMINGR_SCLL(91U),
  0,
  0};

// Servos i2c
static const I2CConfig i2c2cfg = {
  //  0x00702991, // Computed with CubeMX, but also equals to:
  STM32_TIMINGR_PRESC(0U) | STM32_TIMINGR_SCLDEL(7U) |
    STM32_TIMINGR_SDADEL(0U) | STM32_TIMINGR_SCLH(41U) |
    STM32_TIMINGR_SCLL(91U),
  0,
  0};

Board::Board()
  : // Components
    leftMotor{&PWMD3, 3, false, GPIOB, 4, GPIOB, 5, NULL, 0},
    rightMotor{&PWMD3, 4, false, GPIOD, 2, GPIOC, 12, NULL, 0},
    motors(leftMotor, rightMotor), qei{&QEID1, false, &QEID2, false},
    starter{GPIOC, 13}, colorSwitch{GPIOC, 0},
    eStop{GPIOC, 1, PAL_MODE_INPUT_PULLUP}, pump{GPIOB, 0},
    servos{&I2CD2, &i2c2cfg}, // leftVlx(&I2CD1),

    motorsCurrentChecker{&ADCD1, &GPTD6, 1000}, timeStartOverCurrent{0},
    pidTimerPeriodMs{25},
    leftMotorPid{
      &leftMotorSpeed, &leftMotorPwm, &leftMotorCommand, 1, 0, 0, DIRECT},
    rightMotorPid{
      &rightMotorSpeed, &rightMotorPwm, &rightMotorCommand, 1, 0, 0, DIRECT},
    lastLeftTicks{0}, lastRightTicks{0}, statusPub{"status", &statusMsg},
    // ROS related
    encodersPub{"encoders", &encodersMsg},
    motorsSpeedSub{"motors_speed", &Board::motorsSpeedCb, this},
    leftMotorPidSub{"left_motor_pid", &Board::leftMotorPidCb, this},
    rightMotorPidSub{"right_motor_pid", &Board::rightMotorPidCb, this},
    resetStatusSub{"reset_status", &Board::resetStatusCb, this},
    armServoSub{"arm_servo", &Board::armServoCb, this},
    graspServoSub{"grasp_servo", &Board::graspServoCb, this},
    pumpSub{"pump", &Board::pumpCb, this},
    launchServoSub{"launch_servo", &Board::launchServoCb, this}
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
  // see p.73, chap 4, table 16 of STM32L476 datasheet

  // UART1: PB6 (TX), PB7 (RX)
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

  // UART2: PA2 (TX), PA3 (RX)
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  // PWM Motor left: PC8 = TIM3_CH3
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2));
  // PWM Motor right: PC9 = TIM3_CH4
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(2));

  // ADC motor left: PC2 = ADC1_IN3
  palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
  // ADC motor left: PC3 = ADC1_IN4
  palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);

  // QEI Left: PA8, A9 = TIM1
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  // QEI Right: PA0, PA1 = TIM2
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(1) | PAL_MODE_INPUT_PULLUP);

  // I2C2: PB11 = SDA | PB10 = SCL
  palSetPadMode(GPIOB,
                10,
                PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGH |
                  PAL_STM32_OTYPE_OPENDRAIN);
  palSetPadMode(GPIOB,
                11,
                PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGH |
                  PAL_STM32_OTYPE_OPENDRAIN);

  // Activate Serial Driver for debug
  sdStart(&DEBUG_DRIVER, NULL);
  sdStart(&SERIAL_DRIVER, NULL);

  // Start Timer
  gptStart(&GPTD7, &gpt7cfg);
  // Timer at 10 kHz so the period = ms * 10
  gptStartContinuous(&GPTD7, pidTimerPeriodMs * 10);

  // Start each component
  qei.begin();
  motors.begin();
  starter.begin();
  eStop.begin();
  colorSwitch.begin();
  pump.begin();
  servos.begin();
  motorsCurrentChecker.begin(&gpt6cfg, &adcConversionGroup);
  // leftVlx.begin(&i2c1cfg);

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
  nh.subscribe(armServoSub);
  nh.subscribe(graspServoSub);
  nh.subscribe(pumpSub);
  nh.subscribe(launchServoSub);

  globalStatus = snd_msgs::Status::STATUS_OK;
}

void Board::publishFeedback()
{
  encodersMsg.header.stamp = nh.now();
  gBoard.qei.getValues(&encodersMsg.left, &encodersMsg.right);
  encodersPub.publish(&encodersMsg);
  // DEBUG("pwm l %d r %d\nin l %d r %d",
  //       static_cast<int16_t>(leftMotorPwm),
  //       static_cast<int16_t>(rightMotorPwm),
  //       static_cast<int16_t>(leftMotorSpeed),
  //       static_cast<int16_t>(rightMotorSpeed));
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

void Board::armServoCb(const std_msgs::UInt16& _msg)
{
  servos.setPWM(kArmServoId, 0, bound(_msg.data, kServoMin, kServoMax));
}

void Board::graspServoCb(const std_msgs::UInt16& _msg)
{
  servos.setPWM(kGraspServoId, 0, bound(_msg.data, kServoMin, kServoMax));
}

void Board::launchServoCb(const std_msgs::UInt16& _msg)
{
  servos.setPWM(kLaunchServoId, 0, bound(_msg.data, kServoMin, kServoMax));
}

void Board::pumpCb(const std_msgs::Bool& _msg)
{
  _msg.data ? pump.set() : pump.clear();
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