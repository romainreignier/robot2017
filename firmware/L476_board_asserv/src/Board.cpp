#include "Board.h"

#include <stdlib.h>
#include <math.h>

# define M_PI		3.14159265358979323846	/* pi */
BaseSequentialStream* dbg = (BaseSequentialStream*)&DEBUG_DRIVER;

#if defined(USE_ROS_LOG)
char logBuffer[LOG_BUFFER_SIZE];
#endif

static void gpt7cb(GPTDriver* _gptd)
{
  (void)_gptd;
  // Compute PID for motors
  gBoard.PIDTimerCb();
}

// Drivers configs
// Timer to trigger PID computation
static const GPTConfig gpt7cfg = {10000, gpt7cb, 0, 0};

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
    motors(leftMotor, rightMotor), qei{&QEID1, true, &QEID2, false},
    starter{GPIOC, 13}, colorSwitch{GPIOC, 1}, selector{GPIOB, 12},
    eStop{GPIOC, 5, PAL_MODE_INPUT_PULLUP},
    frontProximitySensor{GPIOB, 1, PAL_MODE_INPUT_PULLUP},
    rearLeftProximitySensor{GPIOC, 7, PAL_MODE_INPUT_PULLUP},
    rearRightProximitySensor{GPIOC, 0, PAL_MODE_INPUT_PULLUP}, pump{GPIOB, 0},
    greenLed{GPIOA, 11}, servos{&I2CD2, &i2c2cfg}, tcsLed{GPIOA, 15},
    lastLeftTicks{0}, lastRightTicks{0}, maxPwm{3000}, iMinDist{-maxPwm},
    iMaxDist{maxPwm}, iMinAng{-maxPwm}, iMaxAng{maxPwm}, kpDist{15.0},
    kpAng{600}, kiDist{0.15}, kiAng{0.1}, kdDist{650}, kdAng{1500},
    vLinMax{4},          // 200 mm/s -> 4 mm / periode (20ms)
    vAngMax{0.087964594}, // 0.7 tr/s -> rad/periode
    finish(true),
    cptRestOnPosition(0),
    compensation(0)
{
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
  sdStart(&SD1, NULL);
  sdStart(&SERIAL_DRIVER, NULL);

  // Start GPT Peripheral for PID Timer
  gptStart(&PID_TIMER, &gpt7cfg);
  // Actually start the Timer
  gptStartContinuous(&PID_TIMER, pidTimerPeriodMs * 10);

  // Start each component
  qei.begin();
  motors.begin();
  starter.begin();
  eStop.begin();
  colorSwitch.begin();
  selector.begin();
  frontProximitySensor.begin();
  rearLeftProximitySensor.begin();
  rearRightProximitySensor.begin();
  pump.begin();
  greenLed.begin();
  // servos.begin();
  tcsLed.begin();
  tcsLed.clear();
}

void Board::main()
{
}

void Board::startPIDTimer()
{
  // Start Timer at 10 kHz so the period = ms * 10
  gptStartContinuous(&PID_TIMER, pidTimerPeriodMs * 10);
}

void Board::stopPIDTimer()
{
  // Stop the timer
  gptStopTimer(&PID_TIMER);
}

void Board::PIDTimerCb()
{
  // Appele a 20Hz
  if(mustComputeTraj)
  {
    computeTraj();
  }
  if(!finish){
    asserv();
    //lectureCodeur();
  }
}

void Board::moveLinear(float _distance)
{
  // Lock system before modifying data used in interrupt
  chSysLock();
  cibleAngle = 0;
  cibleDistance = _distance;
  consigneDistance = _distance;
  consigneAngle = 0;
  mesureDistance = 0;
  mesureAngle = 0;
  finAsservIterations = 0;
  finish = false;
  mustComputeTraj = false;
  compensation=0;
  cptRestOnPosition=0;
  erreurDistance=0;
  lastErreurDistance=0;
  chSysUnlock();
  while(!finish)
  {
    printErrors();
    chThdSleepMilliseconds(100);
  }
  chprintf(dbg, "Asserv fini\r\n");
  printErrors();
}

void Board::moveAngular(float _angle)
{
  // Lock system before modifying data used in interrupt
  chSysLock();
  cibleDistance = 0;
  cibleAngle = normalize_angle(_angle);
  consigneDistance = 0;
  consigneAngle = cibleAngle; // 0 si mustComputeTraj True
  mesureDistance = 0;
  mesureAngle = 0;
  finAsservIterations = 0;
  finish = false;
  mustComputeTraj = false; // On desactive la rampe avec false
  chSysUnlock();
  while(!finish)
  {
    printErrors();
    chThdSleepMilliseconds(100);
  }
  chprintf(dbg, "Asserv fini\r\n");
  printErrors();
}

void Board::moveLinearEchelon(float _distance)
{
  // Lock system before modifying data used in interrupt
  chSysLock();
  cibleAngle = 0;
  cibleDistance = _distance;
  consigneDistance = _distance;
  consigneAngle = 0;
  mesureDistance = 0;
  mesureAngle = 0;
  finAsservIterations = 0;
  finish = false;
  mustComputeTraj = false;
  chSysUnlock();
  while(!finish)
  {
    printErrors();
    chThdSleepMilliseconds(100);
  }
  chprintf(dbg, "Asserv fini\r\n");
  printErrors();
}

void Board::moveAngularEchelon(float _angle)
{
  // Lock system before modifying data used in interrupt
  chSysLock();
  cibleDistance = 0;
  cibleAngle = _angle;
  consigneDistance = 0;
  consigneAngle = _angle;
  mesureDistance = 0;
  mesureAngle = 0;
  finAsservIterations = 0;
  finish = false;
  mustComputeTraj = false;
  chSysUnlock();
  while(!finish)
  {
    printErrors();
    chThdSleepMilliseconds(100);
  }
  chprintf(dbg, "Asserv fini\r\n");
  printErrors();
}

void Board::computeTraj()
{
  // Increment distance
  if(cibleDistance >= 0)
  {
    if(consigneDistance < cibleDistance)
    {
      consigneDistance += vLinMax;
    }
    else
    {
      consigneDistance = cibleDistance;
    }
  }
  else
  {
    if(consigneDistance > cibleDistance)
    {
      consigneDistance -= vLinMax;
    }
    else
    {
      consigneDistance = cibleDistance;
    }
  }
  // Increment Angle
  if(cibleAngle >= 0)
  {
    if(consigneAngle < cibleAngle)
    {
      consigneAngle += vAngMax;
    }
    else
    {
      consigneAngle = cibleAngle;
    }
  }
  else
  {
    if(consigneAngle > cibleAngle)
    {
      consigneAngle -= vAngMax;
    }
    else
    {
      consigneAngle = cibleAngle;
    }
  }
}

void Board::asserv()
{
  // Recuperation codeurs
  int32_t leftTicks;
  int32_t rightTicks;
  chSysLockFromISR();
  gBoard.qei.getValuesI(&leftTicks, &rightTicks);
  chSysUnlockFromISR();

  // Estimation deplacement
  const float dl = float(leftTicks - lastLeftTicks) *
                   LEFT_TICKS_TO_MM; // calcul du déplacement de la roue droite
  const float dr = float(rightTicks - lastRightTicks) *
                   RIGHT_TICKS_TO_MM; // calcul du déplacement de la roue gauche

  lastLeftTicks = leftTicks;
  lastRightTicks = rightTicks;

  // calcul du déplacement su robot
  const float dD = (dr + dl) / 2;
  // calcul de la variation de l'angle alpha du robot
  const float dA = (dr - dl) / wheelSeparationMM;

  // Incrementation des mesures
  mesureDistance += dD;
  mesureAngle += dA;
  mesureAngle = normalize_angle(mesureAngle);

  // Calcul des erreurs
  erreurDistance = consigneDistance - mesureDistance;
  erreurAngle = normalize_angle(consigneAngle - mesureAngle);


  //iTermDist += kiDist * erreurDistance;
  iTermAng += kiAng * erreurAngle;
  //iTermDist = bound(iTermDist, iMinDist, iMaxDist);
  iTermAng = bound(iTermAng, iMinAng, iMaxAng);

  /*const float correctionDistance =
    (kpDist * erreurDistance + kdDist * (lastErreurDistance - erreurDistance) +
    iTermDist)+ 800;*/
  float correctionDistance=0.0;

  if(erreurDistance >= 0.0)
    correctionDistance = (kpDist * erreurDistance + ( kdDist * (erreurDistance - lastErreurDistance)) )+ 700.0 + compensation;
  else
    correctionDistance = (kpDist * erreurDistance + ( kdDist * (erreurDistance - lastErreurDistance)) )- 700.0 - compensation;

  //correctionDistance = 0.0;

  float correctionAngle = 0.0;

  if(erreurAngle >= 0.0)
      correctionAngle = kpAng * erreurAngle + kdAng * (erreurAngle - lastErreurAngle) + iTermAng + 800.0 ;
  else
      correctionAngle = kpAng * erreurAngle + kdAng * (erreurAngle - lastErreurAngle) + iTermAng - 800.0 ;

  if(erreurDistance == lastErreurDistance)
  {
      ++cptRestOnPosition;
      iTermDist += kiDist * erreurDistance;
      iTermDist = bound(iTermDist, iMinDist, iMaxDist);
  }
  else
      compensation=0;

  if(cptRestOnPosition > 3)
      compensation = iTermDist;

  lastErreurDistance = erreurDistance;
  lastErreurAngle = erreurAngle;

  leftPwm =
    boundPwm(static_cast<int16_t>(correctionDistance - correctionAngle));
  rightPwm =
    boundPwm(static_cast<int16_t>(correctionDistance + correctionAngle));

  chSysLockFromISR();
  motors.pwmI(leftPwm, rightPwm);
  chSysUnlockFromISR();

  if(fabs(erreurDistance) < 1 && fabs(erreurAngle) < 0.02)
  {
    finAsservIterations++;
    if(finAsservIterations > 50)
    {
      finish = true;
      motors.pwmI(0, 0);
    }
  }
}

void
Board::lectureCodeur(){
    // Recuperation codeurs
    int32_t leftTicks;
    int32_t rightTicks;
    chSysLockFromISR();
    gBoard.qei.getValuesI(&leftTicks, &rightTicks);
    chSysUnlockFromISR();

    // Estimation deplacement
    const float dl = float(leftTicks - lastLeftTicks) *
                     LEFT_TICKS_TO_MM; // calcul du déplacement de la roue droite
    const float dr = float(rightTicks - lastRightTicks) *
                     RIGHT_TICKS_TO_MM; // calcul du déplacement de la roue gauche

    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    // calcul du déplacement su robot
    const float dD = (dr + dl) / 2;
    // calcul de la variation de l'angle alpha du robot
    const float dA = (dr - dl) / wheelSeparationMM;

    // Incrementation des mesures
    mesureDistance += dD;
    mesureAngle += dA;
    mesureAngle = normalize_angle(mesureAngle);

    //chprintf(dbg, "\nmesure dist: %f\n", mesureDistance);
    //chprintf(dbg, "mesure ang: %f\n", mesureAngle);
}

void Board::printErrors()
{
  chprintf(dbg, "\ncible dist: %f\r\n", cibleDistance);
  chprintf(dbg, "cible ang: %f\r\n", cibleAngle);
  chprintf(dbg, "consigne dist: %f\r\n", consigneDistance);
  chprintf(dbg, "consigne ang: %f\r\n", consigneAngle);
  chprintf(dbg, "mesure dist: %f\r\n", mesureDistance);
  chprintf(dbg, "mesure ang: %f\r\n", mesureAngle);
  chprintf(dbg, "erreur dist: %f\r\n", lastErreurDistance);
  chprintf(dbg, "erreur ang: %f\r\n", lastErreurAngle);
  chprintf(dbg, "left pwm: %d\r\n", leftPwm);
  chprintf(dbg, "right pwm: %d\r\n", rightPwm);
  chprintf(dbg, "finish: %d\r\n", finish);
  chprintf(dbg, "finIterations: %d\r\n", finAsservIterations);
}

int16_t Board::boundPwm(int16_t _pwm)
{
  if(_pwm > maxPwm) return maxPwm;
  if(_pwm < -maxPwm) return -maxPwm;
  return _pwm;
}

/*!
 * \brief normalize_angle_positive
 *
 *        Normalizes the angle to be 0 to 2*M_PI
 *        It takes and returns radians.
 */
float Board::normalize_angle_positive(float angle)
{
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}


/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
float Board::normalize_angle(float angle)
{
  float a = normalize_angle_positive(angle);
  if (a > M_PI)
    a -= 2.0 *M_PI;
  return a;
}

Board gBoard;
