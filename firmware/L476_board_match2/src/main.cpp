/**
 * Supmeca Never Dies 2017
 * \date 27/04/2017
 * \author Romain Reignier
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "Board.h"

#define M_PI 3.14159265358979323846

// Green LED blinker thread
static THD_WORKING_AREA(waThreadBlinker, 256);
static THD_FUNCTION(ThreadBlinker, arg)
{
  (void)arg;
  chRegSetThreadName("blinker");
  while(true)
  {
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(10);
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(300);
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(10);
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(1000);
  }
}

systime_t startTime;

const float wheelSeparationMM = 102.1;
const int32_t encoderResolution = 2400;
const float leftWheelRadius = 26.125;
const float rightWheelRadius = 26.075;
const float LEFT_TICKS_TO_MM = (2 * M_PI * leftWheelRadius) / encoderResolution;
const float RIGHT_TICKS_TO_MM =
  (2 * M_PI * rightWheelRadius) / encoderResolution;

float kpDist = 8.0;
// float kpRot = 1.0;
float kpRot = 10000;

int16_t maxPwm = 2000;

int16_t boundPWM(int16_t _pwm)
{
  if(_pwm > maxPwm) return maxPwm;
  if(_pwm < -maxPwm) return -maxPwm;
  return _pwm;
}

void asserv(float _dist, float _angle, bool _oas = false)
{
  static int32_t oldCptR;
  static int32_t oldCptL;
  static float dr;
  static float dl;
  static float dA;
  static float dD;
  static float dX;
  static float dY;

  systime_t moveStartTime = chVTGetSystemTimeX();
  systime_t timeout = S2ST(3);
  bool firstRun = true;

  int16_t maxPWM = 2000;

  float X = 0;
  float Y = 0;
  float A = 0;

  float distance = 0;
  float rotation = 0;

  float errDist;

  bool isAlreadyBlocked = false;

  if(chVTGetSystemTimeX() - startTime > S2ST(85))
  {
    return;
  }

  do
  {
    int32_t cptL;
    int32_t cptR;
    gBoard.qei.getValues(&cptL, &cptR);

    if(firstRun)
    {
      oldCptR = cptR;
      oldCptL = cptL;
      firstRun = false;
    }

    dr = float(cptR - oldCptR) *
         LEFT_TICKS_TO_MM; // calcul du déplacement de la roue droite
    dl = float(cptL - oldCptL) *
         RIGHT_TICKS_TO_MM; // calcul du déplacement de la roue gauche

    oldCptR = cptR;
    oldCptL = cptL;

    dD = (dr + dl) / 2; // calcul du déplacement su robot
    dA = (dr - dl) /
         wheelSeparationMM; // calcul de la variation de l'angle alpha du robot

    A += dA; // calcul de la nouvelle valeur de l'angle
             // alpha du robot

    if(A > M_PI)
    {
      A -= 2 * M_PI;
    }
    else if(A <= -M_PI)
    {
      A += 2 * M_PI;
    }

    dX = cos(A) * dD; // calcul du déplacement selon X
    dY = sin(A) * dD; // calcul du déplacement selon Y

    X += dX; // calcul de la nouvelle valeur de X
    Y += dY; // calcul de la nouvelle valeur de Y

    distance += dD;
    rotation += dA;

    errDist = _dist - distance;
    float errRot = _angle - rotation;
    float correctDistance = kpDist * errDist;
    float correctRotation = kpRot * errRot;

    chprintf(dbg, "rot: %d errRot: %d correctRot %d\n", (int)(rotation *
    1000), (int)(errRot * 1000), (int)(correctRotation * 1000));
    chprintf(dbg, "X %d Y %d A %d\n", (int)(X * 1000), (int)(Y * 1000),
    (int)((A * 180) / M_PI * 1000));

    if(_oas && !gBoard.frontProximitySensor.read())
    {
      gBoard.motors.stop();
      // reset timeout
      moveStartTime = chVTGetSystemTimeX();
    }
    else
    {
      int16_t leftPwm = correctDistance - correctRotation;
      int16_t rightPwm = correctDistance + correctRotation;

      gBoard.motors.pwm(boundPWM(leftPwm), boundPWM(rightPwm));
    }
    // If distance
    if(_angle == 0)
    {
      if(errDist < 10)
      {
        chprintf(dbg, "angle==0 and errDist < 5, finish\n");
        gBoard.motors.stop();
        chThdSleepMilliseconds(500);
        gBoard.motors.brake();
        chThdSleepMilliseconds(500);
        return;
      }
    }
    // if angular
    else if(_dist == 0)
    {
      if(abs(errRot) < 0.01)
      {
        chprintf(dbg, "dist==0 and errRot < 0.01, finish\n");
        gBoard.motors.stop();
        chThdSleepMilliseconds(500);
        gBoard.motors.brake();
        chThdSleepMilliseconds(500);
        return;
      }
    }

    if(chVTGetSystemTimeX() - moveStartTime > timeout)
    {
      gBoard.motors.stop();
      return;
    }

    chThdSleepMilliseconds(20);
  } while(chVTGetSystemTimeX() - startTime < S2ST(85));
}

int main(void)
{
  // System initializations.
  halInit();
  chSysInit();
  gBoard.begin();

  // Creates the blinker thread.
  chThdCreateStatic(
    waThreadBlinker, sizeof(waThreadBlinker), NORMALPRIO, ThreadBlinker, NULL);

  DEBUG("Supmeca Never Dies!!!!");
  int16_t leftPwm = 1500;
  int16_t rightPwm = 1975;
  gBoard.servos.setPWM(gBoard.kLaunchServoId, 0, 270);
  gBoard.servos.setPWM(gBoard.kArmServoId, 0, 208);
  // gBoard.main();
  while(gBoard.starter.read())
    ;
  startTime = chVTGetSystemTimeX();
  if(gBoard.colorSwitch.read())
  {
    // Yellow
    gBoard.greenLed.set();
    asserv(160, 0);
    asserv(0, -1.3);
    asserv(500, 0, true);
    asserv(0, 1.3);
    asserv(480, 0);
    asserv(0, 1.9);
    asserv(243, 0);
    asserv(0, 1.6);
    asserv(600, 0);
    gBoard.greenLed.clear();
  }
  else
  {
    // Blue
    gBoard.greenLed.set();
    asserv(0, -1.1);
    asserv(426, 0, true);
    asserv(0, -1.3);
    asserv(500, 0, true);
    asserv(0, -1.57);
    asserv(300, 0);
    asserv(0, -1.7);
    asserv(600, 0);
    gBoard.greenLed.clear();
  }
  gBoard.motors.pwm(0, 0);
  chThdSleepUntil(startTime + S2ST(90));
  gBoard.servos.setPWM(gBoard.kLaunchServoId, 0, 110);
  while(true)
  {
  }
}
