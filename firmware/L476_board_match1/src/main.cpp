/**
 * Supmeca Never Dies 2017
 * \date 27/04/2017
 * \author Romain Reignier
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "Board.h"

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
  gBoard.servos.setPWM(gBoard.kLaunchServoId, 0, 270);
  // gBoard.main();
  while(gBoard.starter.read())
    ;
  systime_t startTime = chVTGetSystemTimeX();
  gBoard.motors.pwm(1500, 1500);
  chThdSleepMilliseconds(2000);
  gBoard.motors.pwm(0, 0);
  chThdSleepUntil(startTime + S2ST(90));
  gBoard.servos.setPWM(gBoard.kLaunchServoId, 0, 110);

  while(true)
  {
  }
}
