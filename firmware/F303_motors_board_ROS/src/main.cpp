/**
 * Supmeca Never Dies 2017
 * \date 16/04/2017
 * \author Romain Reignier
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

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

  palSetPadMode(GPIOA, 11, PAL_MODE_OUTPUT_PUSHPULL);

  chprintf(dbg, "Supmeca Never Dies!!!!\n");

  // Creates the blinker thread.
  chThdCreateStatic(
    waThreadBlinker, sizeof(waThreadBlinker), NORMALPRIO, ThreadBlinker, NULL);

  systime_t timeLastStatus = chVTGetSystemTimeX();
  const systime_t statusPeriod = MS2ST(100);
  const systime_t feedbackPeriod = MS2ST(20);
  while(true)
  {
    systime_t time = chVTGetSystemTimeX();
    palSetPad(GPIOA, 11);
    if(time - timeLastStatus >= statusPeriod)
    {
      timeLastStatus = time;
      gBoard.publishStatus();
    }
    gBoard.publishFeedback();
    gBoard.nh.spinOnce();
    time += feedbackPeriod;
    palClearPad(GPIOA, 11);
    chThdSleepUntil(time);
  }
}
