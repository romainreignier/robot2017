/**
 * Supmeca Never Dies 2017
 * \date 15/03/2017
 * \author Romain Reignier
 */

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "Board.h"

// Green LED blinker thread
/*
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
*/

int main(void)
{
  // System initializations.
  halInit();
  chSysInit();
  gBoard.begin();

  chprintf(dbg, "Supmeca Never Dies!!!!\n");

  // Creates the blinker thread.
  // chThdCreateStatic(waThreadBlinker,
  //                   sizeof(waThreadBlinker),
  //                   NORMALPRIO,
  //                   ThreadBlinker,
  //                   NULL);

  while(true)
  {
    chThdSleepMilliseconds(500);
  }
}
