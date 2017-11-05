/**
 * Supmeca Never Dies 2017
 * \date 27/05/2017
 * \author Romain Reignier
 */

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "shell.h"

#include <stdlib.h>
#include <string.h>

#include "RosPublisher.h"
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

systime_t startTime;
int16_t maxPwm = 2000;
int16_t boundPWM(int16_t _pwm)
{
  if(_pwm > maxPwm) return maxPwm;
  if(_pwm < -maxPwm) return -maxPwm;
  return _pwm;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/
#define SHELL_WA_SIZE THD_WORKING_AREA_SIZE(2048)

static void cmd_kpa(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.kpAng = val;
    chprintf(chp, "Kp Angulaire = %f\r\n", val);
  }
}

static void cmd_kpd(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.kpDist = val;
    chprintf(chp, "Kp Distance = %f\r\n", val);
  }
}

static void cmd_kda(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.kdAng = val;
    chprintf(chp, "Kd Angulaire = %f\r\n", val);
  }
}

static void cmd_kdd(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.kdDist = val;
    chprintf(chp, "Kd Distance = %f\r\n", val);
  }
}

static void cmd_kia(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.kiAng = val;
    chprintf(chp, "Ki Angulaire = %f\r\n", val);
  }
}

static void cmd_kid(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.kiDist = val;
    chprintf(chp, "Ki Distance = %f\r\n", val);
  }
}

static void cmd_print_kpa(BaseSequentialStream* chp, int argc, char* argv[])
{
    chprintf(chp, "Kp Angulaire = %f\r\n", gBoard.kpAng);
}

static void cmd_print_kpd(BaseSequentialStream* chp, int argc, char* argv[])
{
    chprintf(chp, "Kp Distance = %f\r\n", gBoard.kpDist);
}

static void cmd_print_kda(BaseSequentialStream* chp, int argc, char* argv[])
{
    chprintf(chp, "Kd Angulaire = %f\r\n", gBoard.kdAng);
}

static void cmd_print_kdd(BaseSequentialStream* chp, int argc, char* argv[])
{
    chprintf(chp, "Kd Distance = %f\r\n", gBoard.kdDist);
}

static void cmd_print_kia(BaseSequentialStream* chp, int argc, char* argv[])
{
    chprintf(chp, "Ki Angulaire = %f\r\n", gBoard.kiAng);
}

static void cmd_print_kid(BaseSequentialStream* chp, int argc, char* argv[])
{
    chprintf(chp, "Ki Distance = %f\r\n", gBoard.kiDist);
}

static void cmd_vlinmax(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.vLinMax = val / (1 / (gBoard.pidTimerPeriodMs * 0.001));
    chprintf(chp,
             "Vitesse Lineaire Max = %f mm/s -> %f mm/periode\r\n",
             val,
             gBoard.vLinMax);
  }
}

static void cmd_vangmax(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.vAngMax = val / (1 / (gBoard.pidTimerPeriodMs * 0.001));
    chprintf(chp,
             "Vitesse Angulaire Max = %f rad/s -> %f rad/Periode\r\n",
             val,
             gBoard.vAngMax);
  }
}

static void cmd_pwmmax(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const int val = atoi(argv[0]);
    gBoard.maxPwm = val;
    chprintf(chp, "PWM Max = %d\r\n", val);
  }
}

static void cmd_avance(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc < 1)
  {
    chprintf(chp, "USAGE: avance [dist_mm]\r\n");
    return;
  }
  const float val = atof(argv[0]);
  chprintf(chp, "Avance de %f mm\r\n", val);
  gBoard.moveLinear(val);
}

static void cmd_tourne(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc < 1)
  {
    chprintf(chp, "USAGE: tourne [angle_rad]\r\n");
    return;
  }
  const float val = atof(argv[0]);
  chprintf(chp, "tourne de %f rad\r\n", val);
  gBoard.moveAngular(val);
}

static void cmd_tourne_deg(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc < 1)
  {
    chprintf(chp, "USAGE: tourne_deg [angle_deg]\r\n");
    return;
  }
  const float val = atof(argv[0]);
  const float valRad = val * gBoard.kPi / 180.0;
  chprintf(chp, "tourne de %f degres -> %f rad\r\n", val, valRad);
  gBoard.moveAngular(valRad);
}

static void cmd_avance_ech(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc < 1)
  {
    chprintf(chp, "USAGE: avance_ech [dist_mm]\r\n");
    return;
  }
  const float val = atof(argv[0]);
  chprintf(chp, "Avance de %f mm\r\n", val);
  gBoard.moveLinearEchelon(val);
}

static void cmd_tourne_ech(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc < 1)
  {
    chprintf(chp, "USAGE: tourne_ech [angle_rad]\r\n");
    return;
  }
  const float val = atof(argv[0]);
  chprintf(chp, "tourne de %f rad\r\n", val);
  gBoard.moveAngularEchelon(val);
}

static void cmd_tourne_ech_deg(BaseSequentialStream* chp, int argc,
                               char* argv[])
{
  if(argc < 1)
  {
    chprintf(chp, "USAGE: tourne_ech_deg [angle_deg]\r\n");
    return;
  }
  const float val = atof(argv[0]);
  const float valRad = val * gBoard.kPi / 180.0;
  chprintf(chp, "tourne de %f degres -> %f rad\r\n", val, valRad);
  gBoard.moveAngularEchelon(valRad);
}

static void cmd_dist(BaseSequentialStream* chp, int argc, char* argv[])
{
 chprintf(chp, "Lecture Distance= %f\r\n", gBoard.mesureDistance);
}

static const ShellCommand commands[] = {{"kpd", cmd_kpd},
                                        {"kid", cmd_kid},
                                        {"kdd", cmd_kdd},
                                        {"kpa", cmd_kpa},
                                        {"kia", cmd_kia},
                                        {"kda", cmd_kda},
                                        {"kpd?", cmd_print_kpd},
                                        {"kid?", cmd_print_kid},
                                        {"kdd?", cmd_print_kdd},
                                        {"kpa?", cmd_print_kpa},
                                        {"kia?", cmd_print_kia},
                                        {"kda?", cmd_print_kda},
                                        {"vlinmax", cmd_vlinmax},
                                        {"vangmax", cmd_vangmax},
                                        {"pwmmax", cmd_pwmmax},
                                        {"av", cmd_avance},
                                        {"to", cmd_tourne},
                                        {"tod", cmd_tourne_deg},
                                        {"ave", cmd_avance_ech},
                                        {"toe", cmd_tourne_ech},
                                        {"toed", cmd_tourne_ech_deg},
                                        {"dist", cmd_dist},
                                        {NULL, NULL}};

static const ShellConfig shell_cfg1 = {(BaseSequentialStream*)&SD2, commands};

int main(void)
{
  // System initializations.
  halInit();
  chSysInit();
  gBoard.begin();
  shellInit();

  // Creates the blinker thread.
  chThdCreateStatic(
    waThreadBlinker, sizeof(waThreadBlinker), NORMALPRIO, ThreadBlinker, NULL);

  chThdCreateStatic(
    waThreadRosserial, sizeof(waThreadRosserial), NORMALPRIO, ThreadRosserial, NULL);

  DEBUG("Supmeca Never Dies!!!!");

  while(true)
  {
    thread_t* shelltp = chThdCreateFromHeap(NULL,
                                            SHELL_WA_SIZE,
                                            "shell",
                                            NORMALPRIO + 1,
                                            shellThread,
                                            (void*)&shell_cfg1);
    chThdWait(shelltp); /* Waiting termination. */
    //chThdSleepMilliseconds(2000);
    //chSysLockFromISR();
    //gBoard.motors.pwmI(620, 620);
    //chSysUnlockFromISR();
    //DEBUG("FIN!!!!");
    //chThdSleepMilliseconds(500);
    //gBoard.motors.pwmI(0, 0);
    //chThdSleepMilliseconds(1000);
  }
}
