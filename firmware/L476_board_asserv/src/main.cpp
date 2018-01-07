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

#include "trajecto.h"
#include "Board.h"
#include "RosPublisher.h"

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

static int16_t maxPwm = 2000;
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
  (void)argc;
  (void)argv;
  chprintf(chp, "Kp Angulaire = %f\r\n", gBoard.kpAng);
}

static void cmd_print_kpd(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "Kp Distance = %f\r\n", gBoard.kpDist);
}

static void cmd_print_kda(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "Kd Angulaire = %f\r\n", gBoard.kdAng);
}

static void cmd_print_kdd(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "Kd Distance = %f\r\n", gBoard.kdDist);
}

static void cmd_print_kia(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "Ki Angulaire = %f\r\n", gBoard.kiAng);
}

static void cmd_print_kid(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "Ki Distance = %f\r\n", gBoard.kiDist);
}

static void cmd_vlinmax(BaseSequentialStream* chp, int argc, char* argv[])
{
  if(argc > 0)
  {
    const float val = atof(argv[0]);
    gBoard.vLinMax = val / (1 / (gBoard.kPidTimerPeriodMs * 0.001f));
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
    gBoard.vAngMax = val / (1 / (gBoard.kPidTimerPeriodMs * 0.001f));
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
  const float valRad = val * gBoard.kPi / 180.0f;
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
  const float valRad = val * gBoard.kPi / 180.0f;
  chprintf(chp, "tourne de %f degres -> %f rad\r\n", val, valRad);
  gBoard.moveAngularEchelon(valRad);
}

static void cmd_dist(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "Lecture Distance= %f\r\n", gBoard.mesureDistance);
}

static void cmd_graph(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  chprintf(chp, "lancement fonction tracage de courbe moteur\r\n");
  gBoard.needMotorGraph();
}

static void cmd_trace(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  (void) chp;

    while(true){
        chprintf(dbg, "\n\nG_X_mm : %f\r\n", gBoard.G_X_mm);
        chprintf(dbg, "G_Y_mm: %f\r\n", gBoard.G_Y_mm);
        chprintf(dbg, "G_Theta_rad[deg]: %f\r\n", gBoard.G_Theta_rad * RTOD);
        chThdSleepMilliseconds(200);
    }
}

static void cmd_square(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  (void) chp;
  MoveSquare();
}

static void cmd_arm_auto(BaseSequentialStream* chp, int argc, char* argv[])
{
  (void)argc;
  (void)argv;
  (void) chp;
  while(1){
    if(gBoard.starter.read())
        gBoard.servos.setPWM(gBoard.kLaunchServoId, 0, 270);
    else
        gBoard.servos.setPWM(gBoard.kLaunchServoId, 0, 110);
  }

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
                                        {"graph", cmd_graph},
                                        {"trace", cmd_trace},
                                        {"square", cmd_square},
                                        {"auto", cmd_arm_auto},
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

  chThdCreateStatic(waThreadRosserial,
                    sizeof(waThreadRosserial),
                    NORMALPRIO,
                    ThreadRosserial,
                    NULL);

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
  }
}
