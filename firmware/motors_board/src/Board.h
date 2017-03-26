#pragma once

#include "Communication.h"
#include "MonsterShield.h"
#include "Qei.h"
#include "SerialComm.h"

#define SERIAL_DRIVER SD2
#define DEBUG_DRIVER SD3

extern BaseSequentialStream* dbg;
extern BaseSequentialStream* ser;

// Here we use a struct instead of a class to ease the use of the object
// So no need to use getters
// Note that a Singleton Design Pattern could be used but
// it needs dynamic memory allocation to be used.
struct Board
{
  Board();
  void begin();

  MonsterShield leftMotor;
  MonsterShield rightMotor;
  Qei qei;
  SerialComm serial;
  Communication comm;
};

extern Board gBoard;
