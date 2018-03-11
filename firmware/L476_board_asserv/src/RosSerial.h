#pragma once

#include "ch.h"

extern THD_WORKING_AREA(waThreadRosserial, 2048);
extern THD_FUNCTION(ThreadRosserial, arg);
