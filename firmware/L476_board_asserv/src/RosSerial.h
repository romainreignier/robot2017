#pragma once

#include "ch.h"
#include <snd_msgs/Encoders.h>

extern THD_WORKING_AREA(waThreadRosserial, 2048);
extern THD_FUNCTION(ThreadRosserial, arg);
