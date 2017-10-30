#pragma once

#include "Motors.h"
#include "L298.h"

class L298Motors : public Motors
{
public:
    L298Motors(L298& _leftMotor, L298& _rightMotor);
    void begin() override;
};
