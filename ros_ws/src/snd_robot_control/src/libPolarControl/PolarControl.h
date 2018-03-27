#pragma once

#include <tuple>

#include "Component.h"
#include "Pose.h"

class PolarControl
{
public:

    enum
    {
        LEFT,
        RIGHT
    };

    enum class State
    {
        ROTATION_ALONG_PATH,
        TRANSLATION_ALONG_PATH,
        ROTATION_ALONG_TARGET,
    };

    PolarControl();
    void setMaxOutput(float _max);
    void setTargetPose(const Pose& _target);
    std::pair<float, float> computeMotorsCommands(const Pose& _currentPose);
    bool isGoalReached() const { return m_isGoalReached; }
    Component& getLinearComponent() { return m_linear; }
    Component& getAngularComponent() { return m_angular; }

protected:
    float bound(float _in, float _min, float _max);

protected:
    State m_state = State::ROTATION_ALONG_PATH;
    Pose m_targetPose;
    Component m_linear;
    Component m_angular;
    float m_maxOutput = 1;
    int m_endControlIterations = 0;
    int m_numberIterationsEndControl = 100;
    bool m_isGoalReached;
};
