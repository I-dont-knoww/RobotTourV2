#include "regulators/CurrentRegulator.hpp"

#include "Constants.hpp"

#include "drivers/Motors.hpp"

#include "state/Vector.hpp"

#include <algorithm>
#include <cmath>

CurrentRegulator::CurrentRegulator(Motors& motors, float batteryVoltage)
    : m_leftController{ { Regulators::Current::kP },
                        { Regulators::Current::kI, -batteryVoltage, batteryVoltage } },
      m_rightController{ { Regulators::Current::kP },
                         { Regulators::Current::kI, -batteryVoltage, batteryVoltage } },
      m_motors{ motors } {}

float getCurrentSign(float appliedVoltage, float motorVoltage) {
    if (std::fabsf(appliedVoltage) > std::fabsf(motorVoltage))
        return appliedVoltage >= 0.0f ? 1.0f : -1.0f;
    else return motorVoltage >= 0.0f ? -1.0f : 1.0f;
}

void CurrentRegulator::update(Vec2 const& wheelSpeeds, float batteryVoltage, float dt) {
    Vec2 const rawCurrents = m_motors.current();

    Vec2 const appliedVoltages = batteryVoltage * m_motors.power();
    Vec2 const motorVoltages = Regulators::Current::kV * wheelSpeeds;

    Vec2 const signedCurrents = rawCurrents *
                                Vec2{ getCurrentSign(appliedVoltages.x, motorVoltages.x),
                                      getCurrentSign(appliedVoltages.y, motorVoltages.y) };
    Vec2 const filteredActualCurrents{ m_leftCurrentFilter.update(signedCurrents.x, dt),
                                       m_rightCurrentFilter.update(signedCurrents.y, dt) };

    Vec2 const filteredTargetCurrents{ m_leftTargetFilter.update(m_targetLeftCurrent, dt),
                                       m_rightTargetFilter.update(m_targetRightCurrent, dt) };

    float const voltageLeft = m_leftController.update(filteredTargetCurrents.x,
                                                      filteredActualCurrents.x, dt);
    float const voltageRight = m_rightController.update(filteredTargetCurrents.y,
                                                        filteredActualCurrents.y, dt);

    float const voltageToPWM = static_cast<float>(Drivers::Motors::MAX_POWER) / batteryVoltage;

    int const pwmLeft =
        static_cast<int>(std::clamp(voltageLeft, -batteryVoltage, batteryVoltage) * voltageToPWM);
    int const pwmRight =
        static_cast<int>(std::clamp(voltageRight, -batteryVoltage, batteryVoltage) * voltageToPWM);

    // std::printf(">lefttarget:%.5f\n>leftmeasured:%.5f\n>leftoutput:%.5f\n>leftrawOutput:%d\n",
    //             filteredTargetCurrents.x, filteredActualCurrents.x, voltageLeft, pwmLeft);

    // std::printf(">righttarget:%.5f\n>rightmeasured:%.5f\n>rightoutput:%.5f\n>rightrawOutput:%d\n",
    //             filteredTargetCurrents.y, filteredActualCurrents.y, voltageRight, pwmRight);

    m_motors.spin(pwmLeft, pwmRight);
}
