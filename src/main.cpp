#include "Constants.hpp"

#include "drivers/Battery.hpp"
#include "drivers/Button.hpp"
#include "drivers/Compass.hpp"
#include "drivers/Encoders.hpp"
#include "drivers/Gyroscope.hpp"
#include "drivers/LedRGB.hpp"
#include "drivers/Motors.hpp"
#include "drivers/Time.hpp"

#include "filters/RCFilter.hpp"

#include "fusion/Fusion.hpp"

#include "kinematics/ForwardKinematics.hpp"

#include "managers/Follower.hpp"

#include "path/Competition.hpp"

#include "regulators/CurrentRegulator.hpp"
#include "regulators/VelocityRegulator.hpp"

#include "storage/Atomic.hpp"
// #include "storage/Flash.hpp"

#include "hardware/irq.h"
#include "hardware/timer.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>

static Atomic<ForwardKinematics::State> atomicForwardKinematicsState{};

static bool volatile core1Ready = false;

void core0() {
    auto core0Loop = [&]() -> void {
        auto const state = atomicForwardKinematicsState.load();

        std::printf(">location:%.5f:%.5f|xy\n", state.position.x, state.position.y);
        std::printf(">x:%.5f\n>y:%.5f\n>angle:%.5f\n>anglularVelocity:%.5f\n", state.position.x,
                    state.position.y, state.angle, state.angularVelocity);
        std::printf(">xVel:%.5f\n>yVel:%.5f\n", state.velocity.x, state.velocity.y);
        std::printf(">leftWheelSpeed:%.5f\n>rightWheelSpeed:%.5f\n", state.wheelSpeeds.x,
                    state.wheelSpeeds.y);
    };

    auto thunk = [](repeating_timer_t* timer) -> bool {
        (*static_cast<decltype(core0Loop)*>(timer->user_data))();
        return true;
    };

    alarm_pool_t* core0AlarmPool = alarm_pool_create_with_unused_hardware_alarm(1u);
    irq_set_priority(hardware_alarm_get_irq_num(alarm_pool_hardware_alarm_num(core0AlarmPool)),
                     0b11000000);

    while (!core1Ready) tight_loop_contents();

    repeating_timer_t timer;
    alarm_pool_add_repeating_timer_us(core0AlarmPool, -Integration::SLOW_LOOP_US, thunk, &core0Loop,
                                      &timer);

    while (true) tight_loop_contents();
}

void core1() {
    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button button{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };

    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };

    Fusion fusion{};
    ForwardKinematics forwardKinematics{ encoders.data() };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    button.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    Gyroscope gyroscope{ pio0,
                         Pins::Gyroscope::CS,
                         Pins::Gyroscope::SCK,
                         Pins::Gyroscope::MISO,
                         Pins::Gyroscope::MOSI,
                         Pins::Gyroscope::INT };

    ledRGB.setRGB(Status::READY_TO_RUN);
    button.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    auto core1Loop = [&]() -> void {
        forwardKinematics.update(
            encoders.data(), fusion.update(gyroscope.angularVelocity(), Integration::FAST_LOOP_DT),
            gyroscope.angularVelocity(), Integration::FAST_LOOP_DT);
        atomicForwardKinematicsState.store(forwardKinematics.state());
    };

    auto thunk = [](repeating_timer_t* timer) -> bool {
        (*static_cast<decltype(core1Loop)*>(timer->user_data))();
        return true;
    };

    alarm_pool_t* core1AlarmPool = alarm_pool_create_with_unused_hardware_alarm(1u);
    irq_set_priority(hardware_alarm_get_irq_num(alarm_pool_hardware_alarm_num(core1AlarmPool)),
                     0b11000000);

    core1Ready = true;

    repeating_timer_t timer;
    alarm_pool_add_repeating_timer_us(core1AlarmPool, -Integration::FAST_LOOP_US, thunk, &core1Loop,
                                      &timer);

    while (true) tight_loop_contents();
}

int main() {
    stdio_init_all();

    multicore_launch_core1(core1);
    core0();
}
