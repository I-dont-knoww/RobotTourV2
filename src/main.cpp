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

#include "hardware/irq.h"
#include "hardware/timer.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <cstdint>
#include <cstdio>

static Atomic<ForwardKinematics::State> atomicForwardKinematicsState{};

enum CoreStatus : uint8_t {
    INITIALIZING = 0u,
    INITIALIZED = 1u,
    CALIBRATING = 2u,
    CALIBRATED = 3u,
    STARTING = 4u,
    RUNNING = 5u,
    FINISHED = 6u,
};
static CoreStatus volatile core0Status = INITIALIZING;
static CoreStatus volatile core1Status = INITIALIZING;

void core0() {
    Button button{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };

    CurrentRegulator currentRegulator{};
    VelocityRegulator velocityRegulator{};
    Follower follower{ Competition::PATH, Competition::TARGET_TIMES };

    core0Status = INITIALIZED;
    while (core1Status < INITIALIZED) tight_loop_contents();

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    button.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);
    core0Status = CALIBRATING;

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };

    core0Status = CALIBRATED;
    while (core1Status < CALIBRATED) tight_loop_contents();

    ledRGB.setRGB(Status::READY_TO_RUN);
    button.waitForClick();
    ledRGB.setRGB(Status::RUNNING);
    core0Status = STARTING;

    Time time{};
    time.reset();

    core0Status = RUNNING;

    auto core0Loop = [&]() {
        auto const state = atomicForwardKinematicsState.load();

        time.update();
        Vec2 const targetSpeeds = follower.update(state, time.elapsed(), Integration::SLOW_LOOP_DT);

        velocityRegulator.setTargets(targetSpeeds.x, targetSpeeds.y);
        Vec2 const targetVoltages = velocityRegulator.update(
            state.velocity, state.angle, state.angularVelocity, battery.voltage(),
            Integration::SLOW_LOOP_DT);

        currentRegulator.setTargetVoltage(targetVoltages);
        Vec2 const motorVoltages = currentRegulator.update(state.wheelSpeeds, battery.voltage(),
                                                           Integration::SLOW_LOOP_DT);
        motors.spin(static_cast<int>(motorVoltages.x), static_cast<int>(motorVoltages.y));

        if (follower.finished()) {
            core0Status = FINISHED;
            return false;
        }
        return true;
    };

    auto thunk = [](repeating_timer_t* timer) {
        return (*static_cast<decltype(core0Loop)*>(timer->user_data))();
    };

    alarm_pool_t* core0AlarmPool = alarm_pool_create_with_unused_hardware_alarm(1u);
    irq_set_priority(hardware_alarm_get_irq_num(alarm_pool_hardware_alarm_num(core0AlarmPool)),
                     0b11110000);

    repeating_timer_t timer{};
    alarm_pool_add_repeating_timer_us(core0AlarmPool, -Integration::SLOW_LOOP_US, thunk, &core0Loop,
                                      &timer);

    while (core0Status < FINISHED) tight_loop_contents();

    motors.spin(0.0f);
    ledRGB.setRGB(Status::FINISHED);

    float const finalTime = time.elapsed();
    sleep_ms(static_cast<uint32_t>(Integration::FINAL_STATE_MEASUREMENT_DELAY * 1000.0f));
    Vec2 const finalPosition = atomicForwardKinematicsState.load().position;
    float const finalAngle = atomicForwardKinematicsState.load().angle;

    while (true) {
        std::printf("Finished with position (%.5f, %.5f), angle %.5f, and time %.5f.\n",
                    finalPosition.x, finalPosition.y, finalAngle, finalTime);
        sleep_ms(1000);
    }
}

void core1() {
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };
    Fusion fusion{};

    core1Status = INITIALIZED;
    while (core0Status < CALIBRATING) tight_loop_contents();
    core1Status = CALIBRATING;

    sleep_ms(static_cast<uint32_t>(Integration::CALIBRATION_DELAY * 1000.0f));

    ForwardKinematics forwardKinematics{ encoders.data() };
    Gyroscope gyroscope{ pio0,
                         Pins::Gyroscope::CS,
                         Pins::Gyroscope::SCK,
                         Pins::Gyroscope::MISO,
                         Pins::Gyroscope::MOSI,
                         Pins::Gyroscope::INT };

    core1Status = CALIBRATED;
    while (core0Status < RUNNING) tight_loop_contents();

    auto core1Loop = [&]() {
        float const angularVelocity = gyroscope.angularVelocity();
        float const angle = fusion.update(angularVelocity, Integration::FAST_LOOP_DT);

        forwardKinematics.update(encoders.data(), angle, angularVelocity,
                                 Integration::FAST_LOOP_DT);

        auto const& state = forwardKinematics.state();
        atomicForwardKinematicsState.store(forwardKinematics.state());
    };

    auto thunk = [](repeating_timer_t* timer) {
        (*static_cast<decltype(core1Loop)*>(timer->user_data))();
        return true;
    };

    alarm_pool_t* core1AlarmPool = alarm_pool_create_with_unused_hardware_alarm(1u);
    irq_set_priority(hardware_alarm_get_irq_num(alarm_pool_hardware_alarm_num(core1AlarmPool)),
                     0b11110000);

    repeating_timer_t timer{};
    alarm_pool_add_repeating_timer_us(core1AlarmPool, -Integration::FAST_LOOP_US, thunk, &core1Loop,
                                      &timer);

    while (true) tight_loop_contents();
}

int main() {
    stdio_init_all();

    multicore_launch_core1(core1);
    core0();
}
