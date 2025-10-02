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

#include "kinematics/ForwardKinematics.hpp"

#include "managers/Follower.hpp"

#include "path/Competition.hpp"

#include "regulators/CurrentRegulator.hpp"
#include "regulators/VelocityRegulator.hpp"

#include "storage/Atomic.hpp"
#include "storage/Flash.hpp"

#include "hardware/timer.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <cstdint>
#include <cstdio>
#include <optional>

void testLedRGB() {
    stdio_init_all();

    // Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    // Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT,
    // Pins::Encoders::SCK,
    //                    Pins::Encoders::MISO };
    // ForwardKinematics forwardKinematics{ encoders.data() };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    // Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
    //                Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
    //                Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    // CurrentRegulator currentRegulator{ motors, battery.voltage() };
    // VelocityRegulator velocityRegulator{};

    // Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    constexpr float MAX = 255.0f;

    while (true) {
        for (uint16_t i = 0u; i < MAX; ++i) {
            ledRGB.setRGB({ i / MAX, 0, 1.0f - (i / MAX) });
            sleep_ms(1);
        }
        for (uint16_t i = 0u; i < MAX; ++i) {
            ledRGB.setRGB({ 1.0f - (i / MAX), i / MAX, 0 });
            sleep_ms(1);
        }
        for (uint16_t i = 0u; i < MAX; ++i) {
            ledRGB.setRGB({ 0, 1.0f - (i / MAX), i / MAX });
            sleep_ms(1);
        }
    }
}

void testEncoders() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };
    ForwardKinematics forwardKinematics{ encoders.data() };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    while (true) {
        auto const& angle = encoders.data();

        std::printf(">left:%.8f\n>right:%.8f\n", angle.x, angle.y);
        sleep_ms(100);
    }
}

void testMotors() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };
    RCFilter left{ 25.0f };
    RCFilter right{ 25.0f };
    ForwardKinematics forwardKinematics{ encoders.data() };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    time.reset();
    while (true) {
        for (uint16_t i = 0u; i < Drivers::Motors::MAX_POWER; ++i) {
            time.update();
            // motors.spin(i, Drivers::Motors::MAX_POWER - i);

            auto const& current = motors.current();
            float filteredLeft = left.update(current.x, time.delta());
            float filteredRight = right.update(current.y, time.delta());
            std::printf(
                ">filteredLeft:%.5f\n>filteredRight:%.5f\n>currentLeft:%.5f\n>currentRight:%.5f\n",
                filteredLeft, filteredRight, current.x, current.y);

            sleep_ms(1);
        }
        for (uint16_t i = 0u; i < Drivers::Motors::MAX_POWER; ++i) {
            time.update();
            // motors.spin(Drivers::Motors::MAX_POWER - i, i);

            auto const& current = motors.current();
            float filteredLeft = left.update(current.x, time.delta());
            float filteredRight = right.update(current.y, time.delta());
            std::printf(
                ">filteredLeft:%.5f\n>filteredRight:%.5f\n>currentLeft:%.5f\n>currentRight:%.5f\n",
                filteredLeft, filteredRight, current.x, current.y);

            sleep_ms(1);
        }
    }
}

void testEncodersAndMotors() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };
    ForwardKinematics forwardKinematics{ encoders.data() };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    while (true) {
        auto const& angle = encoders.data();

        std::printf(">left:%.5f\n>right:%.5f\n", angle.x, angle.y);
        sleep_ms(10);
    }
}

void testGyroscope() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    ForwardKinematics forwardKinematics{ encoders.data() };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    Gyroscope gyroscope{ pio0,
                         Pins::Gyroscope::CS,
                         Pins::Gyroscope::SCK,
                         Pins::Gyroscope::MISO,
                         Pins::Gyroscope::MOSI,
                         Pins::Gyroscope::INT };

    time.reset();
    while (true) {
        time.update();

        // auto const& wheels = encoders.data();

        std::printf(">angularVelocity:%.5f\n", gyroscope.angularVelocity());
        // std::printf(">leftWheelSpeed:%.5f\n>rightWheelSpeed:%.5f\n", wheels.x, wheels.y);

        sleep_ms(10);
    }
}

void testCompass() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    Gyroscope gyroscope{ pio0,
                         Pins::Gyroscope::CS,
                         Pins::Gyroscope::SCK,
                         Pins::Gyroscope::MISO,
                         Pins::Gyroscope::MOSI,
                         Pins::Gyroscope::INT };
    ForwardKinematics forwardKinematics{ encoders.data() };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    ledRGB.setRGB(Status::READY_FOR_MOTION_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTION_CALIBRATING);

    Compass compass{ gyroscope.down(), Pins::Compass::SDA, Pins::Compass::SCL };
    // Compass::Calibration compassCalibration =
    //     compass.getCalibration([&startButton](Vec3 const& sample, Vec3 const& minimums,
    //                                           Vec3 const& maximums, int sampleCount) {
    //         if (sampleCount % 40 == 0) {
    //             std::printf(">samplexy:%.5f:%.5f|xy\n", sample.x, sample.y);
    //             std::printf(">sampleyz:%.5f:%.5f|xy\n", sample.y, sample.z);
    //             std::printf(">samplexz:%.5f:%.5f|xy\n", sample.x, sample.z);

    //             // std::printf(">minimumsxy:%.5f:%.5f|xy\n", minimums.x, minimums.y);
    //             // std::printf(">minimumsyz:%.5f:%.5f|xy\n", minimums.y, minimums.z);
    //             // std::printf(">minimumsxz:%.5f:%.5f|xy\n", minimums.x, minimums.z);

    //             // std::printf(">maximumsxy:%.5f:%.5f|xy\n", maximums.x, maximums.y);
    //             // std::printf(">maximumsyz:%.5f:%.5f|xy\n", maximums.y, maximums.z);
    //             // std::printf(">maximumsxz:%.5f:%.5f|xy\n", maximums.x, maximums.z);
    //         }
    //         return startButton.isPressed();
    //     });
    // Flash::store(compassCalibration);
    compass.setCalibration(Flash::load<Compass::Calibration>());
    compass.switchToPIO(pio1, Pins::Compass::SDA, Pins::Compass::SCL);

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    Time time{};

    time.reset();
    while (true) {
        time.update();

        auto const data = compass.getData();
        std::printf(">heading:%.5f\n", data);

        sleep_ms(50);
    }
}

void testForwardKinematics() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    ForwardKinematics forwardKinematics{ encoders.data() };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    time.reset();
    while (true) {
        for (int i = 0; i < 10000; ++i) {
            time.update();
            forwardKinematics.update(encoders.data(), std::nullopt, time.delta());
        }

        auto const& state = forwardKinematics.state();

        std::printf(">location:%.5f:%.5f|xy\n", state.position.x, state.position.y);
        std::printf(">x:%.5f\n>y:%.5f\n>angle:%.5f\n>anglularVelocity:%.5f\n", state.position.x,
                    state.position.y, state.angle, state.angularVelocity);
        std::printf(">xVel:%.5f\n>yVel:%.5f\n", state.velocity.x, state.velocity.y);
        std::printf(">leftWheelSpeed:%.5f\n>rightWheelSpeed:%.5f\n", state.wheelSpeeds.x,
                    state.wheelSpeeds.y);
    }
}

void testCurrentRegulator() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    ForwardKinematics forwardKinematics{ encoders.data() };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    currentRegulator.spin(-0.1f);

    time.reset();

    while (true) {
        time.update();

        forwardKinematics.update(encoders.data(), std::nullopt, time.delta());
        currentRegulator.update(forwardKinematics.state().wheelSpeeds, battery.voltage(),
                                time.delta());
        sleep_us(500);
    }
}

void testVelocityRegulator() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };
    ForwardKinematics forwardKinematics{ encoders.data() };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    time.reset();

    velocityRegulator.move(0.0f, Constants::PI / 2.0f);

    while (true) {
        time.update();

        forwardKinematics.update(encoders.data(), std::nullopt, time.delta());
        auto const& state = forwardKinematics.state();

        std::printf(">angle:%.5f\n", state.angle);

        Vec2 const targetWheelSpeeds = velocityRegulator.update(
            state.velocity, state.angle, state.angularVelocity, time.delta());
        currentRegulator.spin(targetWheelSpeeds.x, targetWheelSpeeds.y);
        currentRegulator.update(state.wheelSpeeds, battery.voltage(), time.delta());
    }
}

void testRun() {
    stdio_init_all();

    Battery battery{ Pins::Battery::VOLTAGE_SENSE };
    Button startButton{ Pins::BUTTON };
    LedRGB ledRGB{ Pins::LedRGB::RED, Pins::LedRGB::GREEN, Pins::LedRGB::BLUE };
    Encoders encoders{ pio0, Pins::Encoders::CS_LEFT, Pins::Encoders::CS_RIGHT, Pins::Encoders::SCK,
                       Pins::Encoders::MISO };

    ledRGB.setRGB(Status::READY_FOR_MOTIONLESS_CALIBRATION);
    startButton.waitForClick();
    ledRGB.setRGB(Status::MOTIONLESS_CALIBRATING);

    ForwardKinematics forwardKinematics{ encoders.data() };
    Motors motors{ Pins::Motors::LEFT_MOTOR_IN1,     Pins::Motors::LEFT_MOTOR_IN2,
                   Pins::Motors::RIGHT_MOTOR_IN1,    Pins::Motors::RIGHT_MOTOR_IN2,
                   Pins::Motors::LEFT_MOTOR_CURRENT, Pins::Motors::RIGHT_MOTOR_CURRENT };
    CurrentRegulator currentRegulator{ motors, battery.voltage() };
    VelocityRegulator velocityRegulator{};

    Time time{};

    ledRGB.setRGB(Status::READY_TO_RUN);
    startButton.waitForClick();
    ledRGB.setRGB(Status::RUNNING);

    Follower follower{ Competition::PATH, Competition::TARGET_TIMES };

    time.reset();
    while (!follower.finished()) {
        time.update();

        forwardKinematics.update(encoders.data(), std::nullopt, time.delta());
        auto const& state = forwardKinematics.state();

        Vec2 const targetSpeeds = follower.update(state.position, state.angle, time.elapsed(),
                                                  time.delta());
        velocityRegulator.move(targetSpeeds.x, targetSpeeds.y);

        Vec2 const targetWheelSpeeds = velocityRegulator.update(
            state.velocity, state.angle, state.angularVelocity, time.delta());
        currentRegulator.spin(targetWheelSpeeds.x, targetWheelSpeeds.y);
        currentRegulator.update(state.wheelSpeeds, battery.voltage(), time.delta());

        // std::printf(">x:%.5f\n>y:%.5f\n>angle:%.5f\n", state.position.x, state.position.y,
        //             state.angle);
        sleep_us(500);
    }

    motors.spin(0.0f);
    ledRGB.setRGB(Status::FINISHED);
    Vec2 const finishPosition = forwardKinematics.state().position;
    float const finishTime = time.elapsed();

    while (true) {
        std::printf("Finished with time %.5f and position (%.5f, %.5f)!\n", finishTime,
                    finishPosition.x, finishPosition.y);
        sleep_ms(1000);
    }
}

int main() { testCompass(); }
