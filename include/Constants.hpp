#pragma once

#include "state/Vector.hpp"

#include <cstddef>
#include <cstdint>
#include <numbers>

using uint = unsigned int;

namespace Constants {
    inline constexpr float PI = std::numbers::pi_v<float>;
    inline constexpr float E = std::numbers::e_v<float>;
}

namespace Drivers {
    namespace Battery {
        inline constexpr size_t SAMPLE_COUNT = 16384u;

        inline constexpr float R1_VALUE = 20.0e3f;
        inline constexpr float R2_VALUE = 5.1e3f;

        inline constexpr float DIVIDER_VALUE = R2_VALUE / (R1_VALUE + R2_VALUE);
    }

    namespace Compass {
        inline constexpr uint8_t I2C_ADDRESS = 0x7c;
    }

    namespace Gyroscope {
        inline constexpr float RESOLUTION = 131.0f;

        inline constexpr size_t GYRO_CALIBRATION_SAMPLE_COUNT = 2000u;
        inline constexpr size_t DOWN_DIRECTION_SAMPLE_COUNT = 2000u;
    }

    namespace LedRGB {
        inline constexpr uint16_t MAX_BRIGHTNESS = 255u;
        inline constexpr uint16_t MAX_POWER = 1024u;
        inline constexpr uint32_t PWM_FREQUENCY = 100u;

        inline constexpr uint16_t PWM_WRAP = MAX_POWER - 1u;
        inline constexpr uint32_t TARGET_CLK_FREQUENCY = PWM_FREQUENCY * MAX_POWER;
    }

    namespace Motors {
        inline constexpr uint16_t MAX_POWER = 16384u;
        inline constexpr size_t SAMPLE_COUNT = 4096u;

        inline constexpr uint32_t PWM_FREQUENCY = 8'000u;

        inline constexpr uint16_t PWM_WRAP = MAX_POWER - 1u;
        inline constexpr uint32_t TARGET_CLK_FREQUENCY = PWM_FREQUENCY * MAX_POWER;
    }
}

namespace Kinematics {
    inline constexpr float AXLE_LENGTH = 13.35f;
    inline constexpr float WHEEL_RADIUS = 3.01625f;

    namespace Forward {
        inline constexpr size_t MA_FILTER_LENGTH = 50u;
        inline constexpr float RC_FILTER_CUTOFF_FREQUENCY = 10u;
    }

    namespace Inverse {
        inline constexpr float RAD_PER_SEC_12V = 95.984f;
    }
}

namespace Manager {
    namespace Follower {
        inline constexpr float DISTANCE_THRESHOLD_ACCURATE = 1.0f;
        inline constexpr float DISTANCE_THRESHOLD_FAST = 3.0f;
        inline constexpr float TURNING_RADIUS = 3.0f;
        // inline constexpr float DISTANCE_THRESHOLD_ACCURATE = 0.0f;
        // inline constexpr float DISTANCE_THRESHOLD_FAST = 0.0f;
        // inline constexpr float TURNING_RADIUS = 0.0f;

        inline constexpr float ANGLE_THRESHOLD = 0.05f;

        inline constexpr float ANGLE_CONTROL_MIN_POWER_BUDGET = 0.7f;
        inline constexpr float LINEAR_CONTROL_POWER_BUDGET = 1.0f - ANGLE_CONTROL_MIN_POWER_BUDGET;
    }

    namespace Heading {
        inline constexpr float kP = 4.0f;
    }

    namespace Position {
        inline constexpr float kP = 3.0f;
        inline constexpr float kI = 0.001f;

        inline constexpr float MIN_SPEED = 3.0f;
        inline constexpr float INTEGRATOR_BOUND = 0.05f;
    }

    namespace Rotation {
        inline constexpr float kP = 5.0f;
        inline constexpr float kI = 1.0f;

        inline constexpr float INTEGRATOR_BOUND = 1.0f;
        // inline constexpr float kP = 0.0f;
    }

    namespace Speed {
        inline constexpr float kP = 0.3f;
        inline constexpr float kD = 0.4f;

        inline constexpr float MAX_SPEED = 50.0f;

        inline constexpr float GIVE_UP_TIME = 0.1f;
    }
}

namespace Pins {
    namespace Battery {
        inline constexpr uint VOLTAGE_SENSE = 27u;
    }

    namespace Compass {
        inline constexpr uint SDA = 8u;
        inline constexpr uint SCL = 5u;
    }

    inline constexpr uint BUTTON = 4u;

    namespace Encoders {
        inline constexpr uint CS_LEFT = 0u;
        inline constexpr uint CS_RIGHT = 1u;
        inline constexpr uint SCK = 2u;
        inline constexpr uint MISO = 3u;
    }

    namespace Gyroscope {
        inline constexpr uint CS = 15u;
        inline constexpr uint SCK = 14u;
        inline constexpr uint MISO = 12u;
        inline constexpr uint MOSI = 11u;
        inline constexpr uint INT = 10u;
    }

    namespace LedRGB {
        inline constexpr uint RED = 16u;
        inline constexpr uint GREEN = 17u;
        inline constexpr uint BLUE = 18u;
    }

    namespace Motors {
        inline constexpr uint LEFT_MOTOR_IN1 = 24u;
        inline constexpr uint LEFT_MOTOR_IN2 = 25u;
        inline constexpr uint LEFT_MOTOR_CURRENT = 28u;

        inline constexpr uint RIGHT_MOTOR_IN1 = 22u;
        inline constexpr uint RIGHT_MOTOR_IN2 = 23u;
        inline constexpr uint RIGHT_MOTOR_CURRENT = 26u;
    }
}

namespace Regulators {
    namespace Current {
        inline constexpr float kP = 60.0f;
        inline constexpr float kI = 30.0f;

        inline constexpr float RC_FILTER_CUTOFF_FREQUENCY = 10.0f;
        inline constexpr float LAG_FILTER_K = 0.1f;

        inline constexpr float RAD_PER_SEC_12V = 95.923f;
        inline constexpr float kV = 12.0f / RAD_PER_SEC_12V;
    }

    namespace Velocity {
        inline constexpr float MAX_CURRENT = 0.45f;
        // inline constexpr float MAX_CURRENT = 1.0f;
        inline constexpr float ANGULAR_VELOCITY_MIN_CURRENT_BUDGET = 0.3f;
        inline constexpr float LINEAR_VELOCITY_CURRENT_BUDGET = 1.0f -
                                                                ANGULAR_VELOCITY_MIN_CURRENT_BUDGET;

        namespace Linear {
            // inline constexpr float kP = 0.03f;
            // inline constexpr float kI = 0.1f; // 0.1f
            // inline constexpr float kD = 0.1f;
            // inline constexpr float kPIntegrator = 0.03f; // 0.5f
            // inline constexpr float kPControl = 0.0f; // 0.1f
            // inline constexpr float kD = 0.0f; // 0.2f
            inline constexpr float kV = 0.001f;
            inline constexpr float kA = 0.0f;
            inline constexpr float kP = 0.00f;

            inline constexpr float MIN_INT = -1.0f;
            inline constexpr float MAX_INT = 1.0f;

            inline constexpr float CUTOFF_FREQUENCY = 10.0f; // 0.1f

            inline constexpr float LAG_FILTER_K = 0.005f;
        }

        namespace Angular {
            // inline constexpr float kP = 0.5f;
            // inline constexpr float kI = 0.0f; // 0.6f
            // inline constexpr float kD = 0.0f; // 0.5f
            inline constexpr float kV = 0.0f;
            inline constexpr float kA = 0.0f;
            inline constexpr float kP = 2.0f;
            inline constexpr float kD = 0.0f;

            inline constexpr float MIN_INT = -1.0f;
            inline constexpr float MAX_INT = 1.0f;

            inline constexpr float CUTOFF_FREQUENCY = 0.1f; // 0.1f
            inline constexpr float ALPHA = 0.5f;

            inline constexpr float LAG_FILTER_K = 0.5f;
        }
    }
}

namespace Status {
    inline constexpr Vec3 READY_FOR_MOTIONLESS_CALIBRATION{ 1.0f, 0.0f, 0.0f };
    inline constexpr Vec3 MOTIONLESS_CALIBRATING{ 1.0f, 1.0f, 0.0f };

    inline constexpr Vec3 READY_FOR_MOTION_CALIBRATION{ 1.0f, 0.0f, 1.0f };
    inline constexpr Vec3 MOTION_CALIBRATING{ 1.0f, 1.0f, 1.0f };

    inline constexpr Vec3 READY_TO_RUN{ 0.0f, 1.0f, 0.0f };
    inline constexpr Vec3 RUNNING{ 0.0f, 0.0f, 1.0f };

    inline constexpr Vec3 FINISHED{ 0.0f, 1.0f, 0.0f };
}

namespace Storage {
    namespace Flash {
        
    }
}

namespace Track {
    inline constexpr float SQUARE_SIZE = 50.0f;
}
