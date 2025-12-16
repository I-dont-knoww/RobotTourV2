#pragma once

#include "state/Vector.hpp"

#include <cstddef>
#include <cstdint>
#include <numbers>

using uint = unsigned int;

namespace Chassis {
    inline constexpr float AXLE_LENGTH = 13.35f;
    inline constexpr float WHEEL_RADIUS = 3.01625f;

    inline constexpr float DOWEL_DISTANCE = 2.314066f;

    inline constexpr float MASS = 0.786f;
}

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
        inline constexpr float RESOLUTION = 32.8f * 180.0f / Constants::PI;

        inline constexpr size_t GYRO_CALIBRATION_SAMPLE_COUNT = 131072u;
        inline constexpr size_t DOWN_DIRECTION_SAMPLE_COUNT = 4096u;
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

namespace Integration {
    inline constexpr float TARGET_FAST_LOOP_HZ = 32.0e3f;
    // inline constexpr float TARGET_SLOW_LOOP_HZ = 10.0e3f;
    inline constexpr float TARGET_SLOW_LOOP_HZ = 2.0e3f;

    inline constexpr float CALIBRATION_DELAY = 1.0f;
    inline constexpr float FINAL_STATE_MEASUREMENT_DELAY = 1.0f;

    inline constexpr float TARGET_FAST_LOOP_DT = 1.0f / TARGET_FAST_LOOP_HZ;
    inline constexpr float TARGET_SLOW_LOOP_DT = 1.0f / TARGET_SLOW_LOOP_HZ;

    inline constexpr int64_t FAST_LOOP_US = static_cast<uint64_t>(TARGET_FAST_LOOP_DT * 1.0e6f);
    inline constexpr int64_t SLOW_LOOP_US = static_cast<uint64_t>(TARGET_SLOW_LOOP_DT * 1.0e6f);

    inline constexpr float FAST_LOOP_DT = static_cast<float>(FAST_LOOP_US * 1.0e-6f);
    inline constexpr float SLOW_LOOP_DT = static_cast<float>(SLOW_LOOP_US * 1.0e-6f);
}

namespace Kinematics {
    namespace Forward {
        inline constexpr size_t MA_FILTER_LENGTH = 50u;

        inline constexpr float VELOCITY_CUTOFF_FREQUENCY = 10.0f;
        inline constexpr float WHEEL_SPEED_CUTOFF_FREQUENCY = 100.0f;
    }

    namespace Inverse {
        inline constexpr float RAD_PER_SEC_12V = 95.984f;
    }
}

namespace Manager {
    namespace Follower {
        inline constexpr float DISTANCE_THRESHOLD_ACCURATE = 0.0f;
        inline constexpr float DISTANCE_THRESHOLD_FAST = 0.0f;
        inline constexpr float TURNING_RADIUS = 20.0f;

        inline constexpr float ANGLE_THRESHOLD = 0.05f;
    }

    namespace Straight {
        inline constexpr float slowdownKp = 23.0f;
        inline constexpr float slowdownKh = 0.5f;
        inline constexpr float slowdownKs = 7.0f;

        inline constexpr float angularKp = 20.0f;
        inline constexpr float angularKd = 0.2f;

        inline constexpr float linearKp = 2.0f;
        inline constexpr float linearKd = 0.1f;
        inline constexpr float LINEAR_AUTHORITY = 0.3f;

        inline constexpr float FILTER_ALPHA = 1.0f;

        inline constexpr float MAX_CENTRIPETAL = 120.0f;
        inline constexpr float TURN_ANGULAR_SPEED = 3.5f;
        inline constexpr float MAX_LINEAR_SPEED = 500.0f;
    }

    namespace Rotation {
        inline constexpr float kS = 2.0f;
        inline constexpr float kP = 3.5f;

        inline constexpr float GRABBING_SPEED = 1.0f;

        inline constexpr float MAX_SPEED = 2.0f;
        inline constexpr float TURN_TIME_OFFSET = 1.0f;
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
        inline constexpr float MAX_CURRENT = 0.40f;

        inline constexpr Vec2 RESISTANCE{ 3.30982f, 3.33778f };
        inline constexpr Vec2 FREE_ANGULAR_VEL{ 91.0553f, 90.0371f };
        inline constexpr Vec2 FREE_VOLTAGE{ 10.5f, 10.5f };
        inline constexpr Vec2 FREE_CURRENT{ 0.0576f, 0.0608f };

        inline constexpr Vec2 KV = FREE_ANGULAR_VEL / (FREE_VOLTAGE - FREE_CURRENT * RESISTANCE);
    }

    namespace Velocity {
        inline constexpr float ANGLE_CONTROL_MIN_VOLTAGE_BUDGET = 0.3f;
        inline constexpr float LINEAR_VELOCITY_VOLTAGE_BUDGET = 1.0f -
                                                                ANGLE_CONTROL_MIN_VOLTAGE_BUDGET;
        inline constexpr float OFFSET = 0.85f;

        namespace Linear {
            inline constexpr float kS = 0.4f;
            inline constexpr float kV = 0.037f;
            inline constexpr float kA = 0.0f;
            inline constexpr float kP = 0.08f;
            inline constexpr float kI = 0.08f;

            inline constexpr float MIN_INT = -1.0f;
            inline constexpr float MAX_INT = 1.0f;

            inline constexpr float CUTOFF_FREQUENCY = 10.0f;

            inline constexpr float LAG_FILTER_K = 0.05f;
        }

        namespace Angular {
            inline constexpr float kS = 0.0f;
            inline constexpr float kV = 0.7f;
            inline constexpr float kA = 0.0f;
            inline constexpr float kP = 1.0f;
            inline constexpr float kI = 0.0f;

            inline constexpr float MIN_INT = -10.0f;
            inline constexpr float MAX_INT = 10.0f;

            inline constexpr float CUTOFF_FREQUENCY = 50.0f;

            inline constexpr float LAG_FILTER_K = 0.05f;
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

namespace Track {
    inline constexpr float SQUARE_SIZE = 50.0f;
}
