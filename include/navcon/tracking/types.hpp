#pragma once

#include "navcon/types.hpp"
#include <optional>
#include <string>

namespace navcon {
    namespace tracking {

        // Goal specification for tracking
        struct Goal {
            Pose target_pose;
            std::optional<Velocity> target_velocity;
            double tolerance_position = 0.1;    // meters
            double tolerance_orientation = 0.1; // radians
        };

        // Control output types
        enum class OutputType {
            VELOCITY_COMMAND // Linear + angular velocity
        };

        // Output units mode
        enum class OutputUnits {
            NORMALIZED, // Output in range [-1, 1] (default for most simulators)
            PHYSICAL    // Output in physical units (m/s, rad/s) - standard for real robots
        };

        // Base control output
        struct ControlOutput {
            bool valid = false;
            std::string status_message;
            OutputType type = OutputType::VELOCITY_COMMAND;
        };

        // Velocity-based output (twist)
        struct VelocityCommand : public ControlOutput {
            double linear_velocity = 0.0;  // Either normalized [-1,1] or m/s depending on mode
            double angular_velocity = 0.0; // Either normalized [-1,1] or rad/s depending on mode
            double lateral_velocity = 0.0; // Either normalized [-1,1] or m/s depending on mode

            inline VelocityCommand() { type = OutputType::VELOCITY_COMMAND; }
        };

        // Controller configuration
        struct ControllerConfig {
            // Output units mode
            OutputUnits output_units = OutputUnits::NORMALIZED; // Default: normalized [-1, 1]

            // PID gains
            double kp_linear = 1.0;
            double ki_linear = 0.0;
            double kd_linear = 0.0;

            double kp_angular = 1.0;
            double ki_angular = 0.0;
            double kd_angular = 0.0;

            // Path following
            double lookahead_distance = 1.0; // Pure pursuit
            double k_cross_track = 1.0;      // Stanley controller
            double k_heading = 1.0;          // Stanley controller

            // Behavior
            bool allow_reverse = false;
            double goal_tolerance = 0.1;    // meters
            double angular_tolerance = 0.1; // radians
        };

        // Controller status
        struct ControllerStatus {
            bool goal_reached = false;
            double distance_to_goal = 0.0;
            double cross_track_error = 0.0;
            double heading_error = 0.0;
            std::string mode; // e.g., "tracking", "turning", "stopped"
        };

    } // namespace tracking

    // Convenience aliases in navcon namespace for backward compatibility
    using Goal = tracking::Goal;
    using OutputType = tracking::OutputType;
    using OutputUnits = tracking::OutputUnits;
    using ControlOutput = tracking::ControlOutput;
    using VelocityCommand = tracking::VelocityCommand;
    using ControllerConfig = tracking::ControllerConfig;
    using ControllerStatus = tracking::ControllerStatus;

} // namespace navcon
