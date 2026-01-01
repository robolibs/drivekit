#pragma once

#include "datapod/spatial.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace drivekit {

    // Use Datapod types directly
    using Pose = datapod::Pose;
    using Point = datapod::Point;
    using Size = datapod::Size;
    using Euler = datapod::Euler;
    using Quaternion = datapod::Quaternion;

    // Steering type
    enum class SteeringType {
        DIFFERENTIAL, // Differential drive (can turn in place)
        ACKERMANN,    // Ackermann steering (needs forward motion to turn)
        HOLONOMIC,    // Omnidirectional (can move in any direction)
        SKID_STEER    // Skid steering (like differential but with lateral motion)
    };

    // Velocity state
    struct Velocity {
        double linear = 0.0;  // Forward velocity (m/s)
        double angular = 0.0; // Angular velocity (rad/s)
        double lateral = 0.0; // Lateral velocity for holonomic (m/s)
    };

    // Robot state
    struct RobotState {
        Pose pose;                  // Primary robot pose (e.g. tractor)
        Velocity velocity;          // Primary robot velocity
        double timestamp = 0.0;     // Time in seconds
        bool allow_reverse = false; // Allow reverse motion this tick
        bool turn_first = false;    // For diff/skid: rotate in place before translating
        bool allow_move = true;     // Allow movement (false = send zero velocity)

        // Optional articulated follower (e.g. trailer)
        bool has_trailer = false;
        Pose trailer_pose; // Pose of first follower in chain when available
    };

    // Path specification
    struct Path {
        std::vector<Pose> drivekits;
        std::vector<double> speeds; // Optional speed at each drivekit
        bool is_closed = false;     // Loop back to start
    };

    // Robot physical parameters (static, set at initialization)
    struct RobotConstraints {
        // Kinematic parameters
        SteeringType steering_type = SteeringType::ACKERMANN; // Default to Ackermann steering
        double wheelbase = 1.0;                               // Distance between axles (m)
        double track_width = 0.5;                             // Distance between wheels (m)
        double wheel_radius = 0.1;                            // Wheel radius (m)

        // Dynamic limits
        double max_linear_velocity = 1.0;      // m/s
        double min_linear_velocity = -0.5;     // m/s (negative for reverse)
        double max_angular_velocity = 1.0;     // rad/s
        double max_linear_acceleration = 1.0;  // m/s²
        double max_angular_acceleration = 1.0; // rad/s²

        // Steering limits (for Ackermann)
        double max_steering_angle = 0.5; // radians
        double max_steering_rate = 1.0;  // rad/s
        double min_turning_radius = 1.0; // meters

        // Multi-axle specific
        double rear_wheelbase = 0.0;          // Distance to rear axle
        double max_rear_steering_angle = 0.0; // radians

        double robot_width = 0.0;  // Robot width
        double robot_length = 0.0; // Robot length
    };

    // Zone representation for world constraints
    struct Zone {
        std::vector<Point> boundary; // Polygon boundary
        double max_speed = 0.0;      // Maximum speed in zone (0 = forbidden/no-go zone)
    };

    // Obstacle representation with uncertainty prediction
    struct Obstacle {
        size_t id = 0;
        double radius = 0.3; // Obstacle radius (m)

        // Gaussian mode for trajectory prediction
        struct GaussianMode {
            double weight = 1.0;        // Mode probability (sum to 1.0 across modes)
            std::vector<double> mean_x; // Mean x position per timestep
            std::vector<double> mean_y; // Mean y position per timestep
            std::vector<double> std_x;  // Standard deviation x per timestep
            std::vector<double> std_y;  // Standard deviation y per timestep
        };

        std::vector<GaussianMode> modes; // Multiple modes for multi-modal predictions
    };

    // World constraints (can change at every tick - obstacles, zones, etc.)
    struct WorldConstraints {
        std::vector<Zone> zones;         // Dynamic zones (no-go areas, speed limits, etc.)
        std::vector<Obstacle> obstacles; // Dynamic obstacles with Gaussian predictions
    };

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

} // namespace drivekit
