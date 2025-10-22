#pragma once

#include <cstddef>
#include <vector>
#include <string>
#include <optional>
#include "concord/concord.hpp"

namespace navcon {

// Use Concord types directly
using Pose = concord::Pose;
using Point = concord::Point;
using Size = concord::Size;
using Euler = concord::Euler;

// Velocity state
struct Velocity {
    double linear = 0.0;   // Forward velocity (m/s)
    double angular = 0.0;  // Angular velocity (rad/s)
    double lateral = 0.0;  // Lateral velocity for holonomic (m/s)
};

// Robot state
struct RobotState {
    Pose pose;
    Velocity velocity;
    double timestamp = 0.0;  // Time in seconds
};

// Goal specification
struct Goal {
    Pose target_pose;
    std::optional<Velocity> target_velocity;
    double tolerance_position = 0.1;   // meters
    double tolerance_orientation = 0.1; // radians
};

// Path specification
struct Path {
    std::vector<Pose> waypoints;
    std::vector<double> speeds;  // Optional speed at each waypoint
    bool is_closed = false;      // Loop back to start
};

// Robot physical parameters
struct RobotConstraints {
    // Kinematic parameters
    double wheelbase = 1.0;        // Distance between axles (m)
    double track_width = 0.5;      // Distance between wheels (m)
    double wheel_radius = 0.1;     // Wheel radius (m)
    
    // Dynamic limits
    double max_linear_velocity = 1.0;    // m/s
    double min_linear_velocity = -0.5;   // m/s (negative for reverse)
    double max_angular_velocity = 1.0;   // rad/s
    double max_linear_acceleration = 1.0;  // m/s²
    double max_angular_acceleration = 1.0; // rad/s²
    
    // Steering limits (for Ackermann)
    double max_steering_angle = 0.5;     // radians
    double max_steering_rate = 1.0;      // rad/s
    double min_turning_radius = 1.0;     // meters
    
    // Multi-axle specific
    double rear_wheelbase = 0.0;         // Distance to rear axle
    double max_rear_steering_angle = 0.0; // radians
};

// Control output types
enum class OutputType {
    VELOCITY_COMMAND      // Linear + angular velocity
};

// Base control output
struct ControlOutput {
    bool valid = false;
    std::string status_message;
    OutputType type = OutputType::VELOCITY_COMMAND;
};

// Velocity-based output (twist)
struct VelocityCommand : public ControlOutput {
    double linear_velocity = 0.0;   // m/s
    double angular_velocity = 0.0;  // rad/s
    double lateral_velocity = 0.0;  // m/s (holonomic only)
    
    inline VelocityCommand() { type = OutputType::VELOCITY_COMMAND; }
};

// Controller configuration
struct ControllerConfig {
    // PID gains
    double kp_linear = 1.0;
    double ki_linear = 0.0;
    double kd_linear = 0.0;
    
    double kp_angular = 1.0;
    double ki_angular = 0.0;
    double kd_angular = 0.0;
    
    // Path following
    double lookahead_distance = 1.0;     // Pure pursuit
    double k_cross_track = 1.0;          // Stanley controller
    double k_heading = 1.0;              // Stanley controller
    
    // Behavior
    bool allow_reverse = false;
    double goal_tolerance = 0.1;         // meters
    double angular_tolerance = 0.1;      // radians
};

// Controller status
struct ControllerStatus {
    bool goal_reached = false;
    double distance_to_goal = 0.0;
    double cross_track_error = 0.0;
    double heading_error = 0.0;
    std::string mode;  // e.g., "tracking", "turning", "stopped"
};

} // namespace navcon