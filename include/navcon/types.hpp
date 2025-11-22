#pragma once

#include "concord/concord.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace navcon {

    // Use Concord types directly
    using Pose = concord::Pose;
    using Point = concord::Point;
    using Size = concord::Size;
    using Euler = concord::Euler;

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
        Pose pose;
        Velocity velocity;
        double timestamp = 0.0;     // Time in seconds
        bool allow_reverse = false; // Allow backward maneuvers for tight turns
    };

    // Path specification
    struct Path {
        std::vector<Pose> waypoints;
        std::vector<double> speeds; // Optional speed at each waypoint
        bool is_closed = false;     // Loop back to start
    };

    // Robot physical parameters (static, set at initialization)
    struct RobotConstraints {
        // Kinematic parameters
        SteeringType steering_type = SteeringType::DIFFERENTIAL; // Default to differential drive
        double wheelbase = 1.0;                                  // Distance between axles (m)
        double track_width = 0.5;                                // Distance between wheels (m)
        double wheel_radius = 0.1;                               // Wheel radius (m)

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

    // World constraints (can change at every tick - obstacles, zones, etc.)
    struct WorldConstraints {
        std::vector<Zone> zones; // Dynamic zones (no-go areas, speed limits, etc.)
    };

} // namespace navcon
