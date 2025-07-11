#pragma once

#include "navcon/types.hpp"
#include <cmath>

namespace navcon {
namespace kinematics {

class DifferentialDrive {
public:
    // Convert velocity command to wheel speeds
    static DifferentialCommand velocity_to_wheels(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints
    ) {
        DifferentialCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        // Differential drive kinematics
        // left_speed = linear_vel - (angular_vel * track_width / 2)
        // right_speed = linear_vel + (angular_vel * track_width / 2)
        
        double linear = cmd.linear_velocity;
        double angular = cmd.angular_velocity;
        double half_track = constraints.track_width / 2.0;
        
        output.left_speed = linear - (angular * half_track);
        output.right_speed = linear + (angular * half_track);
        
        return output;
    }
    
    // Convert wheel speeds to velocity command
    static VelocityCommand wheels_to_velocity(
        const DifferentialCommand& cmd,
        const RobotConstraints& constraints
    ) {
        VelocityCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        // Inverse kinematics
        // linear_vel = (left_speed + right_speed) / 2
        // angular_vel = (right_speed - left_speed) / track_width
        
        output.linear_velocity = (cmd.left_speed + cmd.right_speed) / 2.0;
        output.angular_velocity = (cmd.right_speed - cmd.left_speed) / constraints.track_width;
        
        return output;
    }
    
    // Convert normalized command to velocity
    static VelocityCommand normalized_to_velocity(
        const NormalizedCommand& cmd,
        const RobotConstraints& constraints
    ) {
        VelocityCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        // Map throttle to linear velocity
        if (cmd.throttle >= 0) {
            output.linear_velocity = cmd.throttle * constraints.max_linear_velocity;
        } else {
            output.linear_velocity = cmd.throttle * std::abs(constraints.min_linear_velocity);
        }
        
        // Map steering to angular velocity
        output.angular_velocity = cmd.steering * constraints.max_angular_velocity;
        
        return output;
    }
    
    // Check if robot can rotate in place
    static bool can_rotate_in_place() { return true; }
    
    // Calculate minimum turning radius (infinite for diff drive)
    static double minimum_turning_radius(const RobotConstraints& constraints) {
        return 0.0; // Can rotate in place
    }
    
    // Forward kinematics: predict next pose given current state and command
    static Pose predict_pose(
        const Pose& current,
        const VelocityCommand& cmd,
        double dt
    ) {
        Pose next = current;
        
        if (std::abs(cmd.angular_velocity) < 1e-6) {
            // Straight line motion
            next.point.x += cmd.linear_velocity * std::cos(current.angle.yaw) * dt;
            next.point.y += cmd.linear_velocity * std::sin(current.angle.yaw) * dt;
        } else {
            // Arc motion
            double radius = cmd.linear_velocity / cmd.angular_velocity;
            double angle_change = cmd.angular_velocity * dt;
            
            next.point.x += radius * (std::sin(current.angle.yaw + angle_change) - std::sin(current.angle.yaw));
            next.point.y += radius * (std::cos(current.angle.yaw) - std::cos(current.angle.yaw + angle_change));
            next.angle.yaw += angle_change;
        }
        
        return next;
    }
};

} // namespace kinematics
} // namespace navcon