#pragma once

#include "navcon/types.hpp"
#include <cmath>
#include <algorithm>

namespace navcon {
namespace kinematics {

class Ackermann {
public:
    // Convert velocity command to Ackermann command
    static AckermannCommand velocity_to_ackermann(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints
    ) {
        AckermannCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        output.speed = cmd.linear_velocity;
        
        // Calculate steering angle from angular velocity
        // angular_vel = (speed / wheelbase) * tan(steering_angle)
        // Therefore: steering_angle = atan((angular_vel * wheelbase) / speed)
        
        if (std::abs(cmd.linear_velocity) > 1e-6) {
            double steering_angle = std::atan((cmd.angular_velocity * constraints.wheelbase) / cmd.linear_velocity);
            output.steering_angle = std::clamp(steering_angle, -constraints.max_steering_angle, constraints.max_steering_angle);
        } else {
            output.steering_angle = 0.0;
        }
        
        return output;
    }
    
    // Convert Ackermann command to velocity command
    static VelocityCommand ackermann_to_velocity(
        const AckermannCommand& cmd,
        const RobotConstraints& constraints
    ) {
        VelocityCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        output.linear_velocity = cmd.speed;
        
        // Calculate angular velocity from steering angle
        if (std::abs(cmd.speed) > 1e-6) {
            output.angular_velocity = (cmd.speed / constraints.wheelbase) * std::tan(cmd.steering_angle);
        } else {
            output.angular_velocity = 0.0;
        }
        
        return output;
    }
    
    // Convert normalized command to Ackermann
    static AckermannCommand normalized_to_ackermann(
        const NormalizedCommand& cmd,
        const RobotConstraints& constraints
    ) {
        AckermannCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        // Map throttle to speed
        if (cmd.throttle >= 0) {
            output.speed = cmd.throttle * constraints.max_linear_velocity;
        } else {
            output.speed = cmd.throttle * std::abs(constraints.min_linear_velocity);
        }
        
        // Map steering to angle
        output.steering_angle = cmd.steering * constraints.max_steering_angle;
        
        return output;
    }
    
    // Check if robot can rotate in place
    static bool can_rotate_in_place() { return false; }
    
    // Calculate minimum turning radius
    static double minimum_turning_radius(const RobotConstraints& constraints) {
        if (std::abs(constraints.max_steering_angle) < 1e-6) {
            return std::numeric_limits<double>::infinity();
        }
        return constraints.wheelbase / std::tan(constraints.max_steering_angle);
    }
    
    // Calculate turning radius for given steering angle
    static double turning_radius(double steering_angle, const RobotConstraints& constraints) {
        if (std::abs(steering_angle) < 1e-6) {
            return std::numeric_limits<double>::infinity();
        }
        return constraints.wheelbase / std::tan(steering_angle);
    }
    
    // Forward kinematics: predict next pose
    static Pose predict_pose(
        const Pose& current,
        const AckermannCommand& cmd,
        double dt
    ) {
        Pose next = current;
        
        if (std::abs(cmd.steering_angle) < 1e-6) {
            // Straight line motion
            next.point.x += cmd.speed * std::cos(current.angle.yaw) * dt;
            next.point.y += cmd.speed * std::sin(current.angle.yaw) * dt;
        } else {
            // Arc motion
            double radius = turning_radius(cmd.steering_angle, {.wheelbase = 1.0}); // Need constraints
            double angular_vel = cmd.speed / radius;
            double angle_change = angular_vel * dt;
            
            next.point.x += radius * (std::sin(current.angle.yaw + angle_change) - std::sin(current.angle.yaw));
            next.point.y += radius * (std::cos(current.angle.yaw) - std::cos(current.angle.yaw + angle_change));
            next.angle.yaw += angle_change;
        }
        
        return next;
    }
    
    // Calculate instantaneous center of rotation
    static Point icr_position(const Pose& vehicle_pose, double steering_angle, const RobotConstraints& constraints) {
        double radius = turning_radius(steering_angle, constraints);
        
        // ICR is perpendicular to the vehicle at the rear axle
        Point icr;
        if (steering_angle > 0) {
            // Turning left, ICR is to the left
            icr.x = vehicle_pose.point.x - radius * std::sin(vehicle_pose.angle.yaw);
            icr.y = vehicle_pose.point.y + radius * std::cos(vehicle_pose.angle.yaw);
        } else {
            // Turning right, ICR is to the right
            icr.x = vehicle_pose.point.x + radius * std::sin(vehicle_pose.angle.yaw);
            icr.y = vehicle_pose.point.y - radius * std::cos(vehicle_pose.angle.yaw);
        }
        
        return icr;
    }
};

} // namespace kinematics
} // namespace navcon