#pragma once

#include "navcon/types.hpp"
#include <cmath>
#include <algorithm>

namespace navcon {
namespace kinematics {

class MultiAxle {
public:
    enum class SteeringMode {
        FRONT_ONLY,      // Traditional front steering
        COORDINATED,     // Both axles for tighter turns
        COUNTER,         // Opposite steering for stability
        CRAB,           // Same angle for lateral motion
        ADAPTIVE        // Speed-dependent mode selection
    };
    
    // Convert velocity command to multi-axle command
    static MultiAxleCommand velocity_to_multi_axle(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints,
        SteeringMode mode = SteeringMode::COORDINATED
    ) {
        MultiAxleCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        output.speed = cmd.linear_velocity;
        
        if (std::abs(cmd.linear_velocity) < 1e-6) {
            output.front_steering_angle = 0.0;
            output.rear_steering_angle = 0.0;
            return output;
        }
        
        switch (mode) {
            case SteeringMode::FRONT_ONLY:
                output.front_steering_angle = calculate_front_only_angle(cmd, constraints);
                output.rear_steering_angle = 0.0;
                break;
                
            case SteeringMode::COORDINATED:
                calculate_coordinated_steering(cmd, constraints, output);
                break;
                
            case SteeringMode::COUNTER:
                calculate_counter_steering(cmd, constraints, output);
                break;
                
            case SteeringMode::CRAB:
                calculate_crab_steering(cmd, constraints, output);
                break;
                
            case SteeringMode::ADAPTIVE:
                calculate_adaptive_steering(cmd, constraints, output);
                break;
        }
        
        // Apply steering limits
        output.front_steering_angle = std::clamp(output.front_steering_angle, 
            -constraints.max_steering_angle, constraints.max_steering_angle);
        output.rear_steering_angle = std::clamp(output.rear_steering_angle, 
            -constraints.max_rear_steering_angle, constraints.max_rear_steering_angle);
        
        return output;
    }
    
    // Check if robot can rotate in place
    static bool can_rotate_in_place() { return false; }
    
    // Calculate minimum turning radius with coordinated steering
    static double minimum_turning_radius(const RobotConstraints& constraints) {
        // With coordinated steering, turning radius can be reduced
        double total_wheelbase = constraints.wheelbase + constraints.rear_wheelbase;
        double max_total_angle = constraints.max_steering_angle + constraints.max_rear_steering_angle;
        
        if (std::abs(max_total_angle) < 1e-6) {
            return std::numeric_limits<double>::infinity();
        }
        
        // Approximate minimum radius with both axles steering
        return total_wheelbase / (2.0 * std::sin(max_total_angle / 2.0));
    }
    
private:
    static double calculate_front_only_angle(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints
    ) {
        // Standard Ackermann calculation
        return std::atan((cmd.angular_velocity * constraints.wheelbase) / cmd.linear_velocity);
    }
    
    static void calculate_coordinated_steering(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints,
        MultiAxleCommand& output
    ) {
        // Distribute steering angle to minimize turning radius
        double total_wheelbase = constraints.wheelbase + constraints.rear_wheelbase;
        double desired_radius = cmd.linear_velocity / cmd.angular_velocity;
        
        // Optimal angle distribution (simplified)
        double front_ratio = constraints.wheelbase / total_wheelbase;
        double rear_ratio = constraints.rear_wheelbase / total_wheelbase;
        
        double total_angle = std::atan(total_wheelbase / desired_radius);
        
        output.front_steering_angle = total_angle * front_ratio;
        output.rear_steering_angle = total_angle * rear_ratio;
    }
    
    static void calculate_counter_steering(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints,
        MultiAxleCommand& output
    ) {
        // Counter-steering for high-speed stability
        output.front_steering_angle = calculate_front_only_angle(cmd, constraints);
        
        // Small opposite rear steering for stability
        double stability_factor = 0.2; // Tunable parameter
        output.rear_steering_angle = -output.front_steering_angle * stability_factor;
    }
    
    static void calculate_crab_steering(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints,
        MultiAxleCommand& output
    ) {
        // Same angle for lateral movement
        // This assumes lateral velocity component in cmd
        if (std::abs(cmd.lateral_velocity) > 1e-6) {
            double crab_angle = std::atan2(cmd.lateral_velocity, cmd.linear_velocity);
            output.front_steering_angle = crab_angle;
            output.rear_steering_angle = crab_angle;
        } else {
            // Fall back to coordinated steering
            calculate_coordinated_steering(cmd, constraints, output);
        }
    }
    
    static void calculate_adaptive_steering(
        const VelocityCommand& cmd,
        const RobotConstraints& constraints,
        MultiAxleCommand& output
    ) {
        // Choose mode based on speed
        double speed_ratio = std::abs(cmd.linear_velocity) / constraints.max_linear_velocity;
        
        if (speed_ratio < 0.3) {
            // Low speed: coordinated for tight turns
            calculate_coordinated_steering(cmd, constraints, output);
        } else if (speed_ratio < 0.7) {
            // Medium speed: front only
            output.front_steering_angle = calculate_front_only_angle(cmd, constraints);
            output.rear_steering_angle = 0.0;
        } else {
            // High speed: counter-steering for stability
            calculate_counter_steering(cmd, constraints, output);
        }
    }
};

} // namespace kinematics
} // namespace navcon