#pragma once

#include "navcon/robot_config.hpp"
#include "navcon/types.hpp"
#include <unordered_map>
#include <cmath>

namespace navcon {

// Individual wheel command
struct WheelCommand {
    double steering_angle = 0.0;  // Steering angle in radians
    double throttle = 0.0;        // Throttle value (normalized or m/s)
    bool brake = false;           // Brake applied
};

// Collection of wheel commands
using WheelCommands = std::unordered_map<std::string, WheelCommand>;

// Converts high-level control commands to individual wheel commands
class WheelController {
public:
    WheelController(const RobotConfig& config) : config_(config) {}
    
    // Convert velocity command to wheel commands
    WheelCommands velocity_to_wheels(const VelocityCommand& cmd) {
        WheelCommands result;
        
        switch (config_.detect_drive_type()) {
            case RobotConfig::DriveType::DIFFERENTIAL:
                return differential_drive(cmd);
            
            case RobotConfig::DriveType::ACKERMANN:
                return ackermann_drive(cmd);
            
            case RobotConfig::DriveType::FOUR_WHEEL:
                return four_wheel_steering(cmd);
            
            case RobotConfig::DriveType::MULTI_AXLE:
                return multi_axle_drive(cmd);
            
            default:
                return result;
        }
    }
    
    // Convert normalized command to wheel commands
    WheelCommands normalized_to_wheels(const NormalizedCommand& cmd) {
        WheelCommands result;
        
        // Simple mapping for normalized commands
        for (const auto& wheel : config_.wheels) {
            WheelCommand wheel_cmd;
            
            // Apply steering
            if (wheel.is_steerable()) {
                wheel_cmd.steering_angle = cmd.steering * wheel.max_steer_angle;
                wheel_cmd.steering_angle += cmd.steering * wheel.steer_differential;
            }
            
            // Apply throttle
            if (wheel.is_powered()) {
                wheel_cmd.throttle = cmd.throttle * wheel.max_throttle;
                wheel_cmd.throttle += cmd.throttle * wheel.throttle_differential;
            }
            
            result[wheel.name] = wheel_cmd;
        }
        
        return result;
    }
    
private:
    const RobotConfig& config_;
    
    WheelCommands differential_drive(const VelocityCommand& cmd) {
        WheelCommands result;
        
        // For differential drive, convert linear and angular velocities to wheel speeds
        double left_speed = cmd.linear_velocity - (cmd.angular_velocity * config_.track_width / 2.0);
        double right_speed = cmd.linear_velocity + (cmd.angular_velocity * config_.track_width / 2.0);
        
        for (const auto& wheel : config_.wheels) {
            WheelCommand wheel_cmd;
            
            if (wheel.is_powered()) {
                if (wheel.is_left_side()) {
                    wheel_cmd.throttle = left_speed / config_.max_linear_velocity;
                } else {
                    wheel_cmd.throttle = right_speed / config_.max_linear_velocity;
                }
                
                // Apply differential factors
                wheel_cmd.throttle *= wheel.max_throttle;
                wheel_cmd.throttle += wheel_cmd.throttle * wheel.throttle_differential;
            }
            
            result[wheel.name] = wheel_cmd;
        }
        
        return result;
    }
    
    WheelCommands ackermann_drive(const VelocityCommand& cmd) {
        WheelCommands result;
        
        // Calculate required steering angle for desired angular velocity
        double steering_angle = 0.0;
        if (std::abs(cmd.linear_velocity) > 1e-6) {
            steering_angle = std::atan((cmd.angular_velocity * config_.wheelbase) / cmd.linear_velocity);
        }
        
        // Calculate turning radius
        double turning_radius = std::numeric_limits<double>::max();
        if (std::abs(steering_angle) > 1e-6) {
            turning_radius = config_.wheelbase / std::tan(steering_angle);
        }
        
        for (const auto& wheel : config_.wheels) {
            WheelCommand wheel_cmd;
            
            // Steering (Ackermann geometry)
            if (wheel.is_steerable() && wheel.is_front()) {
                if (std::abs(turning_radius) < 1e6) {
                    // Calculate individual wheel angles for Ackermann
                    double wheel_offset = wheel.is_left_side() ? -config_.front_track/2 : config_.front_track/2;
                    double inner_radius = turning_radius - wheel_offset;
                    wheel_cmd.steering_angle = std::atan(config_.wheelbase / inner_radius);
                    
                    // Apply limits and differential
                    wheel_cmd.steering_angle = std::clamp(wheel_cmd.steering_angle, 
                        -wheel.max_steer_angle, wheel.max_steer_angle);
                    wheel_cmd.steering_angle += wheel_cmd.steering_angle * wheel.steer_differential;
                } else {
                    wheel_cmd.steering_angle = 0.0;
                }
            }
            
            // Throttle
            if (wheel.is_powered()) {
                wheel_cmd.throttle = cmd.linear_velocity / config_.max_linear_velocity;
                wheel_cmd.throttle *= wheel.max_throttle;
                wheel_cmd.throttle += wheel_cmd.throttle * wheel.throttle_differential;
            }
            
            result[wheel.name] = wheel_cmd;
        }
        
        return result;
    }
    
    WheelCommands four_wheel_steering(const VelocityCommand& cmd) {
        WheelCommands result;
        
        // For 4WS, we can use different strategies
        // Here we'll use a simple proportional approach
        double front_steering = 0.0;
        double rear_steering = 0.0;
        
        if (std::abs(cmd.linear_velocity) > 1e-6) {
            // At low speeds, counter-steer for tighter turns
            // At high speeds, same direction for stability
            double speed_factor = std::abs(cmd.linear_velocity) / config_.max_linear_velocity;
            
            if (speed_factor < 0.3) {
                // Low speed: opposite steering
                front_steering = std::atan((cmd.angular_velocity * config_.wheelbase * 0.7) / cmd.linear_velocity);
                rear_steering = -front_steering * 0.3;
            } else {
                // High speed: same direction
                front_steering = std::atan((cmd.angular_velocity * config_.wheelbase) / cmd.linear_velocity);
                rear_steering = front_steering * 0.1;
            }
        }
        
        for (const auto& wheel : config_.wheels) {
            WheelCommand wheel_cmd;
            
            if (wheel.is_steerable()) {
                if (wheel.is_front()) {
                    wheel_cmd.steering_angle = front_steering;
                } else {
                    wheel_cmd.steering_angle = rear_steering;
                }
                
                wheel_cmd.steering_angle = std::clamp(wheel_cmd.steering_angle,
                    -wheel.max_steer_angle, wheel.max_steer_angle);
                wheel_cmd.steering_angle += wheel_cmd.steering_angle * wheel.steer_differential;
            }
            
            if (wheel.is_powered()) {
                wheel_cmd.throttle = cmd.linear_velocity / config_.max_linear_velocity;
                wheel_cmd.throttle *= wheel.max_throttle;
                wheel_cmd.throttle += wheel_cmd.throttle * wheel.throttle_differential;
            }
            
            result[wheel.name] = wheel_cmd;
        }
        
        return result;
    }
    
    WheelCommands multi_axle_drive(const VelocityCommand& cmd) {
        // For multi-axle vehicles like trucks
        // Use coordinated steering with different angles per axle
        WheelCommands result;
        
        // Group wheels by axle (Y position)
        std::map<double, std::vector<const Wheel*>> axles;
        for (const auto& wheel : config_.wheels) {
            axles[wheel.position.y].push_back(&wheel);
        }
        
        // Calculate steering for each axle
        double base_steering = 0.0;
        if (std::abs(cmd.linear_velocity) > 1e-6) {
            base_steering = std::atan((cmd.angular_velocity * config_.wheelbase) / cmd.linear_velocity);
        }
        
        int axle_idx = 0;
        for (const auto& [y_pos, wheels] : axles) {
            double axle_factor = 1.0 - (axle_idx * 0.2); // Reduce steering for rear axles
            
            for (const auto* wheel : wheels) {
                WheelCommand wheel_cmd;
                
                if (wheel->is_steerable()) {
                    wheel_cmd.steering_angle = base_steering * axle_factor;
                    wheel_cmd.steering_angle = std::clamp(wheel_cmd.steering_angle,
                        -wheel->max_steer_angle, wheel->max_steer_angle);
                    wheel_cmd.steering_angle += wheel_cmd.steering_angle * wheel->steer_differential;
                }
                
                if (wheel->is_powered()) {
                    wheel_cmd.throttle = cmd.linear_velocity / config_.max_linear_velocity;
                    wheel_cmd.throttle *= wheel->max_throttle;
                    wheel_cmd.throttle += wheel_cmd.throttle * wheel->throttle_differential;
                }
                
                result[wheel->name] = wheel_cmd;
            }
            
            axle_idx++;
        }
        
        return result;
    }
};

} // namespace navcon