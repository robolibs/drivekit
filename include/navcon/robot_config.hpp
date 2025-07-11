#pragma once

#include "navcon/types.hpp"
#include <vector>
#include <string>

namespace navcon {

// Wheel configuration
struct Wheel {
    std::string name;
    Point position;      // Position relative to robot center
    Size size;           // Wheel dimensions (width, height/diameter)
    
    // Control parameters
    double max_steer_angle = 0.0;    // Maximum steering angle (radians)
    double steer_differential = 0.0;  // Differential steering factor
    double max_throttle = 1.0;        // Maximum throttle value
    double throttle_differential = 0.0; // Differential throttle factor
    
    // Derived properties
    bool is_steerable() const { return std::abs(max_steer_angle) > 1e-6; }
    bool is_powered() const { return std::abs(max_throttle) > 1e-6; }
    bool is_left_side() const { return position.x < 0; }
    bool is_front() const { return position.y > 0; }
};

// Complete robot configuration
struct RobotConfig {
    // Basic info
    std::string name = "robot";
    std::string type = "generic";
    
    // Physical dimensions
    Size dimensions;     // Overall robot size
    double mass = 100.0; // Robot mass (kg)
    
    // Wheels
    std::vector<Wheel> wheels;
    
    // Derived kinematic properties (computed from wheels)
    double wheelbase = 0.0;      // Distance between front and rear axles
    double track_width = 0.0;    // Distance between left and right wheels
    double front_track = 0.0;    // Front axle track width
    double rear_track = 0.0;     // Rear axle track width
    
    // Control limits (can be overridden or computed from wheels)
    double max_linear_velocity = 10.0;   // m/s
    double min_linear_velocity = -5.0;   // m/s (reverse)
    double max_angular_velocity = 2.0;   // rad/s
    double max_steering_angle = 0.5;     // rad
    
    // Compute derived properties from wheel configuration
    void compute_properties() {
        if (wheels.empty()) return;
        
        // Find wheel extents
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        
        double front_left_x = 0, front_right_x = 0;
        double rear_left_x = 0, rear_right_x = 0;
        int front_count = 0, rear_count = 0;
        
        double max_steer = 0.0;
        
        for (const auto& wheel : wheels) {
            min_x = std::min(min_x, wheel.position.x);
            max_x = std::max(max_x, wheel.position.x);
            min_y = std::min(min_y, wheel.position.y);
            max_y = std::max(max_y, wheel.position.y);
            
            max_steer = std::max(max_steer, std::abs(wheel.max_steer_angle));
            
            // Track width calculations
            if (wheel.is_front()) {
                if (wheel.is_left_side()) {
                    front_left_x += wheel.position.x;
                } else {
                    front_right_x += wheel.position.x;
                }
                front_count++;
            } else {
                if (wheel.is_left_side()) {
                    rear_left_x += wheel.position.x;
                } else {
                    rear_right_x += wheel.position.x;
                }
                rear_count++;
            }
        }
        
        // Set derived properties
        wheelbase = max_y - min_y;
        track_width = max_x - min_x;
        
        if (front_count > 0) {
            front_track = front_right_x - front_left_x;
        }
        if (rear_count > 0) {
            rear_track = rear_right_x - rear_left_x;
        }
        
        if (max_steer > 0) {
            max_steering_angle = max_steer;
        }
    }
    
    // Helper methods
    std::vector<const Wheel*> get_steerable_wheels() const {
        std::vector<const Wheel*> result;
        for (const auto& wheel : wheels) {
            if (wheel.is_steerable()) {
                result.push_back(&wheel);
            }
        }
        return result;
    }
    
    std::vector<const Wheel*> get_powered_wheels() const {
        std::vector<const Wheel*> result;
        for (const auto& wheel : wheels) {
            if (wheel.is_powered()) {
                result.push_back(&wheel);
            }
        }
        return result;
    }
    
    // Determine robot type from configuration
    enum class DriveType {
        DIFFERENTIAL,    // Tank-like, skid steer
        ACKERMANN,      // Car-like, front steering
        FOUR_WHEEL,     // 4WS, front and rear steering
        MULTI_AXLE,     // Truck with multiple axles
        OMNIDIRECTIONAL // Mecanum or omni wheels
    };
    
    DriveType detect_drive_type() const {
        auto steerable = get_steerable_wheels();
        auto powered = get_powered_wheels();
        
        if (steerable.empty()) {
            return DriveType::DIFFERENTIAL;
        }
        
        // Check if rear wheels can steer
        bool has_rear_steering = false;
        bool has_front_steering = false;
        
        for (const auto* wheel : steerable) {
            if (wheel->is_front()) {
                has_front_steering = true;
            } else {
                has_rear_steering = true;
            }
        }
        
        if (has_front_steering && has_rear_steering) {
            if (wheels.size() > 4) {
                return DriveType::MULTI_AXLE;
            } else {
                return DriveType::FOUR_WHEEL;
            }
        }
        
        return DriveType::ACKERMANN;
    }
    
    // Convert to navcon RobotConstraints
    RobotConstraints to_constraints() const {
        RobotConstraints constraints;
        
        constraints.wheelbase = wheelbase;
        constraints.track_width = track_width;
        constraints.max_linear_velocity = max_linear_velocity;
        constraints.min_linear_velocity = min_linear_velocity;
        constraints.max_angular_velocity = max_angular_velocity;
        constraints.max_steering_angle = max_steering_angle;
        
        // For multi-axle, find rear wheelbase
        if (detect_drive_type() == DriveType::MULTI_AXLE) {
            // Find the furthest rear steerable axle
            double rear_y = 0.0;
            for (const auto& wheel : wheels) {
                if (wheel.is_steerable() && !wheel.is_front()) {
                    rear_y = std::min(rear_y, wheel.position.y);
                }
            }
            constraints.rear_wheelbase = std::abs(rear_y);
            
            // Find max rear steering angle
            double max_rear_steer = 0.0;
            for (const auto& wheel : wheels) {
                if (!wheel.is_front() && wheel.is_steerable()) {
                    max_rear_steer = std::max(max_rear_steer, std::abs(wheel.max_steer_angle));
                }
            }
            constraints.max_rear_steering_angle = max_rear_steer;
        }
        
        return constraints;
    }
};

// Builder pattern for easy configuration
class RobotConfigBuilder {
public:
    RobotConfigBuilder& with_name(const std::string& name) {
        config_.name = name;
        return *this;
    }
    
    RobotConfigBuilder& with_type(const std::string& type) {
        config_.type = type;
        return *this;
    }
    
    RobotConfigBuilder& with_dimensions(double width, double length, double height = 0.0) {
        config_.dimensions = Size{width, length, height};
        return *this;
    }
    
    RobotConfigBuilder& with_mass(double mass) {
        config_.mass = mass;
        return *this;
    }
    
    RobotConfigBuilder& add_wheel(const Wheel& wheel) {
        config_.wheels.push_back(wheel);
        return *this;
    }
    
    RobotConfigBuilder& with_velocity_limits(double max_linear, double min_linear, double max_angular) {
        config_.max_linear_velocity = max_linear;
        config_.min_linear_velocity = min_linear;
        config_.max_angular_velocity = max_angular;
        return *this;
    }
    
    RobotConfig build() {
        config_.compute_properties();
        return config_;
    }
    
private:
    RobotConfig config_;
};

} // namespace navcon