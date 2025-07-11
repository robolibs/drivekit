#pragma once

#include "navcon/types.hpp"
#include <memory>

namespace navcon {

// Abstract base controller interface
template<typename InputState = RobotState, typename OutputCommand = VelocityCommand>
class Controller {
public:
    virtual ~Controller() = default;
    
    // Main control computation
    virtual OutputCommand compute_control(
        const InputState& current_state,
        const Goal& goal,
        const RobotConstraints& constraints,
        double dt
    ) = 0;
    
    // Optional path following
    virtual void set_path(const Path& path) {
        path_ = path;
        path_index_ = 0;
    }
    
    // Reset controller state
    virtual void reset() {
        path_.waypoints.clear();
        path_index_ = 0;
        status_ = ControllerStatus{};
    }
    
    // Get current status
    virtual ControllerStatus get_status() const {
        return status_;
    }
    
    // Configure controller
    virtual void set_config(const ControllerConfig& config) {
        config_ = config;
    }
    
    virtual ControllerConfig get_config() const {
        return config_;
    }
    
    // Get controller type name
    virtual std::string get_type() const = 0;
    
protected:
    ControllerConfig config_;
    ControllerStatus status_;
    Path path_;
    size_t path_index_ = 0;
    
    // Helper functions
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    double calculate_distance(const Point& p1, const Point& p2) {
        return p1.distance_to(p2);
    }
    
    double calculate_heading_error(const Pose& current, const Point& target) {
        double dx = target.x - current.point.x;
        double dy = target.y - current.point.y;
        double desired_heading = std::atan2(dy, dx);
        return normalize_angle(desired_heading - current.angle.yaw);
    }
    
    bool is_goal_reached(const Pose& current, const Pose& goal) {
        double dist = calculate_distance(current.point, goal.point);
        double angle_diff = std::abs(normalize_angle(goal.angle.yaw - current.angle.yaw));
        
        status_.distance_to_goal = dist;
        status_.heading_error = angle_diff;
        
        return dist < config_.goal_tolerance && angle_diff < config_.angular_tolerance;
    }
};

// Type aliases for common controller types
using VelocityController = Controller<RobotState, VelocityCommand>;
using AckermannController = Controller<RobotState, AckermannCommand>;
using DifferentialController = Controller<RobotState, DifferentialCommand>;
using NormalizedController = Controller<RobotState, NormalizedCommand>;

} // namespace navcon