#pragma once

#include "navcon/controller.hpp"
#include <algorithm>

namespace navcon {
namespace controllers {

// Simple carrot chasing controller for basic navigation
template<typename OutputCommand = VelocityCommand>
class CarrotController : public Controller<RobotState, OutputCommand> {
public:
    using Base = Controller<RobotState, OutputCommand>;
    using Base::config_;
    using Base::status_;
    
    OutputCommand compute_control(
        const RobotState& current_state,
        const Goal& goal,
        const RobotConstraints& constraints,
        double dt
    ) override {
        OutputCommand cmd;
        
        // Check if goal is reached
        if (this->is_goal_reached(current_state.pose, goal.target_pose)) {
            cmd.valid = true;
            cmd.status_message = "Goal reached";
            status_.goal_reached = true;
            status_.mode = "stopped";
            return cmd;
        }
        
        // Calculate distance and heading to goal
        double dx = goal.target_pose.point.x - current_state.pose.point.x;
        double dy = goal.target_pose.point.y - current_state.pose.point.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double heading_error = this->calculate_heading_error(current_state.pose, goal.target_pose.point);
        
        // Update status
        status_.distance_to_goal = distance;
        status_.heading_error = heading_error;
        status_.goal_reached = false;
        
        // Simple proportional control
        double angular_control = config_.kp_angular * heading_error;
        
        // Reduce speed when turning sharply
        double turn_reduction = 1.0 - std::min(std::abs(heading_error) / M_PI, 0.8);
        
        // Scale speed based on distance (slow down near goal)
        double distance_scale = std::min(distance / (config_.goal_tolerance * 5.0), 1.0);
        
        double linear_control = config_.kp_linear * distance * turn_reduction * distance_scale;
        
        // Apply control based on output type
        apply_carrot_control(cmd, linear_control, angular_control, constraints);
        
        cmd.valid = true;
        cmd.status_message = "Chasing carrot";
        status_.mode = "carrot";
        
        return cmd;
    }
    
    std::string get_type() const override { return "carrot"; }
    
private:
    void apply_carrot_control(
        VelocityCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        cmd.linear_velocity = std::clamp(linear_control,
            0.0, constraints.max_linear_velocity); // Only forward for carrot
        cmd.angular_velocity = std::clamp(angular_control,
            -constraints.max_angular_velocity, constraints.max_angular_velocity);
    }
    
    void apply_carrot_control(
        NormalizedCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        cmd.throttle = std::clamp(linear_control / constraints.max_linear_velocity, 0.0, 1.0);
        cmd.steering = std::clamp(angular_control / constraints.max_angular_velocity, -1.0, 1.0);
    }
    
    void apply_carrot_control(
        AckermannCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        cmd.speed = std::clamp(linear_control, 0.0, constraints.max_linear_velocity);
        
        // Convert angular velocity to steering angle
        if (std::abs(cmd.speed) > 1e-6) {
            double steering_angle = std::atan((angular_control * constraints.wheelbase) / cmd.speed);
            cmd.steering_angle = std::clamp(steering_angle,
                -constraints.max_steering_angle, constraints.max_steering_angle);
        } else {
            cmd.steering_angle = 0.0;
        }
    }
    
    void apply_carrot_control(
        DifferentialCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        // Use kinematics to convert to wheel speeds
        double linear = std::clamp(linear_control, 0.0, constraints.max_linear_velocity);
        double angular = std::clamp(angular_control, -constraints.max_angular_velocity, constraints.max_angular_velocity);
        
        double half_track = constraints.track_width / 2.0;
        cmd.left_speed = linear - (angular * half_track);
        cmd.right_speed = linear + (angular * half_track);
    }
};

} // namespace controllers
} // namespace navcon