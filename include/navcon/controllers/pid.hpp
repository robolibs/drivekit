#pragma once

#include "navcon/controller.hpp"
#include <algorithm>

namespace navcon {
namespace controllers {

// PID controller for basic point-to-point navigation
template<typename OutputCommand = VelocityCommand>
class PIDController : public Controller<RobotState, OutputCommand> {
public:
    using Base = Controller<RobotState, OutputCommand>;
    using Base::config_;
    using Base::status_;
    
    PIDController() { reset(); }
    
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
        
        // Calculate errors
        double dx = goal.target_pose.point.x - current_state.pose.point.x;
        double dy = goal.target_pose.point.y - current_state.pose.point.y;
        double distance_error = std::sqrt(dx * dx + dy * dy);
        double heading_error = this->calculate_heading_error(current_state.pose, goal.target_pose.point);
        
        // Update status
        status_.distance_to_goal = distance_error;
        status_.heading_error = heading_error;
        status_.goal_reached = false;
        
        // PID for angular control
        angular_integral_ += heading_error * dt;
        double angular_derivative = (heading_error - last_heading_error_) / dt;
        last_heading_error_ = heading_error;
        
        double angular_control = config_.kp_angular * heading_error +
                               config_.ki_angular * angular_integral_ +
                               config_.kd_angular * angular_derivative;
        
        // PID for linear control
        linear_integral_ += distance_error * dt;
        double linear_derivative = (distance_error - last_distance_error_) / dt;
        last_distance_error_ = distance_error;
        
        double linear_control = config_.kp_linear * distance_error +
                              config_.ki_linear * linear_integral_ +
                              config_.kd_linear * linear_derivative;
        
        // Reduce linear speed when turning
        double turn_reduction = 1.0 - std::min(std::abs(heading_error) / M_PI, 0.8);
        linear_control *= turn_reduction;
        
        // Apply to command based on output type
        apply_control_to_command(cmd, linear_control, angular_control, constraints);
        
        cmd.valid = true;
        cmd.status_message = "Tracking goal";
        status_.mode = "tracking";
        
        return cmd;
    }
    
    void reset() override {
        Base::reset();
        linear_integral_ = 0.0;
        angular_integral_ = 0.0;
        last_distance_error_ = 0.0;
        last_heading_error_ = 0.0;
    }
    
    std::string get_type() const override { return "pid"; }
    
private:
    double linear_integral_ = 0.0;
    double angular_integral_ = 0.0;
    double last_distance_error_ = 0.0;
    double last_heading_error_ = 0.0;
    
    void apply_control_to_command(
        VelocityCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        cmd.linear_velocity = std::clamp(linear_control, 
            constraints.min_linear_velocity, constraints.max_linear_velocity);
        cmd.angular_velocity = std::clamp(angular_control,
            -constraints.max_angular_velocity, constraints.max_angular_velocity);
    }
    
    void apply_control_to_command(
        NormalizedCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        // Normalize to -1 to 1 range
        cmd.throttle = std::clamp(linear_control / constraints.max_linear_velocity, -1.0, 1.0);
        cmd.steering = std::clamp(angular_control / constraints.max_angular_velocity, -1.0, 1.0);
    }
    
    void apply_control_to_command(
        AckermannCommand& cmd,
        double linear_control,
        double angular_control,
        const RobotConstraints& constraints
    ) {
        cmd.speed = std::clamp(linear_control,
            constraints.min_linear_velocity, constraints.max_linear_velocity);
        
        // Convert angular velocity to steering angle
        if (std::abs(cmd.speed) > 1e-6) {
            double steering_angle = std::atan((angular_control * constraints.wheelbase) / cmd.speed);
            cmd.steering_angle = std::clamp(steering_angle,
                -constraints.max_steering_angle, constraints.max_steering_angle);
        } else {
            cmd.steering_angle = 0.0;
        }
    }
};

} // namespace controllers
} // namespace navcon