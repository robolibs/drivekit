#pragma once

#include "navcon/controller.hpp"
#include <cmath>

namespace navcon {
namespace turners {

class PointTurner : public Controller {
public:
    using Base = Controller;
    using Base::config_;
    using Base::status_;
    using Base::path_;
    using Base::path_index_;
    
    std::string get_type() const override { return "point_turner"; }
    
    void configure_turn(const Pose& position, double target_heading) {
        target_heading_ = target_heading;
        is_configured_ = true;
        is_complete_ = false;
    }
    
    VelocityCommand compute_control(const RobotState& current_state,
                                   const Goal& goal,
                                   const RobotConstraints& constraints,
                                   double dt) override {
        VelocityCommand cmd;
        
        if (!is_configured_) {
            cmd.valid = false;
            return cmd;
        }
        
        if (is_complete_) {
            cmd.valid = true;
            cmd.status_message = "Turn complete";
            return cmd;
        }
        
        // Check current speed - if moving, brake first
        double current_speed = current_state.velocity.linear;
        if (current_speed > 0.1) {
            // Robot is moving - brake to stop
            cmd.linear_velocity = -0.5;  // Brake
            cmd.angular_velocity = 0.0;
            cmd.valid = true;
            cmd.status_message = "Braking to stop";
            return cmd;
        }
        
        // Check if we're close to target heading
        double remaining_angle = normalize_angle(target_heading_ - current_state.pose.angle.yaw);
        if (std::abs(remaining_angle) < 0.1) {
            is_complete_ = true;
            cmd.valid = true;
            cmd.status_message = "Turn complete";
            return cmd;
        }
        
        // Now we're stopped, turn in place
        cmd.linear_velocity = 0.0;
        cmd.angular_velocity = (remaining_angle > 0) ? 0.5 : -0.5;
        cmd.valid = true;
        cmd.status_message = "Turning in place";
        
        return cmd;
    }
    
    void reset() override {
        Controller::reset();
        is_configured_ = false;
        is_complete_ = false;
        target_heading_ = 0.0;
    }
    
    bool is_turn_complete() const { return is_complete_; }
    bool is_configured() const { return is_configured_; }
    double get_target_heading() const { return target_heading_; }
    
private:
    bool is_configured_ = false;
    bool is_complete_ = false;
    double target_heading_ = 0.0;
};

} // namespace turners
} // namespace navcon