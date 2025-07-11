#pragma once

#include "navcon/controller.hpp"
#include <algorithm>

namespace navcon {
namespace controllers {

// Stanley controller for front-axle path following with cross-track error correction
template<typename OutputCommand = AckermannCommand>
class StanleyController : public Controller<RobotState, OutputCommand> {
public:
    using Base = Controller<RobotState, OutputCommand>;
    using Base::config_;
    using Base::status_;
    using Base::path_;
    using Base::path_index_;
    
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
        
        // Find nearest point on path
        auto [nearest_point, path_heading, cross_track_error] = find_nearest_path_point(current_state.pose);
        
        // Calculate heading error
        double heading_error = this->normalize_angle(path_heading - current_state.pose.angle.yaw);
        
        // Stanley control law
        // steering = heading_error + atan(k * cross_track_error / velocity)
        double velocity = std::max(std::abs(current_state.velocity.linear), 0.1); // Avoid division by zero
        double cross_track_term = std::atan(config_.k_cross_track * cross_track_error / velocity);
        double steering_control = config_.k_heading * heading_error + cross_track_term;
        
        // Update status
        status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
        status_.cross_track_error = cross_track_error;
        status_.heading_error = heading_error;
        status_.goal_reached = false;
        status_.mode = "stanley";
        
        // Apply control based on output type
        apply_stanley_control(cmd, steering_control, constraints);
        
        cmd.valid = true;
        cmd.status_message = "Stanley control active";
        
        return cmd;
    }
    
    std::string get_type() const override { return "stanley"; }
    
private:
    struct PathPoint {
        Point point;
        double heading;
        double cross_track_error;
    };
    
    PathPoint find_nearest_path_point(const Pose& current_pose) {
        PathPoint result;
        
        if (path_.waypoints.empty()) {
            // No path, use direct line to goal
            result.point = current_pose.point;
            result.heading = 0.0;
            result.cross_track_error = 0.0;
            return result;
        }
        
        double min_distance = std::numeric_limits<double>::max();
        size_t nearest_idx = path_index_;
        
        // Find nearest waypoint
        for (size_t i = path_index_; i < path_.waypoints.size(); ++i) {
            double dist = current_pose.point.distance_to(path_.waypoints[i].point);
            if (dist < min_distance) {
                min_distance = dist;
                nearest_idx = i;
            }
        }
        
        path_index_ = nearest_idx;
        
        // Calculate path heading at this point
        if (nearest_idx < path_.waypoints.size() - 1) {
            // Use heading to next waypoint
            const auto& current_wp = path_.waypoints[nearest_idx];
            const auto& next_wp = path_.waypoints[nearest_idx + 1];
            
            double dx = next_wp.point.x - current_wp.point.x;
            double dy = next_wp.point.y - current_wp.point.y;
            result.heading = std::atan2(dy, dx);
            
            // Calculate cross-track error (signed distance from path)
            // Using the line from current to next waypoint
            double path_dx = next_wp.point.x - current_wp.point.x;
            double path_dy = next_wp.point.y - current_wp.point.y;
            double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);
            
            if (path_length > 1e-6) {
                // Normal vector to path (rotated 90 degrees)
                double normal_x = -path_dy / path_length;
                double normal_y = path_dx / path_length;
                
                // Vector from path point to robot
                double to_robot_x = current_pose.point.x - current_wp.point.x;
                double to_robot_y = current_pose.point.y - current_wp.point.y;
                
                // Cross track error is projection onto normal
                result.cross_track_error = to_robot_x * normal_x + to_robot_y * normal_y;
            } else {
                result.cross_track_error = 0.0;
            }
            
            result.point = current_wp.point;
        } else {
            // At last waypoint, use its heading
            result.point = path_.waypoints[nearest_idx].point;
            result.heading = path_.waypoints[nearest_idx].angle.yaw;
            result.cross_track_error = 0.0;
        }
        
        return result;
    }
    
    void apply_stanley_control(
        AckermannCommand& cmd,
        double steering_control,
        const RobotConstraints& constraints
    ) {
        cmd.speed = constraints.max_linear_velocity;
        cmd.steering_angle = std::clamp(steering_control,
            -constraints.max_steering_angle, constraints.max_steering_angle);
    }
    
    void apply_stanley_control(
        VelocityCommand& cmd,
        double steering_control,
        const RobotConstraints& constraints
    ) {
        cmd.linear_velocity = constraints.max_linear_velocity;
        
        // Convert steering angle to angular velocity
        // angular_vel = (v / L) * tan(steering_angle)
        cmd.angular_velocity = (cmd.linear_velocity / constraints.wheelbase) * std::tan(steering_control);
        cmd.angular_velocity = std::clamp(cmd.angular_velocity,
            -constraints.max_angular_velocity, constraints.max_angular_velocity);
    }
    
    void apply_stanley_control(
        NormalizedCommand& cmd,
        double steering_control,
        const RobotConstraints& constraints
    ) {
        cmd.throttle = 0.8; // 80% throttle
        cmd.steering = std::clamp(steering_control / constraints.max_steering_angle, -1.0, 1.0);
    }
};

} // namespace controllers
} // namespace navcon