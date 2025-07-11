#pragma once

#include "navcon/controller.hpp"
#include <algorithm>

namespace navcon {
namespace controllers {

// Pure Pursuit controller for smooth path following
template<typename OutputCommand = VelocityCommand>
class PurePursuitController : public Controller<RobotState, OutputCommand> {
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
        
        // Get target point (either from path or direct goal)
        Point target_point;
        bool has_path = !path_.waypoints.empty();
        
        if (has_path) {
            auto lookahead_result = find_lookahead_point(current_state.pose);
            if (!lookahead_result.has_value()) {
                // Reached end of path
                target_point = goal.target_pose.point;
            } else {
                target_point = lookahead_result.value();
            }
        } else {
            target_point = goal.target_pose.point;
        }
        
        // Check if goal is reached
        if (this->is_goal_reached(current_state.pose, goal.target_pose)) {
            cmd.valid = true;
            cmd.status_message = "Goal reached";
            status_.goal_reached = true;
            status_.mode = "stopped";
            return cmd;
        }
        
        // Transform target to robot's local frame
        double dx_global = target_point.x - current_state.pose.point.x;
        double dy_global = target_point.y - current_state.pose.point.y;
        
        double cos_theta = std::cos(current_state.pose.angle.yaw);
        double sin_theta = std::sin(current_state.pose.angle.yaw);
        
        double dx_local = cos_theta * dx_global + sin_theta * dy_global;
        double dy_local = -sin_theta * dx_global + cos_theta * dy_global;
        
        // Calculate curvature using pure pursuit formula
        double lookahead_distance = std::sqrt(dx_local * dx_local + dy_local * dy_local);
        lookahead_distance = std::max(lookahead_distance, config_.lookahead_distance);
        
        double curvature = 2.0 * dy_local / (lookahead_distance * lookahead_distance);
        
        // Update status
        status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
        status_.cross_track_error = std::abs(dy_local);
        status_.goal_reached = false;
        status_.mode = "pure_pursuit";
        
        // Apply control based on output type
        apply_pure_pursuit_control(cmd, curvature, constraints, current_state.velocity.linear);
        
        cmd.valid = true;
        cmd.status_message = "Following path";
        
        return cmd;
    }
    
    std::string get_type() const override { return "pure_pursuit"; }
    
private:
    std::optional<Point> find_lookahead_point(const Pose& current_pose) {
        if (path_.waypoints.empty()) return std::nullopt;
        
        // Find the closest point on the path ahead of the robot
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_idx = path_index_;
        
        // Search forward from current index
        for (size_t i = path_index_; i < path_.waypoints.size(); ++i) {
            double dist = current_pose.point.distance_to(path_.waypoints[i].point);
            if (dist < min_distance) {
                min_distance = dist;
                closest_idx = i;
            }
        }
        
        // Update path index
        path_index_ = closest_idx;
        
        // Find lookahead point
        double accumulated_distance = 0.0;
        Point last_point = path_.waypoints[path_index_].point;
        
        for (size_t i = path_index_; i < path_.waypoints.size(); ++i) {
            Point current_point = path_.waypoints[i].point;
            double segment_length = last_point.distance_to(current_point);
            
            if (accumulated_distance + segment_length >= config_.lookahead_distance) {
                // Interpolate point on this segment
                double remaining = config_.lookahead_distance - accumulated_distance;
                double t = remaining / segment_length;
                
                Point lookahead;
                lookahead.x = last_point.x + t * (current_point.x - last_point.x);
                lookahead.y = last_point.y + t * (current_point.y - last_point.y);
                return lookahead;
            }
            
            accumulated_distance += segment_length;
            last_point = current_point;
        }
        
        // If we're near the end, return the last waypoint
        if (path_index_ == path_.waypoints.size() - 1) {
            return std::nullopt;
        }
        
        return path_.waypoints.back().point;
    }
    
    void apply_pure_pursuit_control(
        VelocityCommand& cmd,
        double curvature,
        const RobotConstraints& constraints,
        double current_speed
    ) {
        // Set linear velocity (could be constant or adaptive)
        cmd.linear_velocity = constraints.max_linear_velocity;
        
        // Angular velocity from curvature
        cmd.angular_velocity = curvature * cmd.linear_velocity;
        cmd.angular_velocity = std::clamp(cmd.angular_velocity,
            -constraints.max_angular_velocity, constraints.max_angular_velocity);
    }
    
    void apply_pure_pursuit_control(
        AckermannCommand& cmd,
        double curvature,
        const RobotConstraints& constraints,
        double current_speed
    ) {
        // Set speed
        cmd.speed = constraints.max_linear_velocity;
        
        // Steering angle from curvature
        // For Ackermann: tan(steering_angle) = curvature * wheelbase
        cmd.steering_angle = std::atan(curvature * constraints.wheelbase);
        cmd.steering_angle = std::clamp(cmd.steering_angle,
            -constraints.max_steering_angle, constraints.max_steering_angle);
    }
    
    void apply_pure_pursuit_control(
        NormalizedCommand& cmd,
        double curvature,
        const RobotConstraints& constraints,
        double current_speed
    ) {
        // Normalized throttle
        cmd.throttle = 0.8; // 80% throttle for path following
        
        // Normalized steering from curvature
        double max_curvature = 1.0 / constraints.min_turning_radius;
        cmd.steering = std::clamp(curvature / max_curvature, -1.0, 1.0);
    }
};

} // namespace controllers
} // namespace navcon