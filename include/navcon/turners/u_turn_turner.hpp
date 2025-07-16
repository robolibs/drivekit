#pragma once

#include "navcon/controller.hpp"
#include <vector>
#include <cmath>

namespace navcon {
namespace turners {

// U-Turn Turner: For headlands, row-to-row transitions
// Input: Start pose + end pose
// Output: U-shaped maneuver from one position to another
class UTurnTurner : public Controller {
public:
    enum class UTurnPattern {
        BULB,           // Simple circular arc
        OMEGA,          // Teardrop shape
        SQUARE,         // Rectangular pattern
        CUSTOM          // User-defined waypoints
    };
    
    using Base = Controller;
    using Base::config_;
    using Base::status_;
    using Base::path_;
    using Base::path_index_;
    
    std::string get_type() const override { return "u_turn_turner"; }
    
    // Configure the U-turn
    void configure_turn(const Pose& start_pose, const Pose& end_pose, 
                       UTurnPattern pattern = UTurnPattern::BULB) {
        start_pose_ = start_pose;
        end_pose_ = end_pose;
        pattern_ = pattern;
        is_configured_ = true;
        is_complete_ = false;
        
        // Generate the turn waypoints
        generate_turn_waypoints();
        current_waypoint_ = 0;
        
        // Set up path for base controller
        path_.waypoints.clear();
        for (const auto& waypoint : turn_waypoints_) {
            path_.waypoints.emplace_back(waypoint);
        }
        path_index_ = 0;
    }
    
    // Quick setup for common headland widths
    void configure_headland_turn(const Pose& start_pose, const Pose& end_pose, 
                                double headland_width) {
        headland_width_ = headland_width;
        configure_turn(start_pose, end_pose, UTurnPattern::BULB);
    }
    
    VelocityCommand compute_control(const RobotState& current_state,
                                   const Goal& goal,
                                   const RobotConstraints& constraints,
                                   double dt) override {
        VelocityCommand cmd;
        
        if (!is_configured_) {
            cmd.valid = false;
            cmd.status_message = "U-turn turner not configured";
            return cmd;
        }
        
        // Check if already complete
        if (is_complete_) {
            cmd.valid = true;
            cmd.status_message = "U-turn complete";
            status_.goal_reached = true;
            status_.mode = "complete";
            return cmd;
        }
        
        // Check if we've reached the final position
        double distance_to_end = current_state.pose.point.distance_to(end_pose_.point);
        double heading_error = std::abs(normalize_angle(end_pose_.angle.yaw - current_state.pose.angle.yaw));
        
        if (distance_to_end < config_.goal_tolerance && heading_error < config_.angular_tolerance) {
            is_complete_ = true;
            cmd.valid = true;
            cmd.status_message = "U-turn complete";
            status_.goal_reached = true;
            status_.mode = "complete";
            return cmd;
        }
        
        // Follow the generated waypoints
        if (current_waypoint_ < turn_waypoints_.size()) {
            const Pose& target_waypoint = turn_waypoints_[current_waypoint_];
            
            // Check if we're close to current waypoint
            double distance_to_waypoint = current_state.pose.point.distance_to(target_waypoint.point);
            if (distance_to_waypoint < 1.0) { // Close enough threshold
                current_waypoint_++;
                if (current_waypoint_ >= turn_waypoints_.size()) {
                    // Use final target
                    return compute_control_to_pose(current_state, end_pose_, constraints);
                }
            }
            
            // Control to current waypoint
            const Pose& current_target = (current_waypoint_ < turn_waypoints_.size()) 
                ? turn_waypoints_[current_waypoint_] : end_pose_;
            
            return compute_control_to_pose(current_state, current_target, constraints);
        }
        
        // Fallback - control directly to end pose
        return compute_control_to_pose(current_state, end_pose_, constraints);
    }
    
    void reset() override {
        Controller::reset();
        is_configured_ = false;
        is_complete_ = false;
        start_pose_ = Pose{};
        end_pose_ = Pose{};
        turn_waypoints_.clear();
        current_waypoint_ = 0;
        headland_width_ = 5.0;
    }
    
    // Status queries
    bool is_turn_complete() const { return is_complete_; }
    bool is_configured() const { return is_configured_; }
    size_t get_current_waypoint() const { return current_waypoint_; }
    size_t get_total_waypoints() const { return turn_waypoints_.size(); }
    double get_progress() const {
        return turn_waypoints_.empty() ? 0.0 : 
               static_cast<double>(current_waypoint_) / turn_waypoints_.size();
    }
    
    // Get the generated waypoints for visualization
    std::vector<Pose> get_turn_waypoints() const { return turn_waypoints_; }
    
private:
    bool is_configured_ = false;
    bool is_complete_ = false;
    Pose start_pose_;
    Pose end_pose_;
    UTurnPattern pattern_ = UTurnPattern::BULB;
    std::vector<Pose> turn_waypoints_;
    size_t current_waypoint_ = 0;
    double headland_width_ = 5.0; // Default headland width
    
    void generate_turn_waypoints() {
        turn_waypoints_.clear();
        
        switch (pattern_) {
            case UTurnPattern::BULB:
                generate_bulb_pattern();
                break;
            case UTurnPattern::OMEGA:
                generate_omega_pattern();
                break;
            case UTurnPattern::SQUARE:
                generate_square_pattern();
                break;
            case UTurnPattern::CUSTOM:
                // Custom waypoints should be set externally
                break;
        }
    }
    
    void generate_bulb_pattern() {
        // Simple circular arc between start and end
        Point center = calculate_arc_center(start_pose_.point, end_pose_.point);
        double radius = start_pose_.point.distance_to(center);
        
        // Calculate start and end angles
        double start_angle = std::atan2(start_pose_.point.y - center.y, start_pose_.point.x - center.x);
        double end_angle = std::atan2(end_pose_.point.y - center.y, end_pose_.point.x - center.x);
        
        // Determine direction of arc
        double angle_diff = normalize_angle(end_angle - start_angle);
        int num_segments = 8;
        
        for (int i = 1; i <= num_segments; ++i) {
            double t = static_cast<double>(i) / num_segments;
            double angle = start_angle + t * angle_diff;
            
            Pose waypoint;
            waypoint.point.x = center.x + radius * std::cos(angle);
            waypoint.point.y = center.y + radius * std::sin(angle);
            waypoint.angle.yaw = angle + M_PI/2; // Tangent to circle
            
            turn_waypoints_.push_back(waypoint);
        }
    }
    
    void generate_omega_pattern() {
        // Teardrop shape - wider at the end
        double distance = start_pose_.point.distance_to(end_pose_.point);
        double width = std::max(headland_width_, distance * 0.3);
        
        // Calculate perpendicular direction
        double path_angle = std::atan2(end_pose_.point.y - start_pose_.point.y,
                                      end_pose_.point.x - start_pose_.point.x);
        double perp_angle = path_angle + M_PI/2;
        
        // Generate waypoints in omega shape
        int num_segments = 12;
        for (int i = 1; i <= num_segments; ++i) {
            double t = static_cast<double>(i) / num_segments;
            
            // Parametric omega curve
            double x_offset = width * std::sin(t * M_PI);
            double y_progress = t;
            
            Pose waypoint;
            waypoint.point.x = start_pose_.point.x + y_progress * (end_pose_.point.x - start_pose_.point.x) +
                              x_offset * std::cos(perp_angle);
            waypoint.point.y = start_pose_.point.y + y_progress * (end_pose_.point.y - start_pose_.point.y) +
                              x_offset * std::sin(perp_angle);
            
            // Calculate tangent direction
            double tangent_angle = path_angle + M_PI * std::cos(t * M_PI);
            waypoint.angle.yaw = tangent_angle;
            
            turn_waypoints_.push_back(waypoint);
        }
    }
    
    void generate_square_pattern() {
        // Rectangular pattern
        double distance = start_pose_.point.distance_to(end_pose_.point);
        double width = std::max(headland_width_, distance * 0.5);
        
        // Calculate perpendicular direction
        double path_angle = std::atan2(end_pose_.point.y - start_pose_.point.y,
                                      end_pose_.point.x - start_pose_.point.x);
        double perp_angle = path_angle + M_PI/2;
        
        // Corner 1: Move perpendicular from start
        Pose corner1 = start_pose_;
        corner1.point.x += width * std::cos(perp_angle);
        corner1.point.y += width * std::sin(perp_angle);
        corner1.angle.yaw = perp_angle;
        turn_waypoints_.push_back(corner1);
        
        // Corner 2: Move toward end position
        Pose corner2 = corner1;
        corner2.point.x += (end_pose_.point.x - start_pose_.point.x);
        corner2.point.y += (end_pose_.point.y - start_pose_.point.y);
        corner2.angle.yaw = path_angle;
        turn_waypoints_.push_back(corner2);
        
        // Corner 3: Move back toward end
        Pose corner3 = corner2;
        corner3.point.x -= width * std::cos(perp_angle);
        corner3.point.y -= width * std::sin(perp_angle);
        corner3.angle.yaw = perp_angle + M_PI;
        turn_waypoints_.push_back(corner3);
    }
    
    Point calculate_arc_center(const Point& start, const Point& end) {
        // Simple center calculation - midpoint offset
        Point midpoint;
        midpoint.x = (start.x + end.x) / 2.0;
        midpoint.y = (start.y + end.y) / 2.0;
        
        // Offset perpendicular to the line
        double line_angle = std::atan2(end.y - start.y, end.x - start.x);
        double perp_angle = line_angle + M_PI/2;
        
        Point center = midpoint;
        center.x += headland_width_ * 0.5 * std::cos(perp_angle);
        center.y += headland_width_ * 0.5 * std::sin(perp_angle);
        
        return center;
    }
    
    VelocityCommand compute_control_to_pose(const RobotState& current_state,
                                           const Pose& target_pose,
                                           const RobotConstraints& constraints) {
        // Simple proportional control to target pose
        VelocityCommand cmd;
        
        // Calculate distance and heading error
        double distance = current_state.pose.point.distance_to(target_pose.point);
        double heading_to_target = std::atan2(target_pose.point.y - current_state.pose.point.y,
                                             target_pose.point.x - current_state.pose.point.x);
        double heading_error = normalize_angle(heading_to_target - current_state.pose.angle.yaw);
        
        // Linear velocity proportional to distance
        cmd.linear_velocity = config_.kp_linear * distance;
        cmd.linear_velocity = std::clamp(cmd.linear_velocity, 0.0, constraints.max_linear_velocity);
        
        // Reduce speed for sharp turns
        double turn_factor = 1.0 - std::min(0.8, std::abs(heading_error) / M_PI);
        cmd.linear_velocity *= std::max(0.2, turn_factor);
        
        // Angular velocity proportional to heading error
        cmd.angular_velocity = config_.kp_angular * heading_error;
        cmd.angular_velocity = std::clamp(cmd.angular_velocity,
                                         -constraints.max_angular_velocity,
                                         constraints.max_angular_velocity);
        
        cmd.valid = true;
        cmd.status_message = "Following U-turn waypoint";
        
        // Update status
        status_.distance_to_goal = distance;
        status_.heading_error = std::abs(heading_error);
        status_.goal_reached = false;
        status_.mode = "u_turning";
        
        return cmd;
    }
};

} // namespace turners
} // namespace navcon