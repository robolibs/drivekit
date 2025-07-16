#pragma once

#include "navcon/controller.hpp"
#include "navcon/followers/stanley.hpp"
#include "navcon/followers/pure_pursuit.hpp"
#include "navcon/turners/point_turner.hpp"
#include "navcon/turners/u_turn_turner.hpp"
#include <memory>
#include <cmath>

namespace navcon {

// Main controller that switches between followers and turners based on path analysis
class PathController : public Controller {
public:
    enum class ControllerType {
        FOLLOWER,
        POINT_TURNER,
        U_TURN_TURNER
    };
    
    enum class FollowerType {
        STANLEY,
        PURE_PURSUIT
    };
    
    PathController(FollowerType follower_type = FollowerType::STANLEY) {
        // Create the follower
        switch (follower_type) {
            case FollowerType::STANLEY:
                follower_ = std::make_unique<followers::StanleyFollower>();
                break;
            case FollowerType::PURE_PURSUIT:
                follower_ = std::make_unique<followers::PurePursuitFollower>();
                break;
        }
        
        // Create turners
        point_turner_ = std::make_unique<turners::PointTurner>();
        u_turn_turner_ = std::make_unique<turners::UTurnTurner>();
        
        // Start with follower
        active_type_ = ControllerType::FOLLOWER;
        active_controller_ = follower_.get();
    }
    
    std::string get_type() const override { return "path_controller"; }
    
    VelocityCommand compute_control(const RobotState& current_state,
                                   const Goal& goal,
                                   double dt) {
        // Use stored constraints
        return compute_control(current_state, goal, constraints_, dt);
    }

    VelocityCommand compute_control(const RobotState& current_state,
                                   const Goal& goal,
                                   const RobotConstraints& constraints,
                                   double dt) override {
        
        // Check if we need to switch controllers based on path analysis
        if (active_type_ == ControllerType::FOLLOWER) {
            analyze_path_and_switch_if_needed(current_state, constraints);
        }
        
        // Execute control with active controller
        auto cmd = active_controller_->compute_control(current_state, goal, constraints, dt);
        
        // Check if turner is complete and switch back to follower
        if (active_type_ == ControllerType::POINT_TURNER) {
            if (point_turner_->is_turn_complete()) {
                switch_to_follower();
            }
        } else if (active_type_ == ControllerType::U_TURN_TURNER) {
            if (u_turn_turner_->is_turn_complete()) {
                switch_to_follower();
            }
        }
        
        return cmd;
    }
    
    void set_path(const Path& path) override {
        Controller::set_path(path);
        // Set path on all controllers
        follower_->set_path(path);
        
        // Reset turner states
        point_turner_->reset();
        u_turn_turner_->reset();
        
        // Start with follower
        active_type_ = ControllerType::FOLLOWER;
        active_controller_ = follower_.get();
    }
    
    void reset() override {
        Controller::reset();
        follower_->reset();
        point_turner_->reset();
        u_turn_turner_->reset();
        
        active_type_ = ControllerType::FOLLOWER;
        active_controller_ = follower_.get();
    }
    
    void set_config(const ControllerConfig& config) override {
        Controller::set_config(config);
        follower_->set_config(config);
        point_turner_->set_config(config);
        u_turn_turner_->set_config(config);
    }
    
    // Configuration
    void set_sharp_turn_threshold(double angle_degrees) {
        sharp_turn_threshold_ = angle_degrees * M_PI / 180.0;
    }
    
    void set_u_turn_threshold(double angle_degrees) {
        u_turn_threshold_ = angle_degrees * M_PI / 180.0;
    }
    
    void set_lookahead_distance(double distance) {
        lookahead_distance_ = distance;
    }
    
    // Status
    ControllerType get_active_type() const { return active_type_; }
    std::string get_active_controller_name() const {
        switch (active_type_) {
            case ControllerType::FOLLOWER:
                return "follower";
            case ControllerType::POINT_TURNER:
                return "point_turner";
            case ControllerType::U_TURN_TURNER:
                return "u_turn_turner";
        }
        return "unknown";
    }
    
    // Get references to sub-controllers for advanced configuration
    Controller* get_follower() const { return follower_.get(); }
    turners::PointTurner* get_point_turner() const { return point_turner_.get(); }
    turners::UTurnTurner* get_u_turn_turner() const { return u_turn_turner_.get(); }
    
    // Set robot constraints
    void set_constraints(const RobotConstraints& constraints) {
        constraints_ = constraints;
    }

private:
    std::unique_ptr<Controller> follower_;
    std::unique_ptr<turners::PointTurner> point_turner_;
    std::unique_ptr<turners::UTurnTurner> u_turn_turner_;
    
    ControllerType active_type_;
    Controller* active_controller_;
    
    // Configuration parameters
    double sharp_turn_threshold_ = 60.0 * M_PI / 180.0;  // 60 degrees
    double u_turn_threshold_ = 120.0 * M_PI / 180.0;     // 120 degrees
    double lookahead_distance_ = 5.0;  // meters
    
    // Stored constraints for compute_control overload
    RobotConstraints constraints_;
    
    void analyze_path_and_switch_if_needed(const RobotState& current_state,
                                          const RobotConstraints& constraints) {
        if (path_.waypoints.empty()) return;
        
        // Look ahead in the path for sharp turns
        auto turn_info = analyze_upcoming_turn(current_state, constraints);
        
        if (turn_info.requires_turn) {
            if (turn_info.is_u_turn) {
                // Switch to U-turn turner
                u_turn_turner_->configure_turn(turn_info.start_pose, turn_info.end_pose);
                switch_to_u_turn_turner();
            } else {
                // Switch to point turner
                point_turner_->configure_turn(turn_info.start_pose, turn_info.target_heading);
                switch_to_point_turner();
            }
        }
    }
    
    struct TurnInfo {
        bool requires_turn = false;
        bool is_u_turn = false;
        Pose start_pose;
        Pose end_pose;
        double target_heading = 0.0;
        double angle_change = 0.0;
    };
    
    TurnInfo analyze_upcoming_turn(const RobotState& current_state,
                                  const RobotConstraints& constraints) {
        TurnInfo info;
        
        // Find current position in path
        size_t current_idx = find_closest_waypoint_index(current_state.pose);
        
        // Look ahead for sharp turns
        for (size_t i = current_idx; i < path_.waypoints.size() - 2; ++i) {
            // Check if we're close enough to consider this turn
            double distance_to_turn = current_state.pose.point.distance_to(path_.waypoints[i].point);
            if (distance_to_turn > lookahead_distance_) {
                continue;  // Too far, keep looking
            }
            
            // Calculate angle change at this waypoint
            const auto& prev = path_.waypoints[i];
            const auto& curr = path_.waypoints[i + 1];
            const auto& next = path_.waypoints[i + 2];
            
            double angle1 = std::atan2(curr.point.y - prev.point.y, curr.point.x - prev.point.x);
            double angle2 = std::atan2(next.point.y - curr.point.y, next.point.x - curr.point.x);
            double angle_change = std::abs(normalize_angle(angle2 - angle1));
            
            // Check if this is a sharp turn
            if (angle_change > sharp_turn_threshold_) {
                info.requires_turn = true;
                info.angle_change = angle_change;
                info.start_pose = curr;
                
                // Determine if it's a U-turn (position change) or point turn (same position)
                if (angle_change > u_turn_threshold_) {
                    // U-turn: go from current position to next position
                    info.is_u_turn = true;
                    info.end_pose = next;
                } else {
                    // Point turn: same position, different heading
                    info.is_u_turn = false;
                    info.target_heading = angle2;
                }
                
                break;  // Found a turn, no need to look further
            }
        }
        
        return info;
    }
    
    size_t find_closest_waypoint_index(const Pose& current_pose) {
        if (path_.waypoints.empty()) return 0;
        
        size_t closest_idx = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < path_.waypoints.size(); ++i) {
            double distance = current_pose.point.distance_to(path_.waypoints[i].point);
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }
    
    void switch_to_follower() {
        active_type_ = ControllerType::FOLLOWER;
        active_controller_ = follower_.get();
        
        // Update follower's path to continue from current position
        follower_->set_path(path_);
    }
    
    void switch_to_point_turner() {
        active_type_ = ControllerType::POINT_TURNER;
        active_controller_ = point_turner_.get();
    }
    
    void switch_to_u_turn_turner() {
        active_type_ = ControllerType::U_TURN_TURNER;
        active_controller_ = u_turn_turner_.get();
    }
};

} // namespace navcon