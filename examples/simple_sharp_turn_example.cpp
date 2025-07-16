#include "navcon/path_controller.hpp"
#include <iostream>
#include <vector>

using namespace navcon;

void print_robot_state(const RobotState& state, const std::string& prefix = "") {
    std::cout << prefix << "Robot at (" << state.pose.point.x << ", " << state.pose.point.y 
              << ") heading " << (state.pose.angle.yaw * 180.0 / M_PI) << "°" << std::endl;
}

int main() {
    std::cout << "Simple Sharp Turn Example" << std::endl;
    std::cout << "=========================" << std::endl;
    
    // Create path controller with Stanley follower
    PathController controller(PathController::FollowerType::STANLEY);
    
    // Configure thresholds
    controller.set_sharp_turn_threshold(45.0);  // 45 degrees
    controller.set_u_turn_threshold(120.0);     // 120 degrees
    controller.set_lookahead_distance(3.0);     // 3 meters
    
    // Create a path with different types of turns
    Path path;
    
    // Straight section
    path.waypoints.emplace_back(Pose{{0, 0}, {0, 0, 0}});       // Start
    path.waypoints.emplace_back(Pose{{10, 0}, {0, 0, 0}});      // Straight
    path.waypoints.emplace_back(Pose{{20, 0}, {0, 0, 0}});      // Straight
    
    // Sharp 90-degree turn (Point Turner)
    path.waypoints.emplace_back(Pose{{20, 0}, {0, 0, M_PI/2}});  // Turn north
    path.waypoints.emplace_back(Pose{{20, 5}, {0, 0, M_PI/2}});  // Continue north
    
    // U-turn for headland (U-Turn Turner)
    path.waypoints.emplace_back(Pose{{20, 10}, {0, 0, M_PI/2}});  // End of row
    path.waypoints.emplace_back(Pose{{15, 15}, {0, 0, M_PI}});    // Headland turn
    path.waypoints.emplace_back(Pose{{0, 15}, {0, 0, M_PI}});     // Next row
    
    controller.set_path(path);
    
    // Robot starts at origin
    RobotState state;
    state.pose = Pose{{0, 0}, {0, 0, 0}};
    state.velocity = {0, 0, 0};
    
    // Goal is the end of the path
    Goal goal;
    goal.target_pose = path.waypoints.back();
    
    // Robot constraints
    RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.0;
    constraints.min_turning_radius = 2.0;
    constraints.wheelbase = 1.5;
    
    // Set controller config
    ControllerConfig config;
    config.goal_tolerance = 0.5;
    config.angular_tolerance = 0.1;
    config.kp_linear = 1.0;
    config.kp_angular = 2.0;
    controller.set_config(config);
    
    std::cout << "Path has " << path.waypoints.size() << " waypoints" << std::endl;
    std::cout << "Starting simulation..." << std::endl << std::endl;
    
    // Simulation loop
    double dt = 0.1;
    std::string last_controller = "";
    
    for (int step = 0; step < 200; ++step) {
        // Check for controller changes
        std::string current_controller = controller.get_active_controller_name();
        if (current_controller != last_controller) {
            print_robot_state(state, ">>> ");
            std::cout << ">>> Controller switched to: " << current_controller << std::endl;
            last_controller = current_controller;
        }
        
        // Compute control
        auto cmd = controller.compute_control(state, goal, constraints, dt);
        
        if (!cmd.valid) {
            std::cout << "Control command invalid: " << cmd.status_message << std::endl;
            break;
        }
        
        // Update robot state (simplified physics)
        state.pose.point.x += cmd.linear_velocity * dt * std::cos(state.pose.angle.yaw);
        state.pose.point.y += cmd.linear_velocity * dt * std::sin(state.pose.angle.yaw);
        state.pose.angle.yaw += cmd.angular_velocity * dt;
        
        // Normalize angle
        while (state.pose.angle.yaw > M_PI) state.pose.angle.yaw -= 2 * M_PI;
        while (state.pose.angle.yaw < -M_PI) state.pose.angle.yaw += 2 * M_PI;
        
        state.velocity.linear = cmd.linear_velocity;
        state.velocity.angular = cmd.angular_velocity;
        
        // Print periodic updates
        if (step % 20 == 0) {
            print_robot_state(state);
            std::cout << "  Controller: " << current_controller 
                      << ", Linear: " << cmd.linear_velocity 
                      << ", Angular: " << cmd.angular_velocity << std::endl;
        }
        
        // Check if goal reached
        double distance_to_goal = state.pose.point.distance_to(goal.target_pose.point);
        if (distance_to_goal < config.goal_tolerance) {
            std::cout << std::endl << "Goal reached!" << std::endl;
            print_robot_state(state, "Final: ");
            break;
        }
        
        // Safety check
        if (std::abs(state.pose.point.x) > 50 || std::abs(state.pose.point.y) > 50) {
            std::cout << "Robot went out of bounds, stopping simulation" << std::endl;
            break;
        }
    }
    
    std::cout << std::endl << "Simulation complete." << std::endl;
    return 0;
}