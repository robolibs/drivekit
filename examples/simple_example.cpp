#include "navcon.hpp"
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>

int main() {
    // 1. Create a navigation controller (using PID controller)
    navcon::Navcon navigator(navcon::NavconControllerType::PID);

    // 2. Set up robot constraints
    navcon::RobotConstraints constraints;
    constraints.max_linear_velocity = 1.0;  // 1 m/s max speed
    constraints.max_angular_velocity = 1.0; // 1 rad/s max turn rate
    constraints.wheelbase = 0.5;            // 0.5m wheelbase

    // 3. Initialize (without visualization for simplicity)
    navigator.init(constraints, nullptr);

    // 4. Set a simple goal: go to point (5, 3)
    navcon::NavigationGoal goal(concord::Point{5.0, 3.0}, // target position
                                0.1,                      // tolerance (0.5m)
                                0.8                       // max speed (0.8 m/s)
    );
    navigator.set_goal(goal);
    std::string goal_str = std::to_string(goal.target.x) + ", " + std::to_string(goal.target.y);
    spdlog::info("Goal set to {}", goal_str);

    // 5. Simulate robot movement
    navcon::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0}; // start at origin
    robot_state.pose.angle.yaw = 0.0;                  // facing east

    float dt = 0.1f;             // 100ms time step
    float print_interval = 0.5f; // Print every 0.5 seconds
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    // Navigation loop
    for (int i = 0; i < 500 && !navigator.is_goal_reached(); ++i) {
        // Get control command
        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            // Apply commands to robot (simulate movement)
            robot_state.velocity.linear = cmd.linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity;

            // Update robot position (simple integration)
            robot_state.pose.point.x += cmd.linear_velocity * cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += cmd.linear_velocity * sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += cmd.angular_velocity * dt;

            // Update time
            current_time += dt;

            // Print progress every print_interval seconds
            if (current_time - last_print_time >= print_interval) {
                spdlog::info("Time {}s: Robot at {}, {}", current_time, robot_state.pose.point.x,
                             robot_state.pose.point.y);
                // std::cout << "Time " << current_time << "s: Robot at (" << robot_state.pose.point.x << ", "
                //     << robot_state.pose.point.y << ")" << std::endl;
                // last_print_time = current_time;
            }

            // sleep thread
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (navigator.is_goal_reached()) {
        std::cout << "Goal reached!" << std::endl;
    } else {
        std::cout << "Timeout - goal not reached" << std::endl;
    }

    return 0;
}
