#include "navcon.hpp"
#include "navcon/utils/visualize.hpp"
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>

int main() {
    // Initialize visualization
    auto rec = std::make_shared<rerun::RecordingStream>("navcon_demo", "navigation");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Clear previous visualization data (allows running multiple times without restarting rerun)
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized");

    // 1. Create a navigation controller (using Pure Pursuit for smooth path following)
    navcon::Navcon navigator(navcon::NavconControllerType::PURE_PURSUIT);

    // 2. Set up robot constraints
    navcon::RobotConstraints constraints;
    constraints.max_linear_velocity = 1.0;  // 1 m/s max speed
    constraints.max_angular_velocity = 1.0; // 1 rad/s max turn rate
    constraints.wheelbase = 0.5;            // 0.5m wheelbase

    // 3. Initialize with visualization
    navigator.init(constraints, rec);

    // 4. Create a large "S" shaped path using many waypoints
    std::vector<concord::Point> s_path_waypoints;

    // Bottom curve of the "S" (starts at origin, curves right)
    s_path_waypoints.push_back({0.0, 0.0});
    s_path_waypoints.push_back({2.0, 0.0});
    s_path_waypoints.push_back({4.0, 0.5});
    s_path_waypoints.push_back({6.0, 1.5});
    s_path_waypoints.push_back({8.0, 3.0});
    s_path_waypoints.push_back({10.0, 5.0});
    s_path_waypoints.push_back({12.0, 7.5});
    s_path_waypoints.push_back({14.0, 10.0});
    s_path_waypoints.push_back({16.0, 12.0});
    s_path_waypoints.push_back({18.0, 13.5});
    s_path_waypoints.push_back({20.0, 14.5});
    s_path_waypoints.push_back({22.0, 15.0});
    s_path_waypoints.push_back({24.0, 15.0});

    // Middle transition (straight section)
    s_path_waypoints.push_back({26.0, 15.0});
    s_path_waypoints.push_back({28.0, 15.0});
    s_path_waypoints.push_back({30.0, 15.0});
    s_path_waypoints.push_back({32.0, 15.0});

    // Top curve of the "S" (curves left - mirror of bottom)
    s_path_waypoints.push_back({34.0, 15.0});
    s_path_waypoints.push_back({36.0, 14.5});
    s_path_waypoints.push_back({38.0, 13.5});
    s_path_waypoints.push_back({40.0, 12.0});
    s_path_waypoints.push_back({42.0, 10.0});
    s_path_waypoints.push_back({44.0, 7.5});
    s_path_waypoints.push_back({46.0, 5.0});
    s_path_waypoints.push_back({48.0, 3.0});
    s_path_waypoints.push_back({50.0, 1.5});
    s_path_waypoints.push_back({52.0, 0.5});
    s_path_waypoints.push_back({54.0, 0.0});
    s_path_waypoints.push_back({56.0, 0.0});

    navcon::PathGoal s_path(s_path_waypoints,
                            0.5,  // tolerance (0.5m)
                            0.8,  // max speed (0.8 m/s)
                            false // don't loop
    );

    navigator.set_path(s_path);
    navigator.smoothen(25.0f); // Add interpolated points every 25cm for smoother path

    spdlog::info("Large S-shaped path set");

    // 5. Setup initial robot state
    navcon::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0}; // start at origin
    robot_state.pose.angle.yaw = 0.0;                  // facing east

    // Visualize start and end poses
    navcon::Pose end_pose;
    end_pose.point = s_path_waypoints.back();
    end_pose.angle.yaw = 0.0;

    float dt = 0.1f;             // 100ms time step
    float print_interval = 0.5f; // Print every 0.5 seconds
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    // Navigation loop
    for (int i = 0; i < 2000 && !navigator.is_path_completed(); ++i) {
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

            // Visualize current robot state
            navcon::visualize::show_robot_state(rec, robot_state, "robot", rerun::Color(255, 255, 0));

            // Update navigation visualization
            navigator.tock();

            // Print progress every print_interval seconds
            if (current_time - last_print_time >= print_interval) {
                spdlog::info("Robot moving along path...");
                last_print_time = current_time;
            }

            // sleep thread
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (navigator.is_path_completed()) {
        spdlog::info("S-path completed!");
    } else {
        spdlog::warn("Timeout - path not completed");
    }

    return 0;
}
