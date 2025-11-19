#include "navcon.hpp"
#include "navcon/tracking/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>

namespace {
    std::vector<concord::Point> build_pid_waypoints() {
        return {{0.0f, 0.0f}, {1.5f, 0.5f}, {3.0f, 1.5f}, {4.5f, 2.0f}, {6.0f, 3.2f}, {7.0f, 3.8f}, {8.0f, 4.0f}};
    }

    navcon::Path make_visual_path(const std::vector<concord::Point> &points) {
        navcon::Path path;
        for (const auto &pt : points) {
            navcon::Pose pose;
            pose.point = pt;
            pose.angle = concord::Euler{0.0f, 0.0f, 0.0f};
            path.waypoints.push_back(pose);
        }
        return path;
    }
} // namespace

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("navcon_pid_demo", "pid");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized for PID demo");

    navcon::Tracker navigator(navcon::TrackerType::PID);

    auto params = navigator.get_controller_params();
    params.linear_kp = 2.5f;
    params.angular_kp = 1.8f;
    params.angular_kd = 0.2f;
    navigator.set_controller_params(params);

    navcon::RobotConstraints constraints;
    constraints.max_linear_velocity = 0.35;
    constraints.max_angular_velocity = 1.0;
    constraints.wheelbase = 0.45;

    navigator.init(constraints, rec);

    const auto waypoints = build_pid_waypoints();
    if (waypoints.empty()) {
        std::cerr << "No PID waypoints defined\n";
        return 1;
    }

    navcon::tracking::visualize::show_path(rec, make_visual_path(waypoints), "pid_path", rerun::Color(0, 120, 255));

    auto set_navigation_goal = [&](size_t index) {
        navcon::NavigationGoal next_goal(waypoints[index], 0.2f, 0.35f);
        navigator.set_goal(next_goal);

        navcon::Goal viz_goal;
        viz_goal.target_pose = concord::Pose{next_goal.target, concord::Euler{0.0f, 0.0f, 0.0f}};
        viz_goal.tolerance_position = next_goal.tolerance;
        navcon::tracking::visualize::show_goal(rec, viz_goal, "pid_goal");
    };

    size_t waypoint_index = 0;
    set_navigation_goal(waypoint_index);

    navcon::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0};
    robot_state.pose.angle.yaw = 0.0;

    float dt = 0.05f;
    float current_time = 0.0f;
    float last_print_time = 0.0f;
    const float print_interval = 0.5f;

    for (int i = 0; i < 2000 && waypoint_index < waypoints.size(); ++i) {
        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            robot_state.velocity.linear = cmd.linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity;

            robot_state.pose.point.x += cmd.linear_velocity * std::cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += cmd.linear_velocity * std::sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += cmd.angular_velocity * dt;

            current_time += dt;

            navcon::tracking::visualize::show_robot_state(rec, robot_state, "robot_pid", rerun::Color(0, 120, 255));
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                spdlog::info("PID controller driving toward goal...");
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        if (navigator.is_goal_reached()) {
            waypoint_index++;
            if (waypoint_index < waypoints.size()) {
                set_navigation_goal(waypoint_index);
            }
        }
    }

    if (waypoint_index >= waypoints.size()) {
        spdlog::info("PID waypoint path completed!");
    } else {
        spdlog::warn("PID demo timed out before completing the waypoint path");
    }

    return 0;
}
