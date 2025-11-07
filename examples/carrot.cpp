#include "navcon.hpp"
#include "navcon/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>

namespace {
    std::vector<concord::Point> build_carrot_waypoints() {
        return {{-2.0f, -6.0f}, {-0.5f, -5.0f}, {1.0f, -4.0f}, {2.5f, -3.0f}, {4.0f, -1.5f}, {5.5f, -0.5f},
                {6.0f, -0.25f}, {6.0f, -3.0f}};
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
    auto rec = std::make_shared<rerun::RecordingStream>("navcon_carrot_demo", "carrot");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized for Carrot demo");

    navcon::Navcon navigator(navcon::NavconControllerType::CARROT);

    auto params = navigator.get_controller_params();
    params.carrot_distance = 1.2f;
    params.linear_kp = 1.2f;
    params.angular_kp = 1.0f;
    navigator.set_controller_params(params);

    navcon::RobotConstraints constraints;
    constraints.max_linear_velocity = 0.3;
    constraints.max_angular_velocity = 0.8;
    constraints.wheelbase = 0.4;

    navigator.init(constraints, rec);

    const auto waypoints = build_carrot_waypoints();
    if (waypoints.empty()) {
        std::cerr << "No carrot waypoints defined\n";
        return 1;
    }

    navcon::visualize::show_path(rec, make_visual_path(waypoints), "carrot_path", rerun::Color(255, 140, 0));

    auto set_navigation_goal = [&](size_t index) {
        navcon::NavigationGoal next_goal(waypoints[index], 0.25f, 0.3f);
        navigator.set_goal(next_goal);

        navcon::Goal viz_goal;
        viz_goal.target_pose = concord::Pose{next_goal.target, concord::Euler{0.0f, 0.0f, 0.0f}};
        viz_goal.tolerance_position = next_goal.tolerance;
        navcon::visualize::show_goal(rec, viz_goal, "carrot_goal", rerun::Color(255, 140, 0));
    };

    size_t waypoint_index = 0;
    set_navigation_goal(waypoint_index);

    navcon::RobotState robot_state;
    robot_state.pose.point = concord::Point{-2.0, -6.0};
    robot_state.pose.angle.yaw = 0.5f;

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

            navcon::visualize::show_robot_state(rec, robot_state, "robot_carrot", rerun::Color(255, 165, 0));
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                spdlog::info("Carrot follower chasing goal...");
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
        spdlog::info("Carrot waypoint path completed!");
    } else {
        spdlog::warn("Carrot demo timed out before reaching the final waypoint");
    }

    return 0;
}
