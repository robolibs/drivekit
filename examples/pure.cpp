#include "navcon.hpp"
#include "navcon/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>

namespace {
    std::vector<concord::Point> build_s_shape_path() {
        return {
            {0.0, 0.0},  {2.0, 0.0},  {4.0, 0.5},  {6.0, 1.5},  {8.0, 3.0},  {10.0, 5.0}, {12.0, 7.5},
            {14.0, 10.0}, {16.0, 12.0}, {18.0, 13.5}, {20.0, 14.5}, {22.0, 15.0}, {24.0, 15.0}, {26.0, 15.0},
            {28.0, 15.0}, {30.0, 15.0}, {32.0, 15.0}, {34.0, 15.0}, {36.0, 14.5}, {38.0, 13.5}, {40.0, 12.0},
            {42.0, 10.0}, {44.0, 7.5}, {46.0, 5.0}, {48.0, 3.0}, {50.0, 1.5}, {52.0, 0.5}, {54.0, 0.0},
            {56.0, 0.0}};
    }
} // namespace

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("navcon_pure_pursuit_demo", "pure_pursuit");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized for Pure Pursuit demo");

    navcon::Navcon navigator(navcon::NavconControllerType::PURE_PURSUIT);

    auto params = navigator.get_controller_params();
    params.lookahead_distance = 2.5f;
    navigator.set_controller_params(params);

    navcon::RobotConstraints constraints;
    constraints.max_linear_velocity = 1.0;
    constraints.max_angular_velocity = 1.0;
    constraints.wheelbase = 0.5;

    navigator.init(constraints, rec);

    navcon::PathGoal path_goal(build_s_shape_path(),
                               0.5f,  // tolerance
                               0.8f,  // max speed
                               false);

    navigator.set_path(path_goal);
    navigator.smoothen(25.0f); // Match smoothing cadence from navi example

    navcon::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0};
    robot_state.pose.angle.yaw = 0.0;

    float dt = 0.1f;
    float print_interval = 0.5f;
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    for (int i = 0; i < 2000 && !navigator.is_path_completed(); ++i) {
        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            robot_state.velocity.linear = cmd.linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity;

            robot_state.pose.point.x += cmd.linear_velocity * std::cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += cmd.linear_velocity * std::sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += cmd.angular_velocity * dt;

            current_time += dt;

            navcon::visualize::show_robot_state(rec, robot_state, "robot_pure_pursuit", rerun::Color(0, 255, 0));
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                spdlog::info("Pure Pursuit: tracking path...");
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (navigator.is_path_completed()) {
        spdlog::info("Pure Pursuit path completed!");
    } else {
        spdlog::warn("Pure Pursuit run timed out before finishing the path");
    }

    return 0;
}
