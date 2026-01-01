#include "drivekit.hpp"
#include "drivekit/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <thread>

namespace {
    std::vector<datapod::Point> build_s_shape_path() {
        return {{0.0, 0.0},   {2.0, 0.0},   {4.0, 0.5},   {6.0, 1.5},   {8.0, 3.0},   {10.0, 5.0},
                {12.0, 7.5},  {14.0, 10.0}, {16.0, 12.0}, {18.0, 13.5}, {20.0, 14.5}, {22.0, 15.0},
                {24.0, 15.0}, {26.0, 15.0}, {28.0, 15.0}, {30.0, 15.0}, {32.0, 15.0}, {34.0, 15.0},
                {36.0, 14.5}, {38.0, 13.5}, {40.0, 12.0}, {42.0, 10.0}, {44.0, 7.5},  {46.0, 5.0},
                {48.0, 3.0},  {50.0, 1.5},  {52.0, 0.5},  {54.0, 0.0},  {56.0, 0.0}};
    }
} // namespace

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("drivekit_stanley_demo", "stanley");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    std::cout << "Visualization initialized for Stanley demo\n";

    drivekit::Tracker navigator(drivekit::TrackerType::STANLEY);

    auto params = navigator.get_controller_params();
    params.cross_track_gain = 2.5f;
    params.softening_gain = 1.0f;
    navigator.set_controller_params(params);

    drivekit::RobotConstraints constraints;
    constraints.max_linear_velocity = 1.0;
    constraints.max_angular_velocity = 1.0;
    constraints.wheelbase = 0.5;

    navigator.init(constraints, rec);

    drivekit::PathGoal path_goal(build_s_shape_path(), 0.5f, 0.8f, false);

    navigator.set_path(path_goal);
    navigator.smoothen(25.0f); // Match smoothing cadence from navi example

    drivekit::RobotState robot_state;
    robot_state.pose.point = datapod::Point{0.0, 0.0};
    robot_state.pose.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);

    float dt = 0.1f;
    float print_interval = 0.5f;
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    for (int i = 0; i < 2000 && !navigator.is_path_completed(); ++i) {
        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            robot_state.velocity.linear = cmd.linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity;

            robot_state.pose.point.x += cmd.linear_velocity * std::cos(robot_state.pose.rotation.to_euler().yaw) * dt;
            robot_state.pose.point.y += cmd.linear_velocity * std::sin(robot_state.pose.rotation.to_euler().yaw) * dt;
            robot_state.pose.rotation = datapod::Quaternion::from_euler(
                0.0, 0.0, robot_state.pose.rotation.to_euler().yaw + cmd.angular_velocity * dt);

            current_time += dt;

            drivekit::visualize::show_robot_state(rec, robot_state, "robot_stanley", rerun::Color(255, 165, 0));
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                std::cout << "Stanley: tracking path...\n";
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (navigator.is_path_completed()) {
        std::cout << "Stanley path completed!\n";
    } else {
        std::cerr << "Stanley run timed out before finishing the path\n";
    }

    return 0;
}
