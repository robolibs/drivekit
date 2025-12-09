#include "waypoint.hpp"
#include "waypoint/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>

namespace {
    std::vector<concord::Point> build_s_shape_path() {
        return {{0.0, 0.0},   {2.0, 0.0},   {4.0, 0.5},   {6.0, 1.5},   {8.0, 3.0},   {10.0, 5.0},
                {12.0, 7.5},  {14.0, 10.0}, {16.0, 12.0}, {18.0, 13.5}, {20.0, 14.5}, {22.0, 15.0},
                {24.0, 15.0}, {26.0, 15.0}, {28.0, 15.0}, {30.0, 15.0}, {32.0, 15.0}, {34.0, 15.0},
                {36.0, 14.5}, {38.0, 13.5}, {40.0, 12.0}, {42.0, 10.0}, {44.0, 7.5},  {46.0, 5.0},
                {48.0, 3.0},  {50.0, 1.5},  {52.0, 0.5},  {54.0, 0.0},  {56.0, 0.0}};
    }
} // namespace

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("waypoint_soc_demo", "soc");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized for SOC demo");

    waypoint::Tracker navigator(waypoint::TrackerType::SOC);

    // Configure SOC parameters
    auto soc_controller = dynamic_cast<waypoint::pred::SOCFollower *>(navigator.get_controller());
    if (soc_controller) {
        auto soc_config = soc_controller->get_soc_config();
        soc_config.horizon_steps = 20;
        soc_config.dt = 0.1;
        soc_config.guide_samples = 64;
        soc_config.num_samples = 500;
        soc_config.temperature = 1.0;
        soc_config.guide_temperature = 1.0;
        soc_config.steering_noise = 0.2;
        soc_config.acceleration_noise = 0.3;
        soc_config.initial_steer_variance = 0.1;
        soc_config.min_steer_variance = 1e-4;
        soc_config.max_steer_variance = 0.5;
        soc_config.weight_cte = 2000.0;
        soc_config.weight_epsi = 2000.0;
        soc_config.weight_vel = 1.0;
        soc_config.weight_steering = 5.0;
        soc_config.weight_acceleration = 5.0;
        soc_config.ref_velocity = 1.0;
        soc_config.guide_iterations = 2;
        soc_config.guide_step_size = 0.2;
        soc_controller->set_soc_config(soc_config);
        spdlog::info("SOC configuration set: horizon={}, dt={}, ref_vel={}, guide_samples={}, samples={}",
                     soc_config.horizon_steps, soc_config.dt, soc_config.ref_velocity, soc_config.guide_samples,
                     soc_config.num_samples);
    }

    waypoint::RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.5;
    constraints.max_linear_acceleration = 1.0;
    constraints.wheelbase = 0.5;
    constraints.max_steering_angle = M_PI / 4; // 45 degrees

    navigator.init(constraints, rec);

    waypoint::PathGoal path_goal(build_s_shape_path(),
                               0.5f, // tolerance
                               1.0f, // max speed
                               false);

    navigator.set_path(path_goal);
    navigator.smoothen(25.0f); // Smooth interpolation

    waypoint::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0}; // Start on path
    robot_state.pose.angle.yaw = 0.0;
    robot_state.velocity.linear = 0.0;
    robot_state.velocity.angular = 0.0;

    float dt = 0.1f;
    float print_interval = 0.5f;
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    spdlog::info("Starting SOC path tracking...");
    if (soc_controller) {
        auto cfg = soc_controller->get_soc_config();
        spdlog::info("SOC: horizon={} steps, dt={} s, guide_samples={}, samples={}",
                     cfg.horizon_steps, cfg.dt, cfg.guide_samples, cfg.num_samples);
    }

    int iteration = 0;

    for (; iteration < 2000 && !navigator.is_path_completed(); ++iteration) {
        if (iteration % 100 == 0) {
            std::cout << "Iteration " << iteration << ", Robot at (" << robot_state.pose.point.x << ","
                      << robot_state.pose.point.y << ")" << std::endl;
        }

        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            // Debug: print commands every 50 iterations
            static int debug_count = 0;
            if (debug_count % 50 == 0) {
                std::cout << "SOC CMD: linear=" << cmd.linear_velocity << ", angular=" << cmd.angular_velocity
                          << " (normalized)" << std::endl;
                std::cout << "  Denormalized: linear=" << (cmd.linear_velocity * constraints.max_linear_velocity)
                          << " m/s, angular=" << (cmd.angular_velocity * constraints.max_angular_velocity) << " rad/s"
                          << std::endl;
            }
            debug_count++;

            // Apply velocity commands (denormalize if needed)
            robot_state.velocity.linear = cmd.linear_velocity * constraints.max_linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity * constraints.max_angular_velocity;

            // Update robot position using bicycle model
            robot_state.pose.point.x += robot_state.velocity.linear * std::cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += robot_state.velocity.linear * std::sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += robot_state.velocity.angular * dt;

            current_time += dt;

            // Visualize
            waypoint::visualize::show_robot_state(rec, robot_state, "robot_soc",
                                                rerun::Color(34, 139, 34)); // Forest green for SOC
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                spdlog::info("SOC: tracking path... (vel={:.2f} m/s, omega={:.2f} rad/s)", robot_state.velocity.linear,
                             robot_state.velocity.angular);
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            spdlog::warn("SOC command invalid at iteration {}", iteration);

            // Slow down if we fail to get a valid command
            robot_state.velocity.linear *= 0.8;
            robot_state.velocity.angular *= 0.8;

            // Update position anyway
            robot_state.pose.point.x += robot_state.velocity.linear * std::cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += robot_state.velocity.linear * std::sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += robot_state.velocity.angular * dt;
        }
    }

    if (navigator.is_path_completed()) {
        spdlog::info("✓ SOC path tracking completed successfully in {} iterations!", iteration);
    } else {
        spdlog::warn("✗ SOC run timed out before finishing the path ({})", iteration);
    }

    return 0;
}

