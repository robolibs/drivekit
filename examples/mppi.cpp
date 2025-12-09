#include "drivekit.hpp"
#include "drivekit/utils/visualize.hpp"
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
    auto rec = std::make_shared<rerun::RecordingStream>("drivekit_mppi_demo", "mppi");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized for MPPI demo");

    drivekit::Tracker navigator(drivekit::TrackerType::MPPI);

    // Configure MPPI parameters
    auto mppi_controller = dynamic_cast<drivekit::pred::MPPIFollower *>(navigator.get_controller());
    if (mppi_controller) {
        auto mppi_config = mppi_controller->get_mppi_config();
        mppi_config.horizon_steps = 20;
        mppi_config.dt = 0.1;
        mppi_config.num_samples = 500;
        mppi_config.temperature = 1.0;
        mppi_config.steering_noise = 0.2;
        mppi_config.acceleration_noise = 0.3;
        mppi_config.weight_cte = 2000.0;
        mppi_config.weight_epsi = 2000.0;
        mppi_config.weight_vel = 1.0;
        mppi_config.weight_steering = 5.0;
        mppi_config.weight_acceleration = 5.0;
        mppi_config.ref_velocity = 1.0;
        mppi_controller->set_mppi_config(mppi_config);
        spdlog::info("MPPI configuration set: horizon={}, dt={}, ref_vel={}, samples={}",
                     mppi_config.horizon_steps, mppi_config.dt, mppi_config.ref_velocity, mppi_config.num_samples);
    }

    drivekit::RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.5;
    constraints.max_linear_acceleration = 1.0;
    constraints.wheelbase = 0.5;
    constraints.max_steering_angle = M_PI / 4; // 45 degrees

    navigator.init(constraints, rec);

    drivekit::PathGoal path_goal(build_s_shape_path(),
                               0.5f, // tolerance
                               1.0f, // max speed
                               false);

    navigator.set_path(path_goal);
    navigator.smoothen(25.0f); // Smooth interpolation

    drivekit::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0}; // Start on path
    robot_state.pose.angle.yaw = 0.0;
    robot_state.velocity.linear = 0.0;
    robot_state.velocity.angular = 0.0;

    float dt = 0.1f;
    float print_interval = 0.5f;
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    spdlog::info("Starting MPPI path tracking...");
    if (mppi_controller) {
        auto cfg = mppi_controller->get_mppi_config();
        spdlog::info("MPPI: horizon={} steps, dt={} s, samples={}", cfg.horizon_steps, cfg.dt, cfg.num_samples);
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
                std::cout << "MPPI CMD: linear=" << cmd.linear_velocity << ", angular=" << cmd.angular_velocity
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
            drivekit::visualize::show_robot_state(rec, robot_state, "robot_mppi",
                                                rerun::Color(255, 140, 0)); // Dark orange for MPPI
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                spdlog::info("MPPI: tracking path... (vel={:.2f} m/s, omega={:.2f} rad/s)", robot_state.velocity.linear,
                             robot_state.velocity.angular);
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            spdlog::warn("MPPI command invalid at iteration {}", iteration);

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
        spdlog::info("✓ MPPI path tracking completed successfully in {} iterations!", iteration);
    } else {
        spdlog::warn("✗ MPPI run timed out before finishing the path ({} iterations)", iteration);
    }

    return 0;
}

