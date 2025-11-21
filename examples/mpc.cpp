#include "navcon.hpp"
#include "navcon/tracking/utils/visualize.hpp"
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
#ifdef HAS_MPC
    auto rec = std::make_shared<rerun::RecordingStream>("navcon_mpc_demo", "mpc");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    spdlog::info("Visualization initialized for MPC demo");

    navcon::Tracker navigator(navcon::TrackerType::MPC);

    // Get and configure MPC parameters
    auto mpc_controller = dynamic_cast<navcon::tracking::path::MPCFollower *>(navigator.get_controller());
    if (mpc_controller) {
        auto mpc_config = mpc_controller->get_mpc_config();
        mpc_config.horizon_steps = 10;              // Prediction horizon
        mpc_config.dt = 0.1;                        // Time step
        mpc_config.ref_velocity = 1.0;              // Reference velocity (m/s)
        mpc_config.weight_cte = 2000.0;             // Cross-track error weight
        mpc_config.weight_epsi = 2000.0;            // Heading error weight
        mpc_config.weight_vel = 1.0;                // Velocity tracking weight
        mpc_config.weight_steering = 5.0;           // Steering effort weight
        mpc_config.weight_acceleration = 5.0;       // Acceleration effort weight
        mpc_config.weight_steering_rate = 200.0;    // Steering smoothness
        mpc_config.weight_acceleration_rate = 10.0; // Acceleration smoothness
        mpc_config.max_solver_time = 0.5;           // IPOPT solver time limit
        mpc_config.print_level = 0;                 // Silent IPOPT output
        mpc_controller->set_mpc_config(mpc_config);
        spdlog::info("MPC configuration set: horizon={}, dt={}, ref_vel={}", mpc_config.horizon_steps, mpc_config.dt,
                     mpc_config.ref_velocity);
    }

    navcon::RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.5;
    constraints.max_linear_acceleration = 1.0;
    constraints.wheelbase = 0.5;
    constraints.max_steering_angle = M_PI / 4; // 45 degrees

    navigator.init(constraints, rec);

    navcon::PathGoal path_goal(build_s_shape_path(),
                               0.5f, // tolerance
                               1.0f, // max speed
                               false);

    navigator.set_path(path_goal);
    navigator.smoothen(25.0f); // Smooth interpolation

    navcon::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0}; // Start on path
    robot_state.pose.angle.yaw = 0.0;
    robot_state.velocity.linear = 0.0;
    robot_state.velocity.angular = 0.0;

    float dt = 0.1f;
    float print_interval = 0.5f;
    float last_print_time = 0.0f;
    float current_time = 0.0f;

    spdlog::info("Starting MPC path tracking...");
    spdlog::info("MPC uses Model Predictive Control with IPOPT solver for optimal trajectory planning");
    spdlog::info("Prediction horizon: {} steps, dt: {} seconds",
                 mpc_controller ? mpc_controller->get_mpc_config().horizon_steps : 10,
                 mpc_controller ? mpc_controller->get_mpc_config().dt : 0.1);

    spdlog::info("Starting main control loop...");
    int solver_failures = 0;

    for (int i = 0; i < 2000 && !navigator.is_path_completed(); ++i) {
        if (i % 100 == 0) {
            std::cout << "Iteration " << i << ", Robot at (" << robot_state.pose.point.x << ","
                      << robot_state.pose.point.y << "), solver failures: " << solver_failures << std::endl;
        }

        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            // Debug: print commands every 50 iterations
            static int debug_count = 0;
            if (debug_count % 50 == 0) {
                std::cout << "MPC CMD: linear=" << cmd.linear_velocity << ", angular=" << cmd.angular_velocity
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
            navcon::tracking::visualize::show_robot_state(rec, robot_state, "robot_mpc",
                                                          rerun::Color(0, 191, 255)); // Deep sky blue color
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                spdlog::info("MPC: tracking path... (vel={:.2f} m/s, omega={:.2f} rad/s)", robot_state.velocity.linear,
                             robot_state.velocity.angular);
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            solver_failures++;
            spdlog::warn("MPC solver failed at iteration {}", i);

            // Continue with reduced velocity if solver fails
            robot_state.velocity.linear *= 0.8;
            robot_state.velocity.angular *= 0.8;

            // Update position anyway
            robot_state.pose.point.x += robot_state.velocity.linear * std::cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += robot_state.velocity.linear * std::sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += robot_state.velocity.angular * dt;
        }
    }

    if (navigator.is_path_completed()) {
        spdlog::info("✓ MPC path tracking completed successfully!");
        spdlog::info("  Total solver failures: {}", solver_failures);
    } else {
        spdlog::warn("✗ MPC run timed out before finishing the path");
        spdlog::warn("  Total solver failures: {}", solver_failures);
    }

    return 0;
#else
    spdlog::error("MPC controller not available - rebuild with -DNAVCON_ENABLE_MPC=ON");
    spdlog::error("Make sure Eigen3, IPOPT, and CppAD are installed");
    return 1;
#endif
}
