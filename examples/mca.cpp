#include "drivekit/pred/mca.hpp"
#include "drivekit/types.hpp"
#include "drivekit/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <thread>

namespace {
    // Build a corridor path
    drivekit::Path build_corridor_path() {
        drivekit::Path path;
        for (double x = 0.0; x <= 35.0; x += 0.5) {
            drivekit::Pose pose;
            pose.point.x = x;
            pose.point.y = 0.0;
            pose.point.z = 0.0;
            pose.angle.yaw = 0.0;
            path.drivekits.push_back(pose);
        }
        return path;
    }

    // Dynamic obstacle (moves)
    struct DynamicObstacle {
        size_t id;
        double x, y;
        double vx, vy;
        double size;
        double activation_distance; // Start moving when robot is this close
        bool is_active;

        void update(double dt, double robot_x, double robot_y) {
            // Check if robot is close enough to activate
            double dist = std::sqrt((x - robot_x) * (x - robot_x) + (y - robot_y) * (y - robot_y));
            if (dist < activation_distance) {
                is_active = true;
            }

            // Only move if active
            if (is_active) {
                x += vx * dt;
                y += vy * dt;
            }
        }
    };

    // Static obstacle (doesn't move)
    struct StaticObstacle {
        size_t id;
        double x, y;
        double size;
    };
} // namespace

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("drivekit_mca_demo");
    rec->spawn().exit_on_failure();

    std::cout << "=== MCA (DRA-MPPI) Demo ===" << std::endl;

    // Create MCA controller
    drivekit::pred::MCAFollower::MCAConfig mca_config;
    mca_config.horizon_steps = 20;
    mca_config.dt = 0.2;
    mca_config.num_samples = 400;
    mca_config.num_mc_samples = 20000;
    mca_config.temperature = 1.0;
    mca_config.steering_noise = 0.5;
    mca_config.acceleration_noise = 0.3;
    mca_config.weight_cte = 100.0;
    mca_config.weight_epsi = 100.0;
    mca_config.weight_vel = 1.0;
    mca_config.weight_steering = 10.0;
    mca_config.weight_acceleration = 5.0;
    mca_config.weight_soft_risk = 100.0;
    mca_config.weight_hard_risk = 1e6;
    mca_config.risk_threshold = 0.05;
    mca_config.ref_velocity = 2.0;

    auto mca_controller = std::make_unique<drivekit::pred::MCAFollower>(mca_config);

    std::cout << "Config: samples=" << mca_config.num_samples
              << ", risk_threshold=" << (mca_config.risk_threshold * 100) << "%" << std::endl;

    // Set up path
    auto path = build_corridor_path();
    mca_controller->set_path(path);

    // Visualize reference path
    drivekit::visualize::show_path(rec, path, "world/path", rerun::Color(150, 150, 150));

    // Create static obstacles (RED - don't move) - RIGHT IN THE PATH
    std::vector<StaticObstacle> static_obstacles;
    static_obstacles.push_back({1, 10.0, 0.0, 0.8});  // In the center of path
    static_obstacles.push_back({2, 20.0, 0.5, 0.8});  // Slightly off-center
    static_obstacles.push_back({3, 30.0, -0.5, 0.8}); // Other side

    // Create dynamic obstacles (GREEN - move when robot gets close)
    std::vector<DynamicObstacle> dynamic_obstacles;
    // Obstacle 1: Crosses upward at x=15, activates when robot within 8m
    dynamic_obstacles.push_back({1, 15.0, -2.5, 0.0, 0.5, 0.6, 8.0, false});
    // Obstacle 2: Moves ALONG path towards robot (head-on), activates when robot within 12m
    dynamic_obstacles.push_back({2, 30.0, 0.0, -1.0, 0.0, 0.6, 12.0, false});
    // Obstacle 3: Crosses downward at x=25, activates when robot within 8m
    dynamic_obstacles.push_back({3, 25.0, 2.5, 0.0, -0.5, 0.6, 8.0, false});

    std::cout << "Static obstacles: " << static_obstacles.size() << " (RED)" << std::endl;
    std::cout << "Dynamic obstacles: " << dynamic_obstacles.size() << " (GREEN)" << std::endl;

    // Robot constraints
    drivekit::RobotConstraints constraints;
    constraints.steering_type = drivekit::SteeringType::DIFFERENTIAL;
    constraints.max_linear_velocity = 3.0;
    constraints.min_linear_velocity = 0.0;
    constraints.max_angular_velocity = 2.0;
    constraints.max_linear_acceleration = 2.0;
    constraints.wheelbase = 0.5;
    constraints.max_steering_angle = M_PI / 4;
    constraints.robot_width = 0.6;
    constraints.robot_length = 0.8;

    drivekit::ControllerConfig ctrl_config;
    ctrl_config.output_units = drivekit::OutputUnits::NORMALIZED;
    ctrl_config.allow_reverse = false;
    mca_controller->set_config(ctrl_config);

    // Initial state
    drivekit::RobotState robot_state;
    robot_state.pose.point = concord::Point{0.0, 0.0, 0.0};
    robot_state.pose.angle.yaw = 0.0;
    robot_state.velocity.linear = 0.0;
    robot_state.velocity.angular = 0.0;

    drivekit::Goal goal;
    goal.target_pose.point = path.drivekits.back().point;
    goal.target_pose.angle.yaw = 0.0;
    goal.tolerance_position = 0.5;

    float dt = 0.1f;
    float current_time = 0.0f;

    std::cout << "\nStarting simulation...\n";

    int iteration = 0;
    const int max_iterations = 3000;

    for (; iteration < max_iterations; ++iteration) {
        rec->set_time_duration_secs("sim_time", current_time);

        // Update dynamic obstacle positions (pass robot position)
        for (auto &dyn : dynamic_obstacles) {
            dyn.update(dt, robot_state.pose.point.x, robot_state.pose.point.y);
        }

        // === VISUALIZE OBSTACLES ===

        // 1. Static obstacles (RED boxes) - z=0 for center AND half-size
        for (const auto &obs : static_obstacles) {
            std::string name = "obstacles/static_" + std::to_string(obs.id);
            rec->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                      {{(float)obs.x, (float)obs.y, 0.0f}},
                                      {{(float)(obs.size / 2.0f), (float)(obs.size / 2.0f), 0.0f}})
                                      .with_colors(rerun::Color(255, 0, 0)));
        }

        // 2. Dynamic obstacles (GREEN boxes) - z=0 for center AND half-size
        for (const auto &dyn : dynamic_obstacles) {
            std::string name = "obstacles/dynamic_" + std::to_string(dyn.id);
            rec->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                      {{(float)dyn.x, (float)dyn.y, 0.0f}},
                                      {{(float)(dyn.size / 2.0f), (float)(dyn.size / 2.0f), 0.0f}})
                                      .with_colors(rerun::Color(0, 255, 0)));
        }

        // 2. Dynamic obstacles (GREEN boxes) - 3D view with z=0
        for (const auto &dyn : dynamic_obstacles) {
            std::string name = "obstacles/dynamic_" + std::to_string(dyn.id);
            rec->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                      {{(float)dyn.x, (float)dyn.y, 0.0f}},
                                      {{(float)(dyn.size / 2.0f), (float)(dyn.size / 2.0f), 0.05f}})
                                      .with_colors(rerun::Color(0, 255, 0)));
        }

        // 2. Dynamic obstacles (GREEN boxes) - 2D only
        for (const auto &dyn : dynamic_obstacles) {
            std::string name = "obstacles/dynamic_" + std::to_string(dyn.id);
            rec->log_static(name, rerun::Boxes2D::from_centers_and_sizes({{(float)dyn.x, (float)dyn.y}},
                                                                         {{(float)dyn.size, (float)dyn.size}})
                                      .with_colors(rerun::Color(0, 255, 0)));
        }

        // 2. Dynamic obstacles (GREEN boxes) - z=0 for center AND half-size
        for (const auto &dyn : dynamic_obstacles) {
            std::string name = "obstacles/dynamic_" + std::to_string(dyn.id);
            rec->log_static(name, rerun::Boxes3D::from_centers_and_half_sizes(
                                      {{(float)dyn.x, (float)dyn.y, 0.0f}},
                                      {{(float)(dyn.size / 2.0f), (float)(dyn.size / 2.0f), 0.0f}})
                                      .with_colors(rerun::Color(0, 255, 0)));
        }

        // Build WorldConstraints with obstacle predictions
        drivekit::WorldConstraints world_constraints;

        // Add dynamic obstacles with predictions
        for (const auto &dyn : dynamic_obstacles) {
            drivekit::Obstacle obs;
            obs.id = dyn.id;
            obs.radius = dyn.size / 2.0;

            drivekit::Obstacle::GaussianMode mode;
            mode.weight = 1.0;
            for (size_t t = 0; t <= mca_config.horizon_steps; ++t) {
                double pred_time = t * mca_config.dt;
                mode.mean_x.push_back(dyn.x + dyn.vx * pred_time);
                mode.mean_y.push_back(dyn.y + dyn.vy * pred_time);
                mode.std_x.push_back(0.3);
                mode.std_y.push_back(0.3);
            }
            obs.modes.push_back(mode);
            world_constraints.obstacles.push_back(obs);
        }

        // Add static obstacles (zero velocity prediction)
        for (const auto &stat : static_obstacles) {
            drivekit::Obstacle obs;
            obs.id = 1000 + stat.id; // Offset ID to avoid collision
            obs.radius = stat.size / 2.0;

            drivekit::Obstacle::GaussianMode mode;
            mode.weight = 1.0;
            for (size_t t = 0; t <= mca_config.horizon_steps; ++t) {
                mode.mean_x.push_back(stat.x);
                mode.mean_y.push_back(stat.y);
                mode.std_x.push_back(0.1); // Less uncertainty for static
                mode.std_y.push_back(0.1);
            }
            obs.modes.push_back(mode);
            world_constraints.obstacles.push_back(obs);
        }

        // Visualize predicted trajectories (light lines)
        for (const auto &obs : world_constraints.obstacles) {
            std::string name = "predictions/pred_" + std::to_string(obs.id);

            if (!obs.modes.empty()) {
                const auto &mode = obs.modes[0];
                std::vector<rerun::Position3D> traj_points;

                for (size_t t = 0; t < mode.mean_x.size(); ++t) {
                    traj_points.push_back({(float)mode.mean_x[t], (float)mode.mean_y[t], 0.0f});
                }

                if (!traj_points.empty()) {
                    rec->log_static(name, rerun::LineStrips3D(rerun::components::LineStrip3D(traj_points))
                                              .with_colors(rerun::Color(255, 255, 0))
                                              .with_radii(0.02f));
                }
            }
        }

        if (iteration % 50 == 0) {
            std::cout << "t=" << current_time << "s, pos=(" << robot_state.pose.point.x << ", "
                      << robot_state.pose.point.y << ")" << std::endl;
        }

        // Compute control with world constraints
        auto cmd = mca_controller->compute_control(robot_state, goal, constraints, dt, &world_constraints);

        if (cmd.valid) {
            robot_state.velocity.linear = cmd.linear_velocity * constraints.max_linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity * constraints.max_angular_velocity;

            robot_state.pose.point.x += robot_state.velocity.linear * std::cos(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.point.y += robot_state.velocity.linear * std::sin(robot_state.pose.angle.yaw) * dt;
            robot_state.pose.angle.yaw += robot_state.velocity.angular * dt;

            current_time += dt;

            // Visualize robot
            drivekit::visualize::show_robot_state(rec, robot_state, "robot", rerun::Color(0, 191, 255));

            // Visualize robot's planned path
            const auto &pred_traj = mca_controller->get_predicted_trajectory();
            if (pred_traj.size() > 1) {
                std::vector<rerun::Position3D> pred_points;
                for (const auto &p : pred_traj) {
                    pred_points.push_back({(float)p.x, (float)p.y, 0.0f});
                }
                rec->log_static("robot/plan", rerun::LineStrips3D(rerun::components::LineStrip3D(pred_points))
                                                  .with_colors(rerun::Color(0, 255, 255))
                                                  .with_radii(0.04f));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            std::cerr << "Control failed: " << cmd.status_message << std::endl;
            break;
        }

        auto status = mca_controller->get_status();
        if (status.goal_reached) {
            std::cout << "\n✓ Goal reached!\n";
            break;
        }
    }

    std::cout << "\n=== Complete ===" << std::endl;
    std::cout << "Legend:" << std::endl;
    std::cout << "  RED boxes = static obstacles (don't move)" << std::endl;
    std::cout << "  GREEN boxes = dynamic obstacles (moving)" << std::endl;
    std::cout << "  YELLOW lines = predicted trajectories" << std::endl;
    std::cout << "  CYAN line = robot's plan" << std::endl;
    std::cout << "  BLUE = robot" << std::endl;

    return 0;
}
