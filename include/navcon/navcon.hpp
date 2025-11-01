#pragma once

#include "concord/concord.hpp"
#include "navcon/controller.hpp"
#include "navcon/followers/carrot.hpp"
#include "navcon/followers/pid.hpp"
#include "navcon/followers/pure_pursuit.hpp"
#include "navcon/path_controller.hpp"
#include "rerun.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace navcon {

    // Goal types for navigation
    struct NavigationGoal {
        concord::Point target;
        float tolerance = 1.0f; // meters
        float max_speed = 1.0f; // m/s

        NavigationGoal(concord::Point t, float tol = 1.0f, float speed = 1.0f)
            : target(t), tolerance(tol), max_speed(speed) {}
    };

    struct PathGoal {
        std::vector<concord::Point> waypoints;
        float tolerance = 1.0f; // meters
        float max_speed = 1.0f; // m/s
        bool loop = false;      // whether to loop back to start

        PathGoal(std::vector<concord::Point> wp, float tol = 1.0f, float speed = 1.0f, bool l = false)
            : waypoints(wp), tolerance(tol), max_speed(speed), loop(l) {}
    };

    // Controller types
    enum class NavconControllerType { PID, PURE_PURSUIT, STANLEY, CARROT, PATH_CONTROLLER };

    // ============================================================================
    // Navcon - High-level navigation controller using navcon library
    // ============================================================================
    class Navcon {
      private:
        NavconControllerType controller_type;
        std::unique_ptr<Controller> controller;          // Simple controller or PathController
        std::unique_ptr<PathController> path_controller; // Advanced path controller

        // Current navigation state
        std::optional<NavigationGoal> current_goal;
        std::optional<PathGoal> current_path;
        size_t current_waypoint_index = 0;
        bool goal_reached = false;
        bool path_completed = false;

        // Robot constraints (set during initialization)
        RobotConstraints constraints_;

        // Minimum turning radius (meters)
        float min_turning_radius_;

        // Recording stream for visualization
        std::shared_ptr<rerun::RecordingStream> rec;

        // Current robot state (updated in tick)
        mutable RobotState current_state_;

        // Controller parameters
        struct ControllerParams {
            // PID parameters - increased gains for proper movement
            float linear_kp = 2.0f, linear_ki = 0.0f, linear_kd = 0.1f;
            float angular_kp = 1.5f, angular_ki = 0.0f, angular_kd = 0.1f;

            // Pure pursuit parameters
            float lookahead_distance = 2.5f; // Shorter distance to stay closer to path
            float lookahead_gain = 0.5f;

            // Stanley parameters
            float cross_track_gain = 2.5f; // Increased for better cross-track correction
            float softening_gain = 1.5f;   // Increased for better heading response

            // Carrot parameters
            float carrot_distance = 1.0f;

            // Path controller parameters
            float sharp_turn_threshold = 60.0f;   // degrees
            float u_turn_threshold = 120.0f;      // degrees
            float path_lookahead_distance = 5.0f; // meters
        } params;

      public:
        Navcon(NavconControllerType type = NavconControllerType::PID, float min_turning_radius = 1.0f)
            : controller_type(type), min_turning_radius_(min_turning_radius) {}
        ~Navcon() = default;

        // Initialize with robot constraints and recording stream
        void init(const RobotConstraints &robot_constraints, std::shared_ptr<rerun::RecordingStream> recording_stream) {
            constraints_ = robot_constraints;
            constraints_.min_turning_radius = min_turning_radius_;
            rec = recording_stream;
            create_controller();
        }

        // Goal management
        void set_goal(const NavigationGoal &goal) {
            std::cout << "Navcon: Setting goal to (" << goal.target.x << ", " << goal.target.y << ") with tolerance "
                      << goal.tolerance << std::endl;
            current_goal = goal;
            current_path.reset();
            goal_reached = false;
            path_completed = false;
        }

        void set_path(const PathGoal &path) {
            current_path = path;
            current_goal.reset();
            current_waypoint_index = 0;
            goal_reached = false;
            path_completed = false;

            // Filter waypoints based on minimum turning radius
            std::vector<concord::Point> filtered_waypoints = filter_waypoints_by_turning_radius(path.waypoints);

            // Update current_path with filtered waypoints
            current_path->waypoints = filtered_waypoints;

            // Convert PathGoal to navcon::Path and set it in the controller
            Path navcon_path;
            for (const auto &waypoint : filtered_waypoints) {
                Pose wp;
                wp.point = waypoint;
                wp.angle = concord::Euler{0.0f, 0.0f, 0.0f};
                navcon_path.waypoints.push_back(wp);
            }
            navcon_path.is_closed = path.loop;

            if (controller) {
                controller->set_path(navcon_path);
            }
            if (path_controller) {
                path_controller->set_path(navcon_path);
            }
        }

        void clear_goal() {
            current_goal.reset();
            goal_reached = false;
        }

        void clear_path() {
            current_path.reset();
            current_waypoint_index = 0;
            path_completed = false;

            // Clear path in the controllers
            Path empty_path;
            if (controller) {
                controller->set_path(empty_path);
            }
            if (path_controller) {
                path_controller->set_path(empty_path);
            }
        }

        // Navigation control - returns velocity command
        VelocityCommand tick(const RobotState &current_state, float dt) {
            // Store current state for visualization
            current_state_ = current_state;
            VelocityCommand cmd;
            cmd.valid = false;

            // Check if we have a valid target
            if (!current_goal.has_value() && !current_path.has_value()) {
                return cmd;
            }

            // Update waypoint progress for path following
            if (current_path.has_value()) {
                update_waypoint_progress(current_state);
            }

            // Get current target goal
            Goal goal;
            if (controller_type == NavconControllerType::PID || controller_type == NavconControllerType::STANLEY ||
                controller_type == NavconControllerType::CARROT) {
                goal = get_current_navcon_goal(); // Point-based controllers need specific targets
            }
            // Pure Pursuit and Path Controller use the path set with set_path()

            // Compute control command
            if (path_controller) {
                // Use path controller
                cmd = path_controller->compute_control(current_state, goal, constraints_, dt);
            } else if (controller) {
                // Use simple controller
                cmd = controller->compute_control(current_state, goal, constraints_, dt);
            }

            // Update goal status
            update_goal_status(current_state);

            return cmd;
        }

        // Status
        bool is_goal_reached() const { return goal_reached; }
        bool is_path_completed() const { return path_completed; }

        // Controller configuration
        void set_controller_type(NavconControllerType type) {
            controller_type = type;
            create_controller();
        }
        void set_controller_params(const ControllerParams &new_params) { params = new_params; }
        const ControllerParams &get_controller_params() const { return params; }

        // Status information
        float get_distance_to_goal() const {
            // This method needs to be called with a robot state parameter
            // For now, return a placeholder value
            return std::numeric_limits<float>::infinity();
        }

        float get_distance_to_current_waypoint() const {
            // This method needs to be called with a robot state parameter
            // For now, return a placeholder value
            return std::numeric_limits<float>::infinity();
        }

        concord::Point get_current_target() const {
            if (current_goal.has_value()) {
                return current_goal->target;
            } else if (current_path.has_value() && current_waypoint_index < current_path->waypoints.size()) {
                return current_path->waypoints[current_waypoint_index];
            }
            return concord::Point{0, 0};
        }

        // Emergency stop
        void emergency_stop() {
            clear_goal();
            clear_path();
        }

        // Smoothen path by adding interpolated points between waypoints
        void smoothen(float interval_cm = 100.0f) {
            if (!current_path.has_value() || current_path->waypoints.size() < 2) {
                return;
            }

            float interval_m = interval_cm / 100.0f; // Convert cm to meters
            std::vector<concord::Point> smoothed_waypoints;

            // Always keep the first waypoint
            smoothed_waypoints.push_back(current_path->waypoints[0]);

            for (size_t i = 0; i < current_path->waypoints.size() - 1; ++i) {
                const auto &start = current_path->waypoints[i];
                const auto &end = current_path->waypoints[i + 1];

                // Calculate distance between waypoints
                float dx = end.x - start.x;
                float dy = end.y - start.y;
                float distance = std::sqrt(dx * dx + dy * dy);

                // Calculate number of segments needed
                int num_segments = static_cast<int>(std::ceil(distance / interval_m));

                // Add interpolated points (skip the first one as it's already added)
                for (int j = 1; j < num_segments; ++j) {
                    float t = static_cast<float>(j) / num_segments;
                    concord::Point interpolated;
                    interpolated.x = start.x + t * dx;
                    interpolated.y = start.y + t * dy;
                    smoothed_waypoints.push_back(interpolated);
                }

                // Add the end waypoint
                smoothed_waypoints.push_back(end);
            }

            // Update the current path with smoothed waypoints
            current_path->waypoints = smoothed_waypoints;
            current_waypoint_index = 0; // Reset to start

            std::cout << "Path smoothened: " << smoothed_waypoints.size() << " waypoints (interval: " << interval_cm
                      << "cm)" << std::endl;

            // Update the path in controllers
            Path navcon_path;
            for (const auto &waypoint : current_path->waypoints) {
                Pose wp;
                wp.point = waypoint;
                wp.angle = concord::Euler{0.0f, 0.0f, 0.0f};
                navcon_path.waypoints.push_back(wp);
            }
            navcon_path.is_closed = current_path->loop;

            if (controller) {
                controller->set_path(navcon_path);
            }
            if (path_controller) {
                path_controller->set_path(navcon_path);
            }
        }

        // Direct access to path controller for advanced usage
        PathController *get_path_controller() { return path_controller.get(); }
        const PathController *get_path_controller() const { return path_controller.get(); }

        // Direct access to simple controller
        Controller *get_controller() { return controller.get(); }
        const Controller *get_controller() const { return controller.get(); }

        // Visualization
        void tock() const {
            if (!rec) {
                return;
            }

            // Visualize current path
            if (current_path.has_value() && !current_path->waypoints.empty()) {
                // Convert waypoints to 3D coordinates for visualization
                std::vector<std::array<float, 3>> path_points;
                for (const auto &waypoint : current_path->waypoints) {
                    path_points.push_back({static_cast<float>(waypoint.x), static_cast<float>(waypoint.y), 0.3f});
                }

                // Draw the planned path as a green line
                if (path_points.size() >= 2) {
                    auto path_line = rerun::components::LineStrip3D(path_points);
                    rec->log_static("navigation/planned_path", rerun::LineStrips3D(path_line)
                                                                   .with_colors({{0, 255, 0}}) // Green for planned path
                                                                   .with_radii({{0.0375f}}));
                }

                // Visualize individual waypoints as green spheres
                std::vector<rerun::components::Position3D> waypoint_positions;
                for (const auto &waypoint : current_path->waypoints) {
                    waypoint_positions.push_back(
                        {static_cast<float>(waypoint.x), static_cast<float>(waypoint.y), 0.3f});
                }

                if (!waypoint_positions.empty()) {
                    rec->log_static("navigation/waypoints", rerun::Points3D(waypoint_positions)
                                                                .with_colors({{0, 255, 0}}) // Green waypoints
                                                                .with_radii({{0.1f}}));
                }

                // Highlight current target waypoint in yellow
                if (current_waypoint_index < current_path->waypoints.size()) {
                    const auto &current_target = current_path->waypoints[current_waypoint_index];
                    rec->log_static("navigation/current_target",
                                    rerun::Points3D({{static_cast<float>(current_target.x),
                                                      static_cast<float>(current_target.y), 0.4f}})
                                        .with_colors({{255, 255, 0}}) // Yellow for current target
                                        .with_radii({{0.15f}}));
                }
            }

            // Visualize current goal
            if (current_goal.has_value()) {
                const auto &target = current_goal->target;

                // Visualize goal point as a red sphere
                rec->log_static("navigation/goal",
                                rerun::Points3D({{static_cast<float>(target.x), static_cast<float>(target.y), 0.3f}})
                                    .with_colors({{255, 0, 0}}) // Red for goal
                                    .with_radii({{0.125f}}));

                // Visualize tolerance circle around goal
                std::vector<std::array<float, 3>> tolerance_circle;
                int segments = 32;
                for (int i = 0; i <= segments; ++i) {
                    float angle = 2.0f * M_PI * i / segments;
                    float x = target.x + current_goal->tolerance * std::cos(angle);
                    float y = target.y + current_goal->tolerance * std::sin(angle);
                    tolerance_circle.push_back({x, y, 0.1f});
                }

                if (!tolerance_circle.empty()) {
                    auto tolerance_line = rerun::components::LineStrip3D(tolerance_circle);
                    rec->log_static("navigation/goal_tolerance",
                                    rerun::LineStrips3D(tolerance_line)
                                        .with_colors({{255, 0, 0, 128}}) // Semi-transparent red
                                        .with_radii({{0.02f}}));
                }
            }

            // Draw line from robot to current target (orange)
            auto target = get_current_target();
            if (target.x != 0 || target.y != 0) {
                std::vector<std::array<float, 3>> direction_line = {
                    {static_cast<float>(current_state_.pose.point.x), static_cast<float>(current_state_.pose.point.y),
                     0.2f},
                    {static_cast<float>(target.x), static_cast<float>(target.y), 0.2f}};

                auto direction_strip = rerun::components::LineStrip3D(direction_line);
                rec->log_static("navigation/direction", rerun::LineStrips3D(direction_strip)
                                                            .with_colors({{255, 165, 0}}) // Orange for direction
                                                            .with_radii({{0.025f}}));
            }
        }

      private:
        // Internal helper methods
        std::vector<concord::Point> filter_waypoints_by_turning_radius(const std::vector<concord::Point> &waypoints) {
            if (waypoints.size() < 2) {
                return waypoints;
            }

            std::vector<concord::Point> filtered;
            filtered.push_back(waypoints[0]);

            float min_distance_threshold = 3.0f * min_turning_radius_;
            size_t skipped_count = 0;

            for (size_t i = 1; i < waypoints.size() - 1; ++i) {
                const auto &prev = filtered.back();
                const auto &current = waypoints[i];

                float dx = current.x - prev.x;
                float dy = current.y - prev.y;
                float distance = std::sqrt(dx * dx + dy * dy);

                if (distance >= min_distance_threshold) {
                    filtered.push_back(current);
                } else {
                    skipped_count++;
                    std::cout << "Navcon: Skipping waypoint " << i << " at (" << current.x << ", " << current.y
                              << ") - too close (" << distance << "m < " << min_distance_threshold
                              << "m threshold based on turning radius " << min_turning_radius_ << "m)" << std::endl;
                }
            }

            filtered.push_back(waypoints.back());

            if (skipped_count > 0) {
                std::cout << "Navcon: Filtered path from " << waypoints.size() << " to " << filtered.size()
                          << " waypoints (skipped " << skipped_count << " waypoints based on min turning radius "
                          << min_turning_radius_ << "m)" << std::endl;
            }

            return filtered;
        }

        Goal get_current_navcon_goal() const {
            Goal goal;

            if (current_goal.has_value()) {
                goal.target_pose = concord::Pose{current_goal->target, concord::Euler{0.0f, 0.0f, 0.0f}};
                goal.tolerance_position = current_goal->tolerance;
            } else if (current_path.has_value() && current_waypoint_index < current_path->waypoints.size()) {
                goal.target_pose =
                    concord::Pose{current_path->waypoints[current_waypoint_index], concord::Euler{0.0f, 0.0f, 0.0f}};
                goal.tolerance_position = current_path->tolerance;
            }

            return goal;
        }

        void update_waypoint_progress(const RobotState &current_state) {
            if (!current_path.has_value() || current_waypoint_index >= current_path->waypoints.size()) {
                return;
            }

            // Calculate distance directly using current state
            const concord::Point &robot_pos = current_state.pose.point;
            const concord::Point &current_target = current_path->waypoints[current_waypoint_index];
            float distance =
                std::sqrt(std::pow(current_target.x - robot_pos.x, 2) + std::pow(current_target.y - robot_pos.y, 2));

            // Debug waypoint progression
            static int waypoint_debug_count = 0;
            if (waypoint_debug_count % 50 == 0) {
                std::cout << "WAYPOINT DEBUG: Index=" << current_waypoint_index << ", Robot(" << robot_pos.x << ","
                          << robot_pos.y << ")" << ", Target(" << current_target.x << "," << current_target.y << ")"
                          << ", Distance=" << distance << ", Tolerance=" << current_path->tolerance << std::endl;
            }
            waypoint_debug_count++;

            if (distance <= current_path->tolerance) {
                std::cout << "WAYPOINT REACHED! Moving to next waypoint. Index was " << current_waypoint_index;
                current_waypoint_index++;
                std::cout << ", now " << current_waypoint_index << std::endl;

                // Check if we've completed the path
                if (current_waypoint_index >= current_path->waypoints.size()) {
                    if (current_path->loop && !current_path->waypoints.empty()) {
                        current_waypoint_index = 0; // Loop back to start
                    } else {
                        path_completed = true;
                    }
                }
            }
        }

        void create_controller() {
            ControllerConfig config;

            switch (controller_type) {
            case NavconControllerType::PID:
                std::cout << "Creating PID controller..." << std::endl;
                controller = std::make_unique<followers::PIDFollower>(min_turning_radius_);
                config.kp_linear = params.linear_kp;
                config.ki_linear = params.linear_ki;
                config.kd_linear = params.linear_kd;
                config.kp_angular = params.angular_kp;
                config.ki_angular = params.angular_ki;
                config.kd_angular = params.angular_kd;
                controller->set_config(config);
                path_controller.reset();
                std::cout << "PID controller created: success" << std::endl;
                break;

            case NavconControllerType::PURE_PURSUIT:
                std::cout << "Creating Pure Pursuit controller..." << std::endl;
                controller = std::make_unique<followers::PurePursuitFollower>(min_turning_radius_);
                config.lookahead_distance = params.lookahead_distance;
                controller->set_config(config);
                path_controller.reset();
                std::cout << "Pure Pursuit controller created: success" << std::endl;
                break;

            case NavconControllerType::STANLEY:
                std::cout << "Stanley controller not implemented yet" << std::endl;
                break;

            case NavconControllerType::CARROT:
                controller = std::make_unique<followers::CarrotFollower>(min_turning_radius_);
                config.lookahead_distance = params.carrot_distance;
                controller->set_config(config);
                path_controller.reset();
                break;

            case NavconControllerType::PATH_CONTROLLER:
                std::cout << "Creating Path controller..." << std::endl;
                path_controller =
                    std::make_unique<PathController>(PathController::FollowerType::PURE_PURSUIT, min_turning_radius_);
                controller.reset();

                // Configure path controller
                path_controller->set_sharp_turn_threshold(params.sharp_turn_threshold);
                path_controller->set_u_turn_threshold(params.u_turn_threshold);
                path_controller->set_lookahead_distance(params.path_lookahead_distance);

                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                config.kp_linear = params.linear_kp;
                config.kp_angular = params.angular_kp;
                path_controller->set_config(config);

                std::cout << "Path controller created: success" << std::endl;
                break;
            }
        }

        void update_goal_status(const RobotState &current_state) {
            // Check goal completion
            if (current_goal.has_value()) {
                const concord::Point &robot_pos = current_state.pose.point;
                const concord::Point &target = current_goal->target;
                float distance = std::sqrt(std::pow(target.x - robot_pos.x, 2) + std::pow(target.y - robot_pos.y, 2));
                goal_reached = distance <= current_goal->tolerance;
            }

            if (current_path.has_value() && current_waypoint_index >= current_path->waypoints.size()) {
                path_completed = true;
            }
        }
    };

} // namespace navcon