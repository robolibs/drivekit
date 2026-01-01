#pragma once

#include "datapod/spatial.hpp"
#include "drivekit/controller.hpp"
#include "drivekit/path/lqr.hpp"
#include "drivekit/path/pure_pursuit.hpp"
#include "drivekit/path/stanley.hpp"
#include "drivekit/point/carrot.hpp"
#include "drivekit/point/pid.hpp"
#include "drivekit/pred/mca.hpp"
#include "drivekit/pred/mpc.hpp"
#include "drivekit/pred/mppi.hpp"
#include "drivekit/pred/soc.hpp"
#include "drivekit/types.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#ifdef HAS_RERUN
#include <rerun.hpp>
#else
// Forward declarations when rerun is not available
namespace rerun {
    class RecordingStream;
}
#endif

namespace drivekit {

    /// Navigation goal for point-to-point navigation.
    struct NavigationGoal {
        datapod::Point target;
        float tolerance = 1.0f; // meters
        float max_speed = 1.0f; // m/s

        inline NavigationGoal(datapod::Point t, float tol = 1.0f, float speed = 1.0f)
            : target(t), tolerance(tol), max_speed(speed) {}
    };

    /// Path goal for waypoint-based navigation.
    struct PathGoal {
        std::vector<datapod::Point> drivekits;
        float tolerance = 1.0f; // meters
        float max_speed = 1.0f; // m/s
        bool loop = false;      // whether to loop back to start

        inline PathGoal(std::vector<datapod::Point> wp, float tol = 1.0f, float speed = 1.0f, bool l = false)
            : drivekits(std::move(wp)), tolerance(tol), max_speed(speed), loop(l) {}
    };

    /// Controller types for tracking.
    enum class TrackerType { PID, PURE_PURSUIT, STANLEY, CARROT, LQR, MPC, MPC_TRAILER, MPPI, SOC, MCA };

    /// High-level path/point tracking controller.
    /// Wraps various low-level controllers and provides a unified interface.
    class Tracker {
      private:
        TrackerType controller_type;
        std::unique_ptr<Controller> controller;

        // Current navigation state
        std::optional<NavigationGoal> current_goal;
        std::optional<PathGoal> current_path;
        size_t current_drivekit_index = 0;
        bool goal_reached = false;
        bool path_completed = false;

        // Robot constraints (set during initialization)
        RobotConstraints constraints_;

#ifdef HAS_RERUN
        // Recording stream for visualization
        std::shared_ptr<rerun::RecordingStream> rec;
#endif

        // Entity prefix for visualization (e.g., "robot_0")
        std::string entity_prefix = "tracking";

        // Current robot state (updated in tick)
        mutable RobotState current_state_;

        // Track previous turn_first state to detect changes
        mutable bool prev_turn_first_ = false;

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
        inline Tracker(TrackerType type = TrackerType::PID) : controller_type(type) {}
        ~Tracker() = default;

        /// Initialize with robot constraints and optional recording stream.
        inline void init(const RobotConstraints &robot_constraints,
                         std::shared_ptr<rerun::RecordingStream> recording_stream = nullptr,
                         const std::string &prefix = "tracking") {
            constraints_ = robot_constraints;
#ifdef HAS_RERUN
            rec = recording_stream;
            entity_prefix = prefix;
#else
            (void)recording_stream;
            (void)prefix;
#endif
            create_controller();
        }

        /// Set a single point goal.
        inline void set_goal(const NavigationGoal &goal) {
            std::cout << "Tracker: Setting goal to (" << goal.target.x << ", " << goal.target.y << ") with tolerance "
                      << goal.tolerance << std::endl;
            current_goal = goal;
            current_path.reset();
            goal_reached = false;
            path_completed = false;
        }

        /// Set a path goal with multiple waypoints.
        inline void set_path(const PathGoal &path) {
            current_path = path;
            current_goal.reset();
            current_drivekit_index = 0;
            goal_reached = false;
            path_completed = false;

            // Convert PathGoal to drivekit::Path and set it in the controller
            Path drivekit_path;
            for (const auto &drivekit : path.drivekits) {
                Pose wp;
                wp.point = drivekit;
                wp.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);
                drivekit_path.drivekits.push_back(wp);
            }
            drivekit_path.is_closed = path.loop;

            if (controller) {
                controller->set_path(drivekit_path);
            }
        }

        /// Clear the current goal.
        inline void clear_goal() {
            current_goal.reset();
            goal_reached = false;
        }

        /// Clear the current path.
        inline void clear_path() {
            current_path.reset();
            current_drivekit_index = 0;
            path_completed = false;

            Path empty_path;
            if (controller) {
                controller->set_path(empty_path);
            }
        }

        /// Main control loop - returns velocity command.
        /// @param current_state Current robot state
        /// @param dt Time step in seconds
        /// @param dynamic_constraints Optional runtime constraints (obstacles, speed limits)
        inline VelocityCommand tick(const RobotState &current_state, float dt,
                                    const WorldConstraints *dynamic_constraints = nullptr) {
            current_state_ = current_state;
            VelocityCommand cmd;
            cmd.valid = false;

            // If movement is not allowed, return zero velocity command
            if (!current_state.allow_move) {
                cmd.valid = true;
                cmd.linear_velocity = 0.0;
                cmd.angular_velocity = 0.0;
                cmd.status_message = "Movement disabled";
                return cmd;
            }

            // Check if we have a valid target
            if (!current_goal.has_value() && !current_path.has_value()) {
                return cmd;
            }

            // Update drivekit progress for path following
            // Only for point-based controllers (PID, CARROT) that don't manage their own path index
            if (current_path.has_value() &&
                (controller_type == TrackerType::PID || controller_type == TrackerType::CARROT)) {
                update_drivekit_progress(current_state);
            }

            // If the path has been completed, output a zero-velocity command
            if (current_path.has_value() && path_completed) {
                cmd.valid = true;
                cmd.status_message = "Path completed";
                return cmd;
            }

            // Get current target goal
            Goal goal;
            if (controller_type == TrackerType::PID || controller_type == TrackerType::CARROT) {
                goal = get_current_tracking_goal();
            } else if (controller_type == TrackerType::PURE_PURSUIT || controller_type == TrackerType::STANLEY ||
                       controller_type == TrackerType::LQR || controller_type == TrackerType::MPC ||
                       controller_type == TrackerType::MPC_TRAILER || controller_type == TrackerType::MPPI ||
                       controller_type == TrackerType::SOC || controller_type == TrackerType::MCA) {
                if (current_path.has_value() && !current_path->drivekits.empty()) {
                    goal.target_pose =
                        datapod::Pose{current_path->drivekits.back(), datapod::Quaternion::from_euler(0.0, 0.0, 0.0)};
                    goal.tolerance_position = current_path->tolerance;
                }
            }

            // Compute control command
            if (controller) {
                auto cfg = controller->get_config();
                cfg.allow_reverse = current_state.allow_reverse;
                controller->set_config(cfg);

                cmd = controller->compute_control(current_state, goal, constraints_, dt, dynamic_constraints);

                // For Ackermann steering, enforce minimum velocity when angular velocity is non-zero
                if (cmd.valid && constraints_.steering_type == SteeringType::ACKERMANN) {
                    const double min_velocity = 0.3;
                    if (std::abs(cmd.angular_velocity) > 0.01) {
                        if (std::abs(cmd.linear_velocity) < min_velocity) {
                            cmd.linear_velocity = (cmd.linear_velocity >= 0) ? min_velocity : -min_velocity;
                        }
                    }
                }

                // Sync path index from controller for visualization
                if (controller_type == TrackerType::PURE_PURSUIT || controller_type == TrackerType::STANLEY ||
                    controller_type == TrackerType::LQR || controller_type == TrackerType::MPC ||
                    controller_type == TrackerType::MPC_TRAILER || controller_type == TrackerType::MPPI ||
                    controller_type == TrackerType::SOC || controller_type == TrackerType::MCA) {
                    current_drivekit_index = controller->get_path_index();

                    if (controller->get_status().goal_reached) {
                        path_completed = true;
                    }
                }
            }

            update_goal_status(current_state);

            return cmd;
        }

        /// Check if the goal has been reached.
        inline bool is_goal_reached() const { return goal_reached; }

        /// Check if the path has been completed.
        inline bool is_path_completed() const { return path_completed; }

        /// Set the controller type.
        inline void set_controller_type(TrackerType type) {
            controller_type = type;
            create_controller();
        }

        /// Set controller parameters.
        inline void set_controller_params(const ControllerParams &new_params) { params = new_params; }

        /// Get controller parameters.
        inline const ControllerParams &get_controller_params() const { return params; }

        /// Get distance to goal.
        inline float get_distance_to_goal() const { return std::numeric_limits<float>::infinity(); }

        /// Get distance to current waypoint.
        inline float get_distance_to_current_drivekit() const { return std::numeric_limits<float>::infinity(); }

        /// Get current target point.
        inline datapod::Point get_current_target() const {
            if (current_goal.has_value()) {
                return current_goal->target;
            } else if (current_path.has_value() && current_drivekit_index < current_path->drivekits.size()) {
                return current_path->drivekits[current_drivekit_index];
            }
            return datapod::Point{0, 0};
        }

        /// Emergency stop - clears all goals and paths.
        inline void emergency_stop() {
            clear_goal();
            clear_path();
        }

        /// Smoothen path by adding interpolated points between waypoints.
        inline void smoothen(float interval_cm = 100.0f) {
            if (!current_path.has_value() || current_path->drivekits.size() < 2) {
                return;
            }

            float interval_m = interval_cm / 100.0f;
            std::vector<datapod::Point> smoothed_drivekits;

            smoothed_drivekits.push_back(current_path->drivekits[0]);

            for (size_t i = 0; i < current_path->drivekits.size() - 1; ++i) {
                const auto &start = current_path->drivekits[i];
                const auto &end = current_path->drivekits[i + 1];

                float dx = end.x - start.x;
                float dy = end.y - start.y;
                float distance = std::sqrt(dx * dx + dy * dy);

                int num_segments = static_cast<int>(std::ceil(distance / interval_m));

                for (int j = 1; j < num_segments; ++j) {
                    float t = static_cast<float>(j) / num_segments;
                    datapod::Point interpolated;
                    interpolated.x = start.x + t * dx;
                    interpolated.y = start.y + t * dy;
                    smoothed_drivekits.push_back(interpolated);
                }

                smoothed_drivekits.push_back(end);
            }

            current_path->drivekits = smoothed_drivekits;
            current_drivekit_index = 0;

            std::cout << "Path smoothened: " << smoothed_drivekits.size() << " drivekits (interval: " << interval_cm
                      << "cm)" << std::endl;

            Path drivekit_path;
            for (const auto &drivekit : current_path->drivekits) {
                Pose wp;
                wp.point = drivekit;
                wp.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);
                drivekit_path.drivekits.push_back(wp);
            }
            drivekit_path.is_closed = current_path->loop;

            if (controller) {
                controller->set_path(drivekit_path);
            }
        }

        /// Get direct access to the underlying controller.
        inline Controller *get_controller() { return controller.get(); }
        inline const Controller *get_controller() const { return controller.get(); }

        /// Visualization callback - call after tick() to update visualization.
        inline void tock() const {
#ifdef HAS_RERUN
            if (!rec) {
                return;
            }

            // Visualize current path
            if (current_path.has_value() && !current_path->drivekits.empty()) {
                std::vector<std::array<float, 3>> path_points;
                for (const auto &drivekit : current_path->drivekits) {
                    path_points.push_back({static_cast<float>(drivekit.x), static_cast<float>(drivekit.y), 0});
                }

                if (path_points.size() >= 2) {
                    auto path_line = rerun::components::LineStrip3D(path_points);
                    rec->log_static(
                        entity_prefix + "/planned_path",
                        rerun::LineStrips3D(path_line).with_colors({{0, 255, 0, 128}}).with_radii({{0.05f}}));
                }

                std::vector<rerun::components::Position3D> drivekit_positions;
                for (const auto &drivekit : current_path->drivekits) {
                    drivekit_positions.push_back({static_cast<float>(drivekit.x), static_cast<float>(drivekit.y), 0});
                }

                if (!drivekit_positions.empty()) {
                    rec->log_static(
                        entity_prefix + "/drivekits",
                        rerun::Points3D(drivekit_positions).with_colors({{0, 255, 0}}).with_radii({{0.07f}}));
                }

                if (current_drivekit_index < current_path->drivekits.size()) {
                    const auto &current_target = current_path->drivekits[current_drivekit_index];
                    rec->log_static(entity_prefix + "/current_target",
                                    rerun::Points3D({{static_cast<float>(current_target.x),
                                                      static_cast<float>(current_target.y), 0.0f}})
                                        .with_colors({{255, 255, 0, 128}})
                                        .with_radii({{0.12f}}));
                }
            }

            // Visualize current goal
            if (current_goal.has_value()) {
                const auto &target = current_goal->target;

                rec->log_static(entity_prefix + "/goal",
                                rerun::Points3D({{static_cast<float>(target.x), static_cast<float>(target.y), 0.3f}})
                                    .with_colors({{255, 0, 0}})
                                    .with_radii({{0.125f}}));

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
                    rec->log_static(
                        entity_prefix + "/goal_tolerance",
                        rerun::LineStrips3D(tolerance_line).with_colors({{255, 0, 0, 128}}).with_radii({{0.02f}}));
                }
            }

            // Draw line from robot to current target waypoint
            if (current_path.has_value() && current_drivekit_index < current_path->drivekits.size()) {
                const auto &target_drivekit = current_path->drivekits[current_drivekit_index];
                std::vector<std::array<float, 3>> target_line = {
                    {static_cast<float>(current_state_.pose.point.x), static_cast<float>(current_state_.pose.point.y),
                     0.2f},
                    {static_cast<float>(target_drivekit.x), static_cast<float>(target_drivekit.y), 0}};

                auto target_strip = rerun::components::LineStrip3D(target_line);
                rec->log_static(
                    entity_prefix + "/target_line",
                    rerun::LineStrips3D(target_strip).with_colors({{255, 255, 0, 128}}).with_radii({{0.02f}}));
            }
#else
            (void)0;
#endif
        }

      private:
        /// Get the current tracking goal based on navigation state.
        inline Goal get_current_tracking_goal() const {
            Goal goal;

            if (current_goal.has_value()) {
                goal.target_pose = datapod::Pose{current_goal->target, datapod::Quaternion::from_euler(0.0, 0.0, 0.0)};
                goal.tolerance_position = current_goal->tolerance;
            } else if (current_path.has_value() && current_drivekit_index < current_path->drivekits.size()) {
                goal.target_pose = datapod::Pose{current_path->drivekits[current_drivekit_index],
                                                 datapod::Quaternion::from_euler(0.0, 0.0, 0.0)};
                goal.tolerance_position = current_path->tolerance;
            }

            return goal;
        }

        /// Update waypoint progress for point-based controllers.
        inline void update_drivekit_progress(const RobotState &current_state) {
            if (!current_path.has_value() || current_drivekit_index >= current_path->drivekits.size()) {
                return;
            }

            const datapod::Point &robot_pos = current_state.pose.point;
            const datapod::Point &current_target = current_path->drivekits[current_drivekit_index];
            float distance =
                std::sqrt(std::pow(current_target.x - robot_pos.x, 2) + std::pow(current_target.y - robot_pos.y, 2));

            static int drivekit_debug_count = 0;
            if (drivekit_debug_count % 50 == 0) {
                std::cout << "WAYPOINT DEBUG: Index=" << current_drivekit_index << ", Robot(" << robot_pos.x << ","
                          << robot_pos.y << ")" << ", Target(" << current_target.x << "," << current_target.y << ")"
                          << ", Distance=" << distance << ", Tolerance=" << current_path->tolerance << std::endl;
            }
            drivekit_debug_count++;

            if (distance <= current_path->tolerance) {
                std::cout << "WAYPOINT REACHED! Moving to next drivekit. Index was " << current_drivekit_index;
                current_drivekit_index++;

                if (current_drivekit_index >= current_path->drivekits.size()) {
                    if (current_path->loop && !current_path->drivekits.empty()) {
                        current_drivekit_index = 0;
                    } else {
                        path_completed = true;
                    }
                }
            }
        }

        /// Create the appropriate controller based on controller_type.
        inline void create_controller() {
            ControllerConfig config;

            switch (controller_type) {
            case TrackerType::PID:
                config.kp_linear = params.linear_kp;
                config.ki_linear = params.linear_ki;
                config.kd_linear = params.linear_kd;
                config.kp_angular = params.angular_kp;
                config.ki_angular = params.angular_ki;
                config.kd_angular = params.angular_kd;
                controller = std::make_unique<point::PIDFollower>();
                controller->set_config(config);
                break;

            case TrackerType::PURE_PURSUIT:
                config.lookahead_distance = params.lookahead_distance;
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<path::PurePursuitFollower>();
                controller->set_config(config);
                break;

            case TrackerType::STANLEY:
                config.k_cross_track = params.cross_track_gain;
                config.k_heading = params.softening_gain;
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                config.allow_reverse = false;
                controller = std::make_unique<path::StanleyFollower>();
                controller->set_config(config);
                break;

            case TrackerType::CARROT:
                config.lookahead_distance = params.carrot_distance;
                controller = std::make_unique<point::CarrotFollower>();
                controller->set_config(config);
                break;

            case TrackerType::LQR:
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<path::LQRFollower>();
                controller->set_config(config);
                break;

            case TrackerType::MPC:
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<pred::MPCFollower>();
                controller->set_config(config);
                break;

            case TrackerType::MPC_TRAILER:
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<pred::MPCFollower>();
                controller->set_config(config);
                break;

            case TrackerType::MPPI: {
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<pred::MPPIFollower>();
                controller->set_config(config);
                break;
            }

            case TrackerType::SOC: {
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<pred::SOCFollower>();
                controller->set_config(config);
                break;
            }

            case TrackerType::MCA: {
                config.goal_tolerance = 0.5f;
                config.angular_tolerance = 0.1f;
                controller = std::make_unique<pred::MCAFollower>();
                controller->set_config(config);
                break;
            }
            }
        }

        /// Update goal/path completion status.
        inline void update_goal_status(const RobotState &current_state) {
            if (current_goal.has_value()) {
                const datapod::Point &robot_pos = current_state.pose.point;
                const datapod::Point &target = current_goal->target;
                float distance = std::sqrt(std::pow(target.x - robot_pos.x, 2) + std::pow(target.y - robot_pos.y, 2));
                goal_reached = distance <= current_goal->tolerance;
            }

            if (current_path.has_value() && current_drivekit_index >= current_path->drivekits.size()) {
                path_completed = true;
            }
        }
    };

} // namespace drivekit
