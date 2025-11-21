#include "navcon/tracker.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace navcon {

    // NavigationGoal constructor
    NavigationGoal::NavigationGoal(concord::Point t, float tol, float speed)
        : target(t), tolerance(tol), max_speed(speed) {}

    // PathGoal constructor
    PathGoal::PathGoal(std::vector<concord::Point> wp, float tol, float speed, bool l)
        : waypoints(std::move(wp)), tolerance(tol), max_speed(speed), loop(l) {}

    // Tracker constructor
    Tracker::Tracker(TrackerType type) : controller_type(type) {}

    // Initialize with robot constraints and recording stream
    void Tracker::init(const RobotConstraints &robot_constraints,
                       std::shared_ptr<rerun::RecordingStream> recording_stream, const std::string &prefix) {
        constraints_ = robot_constraints;
#ifdef HAS_RERUN
        rec = recording_stream;
        entity_prefix = prefix;
#else
        (void)recording_stream; // Suppress unused parameter warning
        (void)prefix;
#endif
        create_controller();
    }

    // Goal management
    void Tracker::set_goal(const NavigationGoal &goal) {
        std::cout << "Tracker: Setting goal to (" << goal.target.x << ", " << goal.target.y << ") with tolerance "
                  << goal.tolerance << std::endl;
        current_goal = goal;
        current_path.reset();
        goal_reached = false;
        path_completed = false;
    }

    void Tracker::set_path(const PathGoal &path) {
        current_path = path;
        current_goal.reset();
        current_waypoint_index = 0;
        goal_reached = false;
        path_completed = false;

        // Convert PathGoal to navcon::Path and set it in the controller
        Path navcon_path;
        for (const auto &waypoint : path.waypoints) {
            Pose wp;
            wp.point = waypoint;
            wp.angle = concord::Euler{0.0f, 0.0f, 0.0f}; // No specific heading required
            navcon_path.waypoints.push_back(wp);
        }
        navcon_path.is_closed = path.loop; // Set loop behavior

        if (controller) {
            controller->set_path(navcon_path);
        }
    }

    void Tracker::clear_goal() {
        current_goal.reset();
        goal_reached = false;
    }

    void Tracker::clear_path() {
        current_path.reset();
        current_waypoint_index = 0;
        path_completed = false;

        // Clear path in the controllers
        Path empty_path;
        if (controller) {
            controller->set_path(empty_path);
        }
    }

    // Navigation control - returns velocity command
    VelocityCommand Tracker::tick(const RobotState &current_state, float dt,
                                  const WorldConstraints *dynamic_constraints) {
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
        if (controller_type == TrackerType::PID || controller_type == TrackerType::CARROT) {
            goal = get_current_tracking_goal(); // Point-based controllers need specific targets
        } else if (controller_type == TrackerType::PURE_PURSUIT || controller_type == TrackerType::STANLEY ||
                   controller_type == TrackerType::LQR) {
            // Pure Pursuit, Stanley, and LQR use the path, but need goal set to END of path for goal-reached check
            if (current_path.has_value() && !current_path->waypoints.empty()) {
                goal.target_pose = concord::Pose{current_path->waypoints.back(), concord::Euler{0.0f, 0.0f, 0.0f}};
                goal.tolerance_position = current_path->tolerance;
            }
        }

        // Compute control command
        if (controller) {
            // Use simple controller, pass dynamic constraints if available
            cmd = controller->compute_control(current_state, goal, constraints_, dt, dynamic_constraints);
        }

        // Update goal status
        update_goal_status(current_state);

        return cmd;
    }

    // Status
    bool Tracker::is_goal_reached() const { return goal_reached; }

    bool Tracker::is_path_completed() const { return path_completed; }

    // Controller configuration
    void Tracker::set_controller_type(TrackerType type) {
        controller_type = type;
        create_controller();
    }

    void Tracker::set_controller_params(const ControllerParams &new_params) { params = new_params; }

    const Tracker::ControllerParams &Tracker::get_controller_params() const { return params; }

    // Status information
    float Tracker::get_distance_to_goal() const {
        // This method needs to be called with a robot state parameter
        // For now, return a placeholder value
        return std::numeric_limits<float>::infinity();
    }

    float Tracker::get_distance_to_current_waypoint() const {
        // This method needs to be called with a robot state parameter
        // For now, return a placeholder value
        return std::numeric_limits<float>::infinity();
    }

    concord::Point Tracker::get_current_target() const {
        if (current_goal.has_value()) {
            return current_goal->target;
        } else if (current_path.has_value() && current_waypoint_index < current_path->waypoints.size()) {
            return current_path->waypoints[current_waypoint_index];
        }
        return concord::Point{0, 0};
    }

    // Emergency stop
    void Tracker::emergency_stop() {
        clear_goal();
        clear_path();
    }

    // Smoothen path by adding interpolated points between waypoints
    void Tracker::smoothen(float interval_cm) {
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
    }

    // Direct access to controller
    tracking::Controller *Tracker::get_controller() { return controller.get(); }

    const tracking::Controller *Tracker::get_controller() const { return controller.get(); }

    // Visualization
    void Tracker::tock() const {
#ifdef HAS_RERUN
        if (!rec) {
            return;
        }

        // Visualize current path
        if (current_path.has_value() && !current_path->waypoints.empty()) {
            // Convert waypoints to 3D coordinates for visualization
            std::vector<std::array<float, 3>> path_points;
            for (const auto &waypoint : current_path->waypoints) {
                path_points.push_back({static_cast<float>(waypoint.x), static_cast<float>(waypoint.y), 0});
            }

            // Draw the planned path as a green line
            if (path_points.size() >= 2) {
                auto path_line = rerun::components::LineStrip3D(path_points);
                rec->log_static(entity_prefix + "/planned_path",
                                rerun::LineStrips3D(path_line)
                                    .with_colors({{0, 255, 0, 128}}) // Green for planned path
                                    .with_radii({{0.05f}}));
            }

            // Visualize individual waypoints as green spheres
            std::vector<rerun::components::Position3D> waypoint_positions;
            for (const auto &waypoint : current_path->waypoints) {
                waypoint_positions.push_back({static_cast<float>(waypoint.x), static_cast<float>(waypoint.y), 0});
            }

            if (!waypoint_positions.empty()) {
                rec->log_static(entity_prefix + "/waypoints", rerun::Points3D(waypoint_positions)
                                                                  .with_colors({{0, 255, 0}}) // Green waypoints
                                                                  .with_radii({{0.07f}}));
            }

            // Highlight current target waypoint in yellow
            if (current_waypoint_index < current_path->waypoints.size()) {
                const auto &current_target = current_path->waypoints[current_waypoint_index];
                rec->log_static(entity_prefix + "/current_target",
                                rerun::Points3D({{static_cast<float>(current_target.x),
                                                  static_cast<float>(current_target.y), 0.0f}})
                                    .with_colors({{255, 255, 0, 128}}) // Yellow for current target
                                    .with_radii({{0.12f}}));
            }
        }

        // Visualize current goal
        if (current_goal.has_value()) {
            const auto &target = current_goal->target;

            // Visualize goal point as a red sphere
            rec->log_static(entity_prefix + "/goal",
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
                rec->log_static(entity_prefix + "/goal_tolerance",
                                rerun::LineStrips3D(tolerance_line)
                                    .with_colors({{255, 0, 0, 128}}) // Semi-transparent red
                                    .with_radii({{0.02f}}));
            }
        }

        // Draw line from robot to current target waypoint
        if (current_path.has_value() && current_waypoint_index < current_path->waypoints.size()) {
            const auto &target_waypoint = current_path->waypoints[current_waypoint_index];
            std::vector<std::array<float, 3>> target_line = {
                {static_cast<float>(current_state_.pose.point.x), static_cast<float>(current_state_.pose.point.y),
                 0.2f},
                {static_cast<float>(target_waypoint.x), static_cast<float>(target_waypoint.y), 0}};

            auto target_strip = rerun::components::LineStrip3D(target_line);
            rec->log_static(entity_prefix + "/target_line", rerun::LineStrips3D(target_strip)
                                                                .with_colors({{255, 255, 0, 128}}) // Yellow
                                                                .with_radii({{0.02f}}));
        }
#else
        // No-op when rerun is not available
        (void)0;
#endif // HAS_RERUN
    }

    // Internal helper methods
    Goal Tracker::get_current_tracking_goal() const {
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

    void Tracker::update_waypoint_progress(const RobotState &current_state) {
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
            // DEBUG:             std::cout << ", now " << current_waypoint_index << std::endl;

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

    void Tracker::create_controller() {
        ControllerConfig config;

        switch (controller_type) {
        case TrackerType::PID:
            // DEBUG:             std::cout << "Creating PID controller..." << std::endl;
            config.kp_linear = params.linear_kp;
            config.ki_linear = params.linear_ki;
            config.kd_linear = params.linear_kd;
            config.kp_angular = params.angular_kp;
            config.ki_angular = params.angular_ki;
            config.kd_angular = params.angular_kd;
            controller = std::make_unique<tracking::point::PIDFollower>();
            controller->set_config(config);
            // DEBUG:             std::cout << "PID controller created: " << (controller ? "success" : "failed") <<
            // std::endl;
            break;

        case TrackerType::PURE_PURSUIT:
            // DEBUG:             std::cout << "Creating Pure Pursuit controller..." << std::endl;
            config.lookahead_distance = params.lookahead_distance;
            config.goal_tolerance = 0.5f;
            config.angular_tolerance = 0.1f;
            controller = std::make_unique<tracking::path::PurePursuitFollower>();
            controller->set_config(config);
            // DEBUG:             std::cout << "Pure Pursuit controller created: " << (controller ? "success" :
            // "failed") << std::endl;
            break;

        case TrackerType::STANLEY:
            // DEBUG:             std::cout << "Creating Stanley controller..." << std::endl;
            config.k_cross_track = params.cross_track_gain;
            config.k_heading = params.softening_gain;
            config.goal_tolerance = 0.5f;
            config.angular_tolerance = 0.1f;
            config.allow_reverse = false; // Can be enabled if needed
            controller = std::make_unique<tracking::path::StanleyFollower>();
            controller->set_config(config);
            // DEBUG:             std::cout << "Stanley controller created: " << (controller ? "success" : "failed") <<
            // std::endl;
            break;

        case TrackerType::CARROT:
            config.lookahead_distance = params.carrot_distance;
            controller = std::make_unique<tracking::point::CarrotFollower>();
            controller->set_config(config);
            break;

        case TrackerType::LQR:
#ifdef HAS_LQR
            config.goal_tolerance = 0.5f;
            config.angular_tolerance = 0.1f;
            controller = std::make_unique<tracking::path::LQRFollower>();
            controller->set_config(config);
#else
            // Fallback to Pure Pursuit if LQR not available
            controller = std::make_unique<tracking::path::PurePursuitFollower>();
            controller->set_config(config);
#endif
            break;

        case TrackerType::MPC:
#ifdef HAS_MPC
            config.goal_tolerance = 0.5f;
            config.angular_tolerance = 0.1f;
            controller = std::make_unique<tracking::path::MPCFollower>();
            controller->set_config(config);
#else
            // Fallback to Pure Pursuit if MPC not available
            controller = std::make_unique<tracking::path::PurePursuitFollower>();
            controller->set_config(config);
#endif
            break;
        }
    }

    void Tracker::update_goal_status(const RobotState &current_state) {
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

} // namespace navcon
