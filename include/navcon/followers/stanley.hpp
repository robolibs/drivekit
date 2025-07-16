#pragma once

#include "navcon/controller.hpp"
#include <algorithm>

namespace navcon {
    namespace followers {

        // Proper Stanley controller implementation based on original research
        // Reference: "Automatic Steering Methods for Autonomous Automobile Path Tracking" by Hoffmann et al.
        class StanleyFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt) override {
                VelocityCommand cmd;

                // Check if goal is reached
                if (this->is_goal_reached(current_state.pose, goal.target_pose)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    return cmd;
                }

                // Find nearest point on path
                auto [nearest_point, path_heading, cross_track_error] = find_nearest_path_point(current_state.pose);

                // Calculate heading error
                double heading_error = this->normalize_angle(path_heading - current_state.pose.angle.yaw);

                // PROPER Stanley control law: δ = ψ_e + arctan(k_e * e / v)
                // where δ = steering angle, ψ_e = heading error, e = cross-track error, v = velocity

                // The key insight: Use the COMMANDED velocity, not the actual velocity
                // This prevents the controller from becoming sluggish at low speeds
                double velocity_for_control = constraints.max_linear_velocity;

                // Stanley cross-track correction term
                double cross_track_term = std::atan(config_.k_cross_track * cross_track_error / velocity_for_control);

                // Complete Stanley steering angle
                double steering_angle = config_.k_heading * heading_error + cross_track_term;

                // Update status
                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.cross_track_error = cross_track_error;
                status_.heading_error = heading_error;
                status_.goal_reached = false;
                status_.mode = "stanley";

                // Apply stanley control for velocity command
                cmd.linear_velocity = constraints.max_linear_velocity;

                // Convert steering angle to angular velocity using bicycle model
                // ω = (v / L) * tan(δ)
                cmd.angular_velocity = (cmd.linear_velocity / constraints.wheelbase) * std::tan(steering_angle);

                // Apply constraints
                cmd.angular_velocity = std::clamp(cmd.angular_velocity, -constraints.max_angular_velocity,
                                                  constraints.max_angular_velocity);

                cmd.valid = true;
                cmd.status_message = "Stanley control active";

                return cmd;
            }

            std::string get_type() const override { return "stanley_follower"; }

          private:
            struct PathPoint {
                Point point;
                double heading;
                double cross_track_error;
            };

            PathPoint find_nearest_path_point(const Pose &current_pose) {
                PathPoint result;

                if (path_.waypoints.empty()) {
                    // No path, use direct line to goal
                    result.point = current_pose.point;
                    result.heading = 0.0;
                    result.cross_track_error = 0.0;
                    return result;
                }

                // Improved path progression logic similar to Pure Pursuit
                // Advance waypoint when close OR when we've passed it
                while (path_index_ < path_.waypoints.size() - 1) {
                    Point current_waypoint = path_.waypoints[path_index_].point;
                    Point next_waypoint = path_.waypoints[path_index_ + 1].point;

                    double dist_to_current = current_pose.point.distance_to(current_waypoint);

                    // Check if we've passed the waypoint by looking at the dot product
                    double dx_to_current = current_waypoint.x - current_pose.point.x;
                    double dy_to_current = current_waypoint.y - current_pose.point.y;
                    double dx_to_next = next_waypoint.x - current_waypoint.x;
                    double dy_to_next = next_waypoint.y - current_waypoint.y;

                    // If dot product is negative, we've passed the waypoint
                    double dot_product = dx_to_current * dx_to_next + dy_to_current * dy_to_next;

                    // Advance if: close to waypoint OR passed it OR too far away (stuck)
                    if (dist_to_current < 2.0 || dot_product < 0 || dist_to_current > 15.0) {
                        path_index_++;
                    } else {
                        break;
                    }
                }

                // Calculate path heading and cross-track error
                if (path_index_ < path_.waypoints.size() - 1) {
                    // Use heading to next waypoint
                    const auto &current_wp = path_.waypoints[path_index_];
                    const auto &next_wp = path_.waypoints[path_index_ + 1];

                    double dx = next_wp.point.x - current_wp.point.x;
                    double dy = next_wp.point.y - current_wp.point.y;
                    result.heading = std::atan2(dy, dx);

                    // Calculate cross-track error using proper coordinate transformation
                    // Transform robot position to path-relative coordinates
                    double path_dx = next_wp.point.x - current_wp.point.x;
                    double path_dy = next_wp.point.y - current_wp.point.y;
                    double path_length = std::sqrt(path_dx * path_dx + path_dy * path_dy);

                    if (path_length > 1e-6) {
                        // Vector from current waypoint to robot
                        double to_robot_x = current_pose.point.x - current_wp.point.x;
                        double to_robot_y = current_pose.point.y - current_wp.point.y;

                        // Unit vector along path direction
                        double path_unit_x = path_dx / path_length;
                        double path_unit_y = path_dy / path_length;

                        // Cross-track error is the perpendicular distance from path
                        // Using the cross product to get signed distance
                        // Positive means robot is to the left of path direction
                        result.cross_track_error = to_robot_x * path_unit_y - to_robot_y * path_unit_x;
                    } else {
                        result.cross_track_error = 0.0;
                    }

                    result.point = current_wp.point;
                } else {
                    // At last waypoint, use its heading
                    result.point = path_.waypoints[path_index_].point;
                    result.heading = path_.waypoints[path_index_].angle.yaw;
                    result.cross_track_error = 0.0;
                }

                return result;
            }
        };

    } // namespace controllers
} // namespace navcon