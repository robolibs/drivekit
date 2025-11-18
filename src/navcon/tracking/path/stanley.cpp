#include "navcon/tracking/path/stanley.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace navcon {
    namespace tracking {
        namespace path {

            VelocityCommand StanleyFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                             const RobotConstraints &constraints, double dt) {
                VelocityCommand cmd;

                // Check if we have a path
                bool has_path = !path_.waypoints.empty();
                if (!has_path) {
                    // No path, just head toward goal
                    return compute_goal_control(current_state, goal, constraints);
                }

                // Check if goal is reached
                if (this->is_goal_reached(current_state.pose, goal.target_pose)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    return cmd;
                }

                // Update path index to find closest point on path
                update_path_index(current_state.pose);

                // Find the closest point on the path and calculate cross-track error
                auto [closest_point, path_heading, cross_track_error] = find_closest_path_point(current_state.pose);

                // Calculate heading error
                double heading_error = normalize_angle(path_heading - current_state.pose.angle.yaw);

                // Debug output
                static int debug_count = 0;
                if (debug_count % 50 == 0) {
                    std::cout << "Stanley: CTE=" << cross_track_error << ", heading_err=" << heading_error
                              << ", path_heading=" << path_heading << ", robot_yaw=" << current_state.pose.angle.yaw
                              << std::endl;
                }
                debug_count++;

                // Determine if we should move forward or backward based on path direction
                double velocity_direction = 1.0;
                if (config_.allow_reverse) {
                    // Check if we should reverse by looking at heading error
                    if (std::abs(heading_error) > M_PI / 2) {
                        velocity_direction = -1.0;
                        // Adjust heading error for reverse motion
                        heading_error = normalize_angle(heading_error + M_PI);
                    }
                }

                // Set linear velocity
                cmd.linear_velocity = velocity_direction * constraints.max_linear_velocity;

                // Calculate cross-track error term
                // Stanley equation: delta = theta_e + arctan(k * e / v)
                double k_cte = 1.0; // Cross-track error gain
                double velocity =
                    std::max(std::abs(cmd.linear_velocity), 0.1); // Minimum velocity to avoid division issues

                // Stanley control law
                double delta_e = std::atan2(k_cte * cross_track_error, velocity);
                double delta = heading_error + delta_e;

                // Normalize to [-pi, pi]
                while (delta > M_PI) delta -= 2 * M_PI;
                while (delta < -M_PI) delta += 2 * M_PI;

                // Set angular velocity
                cmd.angular_velocity = delta;

                // Clamp angular velocity
                cmd.angular_velocity = std::clamp(cmd.angular_velocity, -constraints.max_angular_velocity,
                                                  constraints.max_angular_velocity);

                // Update status
                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.cross_track_error = cross_track_error;
                status_.heading_error = heading_error;
                status_.goal_reached = false;
                status_.mode = velocity_direction > 0 ? "stanley_forward" : "stanley_reverse";

                cmd.valid = true;
                cmd.status_message = velocity_direction > 0 ? "Following path" : "Reversing";

                return cmd;
            }

            std::string StanleyFollower::get_type() const { return "stanley_follower"; }

            VelocityCommand StanleyFollower::compute_goal_control(const RobotState &current_state, const Goal &goal,
                                                                  const RobotConstraints &constraints) {
                VelocityCommand cmd;

                // Simple proportional control to goal
                double heading_error = calculate_heading_error(current_state.pose, goal.target_pose.point);

                cmd.linear_velocity = constraints.max_linear_velocity;
                cmd.angular_velocity = config_.kp_angular * heading_error;
                cmd.angular_velocity = std::clamp(cmd.angular_velocity, -constraints.max_angular_velocity,
                                                  constraints.max_angular_velocity);
                cmd.valid = true;
                cmd.status_message = "Moving to goal";

                return cmd;
            }

            void StanleyFollower::update_path_index(const Pose &current_pose) {
                if (path_.waypoints.empty()) return;

                // Advance path index if we're close to current waypoint or have passed it
                while (path_index_ < path_.waypoints.size() - 1) {
                    Point current_waypoint = path_.waypoints[path_index_].point;
                    double dist = current_pose.point.distance_to(current_waypoint);

                    // Move to next waypoint if close enough
                    if (dist < 1.0) {
                        path_index_++;
                    } else {
                        break;
                    }
                }
            }

            std::tuple<Point, double, double> StanleyFollower::find_closest_path_point(const Pose &current_pose) {
                if (path_.waypoints.empty()) {
                    return {Point{0, 0}, 0.0, 0.0};
                }

                // Search around current path index for closest point
                size_t search_start = (path_index_ > 5) ? path_index_ - 5 : 0;
                size_t search_end = std::min(path_index_ + 20, path_.waypoints.size());

                double min_distance = std::numeric_limits<double>::max();
                size_t closest_idx = path_index_;

                for (size_t i = search_start; i < search_end; ++i) {
                    double dist = current_pose.point.distance_to(path_.waypoints[i].point);
                    if (dist < min_distance) {
                        min_distance = dist;
                        closest_idx = i;
                    }
                }

                // Get closest point
                Point closest_point = path_.waypoints[closest_idx].point;

                // Calculate path heading at this point
                double path_heading = 0.0;
                if (closest_idx < path_.waypoints.size() - 1) {
                    Point next_point = path_.waypoints[closest_idx + 1].point;
                    double dx = next_point.x - closest_point.x;
                    double dy = next_point.y - closest_point.y;
                    path_heading = std::atan2(dy, dx);
                } else if (closest_idx > 0) {
                    Point prev_point = path_.waypoints[closest_idx - 1].point;
                    double dx = closest_point.x - prev_point.x;
                    double dy = closest_point.y - prev_point.y;
                    path_heading = std::atan2(dy, dx);
                }

                // Calculate cross-track error (lateral distance from path)
                double dx = current_pose.point.x - closest_point.x;
                double dy = current_pose.point.y - closest_point.y;

                // Calculate perpendicular distance
                double cross_track_error = std::sqrt(dx * dx + dy * dy);

                // Determine sign: positive if robot is to the left of path
                // Using the method from reference implementation
                double sign_check = (dy * std::cos(path_heading) - dx * std::sin(path_heading));
                if (sign_check > 0) {
                    cross_track_error = -cross_track_error; // Robot is on right side
                }

                return {closest_point, path_heading, cross_track_error};
            }

        } // namespace path
    } // namespace tracking
} // namespace navcon
