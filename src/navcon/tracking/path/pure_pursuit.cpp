#include "navcon/tracking/path/pure_pursuit.hpp"
#include <algorithm>
#include <iostream>

namespace navcon {
    namespace tracking {
        namespace path {

            VelocityCommand PurePursuitFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                                 const RobotConstraints &constraints, double dt) {
                VelocityCommand cmd;

                // Get target point using improved lookahead algorithm
                Point target_point;
                bool has_path = !path_.waypoints.empty();
                double lookahead_distance = config_.lookahead_distance;
                double robot_length = constraints.robot_length > 0.0
                                          ? constraints.robot_length
                                          : (constraints.wheelbase > 0.0 ? constraints.wheelbase : 1.0);
                double progress_distance = compute_progress_distance(robot_length);

                if (has_path) {
                    auto lookahead_result =
                        find_lookahead_point(current_state.pose, lookahead_distance, progress_distance);
                    if (!lookahead_result.has_value()) {
                        // Reached end of path
                        target_point = goal.target_pose.point;
                    } else {
                        target_point = lookahead_result.value();
                    }
                } else {
                    target_point = goal.target_pose.point;
                }

                // Check if goal is reached
                if (this->is_goal_reached(current_state.pose, goal.target_pose)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    return cmd;
                }

                // Transform target to robot's local frame
                double dx_global = target_point.x - current_state.pose.point.x;
                double dy_global = target_point.y - current_state.pose.point.y;

                double cos_theta = std::cos(current_state.pose.angle.yaw);
                double sin_theta = std::sin(current_state.pose.angle.yaw);

                double dx_local = cos_theta * dx_global + sin_theta * dy_global;
                double dy_local = -sin_theta * dx_global + cos_theta * dy_global;

                // Calculate curvature using aggressive pure pursuit formula for sharp turning
                // Much more aggressive curvature calculation - use shorter effective distance for sharper turns
                double curvature = 2.0 * dy_local / (lookahead_distance * lookahead_distance);

                // Amplify curvature for more aggressive turning
                double curvature_multiplier = 2.5; // Make turning much more aggressive
                curvature *= curvature_multiplier;

                // Update status
                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.cross_track_error = std::abs(dy_local);
                status_.goal_reached = false;
                status_.mode = "pure_pursuit";

                // Apply pure pursuit control for velocity command
                // Maintain constant speed for more aggressive turning
                cmd.linear_velocity = constraints.max_linear_velocity;

                // Much more aggressive angular velocity from curvature
                cmd.angular_velocity = curvature * cmd.linear_velocity;

                // Increase angular velocity limits for sharper turns
                double aggressive_angular_limit = constraints.max_angular_velocity * 1.5; // 50% more aggressive
                cmd.angular_velocity =
                    std::clamp(cmd.angular_velocity, -aggressive_angular_limit, aggressive_angular_limit);

                cmd.valid = true;
                cmd.status_message = "Following path";

                return cmd;
            }

            std::string PurePursuitFollower::get_type() const { return "pure_pursuit_follower"; }

            std::optional<Point> PurePursuitFollower::find_lookahead_point(const Pose &current_pose,
                                                                           double lookahead_distance,
                                                                           double progress_distance) {
                if (path_.waypoints.empty()) return std::nullopt;

                // FIXED waypoint progression: advance when close OR when we've passed the waypoint
                // This prevents the robot from getting stuck targeting the first waypoint
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
                    if (dist_to_current < progress_distance || dot_product < 0 || dist_to_current > 15.0) {
                        path_index_++;
                    } else {
                        break;
                    }
                }

                // Find lookahead point starting from current path index
                double accumulated_distance = 0.0;
                Point start_point = path_.waypoints[path_index_].point;

                // Check if we can use the current waypoint directly (if it's far enough)
                double dist_to_start = current_pose.point.distance_to(start_point);
                if (dist_to_start >= lookahead_distance * 0.8) {
                    return start_point; // Use current waypoint if it's at good lookahead distance
                }

                // Otherwise, find interpolated lookahead point along path
                Point last_point = start_point;

                for (size_t i = path_index_ + 1; i < path_.waypoints.size(); ++i) {
                    Point current_point = path_.waypoints[i].point;
                    double segment_length = last_point.distance_to(current_point);

                    if (accumulated_distance + segment_length >= lookahead_distance) {
                        // Interpolate point on this segment
                        double remaining = lookahead_distance - accumulated_distance;
                        double t = segment_length > 0 ? remaining / segment_length : 0.0;

                        Point lookahead;
                        lookahead.x = last_point.x + t * (current_point.x - last_point.x);
                        lookahead.y = last_point.y + t * (current_point.y - last_point.y);
                        return lookahead;
                    }

                    accumulated_distance += segment_length;
                    last_point = current_point;
                }

                // If we've reached the end of the path, return the last waypoint
                return path_.waypoints.back().point;
            }

            double PurePursuitFollower::compute_progress_distance(double robot_length) const {
                double length_based = robot_length * 2.5;
                if (length_based <= 0.0) {
                    length_based = 1.0;
                }
                return std::max(length_based, config_.lookahead_distance);
            }

        } // namespace path
    } // namespace tracking
} // namespace navcon
