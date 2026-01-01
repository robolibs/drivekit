#include "drivekit/path/pure_pursuit.hpp"
#include "drivekit/types.hpp"
#include <algorithm>
#include <iostream>

namespace drivekit {
    namespace path {

        VelocityCommand PurePursuitFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                             const RobotConstraints &constraints, double dt,
                                                             const WorldConstraints *dynamic_constraints) {
            VelocityCommand cmd;

            // Get wheelbase (distance between front and rear axles)
            double wheelbase = constraints.wheelbase > 0.0 ? constraints.wheelbase : 1.0;

            // Calculate rear axle position (reference point for Pure Pursuit)
            // This is more accurate than using the center of the robot
            double yaw = current_state.pose.rotation.to_euler().yaw;
            double rear_x = current_state.pose.point.x - (wheelbase / 2.0) * std::cos(yaw);
            double rear_y = current_state.pose.point.y - (wheelbase / 2.0) * std::sin(yaw);

            Point rear_axle{rear_x, rear_y};

            // Dynamic lookahead distance based on velocity (adaptive)
            // Lf = k * v + Lfc (from PythonRobotics)
            double k_lookahead = 0.1; // Velocity gain (configurable)
            double current_velocity = std::abs(current_state.velocity.linear);
            double base_lookahead = config_.lookahead_distance;
            double lookahead_distance = k_lookahead * current_velocity + base_lookahead;

            // Clamp lookahead distance to reasonable bounds
            lookahead_distance = std::clamp(lookahead_distance, base_lookahead, base_lookahead * 3.0);

            // Get target point using lookahead algorithm
            Point target_point;
            bool has_path = !path_.drivekits.empty();
            double progress_distance = wheelbase * 1.5; // Smaller progress threshold

            if (has_path) {
                auto lookahead_result = find_lookahead_point(rear_axle, yaw, lookahead_distance, progress_distance);
                if (!lookahead_result.has_value()) {
                    // Reached end of path, target the goal
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

            // Determine if this is a diff/skid drive that supports turn_first
            const bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                        constraints.steering_type == SteeringType::SKID_STEER);

            // Calculate angle to target from rear axle (alpha)
            double dx = target_point.x - rear_x;
            double dy = target_point.y - rear_y;
            double alpha = std::atan2(dy, dx) - yaw;

            // Normalize alpha to [-pi, pi]
            alpha = normalize_angle(alpha);

            // Update status
            status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
            status_.cross_track_error = std::abs(std::sin(alpha) * std::hypot(dx, dy));
            status_.goal_reached = false;
            status_.mode = "pure_pursuit";

            // Check turn_first mode for diff/skid drive: rotate in place until aligned
            if (is_diff_drive && current_state.turn_first && std::abs(alpha) > config_.angular_tolerance) {
                // Turn in place - zero linear velocity until aligned
                double kp_angular = 2.5;
                double angular_control = kp_angular * alpha;
                angular_control =
                    std::clamp(angular_control, -constraints.max_angular_velocity, constraints.max_angular_velocity);

                if (config_.output_units == OutputUnits::NORMALIZED) {
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = angular_control / constraints.max_angular_velocity;
                } else {
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = angular_control;
                }

                cmd.valid = true;
                cmd.status_message = "Turning to align";
                status_.mode = "turning";

                return cmd;
            }

            // Calculate desired angular velocity
            double angular_physical;
            double linear_physical;

            // For differential drive: use proportional control on angle error
            // This works better than Pure Pursuit curvature formula for low-speed robots

            // Calculate base angular command proportional to alpha
            // Use a high gain to ensure we can reach full angular velocity
            double kp_angular = 2.5; // Proportional gain for angular control
            angular_physical = kp_angular * alpha;

            // Clamp to maximum angular velocity
            angular_physical =
                std::clamp(angular_physical, -constraints.max_angular_velocity, constraints.max_angular_velocity);

            // Adjust linear velocity based on how much we need to turn
            double alpha_magnitude = std::abs(alpha);

            if (alpha_magnitude > M_PI * 0.66) {
                // Very sharp turn (> 120°): slow down a lot
                linear_physical = constraints.max_linear_velocity * 0.3;
            } else if (alpha_magnitude > M_PI / 3.0) {
                // Moderate turn (> 60°): slow down moderately
                linear_physical = constraints.max_linear_velocity * 0.6;
            } else {
                // Gentle turn: maintain speed
                linear_physical = constraints.max_linear_velocity;
            }

            // Convert to output units based on configuration
            if (config_.output_units == OutputUnits::NORMALIZED) {
                // Normalize to [-1, 1]
                cmd.linear_velocity = linear_physical / constraints.max_linear_velocity;
                cmd.angular_velocity = angular_physical / constraints.max_angular_velocity;
            } else {
                // Physical units (m/s, rad/s)
                cmd.linear_velocity = linear_physical;
                cmd.angular_velocity = angular_physical;
            }

            cmd.valid = true;
            cmd.status_message = "Following path";

            return cmd;
        }

        std::string PurePursuitFollower::get_type() const { return "pure_pursuit_follower"; }

        std::optional<Point> PurePursuitFollower::find_lookahead_point(const Point &rear_axle, double current_yaw,
                                                                       double lookahead_distance,
                                                                       double progress_distance) {
            if (path_.drivekits.empty()) return std::nullopt;

            // Update path index: advance when we've passed drivekits
            // Use rear axle position for consistency
            while (path_index_ < path_.drivekits.size() - 1) {
                Point current_drivekit = path_.drivekits[path_index_].point;
                Point next_drivekit = path_.drivekits[path_index_ + 1].point;

                double dist_to_current = rear_axle.distance_to(current_drivekit);

                // Vector from robot to current drivekit
                double dx_to_current = current_drivekit.x - rear_axle.x;
                double dy_to_current = current_drivekit.y - rear_axle.y;

                // Vector from current to next drivekit (path direction)
                double dx_path = next_drivekit.x - current_drivekit.x;
                double dy_path = next_drivekit.y - current_drivekit.y;

                // Dot product: negative means we've passed the drivekit
                double dot_product = dx_to_current * dx_path + dy_to_current * dy_path;

                // Advance if: very close to drivekit OR passed it
                if (dist_to_current < progress_distance || dot_product < 0) {
                    path_index_++;
                } else {
                    break;
                }
            }

            // Search for lookahead point along the path
            // Start from current drivekit position
            double min_dist_diff = std::numeric_limits<double>::max();
            std::optional<Point> best_point;

            // Search forward from current path index
            for (size_t i = path_index_; i < path_.drivekits.size(); ++i) {
                Point drivekit = path_.drivekits[i].point;
                double dist = rear_axle.distance_to(drivekit);

                // Find point closest to desired lookahead distance
                double dist_diff = std::abs(dist - lookahead_distance);
                if (dist_diff < min_dist_diff && dist >= lookahead_distance * 0.5) {
                    min_dist_diff = dist_diff;
                    best_point = drivekit;
                }

                // Also check interpolated points on segments
                if (i < path_.drivekits.size() - 1) {
                    Point next_drivekit = path_.drivekits[i + 1].point;

                    // Check if lookahead circle intersects this segment
                    auto intersection =
                        find_circle_segment_intersection(rear_axle, lookahead_distance, drivekit, next_drivekit);

                    if (intersection.has_value()) {
                        double dist_to_intersection = rear_axle.distance_to(intersection.value());
                        double diff = std::abs(dist_to_intersection - lookahead_distance);
                        if (diff < min_dist_diff) {
                            min_dist_diff = diff;
                            best_point = intersection.value();
                        }
                    }
                }

                // Stop searching if we're way past the lookahead distance
                if (dist > lookahead_distance * 2.0) {
                    break;
                }
            }

            // If no good point found, use the furthest visible drivekit
            if (!best_point.has_value() && path_index_ < path_.drivekits.size()) {
                best_point = path_.drivekits.back().point;
            }

            return best_point;
        }

        // Helper: find intersection between circle and line segment
        std::optional<Point> PurePursuitFollower::find_circle_segment_intersection(const Point &circle_center,
                                                                                   double radius,
                                                                                   const Point &segment_start,
                                                                                   const Point &segment_end) {

            // Vector from segment start to end
            double dx = segment_end.x - segment_start.x;
            double dy = segment_end.y - segment_start.y;
            double segment_length = std::hypot(dx, dy);

            if (segment_length < 1e-6) {
                return std::nullopt;
            }

            // Normalize direction
            dx /= segment_length;
            dy /= segment_length;

            // Vector from segment start to circle center
            double fx = circle_center.x - segment_start.x;
            double fy = circle_center.y - segment_start.y;

            // Project circle center onto segment line
            double projection = fx * dx + fy * dy;

            // Find closest point on infinite line
            double closest_x = segment_start.x + projection * dx;
            double closest_y = segment_start.y + projection * dy;

            // Distance from circle center to line
            double dist_to_line = std::hypot(circle_center.x - closest_x, circle_center.y - closest_y);

            // No intersection if line is too far from circle
            if (dist_to_line > radius) {
                return std::nullopt;
            }

            // Calculate intersection point(s)
            double half_chord = std::sqrt(radius * radius - dist_to_line * dist_to_line);

            // Two potential intersection points
            double t1 = projection - half_chord;
            double t2 = projection + half_chord;

            // Use the furthest intersection that's still on the segment
            // (we want to look ahead, not behind)
            std::optional<Point> result;

            if (t2 >= 0 && t2 <= segment_length) {
                result = Point{segment_start.x + t2 * dx, segment_start.y + t2 * dy};
            } else if (t1 >= 0 && t1 <= segment_length) {
                result = Point{segment_start.x + t1 * dx, segment_start.y + t1 * dy};
            }

            return result;
        }

    } // namespace path
} // namespace drivekit
