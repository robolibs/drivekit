#include "navcon/tracking/path/lqr.hpp"
#include "navcon/tracking/types.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace navcon {
    namespace tracking {
        namespace path {

            VelocityCommand LQRFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                         const RobotConstraints &constraints, double dt,
                                                         const WorldConstraints *world_constraints) {
                (void)world_constraints; // LQR doesn't use world constraints

                VelocityCommand cmd;
                cmd.valid = false;

                // Check if we have a path
                if (path_.waypoints.empty()) {
                    return cmd;
                }

                // Check if goal is reached
                if (is_goal_reached(current_state.pose, goal.target_pose)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = 0.0;
                    return cmd;
                }

                // Calculate path tracking errors
                PathError error = calculate_path_error(current_state);

                // Calculate error derivatives
                double lateral_error_rate = (error.lateral_error - previous_lateral_error_) / dt;
                double heading_error_rate = (error.heading_error - previous_heading_error_) / dt;

                // Store for next iteration
                previous_lateral_error_ = error.lateral_error;
                previous_heading_error_ = error.heading_error;

                // Get current velocity
                double velocity = current_state.velocity.linear;
                if (std::abs(velocity) < 0.01) {
                    velocity = 0.1; // Minimum velocity to avoid division by zero
                }

                // Compute LQR gain matrix
                Eigen::MatrixXd K = compute_lqr_gain(velocity, constraints, dt);

                // State vector: [lateral_error, lateral_error_rate, heading_error, heading_error_rate]
                Eigen::Vector4d state_error;
                state_error << error.lateral_error, lateral_error_rate, error.heading_error, heading_error_rate;

                // Feedback control: u_fb = -K * x
                double feedback_steering = -(K * state_error)(0);

                // Feedforward control based on path curvature
                double feedforward_steering = std::atan2(constraints.wheelbase * error.path_curvature, 1.0);

                // Total steering command
                double steering_angle = feedforward_steering + feedback_steering;

                // Debug output
                // static int lqr_debug = 0;
                // if (lqr_debug++ % 100 == 0) {
                //     std::cout << "LQR DEBUG: lat_err=" << error.lateral_error << ", head_err=" << error.heading_error
                //               << ", ff_steer=" << feedforward_steering << ", fb_steer=" << feedback_steering
                //               << ", total_steer=" << steering_angle << std::endl;
                // }

                // Clamp steering to limits
                steering_angle =
                    std::clamp(steering_angle, -constraints.max_steering_angle, constraints.max_steering_angle);

                // Update status
                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.cross_track_error = std::abs(error.lateral_error);
                status_.heading_error = std::abs(error.heading_error);
                status_.goal_reached = false;
                status_.mode = "lqr";

                // Set linear velocity (constant for now, could be adaptive)
                double linear_velocity = constraints.max_linear_velocity;

                // For differential drive: convert steering angle to angular velocity
                // Use direct proportional control on the steering angle
                // This is more effective than the bicycle model for diff drive robots
                double kp_steer = 2.0; // Proportional gain for steering to angular velocity
                double angular_velocity = kp_steer * steering_angle;

                // Clamp angular velocity
                angular_velocity =
                    std::clamp(angular_velocity, -constraints.max_angular_velocity, constraints.max_angular_velocity);

                // Convert to output units based on configuration
                if (config_.output_units == OutputUnits::NORMALIZED) {
                    cmd.linear_velocity = linear_velocity / constraints.max_linear_velocity;
                    cmd.angular_velocity = angular_velocity / constraints.max_angular_velocity;
                } else {
                    cmd.linear_velocity = linear_velocity;
                    cmd.angular_velocity = angular_velocity;
                }

                cmd.valid = true;
                cmd.status_message = "LQR tracking";

                // Final debug
                // static int return_debug = 0;
                // if (return_debug++ % 100 == 0) {
                //     std::cout << "LQR RETURN: valid=" << cmd.valid << ", linear=" << cmd.linear_velocity
                //               << ", angular=" << cmd.angular_velocity << std::endl;
                // }

                return cmd;
            }

            std::string LQRFollower::get_type() const { return "lqr_follower"; }

            LQRFollower::PathError LQRFollower::calculate_path_error(const RobotState &current_state) {
                PathError result;

                // Find nearest point on path
                double min_distance = std::numeric_limits<double>::max();
                size_t nearest_idx = path_index_;

                for (size_t i = path_index_; i < path_.waypoints.size(); ++i) {
                    double dist = current_state.pose.point.distance_to(path_.waypoints[i].point);
                    if (dist < min_distance) {
                        min_distance = dist;
                        nearest_idx = i;
                    }
                    // Stop searching if distance starts increasing
                    if (i > path_index_ && dist > min_distance * 1.5) {
                        break;
                    }
                }

                result.nearest_index = nearest_idx;
                result.nearest_point = path_.waypoints[nearest_idx].point;

                // Calculate path heading at nearest point
                if (nearest_idx < path_.waypoints.size() - 1) {
                    Point next_point = path_.waypoints[nearest_idx + 1].point;
                    result.path_heading =
                        std::atan2(next_point.y - result.nearest_point.y, next_point.x - result.nearest_point.x);
                } else {
                    result.path_heading = path_.waypoints[nearest_idx].angle.yaw;
                }

                // Calculate lateral error (cross-track error)
                double dx = current_state.pose.point.x - result.nearest_point.x;
                double dy = current_state.pose.point.y - result.nearest_point.y;

                // Project onto path normal
                result.lateral_error = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);

                // Calculate heading error
                result.heading_error = normalize_angle(current_state.pose.angle.yaw - result.path_heading);

                // Estimate path curvature
                if (nearest_idx > 0 && nearest_idx < path_.waypoints.size() - 1) {
                    Point prev = path_.waypoints[nearest_idx - 1].point;
                    Point curr = path_.waypoints[nearest_idx].point;
                    Point next = path_.waypoints[nearest_idx + 1].point;

                    double heading1 = std::atan2(curr.y - prev.y, curr.x - prev.x);
                    double heading2 = std::atan2(next.y - curr.y, next.x - curr.x);
                    double heading_change = normalize_angle(heading2 - heading1);
                    double arc_length = prev.distance_to(curr) + curr.distance_to(next);

                    result.path_curvature = heading_change / (arc_length / 2.0);
                } else {
                    result.path_curvature = 0.0;
                }

                return result;
            }

            Eigen::MatrixXd LQRFollower::solve_dare(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                                    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R) {
                // Solve Discrete Algebraic Riccati Equation using iterative method
                const int max_iterations = 150;
                const double tolerance = 1e-5;

                Eigen::MatrixXd P = Q; // Initial guess

                for (int i = 0; i < max_iterations; ++i) {
                    Eigen::MatrixXd P_next =
                        A.transpose() * P * A -
                        A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;

                    if ((P_next - P).norm() < tolerance) {
                        return P_next;
                    }
                    P = P_next;
                }

                // Return last iteration if not converged
                return P;
            }

            Eigen::MatrixXd LQRFollower::compute_lqr_gain(double velocity, const RobotConstraints &constraints,
                                                          double dt) {
                // State: [lateral_error, lateral_error_rate, heading_error, heading_error_rate]
                // Control: [steering_angle]

                // Linearized bicycle model matrices
                Eigen::Matrix4d A;
                A << 1.0, dt, 0.0, 0.0, 0.0, 1.0, velocity, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0, 1.0;

                Eigen::Vector4d B;
                B << 0.0, 0.0, 0.0, velocity / constraints.wheelbase;

                // Cost matrices (tunable parameters)
                Eigen::Matrix4d Q;
                Q << 1.0, 0.0, 0.0, 0.0, // Lateral error cost
                    0.0, 0.0, 0.0, 0.0,  // Lateral error rate cost
                    0.0, 0.0, 0.5, 0.0,  // Heading error cost
                    0.0, 0.0, 0.0, 0.0;  // Heading error rate cost

                Eigen::Matrix<double, 1, 1> R;
                R << 0.5; // Control effort cost

                // Solve DARE
                Eigen::MatrixXd P = solve_dare(A, B, Q, R);

                // Compute LQR gain: K = (R + B^T P B)^-1 B^T P A
                Eigen::MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;

                return K;
            }

        } // namespace path
    } // namespace tracking
} // namespace navcon
