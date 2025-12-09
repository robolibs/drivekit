#include "navcon/path/lqr.hpp"
#include "navcon/types.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>

namespace navcon {
    namespace path {

        namespace {

            constexpr int LQR_N = 4;

            using Mat4 = std::array<double, LQR_N * LQR_N>;
            using Vec4 = std::array<double, LQR_N>;

            inline Mat4 mat4_zero() {
                Mat4 M{};
                M.fill(0.0);
                return M;
            }

            inline Mat4 mat4_identity() {
                Mat4 M = mat4_zero();
                for (int i = 0; i < LQR_N; ++i) {
                    M[i * LQR_N + i] = 1.0;
                }
                return M;
            }

            inline Mat4 mat4_add(const Mat4 &A, const Mat4 &B) {
                Mat4 C;
                for (int i = 0; i < LQR_N * LQR_N; ++i) {
                    C[i] = A[i] + B[i];
                }
                return C;
            }

            inline Mat4 mat4_sub(const Mat4 &A, const Mat4 &B) {
                Mat4 C;
                for (int i = 0; i < LQR_N * LQR_N; ++i) {
                    C[i] = A[i] - B[i];
                }
                return C;
            }

            inline Mat4 mat4_mul(const Mat4 &A, const Mat4 &B) {
                Mat4 C = mat4_zero();
                for (int i = 0; i < LQR_N; ++i) {
                    for (int j = 0; j < LQR_N; ++j) {
                        double sum = 0.0;
                        for (int k = 0; k < LQR_N; ++k) {
                            sum += A[i * LQR_N + k] * B[k * LQR_N + j];
                        }
                        C[i * LQR_N + j] = sum;
                    }
                }
                return C;
            }

            inline Mat4 mat4_transpose(const Mat4 &A) {
                Mat4 At;
                for (int i = 0; i < LQR_N; ++i) {
                    for (int j = 0; j < LQR_N; ++j) {
                        At[j * LQR_N + i] = A[i * LQR_N + j];
                    }
                }
                return At;
            }

            inline Vec4 mat4_vec_mul(const Mat4 &A, const Vec4 &x) {
                Vec4 y{};
                for (int i = 0; i < LQR_N; ++i) {
                    double sum = 0.0;
                    for (int j = 0; j < LQR_N; ++j) {
                        sum += A[i * LQR_N + j] * x[j];
                    }
                    y[i] = sum;
                }
                return y;
            }

            inline double vec4_dot(const Vec4 &a, const Vec4 &b) {
                double s = 0.0;
                for (int i = 0; i < LQR_N; ++i) {
                    s += a[i] * b[i];
                }
                return s;
            }

            // Solve Discrete Algebraic Riccati Equation using iterative method
            Mat4 solve_dare(const Mat4 &A, const Vec4 &B, const Mat4 &Q, double R) {
                const int max_iterations = 150;
                const double tolerance = 1e-5;

                Mat4 P = Q; // Initial guess

                for (int it = 0; it < max_iterations; ++it) {
                    Mat4 AT = mat4_transpose(A);

                    // A^T P A
                    Mat4 AT_P = mat4_mul(AT, P);
                    Mat4 AT_P_A = mat4_mul(AT_P, A);

                    // B^T P B (scalar)
                    Vec4 PB = mat4_vec_mul(P, B);
                    double BT_P_B = vec4_dot(B, PB);
                    double denom = R + BT_P_B;
                    if (std::fabs(denom) < 1e-9) {
                        denom = (denom >= 0.0 ? 1e-9 : -1e-9);
                    }
                    double inv_denom = 1.0 / denom;

                    // A^T P B (4x1)
                    Vec4 AT_P_B = mat4_vec_mul(AT_P, B);

                    // B^T P A (1x4)
                    Mat4 P_A = mat4_mul(P, A);
                    Vec4 BT_P_A{};
                    for (int j = 0; j < LQR_N; ++j) {
                        double sum = 0.0;
                        for (int i = 0; i < LQR_N; ++i) {
                            sum += B[i] * P_A[i * LQR_N + j];
                        }
                        BT_P_A[j] = sum;
                    }

                    // A^T P B (R + B^T P B)^-1 B^T P A
                    Mat4 term = mat4_zero();
                    for (int i = 0; i < LQR_N; ++i) {
                        for (int j = 0; j < LQR_N; ++j) {
                            term[i * LQR_N + j] = AT_P_B[i] * BT_P_A[j] * inv_denom;
                        }
                    }

                    Mat4 P_next = mat4_add(mat4_sub(AT_P_A, term), Q);

                    // Check convergence (Frobenius norm)
                    double diff_norm = 0.0;
                    for (int i = 0; i < LQR_N * LQR_N; ++i) {
                        double d = P_next[i] - P[i];
                        diff_norm += d * d;
                    }
                    diff_norm = std::sqrt(diff_norm);
                    P = P_next;

                    if (diff_norm < tolerance) {
                        break;
                    }
                }

                return P;
            }

            // Compute LQR gain: K (1x4 row) given velocity and constraints
            // For differential drive, the control input is angular velocity directly
            // For Ackermann, the control input is steering angle
            std::array<double, 4> compute_lqr_gain(double velocity, const RobotConstraints &constraints, double dt,
                                                   bool is_diff_drive) {
                // Linearized model matrices
                // State: [lateral_error, lateral_error_rate, heading_error, heading_error_rate]
                Mat4 A{};
                A[0 * 4 + 0] = 1.0;
                A[0 * 4 + 1] = dt;
                A[0 * 4 + 2] = 0.0;
                A[0 * 4 + 3] = 0.0;

                A[1 * 4 + 0] = 0.0;
                A[1 * 4 + 1] = 1.0;
                A[1 * 4 + 2] = velocity;
                A[1 * 4 + 3] = 0.0;

                A[2 * 4 + 0] = 0.0;
                A[2 * 4 + 1] = 0.0;
                A[2 * 4 + 2] = 1.0;
                A[2 * 4 + 3] = dt;

                A[3 * 4 + 0] = 0.0;
                A[3 * 4 + 1] = 0.0;
                A[3 * 4 + 2] = 0.0;
                A[3 * 4 + 3] = 1.0;

                Vec4 B{};
                B[0] = 0.0;
                B[1] = 0.0;
                B[2] = 0.0;
                if (is_diff_drive) {
                    // For diff drive: control input (angular velocity) directly affects heading rate
                    B[3] = 1.0;
                } else {
                    // For Ackermann/bicycle: steering angle affects heading via v/L
                    B[3] = velocity / constraints.wheelbase;
                }

                // Cost matrices (tunable parameters)
                Mat4 Q = mat4_zero();
                Q[0 * 4 + 0] = 1.0; // Lateral error cost
                Q[2 * 4 + 2] = 0.5; // Heading error cost

                double R = 0.5; // Control effort cost (scalar)

                Mat4 P = solve_dare(A, B, Q, R);

                // Compute K = (R + B^T P B)^-1 B^T P A (1x4 row)
                Vec4 PB = mat4_vec_mul(P, B);
                double BT_P_B = vec4_dot(B, PB);
                double denom = R + BT_P_B;
                if (std::fabs(denom) < 1e-9) {
                    denom = (denom >= 0.0 ? 1e-9 : -1e-9);
                }
                double inv_denom = 1.0 / denom;

                Mat4 P_A = mat4_mul(P, A);
                std::array<double, 4> K{};
                for (int j = 0; j < LQR_N; ++j) {
                    double sum = 0.0;
                    for (int i = 0; i < LQR_N; ++i) {
                        sum += B[i] * P_A[i * LQR_N + j];
                    }
                    K[j] = inv_denom * sum;
                }

                return K;
            }

        } // namespace

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

            // Determine steering type
            const bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                        constraints.steering_type == SteeringType::SKID_STEER);

            // Get current velocity
            double velocity = current_state.velocity.linear;
            if (std::abs(velocity) < 0.01) {
                velocity = 0.1; // Minimum velocity to avoid division by zero
            }

            // Compute LQR gain vector (1x4 row)
            std::array<double, 4> K = compute_lqr_gain(velocity, constraints, dt, is_diff_drive);

            // State vector: [lateral_error, lateral_error_rate, heading_error, heading_error_rate]
            std::array<double, 4> state_error = {error.lateral_error, lateral_error_rate, error.heading_error,
                                                 heading_error_rate};

            // Feedback control: u_fb = -K * x
            double feedback_control = 0.0;
            for (int i = 0; i < 4; ++i) {
                feedback_control -= K[i] * state_error[i];
            }

            // Update status
            status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
            status_.cross_track_error = std::abs(error.lateral_error);
            status_.heading_error = std::abs(error.heading_error);
            status_.goal_reached = false;
            status_.mode = "lqr";

            // Set linear velocity (constant for now, could be adaptive)
            double linear_velocity = constraints.max_linear_velocity;

            double angular_velocity;
            if (is_diff_drive) {
                // For differential drive: LQR output is angular velocity directly
                // Add feedforward based on path curvature
                double feedforward_omega = velocity * error.path_curvature;
                angular_velocity = feedforward_omega + feedback_control;

                // Clamp angular velocity
                angular_velocity =
                    std::clamp(angular_velocity, -constraints.max_angular_velocity, constraints.max_angular_velocity);
            } else {
                // For Ackermann: LQR output is steering angle
                // Feedforward control based on path curvature
                double feedforward_steering = std::atan2(constraints.wheelbase * error.path_curvature, 1.0);
                double steering_angle = feedforward_steering + feedback_control;

                // Clamp steering to limits
                steering_angle =
                    std::clamp(steering_angle, -constraints.max_steering_angle, constraints.max_steering_angle);

                // Convert steering angle to angular velocity for output
                // Use proportional gain for smoother control
                double kp_steer = 2.0;
                angular_velocity = kp_steer * steering_angle;

                // Clamp angular velocity
                angular_velocity =
                    std::clamp(angular_velocity, -constraints.max_angular_velocity, constraints.max_angular_velocity);
            }

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

            // Update path index to advance along the path
            path_index_ = nearest_idx;

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

    } // namespace path
} // namespace navcon
