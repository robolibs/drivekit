#pragma once

#include "drivekit/controller.hpp"
#include "drivekit/types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>

namespace drivekit {
    namespace path {

        namespace lqr_internal {

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

            /// Solve Discrete Algebraic Riccati Equation using iterative method.
            inline Mat4 solve_dare(const Mat4 &A, const Vec4 &B, const Mat4 &Q, double R) {
                const int max_iterations = 150;
                const double tolerance = 1e-5;

                Mat4 P = Q;

                for (int it = 0; it < max_iterations; ++it) {
                    Mat4 AT = mat4_transpose(A);

                    Mat4 AT_P = mat4_mul(AT, P);
                    Mat4 AT_P_A = mat4_mul(AT_P, A);

                    Vec4 PB = mat4_vec_mul(P, B);
                    double BT_P_B = vec4_dot(B, PB);
                    double denom = R + BT_P_B;
                    if (std::fabs(denom) < 1e-9) {
                        denom = (denom >= 0.0 ? 1e-9 : -1e-9);
                    }
                    double inv_denom = 1.0 / denom;

                    Vec4 AT_P_B = mat4_vec_mul(AT_P, B);

                    Mat4 P_A = mat4_mul(P, A);
                    Vec4 BT_P_A{};
                    for (int j = 0; j < LQR_N; ++j) {
                        double sum = 0.0;
                        for (int i = 0; i < LQR_N; ++i) {
                            sum += B[i] * P_A[i * LQR_N + j];
                        }
                        BT_P_A[j] = sum;
                    }

                    Mat4 term = mat4_zero();
                    for (int i = 0; i < LQR_N; ++i) {
                        for (int j = 0; j < LQR_N; ++j) {
                            term[i * LQR_N + j] = AT_P_B[i] * BT_P_A[j] * inv_denom;
                        }
                    }

                    Mat4 P_next = mat4_add(mat4_sub(AT_P_A, term), Q);

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

            /// Compute LQR gain: K (1x4 row) given velocity and constraints.
            inline std::array<double, 4> compute_lqr_gain(double velocity, const RobotConstraints &constraints,
                                                          double dt, bool is_diff_drive) {
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
                    B[3] = 1.0;
                } else {
                    B[3] = velocity / constraints.wheelbase;
                }

                Mat4 Q = mat4_zero();
                Q[0 * 4 + 0] = 1.0;
                Q[2 * 4 + 2] = 0.5;

                double R = 0.5;

                Mat4 P = solve_dare(A, B, Q, R);

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

        } // namespace lqr_internal

        /// LQR (Linear Quadratic Regulator) controller for optimal path tracking.
        /// Based on "Path tracking simulation with LQR steering control" from PythonRobotics.
        class LQRFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                   const RobotConstraints &constraints, double dt,
                                                   const WorldConstraints *world_constraints = nullptr) override {
                (void)world_constraints;

                VelocityCommand cmd;
                cmd.valid = false;

                if (path_.drivekits.empty()) {
                    return cmd;
                }

                if (is_goal_reached(current_state.pose, goal.target_pose)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = 0.0;
                    return cmd;
                }

                PathError error = calculate_path_error(current_state);

                double lateral_error_rate = (error.lateral_error - previous_lateral_error_) / dt;
                double heading_error_rate = (error.heading_error - previous_heading_error_) / dt;

                previous_lateral_error_ = error.lateral_error;
                previous_heading_error_ = error.heading_error;

                const bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                            constraints.steering_type == SteeringType::SKID_STEER);

                if (is_diff_drive && current_state.turn_first &&
                    std::abs(error.heading_error) > config_.angular_tolerance) {
                    double kp_angular = 2.0;
                    double angular_control = kp_angular * error.heading_error;
                    angular_control = std::clamp(angular_control, -constraints.max_angular_velocity,
                                                 constraints.max_angular_velocity);

                    status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                    status_.cross_track_error = std::abs(error.lateral_error);
                    status_.heading_error = std::abs(error.heading_error);
                    status_.goal_reached = false;
                    status_.mode = "turning";

                    if (config_.output_units == OutputUnits::NORMALIZED) {
                        cmd.linear_velocity = 0.0;
                        cmd.angular_velocity = angular_control / constraints.max_angular_velocity;
                    } else {
                        cmd.linear_velocity = 0.0;
                        cmd.angular_velocity = angular_control;
                    }

                    cmd.valid = true;
                    cmd.status_message = "Turning to align";

                    return cmd;
                }

                double velocity = current_state.velocity.linear;
                if (std::abs(velocity) < 0.01) {
                    velocity = 0.1;
                }

                std::array<double, 4> K = lqr_internal::compute_lqr_gain(velocity, constraints, dt, is_diff_drive);

                std::array<double, 4> state_error = {error.lateral_error, lateral_error_rate, error.heading_error,
                                                     heading_error_rate};

                double feedback_control = 0.0;
                for (int i = 0; i < 4; ++i) {
                    feedback_control -= K[i] * state_error[i];
                }

                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.cross_track_error = std::abs(error.lateral_error);
                status_.heading_error = std::abs(error.heading_error);
                status_.goal_reached = false;
                status_.mode = "lqr";

                double linear_velocity = constraints.max_linear_velocity;

                double angular_velocity;
                if (is_diff_drive) {
                    double feedforward_omega = velocity * error.path_curvature;
                    angular_velocity = feedforward_omega + feedback_control;

                    angular_velocity = std::clamp(angular_velocity, -constraints.max_angular_velocity,
                                                  constraints.max_angular_velocity);
                } else {
                    double feedforward_steering = std::atan2(constraints.wheelbase * error.path_curvature, 1.0);
                    double steering_angle = feedforward_steering + feedback_control;

                    steering_angle =
                        std::clamp(steering_angle, -constraints.max_steering_angle, constraints.max_steering_angle);

                    double kp_steer = 2.0;
                    angular_velocity = kp_steer * steering_angle;

                    angular_velocity = std::clamp(angular_velocity, -constraints.max_angular_velocity,
                                                  constraints.max_angular_velocity);
                }

                if (config_.output_units == OutputUnits::NORMALIZED) {
                    cmd.linear_velocity = linear_velocity / constraints.max_linear_velocity;
                    cmd.angular_velocity = angular_velocity / constraints.max_angular_velocity;
                } else {
                    cmd.linear_velocity = linear_velocity;
                    cmd.angular_velocity = angular_velocity;
                }

                cmd.valid = true;
                cmd.status_message = "LQR tracking";

                return cmd;
            }

            inline std::string get_type() const override { return "lqr_follower"; }

          private:
            double previous_lateral_error_ = 0.0;
            double previous_heading_error_ = 0.0;

            struct PathError {
                size_t nearest_index;
                double lateral_error;
                double heading_error;
                double path_curvature;
                Point nearest_point;
                double path_heading;
            };

            inline PathError calculate_path_error(const RobotState &current_state) {
                PathError result;

                double min_distance = std::numeric_limits<double>::max();
                size_t nearest_idx = path_index_;

                for (size_t i = path_index_; i < path_.drivekits.size(); ++i) {
                    double dist = current_state.pose.point.distance_to(path_.drivekits[i].point);
                    if (dist < min_distance) {
                        min_distance = dist;
                        nearest_idx = i;
                    }
                    if (i > path_index_ && dist > min_distance * 1.5) {
                        break;
                    }
                }

                result.nearest_index = nearest_idx;
                result.nearest_point = path_.drivekits[nearest_idx].point;

                path_index_ = nearest_idx;

                if (nearest_idx < path_.drivekits.size() - 1) {
                    Point next_point = path_.drivekits[nearest_idx + 1].point;
                    result.path_heading =
                        std::atan2(next_point.y - result.nearest_point.y, next_point.x - result.nearest_point.x);
                } else {
                    result.path_heading = path_.drivekits[nearest_idx].rotation.to_euler().yaw;
                }

                double dx = current_state.pose.point.x - result.nearest_point.x;
                double dy = current_state.pose.point.y - result.nearest_point.y;

                result.lateral_error = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);

                result.heading_error =
                    normalize_angle(current_state.pose.rotation.to_euler().yaw - result.path_heading);

                if (nearest_idx > 0 && nearest_idx < path_.drivekits.size() - 1) {
                    Point prev = path_.drivekits[nearest_idx - 1].point;
                    Point curr = path_.drivekits[nearest_idx].point;
                    Point next = path_.drivekits[nearest_idx + 1].point;

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
        };

    } // namespace path
} // namespace drivekit
