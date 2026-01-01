#pragma once

#include "drivekit/controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace drivekit {
    namespace pred {

        /// MPC (Model Predictive Control) controller for optimal path tracking.
        /// Uses a lightweight optimizer (projected gradient descent) for real-time performance.
        class MPCFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            /// MPC-specific configuration.
            struct MPCConfig {
                // Prediction horizon
                size_t horizon_steps = 10;
                double dt = 0.1;

                // Cost function weights
                double weight_cte = 100.0;
                double weight_epsi = 100.0;
                double weight_vel = 1.0;
                double weight_steering = 10.0;
                double weight_acceleration = 5.0;
                double weight_steering_rate = 500.0;
                double weight_acceleration_rate = 50.0;

                // Reference velocity
                double ref_velocity = 1.0;

                // Turn-first behavior parameters
                double turn_first_activation_deg = 60.0;
                double turn_first_release_deg = 15.0;

                // Solver settings
                double max_solver_time = 0.5;
                int print_level = 0;
            };

            /// Path error information.
            struct PathError {
                size_t nearest_index;
                double cte;
                double epsi;
                Point nearest_point;
                double path_heading;
            };

            /// Reference trajectory along the path.
            struct ReferenceTrajectory {
                std::vector<double> x;
                std::vector<double> y;
                std::vector<double> yaw;
                std::vector<double> velocity;
            };

            inline MPCFollower() : MPCFollower(MPCConfig{}) {}

            inline explicit MPCFollower(const MPCConfig &mpc_config)
                : mpc_config_(mpc_config), is_turning_in_place_(false) {
                previous_steering_.resize(mpc_config_.horizon_steps, 0.0);
                previous_acceleration_.resize(mpc_config_.horizon_steps, 0.0);
            }

            inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                   const RobotConstraints &constraints, double dt,
                                                   const WorldConstraints *world_constraints = nullptr) override {
                (void)world_constraints;
                (void)dt;

                VelocityCommand cmd;
                cmd.valid = false;

                if (path_.drivekits.empty()) {
                    return cmd;
                }

                PathError error = calculate_path_error(current_state);

                MPCConfig working_config = mpc_config_;
                bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                      constraints.steering_type == SteeringType::SKID_STEER);

                if (current_state.turn_first && is_diff_drive) {
                    const double activation_threshold_rad = mpc_config_.turn_first_activation_deg * M_PI / 180.0;
                    const double release_threshold_rad = mpc_config_.turn_first_release_deg * M_PI / 180.0;
                    const double heading_error_abs = std::abs(error.epsi);

                    if (!is_turning_in_place_) {
                        if (heading_error_abs > activation_threshold_rad) {
                            is_turning_in_place_ = true;
                        }
                    } else {
                        if (heading_error_abs < release_threshold_rad) {
                            is_turning_in_place_ = false;
                        }
                    }

                    if (is_turning_in_place_) {
                        working_config.ref_velocity = 0.2;
                        working_config.weight_vel = 50.0;
                    }
                } else {
                    is_turning_in_place_ = false;
                }

                const auto &active_config = working_config;

                if (is_goal_reached(current_state.pose, goal.target_pose)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = 0.0;
                    return cmd;
                }

                ReferenceTrajectory ref_traj = calculate_reference_trajectory(error, current_state, constraints);

                MPCSolution solution = solve_mpc(current_state, ref_traj, constraints, active_config);
                if (!solution.success) {
                    cmd.valid = false;
                    cmd.status_message = "MPC optimizer failed";
                    return cmd;
                }

                const size_t N = mpc_config_.horizon_steps;
                if (previous_steering_.size() != N) {
                    previous_steering_.assign(N, 0.0);
                }
                if (previous_acceleration_.size() != N) {
                    previous_acceleration_.assign(N, 0.0);
                }

                if (!solution.steering_sequence.empty() && solution.steering_sequence.size() == N) {
                    for (size_t i = 0; i + 1 < N; ++i) {
                        previous_steering_[i] = solution.steering_sequence[i + 1];
                        previous_acceleration_[i] = solution.acceleration_sequence[i + 1];
                    }
                    previous_steering_[N - 1] = solution.steering_sequence.back();
                    previous_acceleration_[N - 1] = solution.acceleration_sequence.back();
                }

                predicted_trajectory_.clear();
                for (size_t i = 0; i < solution.predicted_x.size(); ++i) {
                    Point p;
                    p.x = solution.predicted_x[i];
                    p.y = solution.predicted_y[i];
                    p.z = 0.0;
                    predicted_trajectory_.push_back(p);
                }

                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.cross_track_error = std::abs(error.cte);
                status_.heading_error = std::abs(error.epsi);
                status_.goal_reached = false;
                status_.mode = "mpc_tracking";

                double target_velocity = active_config.ref_velocity + solution.acceleration * active_config.dt;
                double min_vel = constraints.min_linear_velocity;
                if (!config_.allow_reverse) {
                    min_vel = 0.0;
                }
                target_velocity = std::clamp(target_velocity, min_vel, constraints.max_linear_velocity);

                double angular_output;
                if (is_diff_drive) {
                    angular_output = std::clamp(solution.steering, -constraints.max_angular_velocity,
                                                constraints.max_angular_velocity);
                } else {
                    angular_output =
                        std::clamp(solution.steering, -constraints.max_steering_angle, constraints.max_steering_angle);
                }

                if (config_.output_units == OutputUnits::NORMALIZED) {
                    cmd.linear_velocity = (constraints.max_linear_velocity > 0.0)
                                              ? target_velocity / constraints.max_linear_velocity
                                              : 0.0;
                    if (is_diff_drive) {
                        cmd.angular_velocity = (constraints.max_angular_velocity > 0.0)
                                                   ? angular_output / constraints.max_angular_velocity
                                                   : 0.0;
                    } else {
                        cmd.angular_velocity = (constraints.max_steering_angle > 0.0)
                                                   ? angular_output / constraints.max_steering_angle
                                                   : 0.0;
                    }
                } else {
                    cmd.linear_velocity = target_velocity;
                    cmd.angular_velocity = angular_output;
                }

                cmd.valid = true;
                cmd.status_message = "MPC tracking";

                if (is_turning_in_place_) {
                    cmd.linear_velocity = 0.0;
                }

                return cmd;
            }

            inline std::string get_type() const override { return "mpc_follower"; }

            inline void set_mpc_config(const MPCConfig &config) {
                mpc_config_ = config;
                previous_steering_.resize(config.horizon_steps, 0.0);
                previous_acceleration_.resize(config.horizon_steps, 0.0);
            }

            inline MPCConfig get_mpc_config() const { return mpc_config_; }

            inline const std::vector<Point> &get_predicted_trajectory() const { return predicted_trajectory_; }

          private:
            MPCConfig mpc_config_;
            std::vector<double> previous_steering_;
            std::vector<double> previous_acceleration_;
            std::vector<Point> predicted_trajectory_;
            bool is_turning_in_place_ = false;

            struct MPCSolution {
                bool success;
                double steering;
                double acceleration;
                std::vector<double> predicted_x;
                std::vector<double> predicted_y;
                std::vector<double> steering_sequence;
                std::vector<double> acceleration_sequence;
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

                if (nearest_idx < path_.drivekits.size() - 1) {
                    Point next_point = path_.drivekits[nearest_idx + 1].point;
                    result.path_heading =
                        std::atan2(next_point.y - result.nearest_point.y, next_point.x - result.nearest_point.x);
                } else {
                    result.path_heading = path_.drivekits[nearest_idx].rotation.to_euler().yaw;
                }

                double dx = current_state.pose.point.x - result.nearest_point.x;
                double dy = current_state.pose.point.y - result.nearest_point.y;
                result.cte = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);

                result.epsi = normalize_angle(current_state.pose.rotation.to_euler().yaw - result.path_heading);

                path_index_ = nearest_idx;

                return result;
            }

            inline ReferenceTrajectory calculate_reference_trajectory(const PathError &error,
                                                                      const RobotState &current_state,
                                                                      const RobotConstraints &constraints) {
                (void)current_state;
                (void)constraints;

                ReferenceTrajectory ref;
                size_t start_idx = error.nearest_index;

                for (size_t i = 0; i < mpc_config_.horizon_steps + 1; ++i) {
                    double distance_ahead = mpc_config_.ref_velocity * mpc_config_.dt * i;

                    size_t target_idx = start_idx;
                    double accumulated_dist = 0.0;

                    while (target_idx < path_.drivekits.size() - 1 && accumulated_dist < distance_ahead) {
                        accumulated_dist +=
                            path_.drivekits[target_idx].point.distance_to(path_.drivekits[target_idx + 1].point);
                        if (accumulated_dist < distance_ahead) {
                            target_idx++;
                        }
                    }

                    target_idx = std::min(target_idx, path_.drivekits.size() - 1);

                    ref.x.push_back(path_.drivekits[target_idx].point.x);
                    ref.y.push_back(path_.drivekits[target_idx].point.y);

                    double yaw;
                    if (target_idx < path_.drivekits.size() - 1) {
                        Point next = path_.drivekits[target_idx + 1].point;
                        Point curr = path_.drivekits[target_idx].point;
                        yaw = std::atan2(next.y - curr.y, next.x - curr.x);
                    } else {
                        yaw = path_.drivekits[target_idx].rotation.to_euler().yaw;
                    }
                    ref.yaw.push_back(yaw);

                    ref.velocity.push_back(mpc_config_.ref_velocity);
                }

                return ref;
            }

            inline MPCSolution solve_mpc(const RobotState &current_state, const ReferenceTrajectory &ref_trajectory,
                                         const RobotConstraints &constraints, const MPCConfig &config) {
                MPCSolution sol;
                sol.success = false;

                const size_t horizon_steps = config.horizon_steps;
                if (horizon_steps == 0) {
                    return sol;
                }

                const size_t num_vars = horizon_steps * 2;
                std::vector<double> u(num_vars, 0.0);

                bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                      constraints.steering_type == SteeringType::SKID_STEER);

                // Initialize from previous solution
                for (size_t i = 0; i < horizon_steps; ++i) {
                    double steering = (i < previous_steering_.size()) ? previous_steering_[i] : 0.0;
                    double accel = (i < previous_acceleration_.size()) ? previous_acceleration_[i] : 0.0;
                    u[2 * i] = steering;
                    u[2 * i + 1] = accel;
                }

                auto clamp_controls = [&](std::vector<double> &ctrl) {
                    for (size_t i = 0; i < horizon_steps; ++i) {
                        if (is_diff_drive) {
                            ctrl[2 * i] = std::clamp(ctrl[2 * i], -constraints.max_angular_velocity,
                                                     constraints.max_angular_velocity);
                        } else {
                            ctrl[2 * i] = std::clamp(ctrl[2 * i], -constraints.max_steering_angle,
                                                     constraints.max_steering_angle);
                        }
                        ctrl[2 * i + 1] = std::clamp(ctrl[2 * i + 1], -constraints.max_linear_acceleration,
                                                     constraints.max_linear_acceleration);
                    }
                };

                auto simulate_and_cost = [&](const std::vector<double> &ctrl, std::vector<double> *out_x,
                                             std::vector<double> *out_y) -> double {
                    const double dt = config.dt;
                    const double Lf = constraints.wheelbase;

                    double x = current_state.pose.point.x;
                    double y = current_state.pose.point.y;
                    double yaw = current_state.pose.rotation.to_euler().yaw;
                    double v = current_state.velocity.linear;

                    if (out_x && out_y) {
                        out_x->clear();
                        out_y->clear();
                        out_x->push_back(x);
                        out_y->push_back(y);
                    }

                    double cost = 0.0;

                    for (size_t i = 0; i < horizon_steps; ++i) {
                        double steering_or_omega = ctrl[2 * i];
                        double accel = ctrl[2 * i + 1];

                        x += v * std::cos(yaw) * dt;
                        y += v * std::sin(yaw) * dt;
                        if (is_diff_drive) {
                            yaw += steering_or_omega * dt;
                        } else {
                            yaw += v * steering_or_omega / Lf * dt;
                        }
                        while (yaw > M_PI) yaw -= 2.0 * M_PI;
                        while (yaw < -M_PI) yaw += 2.0 * M_PI;
                        v += accel * dt;
                        v = std::clamp(v, constraints.min_linear_velocity, constraints.max_linear_velocity);

                        if (out_x && out_y) {
                            out_x->push_back(x);
                            out_y->push_back(y);
                        }

                        size_t ref_idx =
                            std::min(i + 1, ref_trajectory.x.empty() ? size_t(0) : (ref_trajectory.x.size() - 1));
                        double ref_x = ref_trajectory.x.empty() ? x : ref_trajectory.x[ref_idx];
                        double ref_y = ref_trajectory.y.empty() ? y : ref_trajectory.y[ref_idx];
                        double ref_yaw = ref_trajectory.yaw.empty() ? yaw : ref_trajectory.yaw[ref_idx];
                        double ref_v =
                            ref_trajectory.velocity.empty() ? config.ref_velocity : ref_trajectory.velocity[ref_idx];

                        double dx = x - ref_x;
                        double dy = y - ref_y;
                        double cte = -dx * std::sin(ref_yaw) + dy * std::cos(ref_yaw);
                        double epsi = yaw - ref_yaw;
                        while (epsi > M_PI) epsi -= 2.0 * M_PI;
                        while (epsi < -M_PI) epsi += 2.0 * M_PI;
                        double vel_error = v - ref_v;

                        cost += config.weight_cte * cte * cte;
                        cost += config.weight_epsi * epsi * epsi;
                        cost += config.weight_vel * vel_error * vel_error;
                        cost += config.weight_steering * steering_or_omega * steering_or_omega;
                        cost += config.weight_acceleration * accel * accel;

                        if (i > 0) {
                            double prev_steering_or_omega = ctrl[2 * (i - 1)];
                            double prev_accel = ctrl[2 * (i - 1) + 1];
                            double d_steer = steering_or_omega - prev_steering_or_omega;
                            double d_acc = accel - prev_accel;
                            cost += config.weight_steering_rate * d_steer * d_steer;
                            cost += config.weight_acceleration_rate * d_acc * d_acc;
                        }
                    }

                    return cost;
                };

                auto smooth_controls = [&](std::vector<double> &ctrl) {
                    if (horizon_steps < 3) {
                        return;
                    }

                    // Apply a simple Savitzky-Golay-like smoothing (length-5 window)
                    std::vector<double> smoothed(ctrl.size());
                    for (int dim = 0; dim < 2; ++dim) {
                        for (size_t i = 0; i < horizon_steps; ++i) {
                            // Indices for a 5-point window, clamped at ends
                            size_t idx_m2 = (i >= 2) ? (i - 2) : 0;
                            size_t idx_m1 = (i >= 1) ? (i - 1) : 0;
                            size_t idx_0 = i;
                            size_t idx_p1 = (i + 1 < horizon_steps) ? (i + 1) : (horizon_steps - 1);
                            size_t idx_p2 = (i + 2 < horizon_steps) ? (i + 2) : (horizon_steps - 1);

                            double y_m2 = ctrl[2 * idx_m2 + dim];
                            double y_m1 = ctrl[2 * idx_m1 + dim];
                            double y_0 = ctrl[2 * idx_0 + dim];
                            double y_p1 = ctrl[2 * idx_p1 + dim];
                            double y_p2 = ctrl[2 * idx_p2 + dim];

                            double smoothed_val =
                                (1.0 / 35.0) * (-3.0 * y_m2 + 12.0 * y_m1 + 17.0 * y_0 + 12.0 * y_p1 - 3.0 * y_p2);
                            smoothed[2 * i + dim] = smoothed_val;
                        }
                    }

                    // Copy back and clamp again
                    ctrl = smoothed;
                    clamp_controls(ctrl);
                };

                clamp_controls(u);
                smooth_controls(u);

                double best_cost = simulate_and_cost(u, nullptr, nullptr);
                if (!std::isfinite(best_cost)) {
                    return sol;
                }

                std::vector<double> best_u = u;
                std::vector<double> grad(num_vars, 0.0);

                const int max_iterations = 15;
                const double eps = 1e-3;
                const double base_step = 0.1;

                for (int iter = 0; iter < max_iterations; ++iter) {
                    // Finite difference gradient
                    for (size_t i = 0; i < num_vars; ++i) {
                        double orig = u[i];
                        u[i] = orig + eps;
                        clamp_controls(u);
                        double cost_plus = simulate_and_cost(u, nullptr, nullptr);
                        u[i] = orig - eps;
                        clamp_controls(u);
                        double cost_minus = simulate_and_cost(u, nullptr, nullptr);
                        grad[i] = (cost_plus - cost_minus) / (2.0 * eps);
                        u[i] = orig;
                    }

                    double grad_norm = 0.0;
                    for (double v : grad) {
                        grad_norm += v * v;
                    }
                    if (grad_norm < 1e-8) {
                        break;
                    }

                    double step = base_step;
                    bool improved = false;
                    std::vector<double> candidate(num_vars);

                    for (int ls = 0; ls < 5; ++ls) {
                        for (size_t i = 0; i < num_vars; ++i) {
                            candidate[i] = u[i] - step * grad[i];
                        }
                        clamp_controls(candidate);
                        smooth_controls(candidate);

                        double cand_cost = simulate_and_cost(candidate, nullptr, nullptr);
                        if (std::isfinite(cand_cost) && cand_cost < best_cost) {
                            best_cost = cand_cost;
                            best_u = candidate;
                            u = candidate;
                            improved = true;
                            break;
                        }

                        step *= 0.5;
                    }

                    if (!improved) {
                        break;
                    }
                }

                std::vector<double> traj_x;
                std::vector<double> traj_y;
                (void)simulate_and_cost(best_u, &traj_x, &traj_y);

                sol.success = true;
                sol.steering = best_u[0];
                sol.acceleration = best_u[1];
                sol.predicted_x = std::move(traj_x);
                sol.predicted_y = std::move(traj_y);

                sol.steering_sequence.resize(horizon_steps);
                sol.acceleration_sequence.resize(horizon_steps);
                for (size_t i = 0; i < horizon_steps; ++i) {
                    sol.steering_sequence[i] = best_u[2 * i];
                    sol.acceleration_sequence[i] = best_u[2 * i + 1];
                }

                return sol;
            }
        };

    } // namespace pred
} // namespace drivekit
