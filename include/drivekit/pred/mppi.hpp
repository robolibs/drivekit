#pragma once

#include "drivekit/controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace drivekit {
    namespace pred {

        /// MPPI (Model Predictive Path Integral) - Sampling-based optimal control.
        /// Uses importance sampling to find optimal control sequences.
        class MPPIFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            /// MPPI-specific configuration.
            struct MPPIConfig {
                // Prediction horizon
                size_t horizon_steps = 20;
                double dt = 0.1;

                // Sampling settings
                size_t num_samples = 1000;
                double temperature = 1.0;
                double steering_noise = 0.5;
                double acceleration_noise = 0.3;

                // Cost weights
                double weight_cte = 100.0;
                double weight_epsi = 100.0;
                double weight_vel = 1.0;
                double weight_steering = 10.0;
                double weight_acceleration = 5.0;

                // Reference velocity
                double ref_velocity = 1.0;

                // Turn-first behavior parameters
                double turn_first_activation_deg = 60.0;
                double turn_first_release_deg = 15.0;

                size_t num_threads = 4;
            };

            /// Path error information.
            struct PathError {
                size_t nearest_index = 0;
                double cte = 0.0;
                double epsi = 0.0;
                Point nearest_point{};
                double path_heading = 0.0;
            };

            /// Reference trajectory along the path.
            struct ReferenceTrajectory {
                std::vector<double> x;
                std::vector<double> y;
                std::vector<double> yaw;
                std::vector<double> velocity;
            };

            inline MPPIFollower() : MPPIFollower(MPPIConfig{}) {}

            inline explicit MPPIFollower(const MPPIConfig &mppi_config)
                : mppi_config_(mppi_config), rng_(std::random_device{}()), is_turning_in_place_(false) {
                mean_steering_.assign(mppi_config_.horizon_steps, 0.0);
                mean_acceleration_.assign(mppi_config_.horizon_steps, 0.0);
            }

            inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                   const RobotConstraints &constraints, double dt,
                                                   const WorldConstraints *world_constraints = nullptr) override {
                (void)world_constraints;
                (void)dt;

                VelocityCommand cmd;
                cmd.valid = false;

                if (path_.drivekits.empty()) {
                    cmd.status_message = "No path set for MPPI";
                    return cmd;
                }

                PathError error = calculate_path_error(current_state);

                MPPIConfig working_config = mppi_config_;
                bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                      constraints.steering_type == SteeringType::SKID_STEER);

                if (current_state.turn_first && is_diff_drive) {
                    const double activation_threshold_rad = mppi_config_.turn_first_activation_deg * M_PI / 180.0;
                    const double release_threshold_rad = mppi_config_.turn_first_release_deg * M_PI / 180.0;
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

                if (mean_steering_.size() != active_config.horizon_steps ||
                    mean_acceleration_.size() != active_config.horizon_steps) {
                    mean_steering_.assign(active_config.horizon_steps, 0.0);
                    mean_acceleration_.assign(active_config.horizon_steps, 0.0);
                }

                const size_t N = active_config.horizon_steps;
                const size_t K = active_config.num_samples;
                const double dt_internal = active_config.dt;

                ReferenceTrajectory ref_traj = calculate_reference_trajectory(error, current_state, constraints);

                std::normal_distribution<double> steering_noise_dist(0.0, active_config.steering_noise);
                std::normal_distribution<double> accel_noise_dist(0.0, active_config.acceleration_noise);

                std::vector<double> costs(K, 0.0);
                std::vector<std::vector<double>> noise_steering(K, std::vector<double>(N, 0.0));
                std::vector<std::vector<double>> noise_accel(K, std::vector<double>(N, 0.0));

                double best_cost = std::numeric_limits<double>::infinity();
                std::vector<Point> best_trajectory;

                for (size_t k = 0; k < K; ++k) {
                    double x = current_state.pose.point.x;
                    double y = current_state.pose.point.y;
                    double yaw = current_state.pose.rotation.to_euler().yaw;
                    double v = current_state.velocity.linear;

                    double sample_cost = 0.0;
                    std::vector<Point> trajectory;
                    trajectory.reserve(N + 1);

                    Point p0;
                    p0.x = x;
                    p0.y = y;
                    p0.z = 0.0;
                    trajectory.push_back(p0);

                    for (size_t i = 0; i < N; ++i) {
                        double eps_delta = steering_noise_dist(rng_);
                        double eps_acc = accel_noise_dist(rng_);
                        noise_steering[k][i] = eps_delta;
                        noise_accel[k][i] = eps_acc;

                        double delta_or_omega = mean_steering_[i] + eps_delta;
                        double a = mean_acceleration_[i] + eps_acc;

                        if (is_diff_drive) {
                            delta_or_omega = std::clamp(delta_or_omega, -constraints.max_angular_velocity,
                                                        constraints.max_angular_velocity);
                        } else {
                            delta_or_omega = std::clamp(delta_or_omega, -constraints.max_steering_angle,
                                                        constraints.max_steering_angle);
                        }
                        a = std::clamp(a, -constraints.max_linear_acceleration, constraints.max_linear_acceleration);

                        double Lf = constraints.wheelbase;

                        x += v * std::cos(yaw) * dt_internal;
                        y += v * std::sin(yaw) * dt_internal;
                        if (is_diff_drive) {
                            yaw += delta_or_omega * dt_internal;
                        } else {
                            yaw += v * delta_or_omega / Lf * dt_internal;
                        }
                        yaw = normalize_angle(yaw);
                        v += a * dt_internal;
                        v = std::clamp(v, constraints.min_linear_velocity, constraints.max_linear_velocity);

                        Point p;
                        p.x = x;
                        p.y = y;
                        p.z = 0.0;
                        trajectory.push_back(p);

                        size_t ref_idx = std::min(i + 1, ref_traj.x.size() - 1);
                        double ref_x = ref_traj.x[ref_idx];
                        double ref_y = ref_traj.y[ref_idx];
                        double ref_yaw = ref_traj.yaw[ref_idx];
                        double ref_v = ref_traj.velocity[ref_idx];

                        double dx = x - ref_x;
                        double dy = y - ref_y;
                        double cte = -dx * std::sin(ref_yaw) + dy * std::cos(ref_yaw);
                        double epsi = normalize_angle(yaw - ref_yaw);
                        double vel_error = v - ref_v;

                        double cost = 0.0;
                        cost += active_config.weight_cte * cte * cte;
                        cost += active_config.weight_epsi * epsi * epsi;
                        cost += active_config.weight_vel * vel_error * vel_error;
                        cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                        cost += active_config.weight_acceleration * a * a;

                        sample_cost += cost * dt_internal;
                    }

                    costs[k] = sample_cost;

                    if (sample_cost < best_cost) {
                        best_cost = sample_cost;
                        best_trajectory = std::move(trajectory);
                    }
                }

                const double temperature = std::max(active_config.temperature, 1e-6);
                const double beta = 1.0 / temperature;

                double min_cost = *std::min_element(costs.begin(), costs.end());
                std::vector<double> weights(K, 0.0);
                double weight_sum = 0.0;

                for (size_t k = 0; k < K; ++k) {
                    double exponent = -beta * (costs[k] - min_cost);
                    exponent = std::max(exponent, -60.0);
                    double w = std::exp(exponent);
                    weights[k] = w;
                    weight_sum += w;
                }

                if (weight_sum < 1e-12) {
                    weight_sum = 1.0;
                }

                for (size_t i = 0; i < N; ++i) {
                    double d_delta = 0.0;
                    double d_acc = 0.0;

                    for (size_t k = 0; k < K; ++k) {
                        double w = weights[k] / weight_sum;
                        d_delta += w * noise_steering[k][i];
                        d_acc += w * noise_accel[k][i];
                    }

                    mean_steering_[i] += d_delta;
                    mean_acceleration_[i] += d_acc;

                    if (is_diff_drive) {
                        mean_steering_[i] = std::clamp(mean_steering_[i], -constraints.max_angular_velocity,
                                                       constraints.max_angular_velocity);
                    } else {
                        mean_steering_[i] = std::clamp(mean_steering_[i], -constraints.max_steering_angle,
                                                       constraints.max_steering_angle);
                    }
                    mean_acceleration_[i] = std::clamp(mean_acceleration_[i], -constraints.max_linear_acceleration,
                                                       constraints.max_linear_acceleration);
                }

                double steering_or_omega = mean_steering_.empty() ? 0.0 : mean_steering_.front();
                double acceleration = mean_acceleration_.empty() ? 0.0 : mean_acceleration_.front();

                if (!mean_steering_.empty()) {
                    for (size_t i = 0; i + 1 < N; ++i) {
                        mean_steering_[i] = mean_steering_[i + 1];
                        mean_acceleration_[i] = mean_acceleration_[i + 1];
                    }
                    mean_steering_[N - 1] = 0.0;
                    mean_acceleration_[N - 1] = 0.0;
                }

                predicted_trajectory_ = best_trajectory;

                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.goal_reached = false;
                status_.mode = "mppi_tracking";

                {
                    PathError cur_error = calculate_path_error(current_state);
                    status_.cross_track_error = std::abs(cur_error.cte);
                    status_.heading_error = std::abs(cur_error.epsi);
                }

                double target_velocity = active_config.ref_velocity + acceleration * dt_internal;

                double min_vel = constraints.min_linear_velocity;
                if (!config_.allow_reverse) {
                    min_vel = 0.0;
                }
                target_velocity = std::clamp(target_velocity, min_vel, constraints.max_linear_velocity);

                double angular_output;
                if (is_diff_drive) {
                    angular_output = std::clamp(steering_or_omega, -constraints.max_angular_velocity,
                                                constraints.max_angular_velocity);
                } else {
                    angular_output =
                        std::clamp(steering_or_omega, -constraints.max_steering_angle, constraints.max_steering_angle);
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
                cmd.status_message = "MPPI tracking";

                if (is_turning_in_place_) {
                    cmd.linear_velocity = 0.0;
                }

                return cmd;
            }

            inline std::string get_type() const override { return "mppi_follower"; }

            inline void set_mppi_config(const MPPIConfig &config) {
                mppi_config_ = config;
                mean_steering_.assign(mppi_config_.horizon_steps, 0.0);
                mean_acceleration_.assign(mppi_config_.horizon_steps, 0.0);
            }

            inline MPPIConfig get_mppi_config() const { return mppi_config_; }

            inline const std::vector<Point> &get_predicted_trajectory() const { return predicted_trajectory_; }

          protected:
            MPPIConfig mppi_config_;
            std::mt19937 rng_;
            std::vector<double> mean_steering_;
            std::vector<double> mean_acceleration_;
            std::vector<Point> predicted_trajectory_;
            bool is_turning_in_place_ = false;

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

                for (size_t i = 0; i < mppi_config_.horizon_steps + 1; ++i) {
                    double distance_ahead = mppi_config_.ref_velocity * mppi_config_.dt * i;

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

                    double ref_vel = mppi_config_.ref_velocity;
                    double dist_to_end = 0.0;
                    for (size_t j = target_idx; j < path_.drivekits.size() - 1; ++j) {
                        dist_to_end += path_.drivekits[j].point.distance_to(path_.drivekits[j + 1].point);
                    }
                    const double decel_distance = 2.0;
                    if (dist_to_end < decel_distance) {
                        ref_vel = mppi_config_.ref_velocity * (dist_to_end / decel_distance);
                        ref_vel = std::max(ref_vel, 0.0);
                    }
                    ref.velocity.push_back(ref_vel);
                }

                return ref;
            }
        };

    } // namespace pred
} // namespace drivekit
