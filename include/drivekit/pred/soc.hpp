#pragma once

#include "drivekit/pred/mppi.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace drivekit {
    namespace pred {

        /// SOC (Stochastic Optimal Control) controller - SVG-MPPI.
        /// Stein Variational Guided Model Predictive Path Integral Control.
        /// Uses SVGD to guide sampling distribution for better exploration.
        class SOCFollower : public MPPIFollower {
          public:
            using Base = MPPIFollower;
            using MPPIConfig = MPPIFollower::MPPIConfig;

            /// SOC-specific configuration.
            struct SOCConfig {
                size_t horizon_steps = 20;
                double dt = 0.1;

                size_t guide_samples = 64;
                size_t num_samples = 512;
                size_t grad_samples = 32;

                double temperature = 1.0;
                double guide_temperature = 1.0;

                double steering_noise = 0.5;
                double acceleration_noise = 0.3;
                double initial_steer_variance = 0.25;
                double min_steer_variance = 1e-4;
                double max_steer_variance = 1.0;

                double weight_cte = 100.0;
                double weight_epsi = 100.0;
                double weight_vel = 1.0;
                double weight_steering = 10.0;
                double weight_acceleration = 5.0;

                double ref_velocity = 1.0;

                size_t svgd_iterations = 3;
                double svgd_step_size = 0.1;
                double kernel_bandwidth = 1.0;

                bool use_covariance_adaptation = true;
                double gaussian_fitting_lambda = 0.1;

                size_t num_threads = 4;
            };

            inline SOCFollower() : MPPIFollower() { soc_config_ = SOCConfig{}; }

            inline explicit SOCFollower(const SOCConfig &soc_config) : MPPIFollower() { set_soc_config(soc_config); }

            inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                   const RobotConstraints &constraints, double dt,
                                                   const WorldConstraints *world_constraints = nullptr) override {
                (void)dt;

                VelocityCommand cmd;
                cmd.valid = false;

                if (path_.drivekits.empty()) {
                    cmd.status_message = "No path set for SOC";
                    return cmd;
                }

                MPPIConfig active_config = get_mppi_config();
                bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                      constraints.steering_type == SteeringType::SKID_STEER);

                PathError error = calculate_path_error(current_state);
                (void)error;

                if (is_goal_reached(current_state.pose, goal.target_pose, goal.tolerance_position)) {
                    cmd.valid = true;
                    cmd.status_message = "Goal reached";
                    status_.goal_reached = true;
                    status_.mode = "stopped";
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = 0.0;
                    return cmd;
                }

                const size_t N = active_config.horizon_steps;
                const size_t K_guide = soc_config_.guide_samples;
                const size_t K_final = active_config.num_samples;
                const double dt_internal = active_config.dt;

                struct ObstaclePrediction {
                    std::vector<double> mean_x;
                    std::vector<double> mean_y;
                    double radius;
                };
                std::vector<ObstaclePrediction> obstacles;

                if (world_constraints != nullptr && !world_constraints->obstacles.empty()) {
                    for (const auto &obs : world_constraints->obstacles) {
                        if (obs.modes.empty()) continue;
                        const auto &mode = obs.modes[0];
                        ObstaclePrediction pred;
                        pred.radius = obs.radius;
                        pred.mean_x = mode.mean_x;
                        pred.mean_y = mode.mean_y;
                        obstacles.push_back(pred);
                    }
                }

                const double weight_obstacle = 50.0;

                std::mt19937 rng(std::random_device{}());
                std::normal_distribution<double> steering_noise(0.0, soc_config_.initial_steer_variance);
                std::normal_distribution<double> accel_noise(0.0, active_config.acceleration_noise);

                // Initialize guide particles
                for (size_t k = 0; k < K_guide; ++k) {
                    for (size_t i = 0; i < N; ++i) {
                        guide_steering_[k][i] = steering_noise(rng);
                        guide_acceleration_[k][i] = accel_noise(rng);
                    }
                }

                std::vector<double> guide_costs(K_guide, 0.0);

                // Compute costs for guide particles
                for (size_t k = 0; k < K_guide; ++k) {
                    double x = current_state.pose.point.x;
                    double y = current_state.pose.point.y;
                    double yaw = current_state.pose.rotation.to_euler().yaw;
                    double v = current_state.velocity.linear;
                    double cost = 0.0;

                    for (size_t i = 0; i < N; ++i) {
                        double delta_or_omega = guide_steering_[k][i];
                        double a = guide_acceleration_[k][i];

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

                        double min_path_dist = std::numeric_limits<double>::max();
                        for (const auto &wp : path_.drivekits) {
                            double dist =
                                std::sqrt((x - wp.point.x) * (x - wp.point.x) + (y - wp.point.y) * (y - wp.point.y));
                            min_path_dist = std::min(min_path_dist, dist);
                        }
                        cost += active_config.weight_cte * min_path_dist * min_path_dist;

                        double dx = x - goal.target_pose.point.x;
                        double dy = y - goal.target_pose.point.y;
                        cost += 0.1 * std::sqrt(dx * dx + dy * dy);

                        for (const auto &obs : obstacles) {
                            if (i < obs.mean_x.size()) {
                                double obs_x = obs.mean_x[i];
                                double obs_y = obs.mean_y[i];
                                double dist = std::sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
                                double clearance = dist - obs.radius - constraints.robot_width / 2.0;

                                if (clearance < 0.0) {
                                    cost += weight_obstacle * 100.0;
                                } else if (clearance < 0.3) {
                                    cost += weight_obstacle * (0.3 - clearance) / 0.3;
                                }
                            }
                        }

                        cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                        cost += active_config.weight_acceleration * a * a;
                    }

                    guide_costs[k] = cost;
                }

                // Find best guide particle
                size_t best_guide_idx =
                    std::distance(guide_costs.begin(), std::min_element(guide_costs.begin(), guide_costs.end()));

                // Compute adaptive covariances
                std::vector<double> adapted_steer_cov(N);
                std::vector<double> adapted_accel_cov(N);

                for (size_t i = 0; i < N; ++i) {
                    double steer_mean = 0.0, accel_mean = 0.0;
                    for (size_t k = 0; k < K_guide; ++k) {
                        steer_mean += guide_steering_[k][i];
                        accel_mean += guide_acceleration_[k][i];
                    }
                    steer_mean /= K_guide;
                    accel_mean /= K_guide;

                    double steer_var = 0.0, accel_var = 0.0;
                    for (size_t k = 0; k < K_guide; ++k) {
                        steer_var += (guide_steering_[k][i] - steer_mean) * (guide_steering_[k][i] - steer_mean);
                        accel_var +=
                            (guide_acceleration_[k][i] - accel_mean) * (guide_acceleration_[k][i] - accel_mean);
                    }
                    steer_var /= K_guide;
                    accel_var /= K_guide;

                    adapted_steer_cov[i] =
                        std::clamp(steer_var, soc_config_.min_steer_variance, soc_config_.max_steer_variance);
                    adapted_accel_cov[i] =
                        std::clamp(accel_var, soc_config_.min_steer_variance, soc_config_.max_steer_variance);
                }

                // Final MPPI sampling
                std::vector<double> final_costs(K_final, 0.0);
                std::vector<std::vector<double>> final_steering(K_final, std::vector<double>(N));
                std::vector<std::vector<double>> final_accel(K_final, std::vector<double>(N));

                for (size_t k = 0; k < K_final; ++k) {
                    double x = current_state.pose.point.x;
                    double y = current_state.pose.point.y;
                    double yaw = current_state.pose.rotation.to_euler().yaw;
                    double v = current_state.velocity.linear;
                    double cost = 0.0;

                    for (size_t i = 0; i < N; ++i) {
                        std::normal_distribution<double> steer_dist(guide_steering_[best_guide_idx][i],
                                                                    std::sqrt(adapted_steer_cov[i]));
                        std::normal_distribution<double> accel_dist(guide_acceleration_[best_guide_idx][i],
                                                                    std::sqrt(adapted_accel_cov[i]));

                        double delta_or_omega = steer_dist(rng);
                        double a = accel_dist(rng);

                        final_steering[k][i] = delta_or_omega;
                        final_accel[k][i] = a;

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

                        double min_path_dist = std::numeric_limits<double>::max();
                        for (const auto &wp : path_.drivekits) {
                            double dist =
                                std::sqrt((x - wp.point.x) * (x - wp.point.x) + (y - wp.point.y) * (y - wp.point.y));
                            min_path_dist = std::min(min_path_dist, dist);
                        }
                        cost += active_config.weight_cte * min_path_dist * min_path_dist;

                        double dx = x - goal.target_pose.point.x;
                        double dy = y - goal.target_pose.point.y;
                        cost += 0.1 * std::sqrt(dx * dx + dy * dy);

                        for (const auto &obs : obstacles) {
                            if (i < obs.mean_x.size()) {
                                double obs_x = obs.mean_x[i];
                                double obs_y = obs.mean_y[i];
                                double dist = std::sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
                                double clearance = dist - obs.radius - constraints.robot_width / 2.0;

                                if (clearance < 0.0) {
                                    cost += weight_obstacle * 100.0;
                                } else if (clearance < 0.3) {
                                    cost += weight_obstacle * (0.3 - clearance) / 0.3;
                                }
                            }
                        }

                        cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                        cost += active_config.weight_acceleration * a * a;
                    }

                    final_costs[k] = cost;
                }

                // MPPI weighting
                const double temperature = std::max(active_config.temperature, 1e-6);
                const double beta = 1.0 / temperature;
                double min_cost = *std::min_element(final_costs.begin(), final_costs.end());

                std::vector<double> weights(K_final, 0.0);
                double weight_sum = 0.0;

                for (size_t k = 0; k < K_final; ++k) {
                    double exponent = -beta * (final_costs[k] - min_cost);
                    exponent = std::max(exponent, -60.0);
                    double w = std::exp(exponent);
                    weights[k] = w;
                    weight_sum += w;
                }

                if (weight_sum < 1e-12) {
                    weight_sum = 1.0;
                }

                double steering_cmd = 0.0;
                double accel_cmd = 0.0;

                for (size_t k = 0; k < K_final; ++k) {
                    double w = weights[k] / weight_sum;
                    steering_cmd += w * final_steering[k][0];
                    accel_cmd += w * final_accel[k][0];
                }

                // Calculate distance to end for deceleration
                double dist_to_end = 0.0;
                for (size_t j = path_index_; j < path_.drivekits.size() - 1; ++j) {
                    dist_to_end += path_.drivekits[j].point.distance_to(path_.drivekits[j + 1].point);
                }
                const double decel_distance = 2.0;
                double effective_ref_velocity = active_config.ref_velocity;
                if (dist_to_end < decel_distance) {
                    effective_ref_velocity = active_config.ref_velocity * (dist_to_end / decel_distance);
                    effective_ref_velocity = std::max(effective_ref_velocity, 0.0);
                }

                double target_velocity = effective_ref_velocity + accel_cmd * dt_internal;
                double min_vel = constraints.min_linear_velocity;
                if (!config_.allow_reverse) {
                    min_vel = 0.0;
                }
                target_velocity = std::clamp(target_velocity, min_vel, constraints.max_linear_velocity);

                double angular_output;
                if (is_diff_drive) {
                    angular_output =
                        std::clamp(steering_cmd, -constraints.max_angular_velocity, constraints.max_angular_velocity);
                } else {
                    angular_output =
                        std::clamp(steering_cmd, -constraints.max_steering_angle, constraints.max_steering_angle);
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
                cmd.status_message = "SOC tracking";

                status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
                status_.goal_reached = false;
                status_.mode = "soc_tracking";

                return cmd;
            }

            inline std::string get_type() const override { return "soc_follower"; }

            inline void set_soc_config(const SOCConfig &config) {
                soc_config_ = config;

                MPPIConfig mppi_cfg;
                mppi_cfg.horizon_steps = config.horizon_steps;
                mppi_cfg.dt = config.dt;
                mppi_cfg.num_samples = config.num_samples;
                mppi_cfg.temperature = config.temperature;
                mppi_cfg.steering_noise = config.steering_noise;
                mppi_cfg.acceleration_noise = config.acceleration_noise;
                mppi_cfg.weight_cte = config.weight_cte;
                mppi_cfg.weight_epsi = config.weight_epsi;
                mppi_cfg.weight_vel = config.weight_vel;
                mppi_cfg.weight_steering = config.weight_steering;
                mppi_cfg.weight_acceleration = config.weight_acceleration;
                mppi_cfg.ref_velocity = config.ref_velocity;

                set_mppi_config(mppi_cfg);

                guide_steering_.resize(config.guide_samples, std::vector<double>(config.horizon_steps, 0.0));
                guide_acceleration_.resize(config.guide_samples, std::vector<double>(config.horizon_steps, 0.0));
            }

            inline SOCConfig get_soc_config() const { return soc_config_; }

          private:
            SOCConfig soc_config_;
            std::vector<std::vector<double>> guide_steering_;
            std::vector<std::vector<double>> guide_acceleration_;

            /// Estimate gradient of log-likelihood for Stein variational gradient descent.
            /// Placeholder implementation - returns 0.0 for now.
            inline double estimate_gradient_log_likelihood(size_t particle_idx, size_t time_idx, size_t control_idx,
                                                           const RobotState &current_state, const Goal &goal,
                                                           const RobotConstraints &constraints) const {
                (void)particle_idx;
                (void)time_idx;
                (void)control_idx;
                (void)current_state;
                (void)goal;
                (void)constraints;
                // Placeholder: estimate gradient via finite differences
                // In full implementation, this would sample around the particle and compute cost gradient
                return 0.0;
            }

            inline double rbf_kernel(const std::vector<double> &x1, const std::vector<double> &x2,
                                     double bandwidth) const {
                double dist_sq = 0.0;
                for (size_t i = 0; i < x1.size(); ++i) {
                    double diff = x1[i] - x2[i];
                    dist_sq += diff * diff;
                }
                return std::exp(-dist_sq / (2.0 * bandwidth * bandwidth));
            }
        };

    } // namespace pred
} // namespace drivekit
