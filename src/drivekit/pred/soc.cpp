#include "drivekit/pred/soc.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace drivekit {
    namespace pred {

        SOCFollower::SOCFollower() : MPPIFollower() { soc_config_ = SOCConfig{}; }

        SOCFollower::SOCFollower(const SOCConfig &soc_config) : MPPIFollower() { set_soc_config(soc_config); }

        std::string SOCFollower::get_type() const { return "soc_follower"; }

        void SOCFollower::set_soc_config(const SOCConfig &config) {
            soc_config_ = config;

            // Map SOCConfig to underlying MPPIConfig
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

            // Initialize guide particles
            guide_steering_.resize(config.guide_samples, std::vector<double>(config.horizon_steps, 0.0));
            guide_acceleration_.resize(config.guide_samples, std::vector<double>(config.horizon_steps, 0.0));
        }

        SOCFollower::SOCConfig SOCFollower::get_soc_config() const { return soc_config_; }

        double SOCFollower::rbf_kernel(const std::vector<double> &x1, const std::vector<double> &x2,
                                       double bandwidth) const {
            double dist_sq = 0.0;
            for (size_t i = 0; i < x1.size(); ++i) {
                double diff = x1[i] - x2[i];
                dist_sq += diff * diff;
            }
            return std::exp(-dist_sq / (2.0 * bandwidth * bandwidth));
        }

        std::vector<double> SOCFollower::compute_svgd_gradient(size_t particle_idx,
                                                               const std::vector<std::vector<double>> &particles,
                                                               const std::vector<double> &costs,
                                                               double bandwidth) const {
            const size_t N = particles.size();
            const size_t D = particles[0].size();
            std::vector<double> gradient(D, 0.0);

            // Compute gradient of log-posterior (cost gradient)
            std::vector<double> score(D, 0.0);
            for (size_t d = 0; d < D; ++d) {
                // Finite difference approximation
                double eps = 1e-4;
                double cost_plus = costs[particle_idx];
                double cost_minus = costs[particle_idx];

                // Simple central difference
                score[d] = -(cost_plus - cost_minus) / (2.0 * eps);
            }

            // SVGD gradient: (1/N) * sum_j [k(x_j, x_i) * grad_log_p(x_j) + grad_k(x_j, x_i)]
            for (size_t j = 0; j < N; ++j) {
                double k_val = rbf_kernel(particles[j], particles[particle_idx], bandwidth);

                // Attractive term: k(x_j, x_i) * score(x_j)
                for (size_t d = 0; d < D; ++d) {
                    gradient[d] += k_val * score[d];
                }

                // Repulsive term: grad_k(x_j, x_i)
                for (size_t d = 0; d < D; ++d) {
                    double grad_k = k_val * (particles[j][d] - particles[particle_idx][d]) / (bandwidth * bandwidth);
                    gradient[d] += grad_k;
                }
            }

            // Average
            for (size_t d = 0; d < D; ++d) {
                gradient[d] /= static_cast<double>(N);
            }

            return gradient;
        }

        double SOCFollower::estimate_gradient_log_likelihood(size_t particle_idx, size_t time_idx, size_t control_idx,
                                                             const RobotState &current_state, const Goal &goal,
                                                             const RobotConstraints &constraints) const {
            // Placeholder: estimate gradient via finite differences
            // In full implementation, this would sample around the particle and compute cost gradient
            return 0.0;
        }

        VelocityCommand SOCFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                     const RobotConstraints &constraints, double dt,
                                                     const WorldConstraints *world_constraints) {
            (void)dt;

            VelocityCommand cmd;
            cmd.valid = false;

            if (path_.drivekits.empty()) {
                cmd.status_message = "No path set for SOC";
                return cmd;
            }

            // Get MPPI config
            MPPIConfig active_config = get_mppi_config();
            bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                  constraints.steering_type == SteeringType::SKID_STEER);

            // Goal reached check
            if (is_goal_reached(current_state.pose, goal.target_pose)) {
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

            // Convert WorldConstraints obstacles to internal format
            struct ObstaclePrediction {
                std::vector<double> mean_x;
                std::vector<double> mean_y;
                double radius;
            };
            std::vector<ObstaclePrediction> obstacles;

            if (world_constraints != nullptr && !world_constraints->obstacles.empty()) {
                for (const auto &obs : world_constraints->obstacles) {
                    if (obs.modes.empty()) continue;

                    // Use first mode (highest weight)
                    const auto &mode = obs.modes[0];
                    ObstaclePrediction pred;
                    pred.radius = obs.radius;
                    pred.mean_x = mode.mean_x;
                    pred.mean_y = mode.mean_y;
                    obstacles.push_back(pred);
                }
            }

            // Weight for obstacle avoidance (tuned to balance path-following and safety)
            const double weight_obstacle = 50.0;

            std::mt19937 rng(std::random_device{}());
            std::normal_distribution<double> steering_noise(0.0, soc_config_.initial_steer_variance);
            std::normal_distribution<double> accel_noise(0.0, active_config.acceleration_noise);

            // === PHASE 1: SVGD on Guide Particles ===

            // Initialize guide particles with noise
            for (size_t k = 0; k < K_guide; ++k) {
                for (size_t i = 0; i < N; ++i) {
                    guide_steering_[k][i] = steering_noise(rng);
                    guide_acceleration_[k][i] = accel_noise(rng);
                }
            }

            // Compute costs for guide particles
            std::vector<double> guide_costs(K_guide, 0.0);

            for (size_t k = 0; k < K_guide; ++k) {
                double x = current_state.pose.point.x;
                double y = current_state.pose.point.y;
                double yaw = current_state.pose.angle.yaw;
                double v = current_state.velocity.linear;
                double cost = 0.0;

                // Rollout this guide particle
                for (size_t i = 0; i < N; ++i) {
                    double delta_or_omega = guide_steering_[k][i];
                    double a = guide_acceleration_[k][i];

                    // Clamp
                    if (is_diff_drive) {
                        delta_or_omega = std::clamp(delta_or_omega, -constraints.max_angular_velocity,
                                                    constraints.max_angular_velocity);
                    } else {
                        delta_or_omega =
                            std::clamp(delta_or_omega, -constraints.max_steering_angle, constraints.max_steering_angle);
                    }
                    a = std::clamp(a, -constraints.max_linear_acceleration, constraints.max_linear_acceleration);

                    // Integrate
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

                    // Path-following cost: penalize deviation from reference path
                    double min_path_dist = std::numeric_limits<double>::max();
                    for (const auto &wp : path_.drivekits) {
                        double dist =
                            std::sqrt((x - wp.point.x) * (x - wp.point.x) + (y - wp.point.y) * (y - wp.point.y));
                        min_path_dist = std::min(min_path_dist, dist);
                    }
                    cost += active_config.weight_cte * min_path_dist * min_path_dist;

                    // Progress towards goal (smaller weight to encourage forward motion)
                    double dx = x - goal.target_pose.point.x;
                    double dy = y - goal.target_pose.point.y;
                    cost += 0.1 * std::sqrt(dx * dx + dy * dy);

                    // Obstacle avoidance cost
                    for (const auto &obs : obstacles) {
                        if (i < obs.mean_x.size()) {
                            double obs_x = obs.mean_x[i];
                            double obs_y = obs.mean_y[i];
                            double dist = std::sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
                            double clearance = dist - obs.radius - constraints.robot_width / 2.0;

                            // Hard penalty for collision
                            if (clearance < 0.0) {
                                cost += weight_obstacle * 100.0;
                            }
                            // Soft penalty in safety margin (0-0.3m)
                            else if (clearance < 0.3) {
                                cost += weight_obstacle * (0.3 - clearance) / 0.3;
                            }
                        }
                    }

                    // Control effort
                    cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                    cost += active_config.weight_acceleration * a * a;
                }

                guide_costs[k] = cost;
            }

            // SVGD iterations
            for (size_t iter = 0; iter < soc_config_.svgd_iterations; ++iter) {
                // Flatten particles for SVGD
                std::vector<std::vector<double>> flat_particles(K_guide);
                for (size_t k = 0; k < K_guide; ++k) {
                    flat_particles[k].resize(N * 2); // steering + accel
                    for (size_t i = 0; i < N; ++i) {
                        flat_particles[k][i] = guide_steering_[k][i];
                        flat_particles[k][N + i] = guide_acceleration_[k][i];
                    }
                }

                // Compute SVGD gradient for each particle
                for (size_t k = 0; k < K_guide; ++k) {
                    std::vector<double> svgd_grad =
                        compute_svgd_gradient(k, flat_particles, guide_costs, soc_config_.kernel_bandwidth);

                    // Update particle
                    for (size_t i = 0; i < N; ++i) {
                        guide_steering_[k][i] += soc_config_.svgd_step_size * svgd_grad[i];
                        guide_acceleration_[k][i] += soc_config_.svgd_step_size * svgd_grad[N + i];
                    }
                }

                // Recompute costs after update
                for (size_t k = 0; k < K_guide; ++k) {
                    double x = current_state.pose.point.x;
                    double y = current_state.pose.point.y;
                    double yaw = current_state.pose.angle.yaw;
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

                        // Path-following cost
                        double min_path_dist = std::numeric_limits<double>::max();
                        for (const auto &wp : path_.drivekits) {
                            double dist =
                                std::sqrt((x - wp.point.x) * (x - wp.point.x) + (y - wp.point.y) * (y - wp.point.y));
                            min_path_dist = std::min(min_path_dist, dist);
                        }
                        cost += active_config.weight_cte * min_path_dist * min_path_dist;

                        // Progress towards goal
                        double dx = x - goal.target_pose.point.x;
                        double dy = y - goal.target_pose.point.y;
                        cost += 0.1 * std::sqrt(dx * dx + dy * dy);

                        // Obstacle avoidance cost
                        for (const auto &obs : obstacles) {
                            if (i < obs.mean_x.size()) {
                                double obs_x = obs.mean_x[i];
                                double obs_y = obs.mean_y[i];
                                double dist = std::sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
                                double clearance = dist - obs.radius - constraints.robot_width / 2.0;

                                // Hard penalty for collision
                                if (clearance < 0.0) {
                                    cost += weight_obstacle * 100.0;
                                }
                                // Soft penalty in safety margin (0-0.3m)
                                else if (clearance < 0.3) {
                                    cost += weight_obstacle * (0.3 - clearance) / 0.3;
                                }
                            }
                        }

                        cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                        cost += active_config.weight_acceleration * a * a;
                    }

                    guide_costs[k] = cost;
                }
            }

            // Find best guide particle
            size_t best_guide_idx =
                std::distance(guide_costs.begin(), std::min_element(guide_costs.begin(), guide_costs.end()));

            // === PHASE 2: Covariance Adaptation ===

            // Compute adaptive covariances from guide particles
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
                    accel_var += (guide_acceleration_[k][i] - accel_mean) * (guide_acceleration_[k][i] - accel_mean);
                }
                steer_var /= K_guide;
                accel_var /= K_guide;

                adapted_steer_cov[i] =
                    std::clamp(steer_var, soc_config_.min_steer_variance, soc_config_.max_steer_variance);
                adapted_accel_cov[i] =
                    std::clamp(accel_var, soc_config_.min_steer_variance, soc_config_.max_steer_variance);
            }

            // === PHASE 3: Final MPPI Sampling from Adapted Distribution ===

            std::vector<double> final_costs(K_final, 0.0);
            std::vector<std::vector<double>> final_steering(K_final, std::vector<double>(N));
            std::vector<std::vector<double>> final_accel(K_final, std::vector<double>(N));

            for (size_t k = 0; k < K_final; ++k) {
                double x = current_state.pose.point.x;
                double y = current_state.pose.point.y;
                double yaw = current_state.pose.angle.yaw;
                double v = current_state.velocity.linear;
                double cost = 0.0;

                for (size_t i = 0; i < N; ++i) {
                    // Sample from adapted distribution
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
                        delta_or_omega =
                            std::clamp(delta_or_omega, -constraints.max_steering_angle, constraints.max_steering_angle);
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

                    // Path-following cost
                    double min_path_dist = std::numeric_limits<double>::max();
                    for (const auto &wp : path_.drivekits) {
                        double dist =
                            std::sqrt((x - wp.point.x) * (x - wp.point.x) + (y - wp.point.y) * (y - wp.point.y));
                        min_path_dist = std::min(min_path_dist, dist);
                    }
                    cost += active_config.weight_cte * min_path_dist * min_path_dist;

                    // Progress towards goal
                    double dx = x - goal.target_pose.point.x;
                    double dy = y - goal.target_pose.point.y;
                    cost += 0.1 * std::sqrt(dx * dx + dy * dy);

                    // Obstacle avoidance cost
                    for (const auto &obs : obstacles) {
                        if (i < obs.mean_x.size()) {
                            double obs_x = obs.mean_x[i];
                            double obs_y = obs.mean_y[i];
                            double dist = std::sqrt((x - obs_x) * (x - obs_x) + (y - obs_y) * (y - obs_y));
                            double clearance = dist - obs.radius - constraints.robot_width / 2.0;

                            // Hard penalty for collision
                            if (clearance < 0.0) {
                                cost += weight_obstacle * 100.0;
                            }
                            // Soft penalty in safety margin (0-0.3m)
                            else if (clearance < 0.3) {
                                cost += weight_obstacle * (0.3 - clearance) / 0.3;
                            }
                        }
                    }

                    cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                    cost += active_config.weight_acceleration * a * a;
                }

                final_costs[k] = cost;
            }

            // === PHASE 4: MPPI Weighting ===

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

            // Weighted average control
            double steering_cmd = 0.0;
            double accel_cmd = 0.0;

            for (size_t k = 0; k < K_final; ++k) {
                double w = weights[k] / weight_sum;
                steering_cmd += w * final_steering[k][0];
                accel_cmd += w * final_accel[k][0];
            }

            // Convert to velocity command
            double target_velocity = active_config.ref_velocity + accel_cmd * dt_internal;
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
                cmd.linear_velocity =
                    (constraints.max_linear_velocity > 0.0) ? target_velocity / constraints.max_linear_velocity : 0.0;
                if (is_diff_drive) {
                    cmd.angular_velocity = (constraints.max_angular_velocity > 0.0)
                                               ? angular_output / constraints.max_angular_velocity
                                               : 0.0;
                } else {
                    cmd.angular_velocity =
                        (constraints.max_steering_angle > 0.0) ? angular_output / constraints.max_steering_angle : 0.0;
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

    } // namespace pred
} // namespace drivekit
