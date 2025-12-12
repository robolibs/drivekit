#include "drivekit/pred/mca.hpp"
#include "drivekit/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace drivekit {
    namespace pred {

        MCAFollower::MCAFollower() : MCAFollower(MCAConfig{}) {}

        MCAFollower::MCAFollower(const MCAConfig &mca_config) : MPPIFollower(), mca_rng_(std::random_device{}()) {
            set_mca_config(mca_config);
        }

        std::string MCAFollower::get_type() const { return "mca_follower"; }

        void MCAFollower::set_mca_config(const MCAConfig &config) {
            mca_config_ = config;

            // Map MCAConfig to underlying MPPIConfig
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
            mppi_cfg.turn_first_activation_deg = config.turn_first_activation_deg;
            mppi_cfg.turn_first_release_deg = config.turn_first_release_deg;
            mppi_cfg.num_threads = config.num_threads;

            set_mppi_config(mppi_cfg);
        }

        MCAFollower::MCAConfig MCAFollower::get_mca_config() const { return mca_config_; }

        void MCAFollower::set_dynamic_obstacles(const std::vector<DynamicObstacle> &obstacles) {
            dynamic_obstacles_ = obstacles;
        }

        double MCAFollower::evaluate_obstacle_pdf(const DynamicObstacle &obstacle, size_t timestep, double x,
                                                  double y) const {
            if (obstacle.modes.empty()) return 0.0;

            double total_pdf = 0.0;

            // Evaluate mixture of Gaussians
            for (const auto &mode : obstacle.modes) {
                if (timestep >= mode.mean_x.size() || timestep >= mode.mean_y.size()) continue;

                double mean_x = mode.mean_x[timestep];
                double mean_y = mode.mean_y[timestep];
                double std_x = mode.std_x[timestep];
                double std_y = mode.std_y[timestep];

                // Handle very small variances
                if (std_x < 1e-6) std_x = 1e-6;
                if (std_y < 1e-6) std_y = 1e-6;

                // Compute 2D Gaussian PDF (assuming independence between x and y)
                double dx = x - mean_x;
                double dy = y - mean_y;

                double exponent_x = -(dx * dx) / (2.0 * std_x * std_x);
                double exponent_y = -(dy * dy) / (2.0 * std_y * std_y);

                double norm_x = 1.0 / (std_x * std::sqrt(2.0 * M_PI));
                double norm_y = 1.0 / (std_y * std::sqrt(2.0 * M_PI));

                double pdf = norm_x * norm_y * std::exp(exponent_x + exponent_y);

                total_pdf += mode.weight * pdf;
            }

            return total_pdf;
        }

        std::vector<double>
        MCAFollower::compute_collision_probabilities(const std::vector<std::vector<Point>> &trajectories,
                                                     size_t timestep, double robot_radius) {

            const size_t K = trajectories.size();
            std::vector<double> collision_probs(K, 0.0);

            // If no obstacles, return zero probabilities
            if (dynamic_obstacles_.empty()) {
                return collision_probs;
            }

            // Find bounding box for all robot positions at this timestep
            double x_min = std::numeric_limits<double>::max();
            double x_max = std::numeric_limits<double>::lowest();
            double y_min = std::numeric_limits<double>::max();
            double y_max = std::numeric_limits<double>::lowest();

            for (size_t k = 0; k < K; ++k) {
                if (timestep >= trajectories[k].size()) continue;
                const Point &p = trajectories[k][timestep];
                x_min = std::min(x_min, p.x);
                x_max = std::max(x_max, p.x);
                y_min = std::min(y_min, p.y);
                y_max = std::max(y_max, p.y);
            }

            // Extend bounding box by robot radius + max obstacle radius
            double max_obstacle_radius = 0.0;
            for (const auto &obs : dynamic_obstacles_) {
                max_obstacle_radius = std::max(max_obstacle_radius, obs.radius);
            }
            double total_radius = robot_radius + max_obstacle_radius;

            x_min -= total_radius;
            x_max += total_radius;
            y_min -= total_radius;
            y_max += total_radius;

            // Sample Monte Carlo points uniformly in the bounding box
            const size_t Nmc = mca_config_.num_mc_samples;
            std::uniform_real_distribution<double> dist_x(x_min, x_max);
            std::uniform_real_distribution<double> dist_y(y_min, y_max);

            std::vector<double> mc_x(Nmc);
            std::vector<double> mc_y(Nmc);
            std::vector<double> joint_prob(Nmc, 0.0);

            // Generate Monte Carlo samples
            for (size_t j = 0; j < Nmc; ++j) {
                mc_x[j] = dist_x(mca_rng_);
                mc_y[j] = dist_y(mca_rng_);
            }

            // Compute joint collision probability for each MC sample
            // P_joint = 1 - ∏(1 - P_obstacle)
            for (size_t j = 0; j < Nmc; ++j) {
                double prob_no_collision = 1.0;

                for (const auto &obs : dynamic_obstacles_) {
                    double pdf = evaluate_obstacle_pdf(obs, timestep, mc_x[j], mc_y[j]);

                    // Marginal collision probability is PDF integrated over collision region
                    // For small regions, approximate as PDF * area
                    double collision_region_radius = robot_radius + obs.radius;
                    double area_element = (x_max - x_min) * (y_max - y_min) / Nmc;
                    double marginal_prob = pdf * area_element;

                    // Clamp to [0, 1]
                    marginal_prob = std::clamp(marginal_prob, 0.0, 1.0);

                    prob_no_collision *= (1.0 - marginal_prob);
                }

                joint_prob[j] = 1.0 - prob_no_collision;
            }

            // For each trajectory sample k, compute collision probability
            double bbox_area = (x_max - x_min) * (y_max - y_min);

            for (size_t k = 0; k < K; ++k) {
                if (timestep >= trajectories[k].size()) continue;

                const Point &robot_pos = trajectories[k][timestep];
                double collision_radius = robot_radius + max_obstacle_radius;
                double collision_area = M_PI * collision_radius * collision_radius;

                // Count MC samples in collision region and sum their probabilities
                double sum_prob = 0.0;
                size_t count_in_region = 0;

                for (size_t j = 0; j < Nmc; ++j) {
                    if (is_in_collision_region(robot_pos.x, robot_pos.y, mc_x[j], mc_y[j], collision_radius)) {
                        sum_prob += joint_prob[j];
                        count_in_region++;
                    }
                }

                // Monte Carlo estimate: (area / count) * sum
                if (count_in_region > 0) {
                    collision_probs[k] = (collision_area / count_in_region) * sum_prob / bbox_area * Nmc;
                    // Clamp to valid probability range
                    collision_probs[k] = std::clamp(collision_probs[k], 0.0, 1.0);
                }
            }

            return collision_probs;
        }

        VelocityCommand MCAFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                     const RobotConstraints &constraints, double dt,
                                                     const WorldConstraints *world_constraints) {
            // Convert WorldConstraints obstacles to MCA internal format
            if (world_constraints && !world_constraints->obstacles.empty()) {
                std::vector<DynamicObstacle> obstacles;
                for (const auto &obs : world_constraints->obstacles) {
                    DynamicObstacle dyn_obs;
                    dyn_obs.id = obs.id;
                    dyn_obs.radius = obs.radius;

                    // Convert modes
                    for (const auto &mode : obs.modes) {
                        DynamicObstacle::GaussianMode dyn_mode;
                        dyn_mode.weight = mode.weight;
                        dyn_mode.mean_x = mode.mean_x;
                        dyn_mode.mean_y = mode.mean_y;
                        dyn_mode.std_x = mode.std_x;
                        dyn_mode.std_y = mode.std_y;
                        dyn_obs.modes.push_back(dyn_mode);
                    }
                    obstacles.push_back(dyn_obs);
                }
                dynamic_obstacles_ = obstacles;
            }

            // If no dynamic obstacles, fall back to regular MPPI
            if (dynamic_obstacles_.empty()) {
                return MPPIFollower::compute_control(current_state, goal, constraints, dt, world_constraints);
            }

            // Otherwise, run custom risk-aware MPPI logic
            (void)world_constraints;
            (void)dt;

            VelocityCommand cmd;
            cmd.valid = false;

            // Require a path
            if (path_.drivekits.empty()) {
                cmd.status_message = "No path set for MCA";
                return cmd;
            }

            // Use MPPI base class method for path error calculation
            // Since it's private, we need to reimplement the necessary logic inline

            // Find nearest point on path
            double min_distance = std::numeric_limits<double>::max();
            size_t nearest_idx = path_index_;
            Point nearest_point{};
            double path_heading = 0.0;

            for (size_t i = path_index_; i < path_.drivekits.size(); ++i) {
                double dist = current_state.pose.point.distance_to(path_.drivekits[i].point);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_idx = i;
                    nearest_point = path_.drivekits[i].point;
                }
                if (i > path_index_ && dist > min_distance * 1.5) {
                    break;
                }
            }

            // Calculate path heading
            if (nearest_idx < path_.drivekits.size() - 1) {
                Point next_point = path_.drivekits[nearest_idx + 1].point;
                path_heading = std::atan2(next_point.y - nearest_point.y, next_point.x - nearest_point.x);
            } else {
                path_heading = path_.drivekits[nearest_idx].angle.yaw;
            }

            // Cross-track error
            double dx = current_state.pose.point.x - nearest_point.x;
            double dy = current_state.pose.point.y - nearest_point.y;
            double cte = -dx * std::sin(path_heading) + dy * std::cos(path_heading);

            // Heading error
            double epsi = normalize_angle(current_state.pose.angle.yaw - path_heading);

            // Advance path index
            path_index_ = nearest_idx;

            // Get configuration
            MPPIConfig active_config = get_mppi_config();
            bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                  constraints.steering_type == SteeringType::SKID_STEER);

            // Turn-first logic
            bool is_turning = false;
            if (current_state.turn_first && is_diff_drive) {
                const double activation_threshold_rad = active_config.turn_first_activation_deg * M_PI / 180.0;
                const double release_threshold_rad = active_config.turn_first_release_deg * M_PI / 180.0;
                const double heading_error_abs = std::abs(epsi);

                static bool is_turning_in_place = false;
                if (!is_turning_in_place) {
                    if (heading_error_abs > activation_threshold_rad) {
                        is_turning_in_place = true;
                    }
                } else {
                    if (heading_error_abs < release_threshold_rad) {
                        is_turning_in_place = false;
                    }
                }

                if (is_turning_in_place) {
                    active_config.ref_velocity = 0.2;
                    active_config.weight_vel = 50.0;
                    is_turning = true;
                }
            }

            // Goal reached check (use goal's tolerance if specified)
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
            const size_t K = active_config.num_samples;
            const double dt_internal = active_config.dt;
            const double ref_velocity = active_config.ref_velocity;

            // Generate reference trajectory along path
            std::vector<double> ref_x, ref_y, ref_yaw, ref_vel;
            for (size_t i = 0; i < N + 1; ++i) {
                double distance_ahead = ref_velocity * dt_internal * i;
                size_t target_idx = nearest_idx;
                double accumulated_dist = 0.0;

                while (target_idx < path_.drivekits.size() - 1 && accumulated_dist < distance_ahead) {
                    accumulated_dist +=
                        path_.drivekits[target_idx].point.distance_to(path_.drivekits[target_idx + 1].point);
                    if (accumulated_dist < distance_ahead) {
                        target_idx++;
                    }
                }

                target_idx = std::min(target_idx, path_.drivekits.size() - 1);

                ref_x.push_back(path_.drivekits[target_idx].point.x);
                ref_y.push_back(path_.drivekits[target_idx].point.y);

                double yaw;
                if (target_idx < path_.drivekits.size() - 1) {
                    Point next = path_.drivekits[target_idx + 1].point;
                    Point curr = path_.drivekits[target_idx].point;
                    yaw = std::atan2(next.y - curr.y, next.x - curr.x);
                } else {
                    yaw = path_.drivekits[target_idx].angle.yaw;
                }
                ref_yaw.push_back(yaw);
                ref_vel.push_back(ref_velocity);
            }

            // Sampling distributions
            std::normal_distribution<double> steering_noise_dist(0.0, active_config.steering_noise);
            std::normal_distribution<double> accel_noise_dist(0.0, active_config.acceleration_noise);

            // Storage
            std::vector<double> costs(K, 0.0);
            std::vector<std::vector<double>> noise_steering(K, std::vector<double>(N, 0.0));
            std::vector<std::vector<double>> noise_accel(K, std::vector<double>(N, 0.0));
            std::vector<std::vector<Point>> all_trajectories(K);

            // Initialize collision probability storage
            sample_collision_probs_.assign(K, std::vector<double>(N, 0.0));

            // Mean control sequences (start from zero)
            std::vector<double> mean_steering(N, 0.0);
            std::vector<double> mean_acceleration(N, 0.0);

            double best_cost = std::numeric_limits<double>::infinity();
            std::vector<Point> best_trajectory;

            // Rollout all samples
            for (size_t k = 0; k < K; ++k) {
                double x = current_state.pose.point.x;
                double y = current_state.pose.point.y;
                double yaw = current_state.pose.angle.yaw;
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
                    double eps_delta = steering_noise_dist(mca_rng_);
                    double eps_acc = accel_noise_dist(mca_rng_);
                    noise_steering[k][i] = eps_delta;
                    noise_accel[k][i] = eps_acc;

                    double delta_or_omega = mean_steering[i] + eps_delta;
                    double a = mean_acceleration[i] + eps_acc;

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

                    Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0.0;
                    trajectory.push_back(p);

                    size_t ref_idx = std::min(i + 1, ref_x.size() - 1);
                    double r_x = ref_x[ref_idx];
                    double r_y = ref_y[ref_idx];
                    double r_yaw = ref_yaw[ref_idx];
                    double r_v = ref_vel[ref_idx];

                    double d_x = x - r_x;
                    double d_y = y - r_y;
                    double cur_cte = -d_x * std::sin(r_yaw) + d_y * std::cos(r_yaw);
                    double cur_epsi = normalize_angle(yaw - r_yaw);
                    double vel_error = v - r_v;

                    double cost = 0.0;
                    cost += active_config.weight_cte * cur_cte * cur_cte;
                    cost += active_config.weight_epsi * cur_epsi * cur_epsi;
                    cost += active_config.weight_vel * vel_error * vel_error;
                    cost += active_config.weight_steering * delta_or_omega * delta_or_omega;
                    cost += active_config.weight_acceleration * a * a;

                    sample_cost += cost * dt_internal;
                }

                costs[k] = sample_cost;
                all_trajectories[k] = trajectory;

                if (sample_cost < best_cost) {
                    best_cost = sample_cost;
                    best_trajectory = trajectory;
                }
            }

            // Compute collision probabilities for each timestep
            // Estimate robot radius from constraints
            double robot_radius = std::max(constraints.robot_width, constraints.robot_length) / 2.0;
            if (robot_radius < 0.1) robot_radius = 0.5; // Default fallback

            for (size_t t = 0; t < N; ++t) {
                std::vector<double> collision_probs =
                    compute_collision_probabilities(all_trajectories, t + 1, robot_radius);

                // Add risk costs to sample costs
                for (size_t k = 0; k < K; ++k) {
                    sample_collision_probs_[k][t] = collision_probs[k];

                    // Soft risk penalty
                    double soft_penalty = mca_config_.weight_soft_risk * collision_probs[k];

                    // Hard risk penalty
                    double hard_penalty = 0.0;
                    if (collision_probs[k] > mca_config_.risk_threshold) {
                        hard_penalty = mca_config_.weight_hard_risk;
                    }

                    costs[k] += (soft_penalty + hard_penalty) * dt_internal;
                }
            }

            // Compute importance weights
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

            // Update mean control sequence
            for (size_t i = 0; i < N; ++i) {
                double d_delta = 0.0;
                double d_acc = 0.0;

                for (size_t k = 0; k < K; ++k) {
                    double w = weights[k] / weight_sum;
                    d_delta += w * noise_steering[k][i];
                    d_acc += w * noise_accel[k][i];
                }

                mean_steering[i] += d_delta;
                mean_acceleration[i] += d_acc;

                if (is_diff_drive) {
                    mean_steering[i] = std::clamp(mean_steering[i], -constraints.max_angular_velocity,
                                                  constraints.max_angular_velocity);
                } else {
                    mean_steering[i] =
                        std::clamp(mean_steering[i], -constraints.max_steering_angle, constraints.max_steering_angle);
                }
                mean_acceleration[i] = std::clamp(mean_acceleration[i], -constraints.max_linear_acceleration,
                                                  constraints.max_linear_acceleration);
            }

            // Extract first control input
            double steering_or_omega = mean_steering.empty() ? 0.0 : mean_steering.front();
            double acceleration = mean_acceleration.empty() ? 0.0 : mean_acceleration.front();

            // Update status
            status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
            status_.goal_reached = false;
            status_.mode = "mca_tracking";
            status_.cross_track_error = std::abs(cte);
            status_.heading_error = std::abs(epsi);

            // Calculate distance to end of path for deceleration
            double dist_to_end = 0.0;
            for (size_t j = path_index_; j < path_.drivekits.size() - 1; ++j) {
                dist_to_end += path_.drivekits[j].point.distance_to(path_.drivekits[j + 1].point);
            }
            // Deceleration zone: slow down within 2m of the end
            const double decel_distance = 2.0;
            double effective_ref_velocity = ref_velocity;
            if (dist_to_end < decel_distance) {
                effective_ref_velocity = ref_velocity * (dist_to_end / decel_distance);
                effective_ref_velocity = std::max(effective_ref_velocity, 0.0);
            }

            // Convert to velocity commands
            double target_velocity = effective_ref_velocity + acceleration * dt_internal;
            double min_vel = constraints.min_linear_velocity;
            if (!config_.allow_reverse) {
                min_vel = 0.0;
            }
            target_velocity = std::clamp(target_velocity, min_vel, constraints.max_linear_velocity);

            double angular_output;
            if (is_diff_drive) {
                angular_output =
                    std::clamp(steering_or_omega, -constraints.max_angular_velocity, constraints.max_angular_velocity);
            } else {
                angular_output =
                    std::clamp(steering_or_omega, -constraints.max_steering_angle, constraints.max_steering_angle);
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
            cmd.status_message = "MCA tracking";

            if (is_turning) {
                cmd.linear_velocity = 0.0;
            }

            return cmd;
        }

    } // namespace pred
} // namespace drivekit
