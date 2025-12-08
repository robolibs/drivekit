#include "navcon/pred/mppi.hpp"
#include "navcon/types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace navcon {
    namespace pred {

        MPPIFollower::MPPIFollower() : MPPIFollower(MPPIConfig{}) {}

        MPPIFollower::MPPIFollower(const MPPIConfig &mppi_config)
            : mppi_config_(mppi_config), rng_(std::random_device{}()) {
            // Initialize mean control sequences
            mean_steering_.assign(mppi_config_.horizon_steps, 0.0);
            mean_acceleration_.assign(mppi_config_.horizon_steps, 0.0);
        }

        VelocityCommand MPPIFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                      const RobotConstraints &constraints, double dt,
                                                      const WorldConstraints *world_constraints) {
            (void)world_constraints; // Currently unused
            (void)dt;                // Use internal dt from configuration

            VelocityCommand cmd;
            cmd.valid = false;

            // Require a path for MPPI operation
            if (path_.waypoints.empty()) {
                cmd.status_message = "No path set for MPPI";
                return cmd;
            }

            // Goal reached check (same semantics as MPCFollower)
            if (is_goal_reached(current_state.pose, goal.target_pose)) {
                cmd.valid = true;
                cmd.status_message = "Goal reached";
                status_.goal_reached = true;
                status_.mode = "stopped";
                cmd.linear_velocity = 0.0;
                cmd.angular_velocity = 0.0;
                return cmd;
            }

            // Ensure mean control sequences match current horizon length
            if (mean_steering_.size() != mppi_config_.horizon_steps ||
                mean_acceleration_.size() != mppi_config_.horizon_steps) {
                mean_steering_.assign(mppi_config_.horizon_steps, 0.0);
                mean_acceleration_.assign(mppi_config_.horizon_steps, 0.0);
            }

            const size_t N = mppi_config_.horizon_steps;
            const size_t K = mppi_config_.num_samples;
            const double dt_internal = mppi_config_.dt;

            // Compute path error and reference trajectory (mirrors MPCFollower)
            PathError error = calculate_path_error(current_state);
            ReferenceTrajectory ref_traj = calculate_reference_trajectory(error, current_state, constraints);

            // Sampling distributions
            std::normal_distribution<double> steering_noise_dist(0.0, mppi_config_.steering_noise);
            std::normal_distribution<double> accel_noise_dist(0.0, mppi_config_.acceleration_noise);

            // Storage for costs and sampled noises
            std::vector<double> costs(K, 0.0);
            std::vector<std::vector<double>> noise_steering(K, std::vector<double>(N, 0.0));
            std::vector<std::vector<double>> noise_accel(K, std::vector<double>(N, 0.0));

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

                // Include initial state in trajectory for visualization
                Point p0;
                p0.x = x;
                p0.y = y;
                p0.z = 0.0;
                trajectory.push_back(p0);

                for (size_t i = 0; i < N; ++i) {
                    // Sample control noise
                    double eps_delta = steering_noise_dist(rng_);
                    double eps_acc = accel_noise_dist(rng_);
                    noise_steering[k][i] = eps_delta;
                    noise_accel[k][i] = eps_acc;

                    // Combine with mean control sequence
                    double delta = mean_steering_[i] + eps_delta;
                    double a = mean_acceleration_[i] + eps_acc;

                    // Clamp controls to robot constraints
                    delta = std::clamp(delta, -constraints.max_steering_angle, constraints.max_steering_angle);
                    a = std::clamp(a, -constraints.max_linear_acceleration, constraints.max_linear_acceleration);

                    // Kinematic bicycle model integration
                    double Lf = constraints.wheelbase;

                    x += v * std::cos(yaw) * dt_internal;
                    y += v * std::sin(yaw) * dt_internal;
                    yaw += v * delta / Lf * dt_internal;
                    yaw = normalize_angle(yaw);
                    v += a * dt_internal;
                    v = std::clamp(v, constraints.min_linear_velocity, constraints.max_linear_velocity);

                    // Add state to trajectory for potential visualization
                    Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0.0;
                    trajectory.push_back(p);

                    // Reference at this timestep (mirror MPC cost structure)
                    size_t ref_idx = std::min(i + 1, ref_traj.x.size() - 1);
                    double ref_x = ref_traj.x[ref_idx];
                    double ref_y = ref_traj.y[ref_idx];
                    double ref_yaw = ref_traj.yaw[ref_idx];
                    double ref_v = ref_traj.velocity[ref_idx];

                    // Cross-track and heading error relative to reference
                    double dx = x - ref_x;
                    double dy = y - ref_y;
                    double cte = -dx * std::sin(ref_yaw) + dy * std::cos(ref_yaw);
                    double epsi = normalize_angle(yaw - ref_yaw);
                    double vel_error = v - ref_v;

                    // Stage cost
                    double cost = 0.0;
                    cost += mppi_config_.weight_cte * cte * cte;
                    cost += mppi_config_.weight_epsi * epsi * epsi;
                    cost += mppi_config_.weight_vel * vel_error * vel_error;
                    cost += mppi_config_.weight_steering * delta * delta;
                    cost += mppi_config_.weight_acceleration * a * a;

                    sample_cost += cost * dt_internal;
                }

                costs[k] = sample_cost;

                if (sample_cost < best_cost) {
                    best_cost = sample_cost;
                    best_trajectory = std::move(trajectory);
                }
            }

            // Compute importance weights
            const double temperature = std::max(mppi_config_.temperature, 1e-6);
            const double beta = 1.0 / temperature;

            double min_cost = *std::min_element(costs.begin(), costs.end());
            std::vector<double> weights(K, 0.0);
            double weight_sum = 0.0;

            for (size_t k = 0; k < K; ++k) {
                double exponent = -beta * (costs[k] - min_cost);
                // Prevent numerical underflow
                exponent = std::max(exponent, -60.0);
                double w = std::exp(exponent);
                weights[k] = w;
                weight_sum += w;
            }

            if (weight_sum < 1e-12) {
                // Degenerate case: fall back to mean controls without update
                weight_sum = 1.0;
            }

            // Update mean control sequence using weighted noise
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

                // Keep updated means within constraints
                mean_steering_[i] =
                    std::clamp(mean_steering_[i], -constraints.max_steering_angle, constraints.max_steering_angle);
                mean_acceleration_[i] = std::clamp(mean_acceleration_[i], -constraints.max_linear_acceleration,
                                                   constraints.max_linear_acceleration);
            }

            // Extract first control input (receding horizon)
            double steering = mean_steering_.empty() ? 0.0 : mean_steering_.front();
            double acceleration = mean_acceleration_.empty() ? 0.0 : mean_acceleration_.front();

            // Shift mean sequences for next iteration
            if (!mean_steering_.empty()) {
                for (size_t i = 0; i + 1 < N; ++i) {
                    mean_steering_[i] = mean_steering_[i + 1];
                    mean_acceleration_[i] = mean_acceleration_[i + 1];
                }
                mean_steering_[N - 1] = 0.0;
                mean_acceleration_[N - 1] = 0.0;
            }

            // Store best predicted trajectory for visualization/debugging
            predicted_trajectory_ = best_trajectory;

            // Update status fields
            status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
            status_.goal_reached = false;
            status_.mode = "mppi_tracking";

            // Use MPC-style error metrics for status
            {
                PathError cur_error = calculate_path_error(current_state);
                status_.cross_track_error = std::abs(cur_error.cte);
                status_.heading_error = std::abs(cur_error.epsi);
            }

            // Convert steering/acceleration to command velocities
            // Use reference velocity as base, modulated by acceleration
            double target_velocity = mppi_config_.ref_velocity + acceleration * dt_internal;

            // Clamp velocity respecting robot constraints; for Ackermann we usually avoid reverse
            double min_vel = constraints.min_linear_velocity;
            if (!config_.allow_reverse) {
                min_vel = 0.0;
            }
            target_velocity = std::clamp(target_velocity, min_vel, constraints.max_linear_velocity);

            double steering_angle =
                std::clamp(steering, -constraints.max_steering_angle, constraints.max_steering_angle);

            if (config_.output_units == OutputUnits::NORMALIZED) {
                cmd.linear_velocity =
                    (constraints.max_linear_velocity > 0.0) ? target_velocity / constraints.max_linear_velocity : 0.0;
                cmd.angular_velocity =
                    (constraints.max_steering_angle > 0.0) ? steering_angle / constraints.max_steering_angle : 0.0;
            } else {
                cmd.linear_velocity = target_velocity;
                // For Ackermann steering, angular_velocity carries the steering angle
                cmd.angular_velocity = steering_angle;
            }

            cmd.valid = true;
            cmd.status_message = "MPPI tracking";

            return cmd;
        }

        std::string MPPIFollower::get_type() const { return "mppi_follower"; }

        void MPPIFollower::set_mppi_config(const MPPIConfig &config) {
            mppi_config_ = config;
            mean_steering_.assign(mppi_config_.horizon_steps, 0.0);
            mean_acceleration_.assign(mppi_config_.horizon_steps, 0.0);
        }

        MPPIFollower::MPPIConfig MPPIFollower::get_mppi_config() const { return mppi_config_; }

        MPPIFollower::PathError MPPIFollower::calculate_path_error(const RobotState &current_state) {
            PathError result;

            // Find nearest point on path (start search from current path_index_)
            double min_distance = std::numeric_limits<double>::max();
            size_t nearest_idx = path_index_;

            for (size_t i = path_index_; i < path_.waypoints.size(); ++i) {
                double dist = current_state.pose.point.distance_to(path_.waypoints[i].point);
                if (dist < min_distance) {
                    min_distance = dist;
                    nearest_idx = i;
                }
                // Stop searching if distance starts increasing significantly
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

            // Cross-track error
            double dx = current_state.pose.point.x - result.nearest_point.x;
            double dy = current_state.pose.point.y - result.nearest_point.y;
            result.cte = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);

            // Heading error
            result.epsi = normalize_angle(current_state.pose.angle.yaw - result.path_heading);

            // Advance controller path index
            path_index_ = nearest_idx;

            return result;
        }

        MPPIFollower::ReferenceTrajectory
        MPPIFollower::calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
                                                     const RobotConstraints &constraints) {
            (void)current_state;
            (void)constraints; // Reserved for future use (speed profile, curvature limits, etc.)

            ReferenceTrajectory ref;
            size_t start_idx = error.nearest_index;

            // Generate reference points along the path for the prediction horizon
            for (size_t i = 0; i < mppi_config_.horizon_steps + 1; ++i) {
                double distance_ahead = mppi_config_.ref_velocity * mppi_config_.dt * i;

                size_t target_idx = start_idx;
                double accumulated_dist = 0.0;

                while (target_idx < path_.waypoints.size() - 1 && accumulated_dist < distance_ahead) {
                    accumulated_dist +=
                        path_.waypoints[target_idx].point.distance_to(path_.waypoints[target_idx + 1].point);
                    if (accumulated_dist < distance_ahead) {
                        target_idx++;
                    }
                }

                target_idx = std::min(target_idx, path_.waypoints.size() - 1);

                ref.x.push_back(path_.waypoints[target_idx].point.x);
                ref.y.push_back(path_.waypoints[target_idx].point.y);

                double yaw;
                if (target_idx < path_.waypoints.size() - 1) {
                    Point next = path_.waypoints[target_idx + 1].point;
                    Point curr = path_.waypoints[target_idx].point;
                    yaw = std::atan2(next.y - curr.y, next.x - curr.x);
                } else {
                    yaw = path_.waypoints[target_idx].angle.yaw;
                }
                ref.yaw.push_back(yaw);

                // Use reference velocity for all points (can be extended with path_.speeds)
                ref.velocity.push_back(mppi_config_.ref_velocity);
            }

            return ref;
        }

    } // namespace pred
} // namespace navcon
