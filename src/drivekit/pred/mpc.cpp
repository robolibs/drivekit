#include "drivekit/pred/mpc.hpp"
#include "drivekit/types.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace drivekit {
    namespace pred {

        // Lightweight single-shooting optimizer for MPCFollower.
        // This replaces the previous IPOPT/CppAD-based solver with a simple
        // projected gradient descent on the control sequence.
        class MPCOptimizer {
          public:
            struct Result {
                bool success = false;
                double steering = 0.0;
                double acceleration = 0.0;
                std::vector<double> predicted_x;
                std::vector<double> predicted_y;
                std::vector<double> steering_sequence;
                std::vector<double> acceleration_sequence;
            };

            MPCOptimizer(const MPCFollower::MPCConfig &config, const RobotConstraints &constraints,
                         const MPCFollower::ReferenceTrajectory &ref_traj, const RobotState &initial_state)
                : config_(config), constraints_(constraints), ref_traj_(ref_traj), initial_state_(initial_state),
                  horizon_steps_(config.horizon_steps) {}

            Result solve(const std::vector<double> &initial_steering,
                         const std::vector<double> &initial_acceleration) const {
                Result res;

                if (horizon_steps_ == 0) {
                    return res;
                }

                const size_t num_vars = horizon_steps_ * 2; // steering + acceleration per step
                std::vector<double> u(num_vars, 0.0);

                // Initialize decision vector from previous solution (warm start)
                for (size_t i = 0; i < horizon_steps_; ++i) {
                    double steering = (i < initial_steering.size()) ? initial_steering[i] : 0.0;
                    double accel = (i < initial_acceleration.size()) ? initial_acceleration[i] : 0.0;
                    u[2 * i] = steering;
                    u[2 * i + 1] = accel;
                }
                clamp_controls(u);
                smooth_controls(u);

                const int max_iterations = 15;
                const double eps = 1e-3;
                const double base_step = 0.1;

                double best_cost = simulate_and_cost(u, nullptr, nullptr);
                if (!std::isfinite(best_cost)) {
                    return res;
                }

                std::vector<double> best_u = u;
                std::vector<double> grad(num_vars, 0.0);

                for (int iter = 0; iter < max_iterations; ++iter) {
                    finite_difference_gradient(u, eps, grad);

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

                // Final trajectory for the best control sequence
                std::vector<double> traj_x;
                std::vector<double> traj_y;
                (void)simulate_and_cost(best_u, &traj_x, &traj_y);

                res.success = true;
                res.steering = best_u[0];
                res.acceleration = best_u[1];
                res.predicted_x = std::move(traj_x);
                res.predicted_y = std::move(traj_y);

                res.steering_sequence.resize(horizon_steps_);
                res.acceleration_sequence.resize(horizon_steps_);
                for (size_t i = 0; i < horizon_steps_; ++i) {
                    res.steering_sequence[i] = best_u[2 * i];
                    res.acceleration_sequence[i] = best_u[2 * i + 1];
                }

                return res;
            }

          private:
            const MPCFollower::MPCConfig &config_;
            const RobotConstraints &constraints_;
            const MPCFollower::ReferenceTrajectory &ref_traj_;
            RobotState initial_state_;
            size_t horizon_steps_;

            static double normalize_angle_local(double angle) {
                while (angle > M_PI) {
                    angle -= 2.0 * M_PI;
                }
                while (angle < -M_PI) {
                    angle += 2.0 * M_PI;
                }
                return angle;
            }

            void clamp_controls(std::vector<double> &u) const {
                for (size_t i = 0; i < horizon_steps_; ++i) {
                    clamp_single(u, 2 * i);
                    clamp_single(u, 2 * i + 1);
                }
            }

            void smooth_controls(std::vector<double> &u) const {
                if (horizon_steps_ < 3) {
                    return;
                }

                // Apply a simple Savitzky–Golay-like smoothing (length-5 window)
                std::vector<double> smoothed(u.size());
                for (int dim = 0; dim < 2; ++dim) {
                    size_t offset = dim;
                    for (size_t i = 0; i < horizon_steps_; ++i) {
                        // Indices for a 5-point window, clamped at ends
                        size_t idx_m2 = (i >= 2) ? (i - 2) : 0;
                        size_t idx_m1 = (i >= 1) ? (i - 1) : 0;
                        size_t idx_0 = i;
                        size_t idx_p1 = (i + 1 < horizon_steps_) ? (i + 1) : (horizon_steps_ - 1);
                        size_t idx_p2 = (i + 2 < horizon_steps_) ? (i + 2) : (horizon_steps_ - 1);

                        double y_m2 = u[2 * idx_m2 + dim];
                        double y_m1 = u[2 * idx_m1 + dim];
                        double y_0 = u[2 * idx_0 + dim];
                        double y_p1 = u[2 * idx_p1 + dim];
                        double y_p2 = u[2 * idx_p2 + dim];

                        double smoothed_val =
                            (1.0 / 35.0) * (-3.0 * y_m2 + 12.0 * y_m1 + 17.0 * y_0 + 12.0 * y_p1 - 3.0 * y_p2);
                        smoothed[2 * i + dim] = smoothed_val;
                    }
                }

                // Copy back and clamp again
                u = smoothed;
                clamp_controls(u);
            }

            void clamp_single(std::vector<double> &u, size_t index) const {
                if (index >= u.size()) {
                    return;
                }
                const bool is_steering_or_omega = (index % 2 == 0);
                const bool is_diff_drive = (constraints_.steering_type == SteeringType::DIFFERENTIAL ||
                                            constraints_.steering_type == SteeringType::SKID_STEER);

                if (is_steering_or_omega) {
                    double &val = u[index];
                    if (is_diff_drive) {
                        // For diff drive: clamp angular velocity
                        val = std::clamp(val, -constraints_.max_angular_velocity, constraints_.max_angular_velocity);
                    } else {
                        // For Ackermann: clamp steering angle
                        val = std::clamp(val, -constraints_.max_steering_angle, constraints_.max_steering_angle);
                    }
                } else {
                    double &accel = u[index];
                    accel =
                        std::clamp(accel, -constraints_.max_linear_acceleration, constraints_.max_linear_acceleration);
                }
            }

            void finite_difference_gradient(const std::vector<double> &u, double eps,
                                            std::vector<double> &grad_out) const {
                const size_t num_vars = u.size();
                grad_out.resize(num_vars);

                std::vector<double> u_plus = u;
                std::vector<double> u_minus = u;

                for (size_t i = 0; i < num_vars; ++i) {
                    double orig = u[i];

                    u_plus[i] = orig + eps;
                    clamp_single(u_plus, i);
                    double cost_plus = simulate_and_cost(u_plus, nullptr, nullptr);

                    u_minus[i] = orig - eps;
                    clamp_single(u_minus, i);
                    double cost_minus = simulate_and_cost(u_minus, nullptr, nullptr);

                    grad_out[i] = (cost_plus - cost_minus) / (2.0 * eps);

                    u_plus[i] = orig;
                    u_minus[i] = orig;
                }
            }

            double simulate_and_cost(const std::vector<double> &u, std::vector<double> *out_x,
                                     std::vector<double> *out_y) const {
                const double dt = config_.dt;
                const double Lf = constraints_.wheelbase;
                const bool is_diff_drive = (constraints_.steering_type == SteeringType::DIFFERENTIAL ||
                                            constraints_.steering_type == SteeringType::SKID_STEER);

                double x = initial_state_.pose.point.x;
                double y = initial_state_.pose.point.y;
                double yaw = initial_state_.pose.angle.yaw;
                double v = initial_state_.velocity.linear;

                if (out_x && out_y) {
                    out_x->clear();
                    out_y->clear();
                    out_x->reserve(horizon_steps_ + 1);
                    out_y->reserve(horizon_steps_ + 1);
                    out_x->push_back(x);
                    out_y->push_back(y);
                }

                double cost = 0.0;

                for (size_t i = 0; i < horizon_steps_; ++i) {
                    const size_t idx_u = 2 * i;
                    double steering_or_omega = u[idx_u];
                    double accel = u[idx_u + 1];

                    // Clamp controls to physical limits
                    if (is_diff_drive) {
                        // For diff drive: first control is angular velocity
                        steering_or_omega = std::clamp(steering_or_omega, -constraints_.max_angular_velocity,
                                                       constraints_.max_angular_velocity);
                    } else {
                        // For Ackermann: first control is steering angle
                        steering_or_omega = std::clamp(steering_or_omega, -constraints_.max_steering_angle,
                                                       constraints_.max_steering_angle);
                    }
                    accel =
                        std::clamp(accel, -constraints_.max_linear_acceleration, constraints_.max_linear_acceleration);

                    // State propagation based on kinematic model
                    x += v * std::cos(yaw) * dt;
                    y += v * std::sin(yaw) * dt;
                    if (is_diff_drive) {
                        // Differential drive: direct angular velocity control
                        yaw += steering_or_omega * dt;
                    } else {
                        // Ackermann/bicycle model: steering angle based
                        yaw += v * steering_or_omega / Lf * dt;
                    }
                    yaw = normalize_angle_local(yaw);
                    v += accel * dt;
                    v = std::clamp(v, constraints_.min_linear_velocity, constraints_.max_linear_velocity);

                    if (out_x && out_y) {
                        out_x->push_back(x);
                        out_y->push_back(y);
                    }

                    // Reference at this timestep
                    size_t ref_idx = std::min(i + 1, ref_traj_.x.empty() ? size_t(0) : (ref_traj_.x.size() - 1));
                    double ref_x = ref_traj_.x.empty() ? x : ref_traj_.x[ref_idx];
                    double ref_y = ref_traj_.y.empty() ? y : ref_traj_.y[ref_idx];
                    double ref_yaw = ref_traj_.yaw.empty() ? yaw : ref_traj_.yaw[ref_idx];
                    double ref_v = ref_traj_.velocity.empty() ? config_.ref_velocity : ref_traj_.velocity[ref_idx];

                    // Tracking errors
                    double dx = x - ref_x;
                    double dy = y - ref_y;
                    double cte = -dx * std::sin(ref_yaw) + dy * std::cos(ref_yaw);
                    double epsi = normalize_angle_local(yaw - ref_yaw);
                    double vel_error = v - ref_v;

                    cost += config_.weight_cte * cte * cte;
                    cost += config_.weight_epsi * epsi * epsi;
                    cost += config_.weight_vel * vel_error * vel_error;

                    // Control effort
                    cost += config_.weight_steering * steering_or_omega * steering_or_omega;
                    cost += config_.weight_acceleration * accel * accel;

                    // Control rate penalties
                    if (i > 0) {
                        double prev_steering_or_omega = u[idx_u - 2];
                        double prev_accel = u[idx_u - 1];
                        double d_steer = steering_or_omega - prev_steering_or_omega;
                        double d_acc = accel - prev_accel;
                        cost += config_.weight_steering_rate * d_steer * d_steer;
                        cost += config_.weight_acceleration_rate * d_acc * d_acc;
                    }
                }

                return cost;
            }
        };

        // -----------------------------------------------------------------------------
        // MPCFollower implementation
        // -----------------------------------------------------------------------------

        MPCFollower::MPCFollower() : MPCFollower(MPCConfig{}) {}

        MPCFollower::MPCFollower(const MPCConfig &mpc_config) : mpc_config_(mpc_config) {
            previous_steering_.resize(mpc_config_.horizon_steps, 0.0);
            previous_acceleration_.resize(mpc_config_.horizon_steps, 0.0);
        }

        VelocityCommand MPCFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                     const RobotConstraints &constraints, double dt,
                                                     const WorldConstraints *world_constraints) {
            (void)world_constraints;
            (void)dt; // MPC uses its own dt from config

            VelocityCommand cmd;
            cmd.valid = false;

            if (path_.drivekits.empty()) {
                return cmd;
            }

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

            // Path error and reference
            PathError error = calculate_path_error(current_state);
            ReferenceTrajectory ref_traj = calculate_reference_trajectory(error, current_state, constraints);

            // Solve MPC
            MPCSolution solution = solve_mpc(current_state, ref_traj, constraints);
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

            // Warm start: shift sequence forward by one step
            if (!solution.steering_sequence.empty() && solution.steering_sequence.size() == N) {
                for (size_t i = 0; i + 1 < N; ++i) {
                    previous_steering_[i] = solution.steering_sequence[i + 1];
                    previous_acceleration_[i] = solution.acceleration_sequence[i + 1];
                }
                previous_steering_[N - 1] = solution.steering_sequence.back();
                previous_acceleration_[N - 1] = solution.acceleration_sequence.back();
            }

            // Update predicted trajectory for visualization
            predicted_trajectory_.clear();
            for (size_t i = 0; i < solution.predicted_x.size(); ++i) {
                Point p;
                p.x = solution.predicted_x[i];
                p.y = solution.predicted_y[i];
                p.z = 0.0;
                predicted_trajectory_.push_back(p);
            }

            // Update status
            status_.distance_to_goal = current_state.pose.point.distance_to(goal.target_pose.point);
            status_.cross_track_error = std::abs(error.cte);
            status_.heading_error = std::abs(error.epsi);
            status_.goal_reached = false;
            status_.mode = "mpc_tracking";

            // Convert steering/angular velocity and acceleration to velocities
            const bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                        constraints.steering_type == SteeringType::SKID_STEER);

            double target_velocity = mpc_config_.ref_velocity + solution.acceleration * mpc_config_.dt;
            double min_vel = constraints.min_linear_velocity;
            if (!config_.allow_reverse) {
                min_vel = 0.0;
            }
            target_velocity = std::clamp(target_velocity, min_vel, constraints.max_linear_velocity);

            double angular_output;
            if (is_diff_drive) {
                // For diff drive: solution.steering is actually angular velocity
                angular_output =
                    std::clamp(solution.steering, -constraints.max_angular_velocity, constraints.max_angular_velocity);
            } else {
                // For Ackermann: solution.steering is steering angle
                angular_output =
                    std::clamp(solution.steering, -constraints.max_steering_angle, constraints.max_steering_angle);
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
            cmd.status_message = "MPC tracking";
            return cmd;
        }

        std::string MPCFollower::get_type() const { return "mpc_follower"; }

        void MPCFollower::set_mpc_config(const MPCConfig &config) {
            mpc_config_ = config;
            previous_steering_.resize(config.horizon_steps, 0.0);
            previous_acceleration_.resize(config.horizon_steps, 0.0);
        }

        MPCFollower::MPCConfig MPCFollower::get_mpc_config() const { return mpc_config_; }

        MPCFollower::PathError MPCFollower::calculate_path_error(const RobotState &current_state) {
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
                result.path_heading = path_.drivekits[nearest_idx].angle.yaw;
            }

            double dx = current_state.pose.point.x - result.nearest_point.x;
            double dy = current_state.pose.point.y - result.nearest_point.y;
            result.cte = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);

            result.epsi = normalize_angle(current_state.pose.angle.yaw - result.path_heading);

            path_index_ = nearest_idx;

            return result;
        }

        MPCFollower::ReferenceTrajectory
        MPCFollower::calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
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
                    yaw = path_.drivekits[target_idx].angle.yaw;
                }
                ref.yaw.push_back(yaw);

                ref.velocity.push_back(mpc_config_.ref_velocity);
            }

            return ref;
        }

        MPCFollower::MPCSolution MPCFollower::solve_mpc(const RobotState &current_state,
                                                        const ReferenceTrajectory &ref_trajectory,
                                                        const RobotConstraints &constraints) {
            MPCOptimizer optimizer(mpc_config_, constraints, ref_trajectory, current_state);
            MPCOptimizer::Result opt = optimizer.solve(previous_steering_, previous_acceleration_);

            MPCSolution sol;
            sol.success = opt.success;
            sol.steering = opt.steering;
            sol.acceleration = opt.acceleration;
            sol.predicted_x = std::move(opt.predicted_x);
            sol.predicted_y = std::move(opt.predicted_y);
            sol.steering_sequence = std::move(opt.steering_sequence);
            sol.acceleration_sequence = std::move(opt.acceleration_sequence);
            return sol;
        }

    } // namespace pred
} // namespace drivekit
