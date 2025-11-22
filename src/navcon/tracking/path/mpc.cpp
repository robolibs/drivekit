#include "navcon/tracking/path/mpc.hpp"
#include "navcon/tracking/types.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace navcon {
    namespace tracking {
        namespace path {

            // Constructor
            MPCFollower::MPCFollower() : MPCFollower(MPCConfig{}) {}

            MPCFollower::MPCFollower(const MPCConfig &mpc_config) : mpc_config_(mpc_config) {
                // Initialize previous control inputs for warm starting
                previous_steering_.resize(mpc_config_.horizon_steps, 0.0);
                previous_acceleration_.resize(mpc_config_.horizon_steps, 0.0);
            }

            VelocityCommand MPCFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                         const RobotConstraints &constraints, double dt,
                                                         const WorldConstraints *world_constraints) {
                (void)world_constraints; // MPC doesn't use world constraints yet
                (void)dt;                // MPC uses its own dt from config

                VelocityCommand cmd;
                cmd.valid = false;

                // Check if we have a path
                if (path_.waypoints.empty()) {
                    return cmd;
                }

                // Check if goal is reached
                // NOTE: Disabled for path tracking - let the tracker handle goal reaching
                // if (is_goal_reached(current_state.pose, goal.target_pose)) {
                //     cmd.valid = true;
                //     cmd.status_message = "Goal reached";
                //     status_.goal_reached = true;
                //     status_.mode = "stopped";
                //     cmd.linear_velocity = 0.0;
                //     cmd.angular_velocity = 0.0;
                //     return cmd;
                // }

                // Calculate path tracking errors
                PathError error = calculate_path_error(current_state);

                // Calculate reference trajectory
                ReferenceTrajectory ref_traj = calculate_reference_trajectory(error, current_state, constraints);

                // Solve MPC optimization problem
                MPCSolution solution = solve_mpc(current_state, ref_traj, constraints, error.cte, error.epsi);

                if (!solution.success) {
                    cmd.valid = false;
                    cmd.status_message = "MPC solver failed";
                    return cmd;
                }

                // Store solution for warm start next iteration
                previous_steering_[0] = solution.steering;
                previous_acceleration_[0] = solution.acceleration;

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

                // Convert steering and acceleration to velocities
                // For differential drive robots
                // Use reference velocity as target, modified by acceleration from MPC
                double target_velocity = mpc_config_.ref_velocity + solution.acceleration * mpc_config_.dt;
                target_velocity = std::clamp(target_velocity, 0.0, constraints.max_linear_velocity);

                // Convert steering angle to angular velocity
                // Using proportional control on steering angle
                double kp_steer = 3.0;
                double angular_velocity = kp_steer * solution.steering;
                angular_velocity =
                    std::clamp(angular_velocity, -constraints.max_angular_velocity, constraints.max_angular_velocity);

                // Convert to output units based on configuration
                if (config_.output_units == OutputUnits::NORMALIZED) {
                    cmd.linear_velocity = target_velocity / constraints.max_linear_velocity;
                    cmd.angular_velocity = angular_velocity / constraints.max_angular_velocity;
                } else {
                    cmd.linear_velocity = target_velocity;
                    cmd.angular_velocity = angular_velocity;
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

                // Find nearest point on path
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

                // Calculate cross-track error (CTE)
                double dx = current_state.pose.point.x - result.nearest_point.x;
                double dy = current_state.pose.point.y - result.nearest_point.y;

                // Project onto path normal
                result.cte = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);

                // Calculate heading error (epsi)
                result.epsi = normalize_angle(current_state.pose.angle.yaw - result.path_heading);

                // Update path index to advance along the path
                path_index_ = nearest_idx;

                return result;
            }

            MPCFollower::ReferenceTrajectory
            MPCFollower::calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
                                                        const RobotConstraints &constraints) {
                (void)constraints; // May be used for speed profile in future

                ReferenceTrajectory ref;
                size_t start_idx = error.nearest_index;

                // Generate reference points along the path for the prediction horizon
                for (size_t i = 0; i < mpc_config_.horizon_steps + 1; ++i) {
                    // Estimate how far along the path we should be at this timestep
                    double distance_ahead = current_state.velocity.linear * mpc_config_.dt * i;

                    // Find corresponding waypoint
                    size_t target_idx = start_idx;
                    double accumulated_dist = 0.0;

                    while (target_idx < path_.waypoints.size() - 1 && accumulated_dist < distance_ahead) {
                        accumulated_dist +=
                            path_.waypoints[target_idx].point.distance_to(path_.waypoints[target_idx + 1].point);
                        if (accumulated_dist < distance_ahead) {
                            target_idx++;
                        }
                    }

                    // Clamp to path bounds
                    target_idx = std::min(target_idx, path_.waypoints.size() - 1);

                    ref.x.push_back(path_.waypoints[target_idx].point.x);
                    ref.y.push_back(path_.waypoints[target_idx].point.y);

                    // Calculate yaw from path
                    double yaw;
                    if (target_idx < path_.waypoints.size() - 1) {
                        Point next = path_.waypoints[target_idx + 1].point;
                        Point curr = path_.waypoints[target_idx].point;
                        yaw = std::atan2(next.y - curr.y, next.x - curr.x);
                    } else {
                        yaw = path_.waypoints[target_idx].angle.yaw;
                    }
                    ref.yaw.push_back(yaw);

                    // Reference velocity
                    ref.velocity.push_back(mpc_config_.ref_velocity);
                }

                return ref;
            }

            MPCFollower::MPCSolution MPCFollower::solve_mpc(const RobotState &current_state,
                                                            const ReferenceTrajectory &ref_trajectory,
                                                            const RobotConstraints &constraints, double cte,
                                                            double epsi) {
                using CppAD::AD;
                using Dvector = CPPAD_TESTVECTOR(double);

                MPCSolution result;
                result.success = false;

                size_t N = mpc_config_.horizon_steps;

                // Number of state variables: x, y, yaw, v, cte, epsi
                size_t n_states = 6;
                // Number of actuators: steering, acceleration
                size_t n_actuators = 2;

                // Total number of variables
                size_t n_vars = n_states * (N + 1) + n_actuators * N;
                // Total number of constraints
                size_t n_constraints = n_states * (N + 1);

                // Initial values of all variables (should be 0 except initial state)
                Dvector vars(n_vars);
                for (size_t i = 0; i < n_vars; i++) {
                    vars[i] = 0.0;
                }

                // Set initial state
                size_t x_start = 0;
                size_t y_start = x_start + N + 1;
                size_t yaw_start = y_start + N + 1;
                size_t v_start = yaw_start + N + 1;
                size_t cte_start = v_start + N + 1;
                size_t epsi_start = cte_start + N + 1;
                size_t steering_start = epsi_start + N + 1;
                size_t acceleration_start = steering_start + N;

                vars[x_start] = current_state.pose.point.x;
                vars[y_start] = current_state.pose.point.y;
                vars[yaw_start] = current_state.pose.angle.yaw;
                vars[v_start] = current_state.velocity.linear;
                vars[cte_start] = cte;
                vars[epsi_start] = epsi;

                // Warm start with previous solution
                for (size_t i = 0; i < N; i++) {
                    if (i < previous_steering_.size()) {
                        vars[steering_start + i] = previous_steering_[i];
                    }
                    if (i < previous_acceleration_.size()) {
                        vars[acceleration_start + i] = previous_acceleration_[i];
                    }
                }

                // Set variable bounds
                Dvector vars_lowerbound(n_vars);
                Dvector vars_upperbound(n_vars);

                // Set all non-actuator variables to very large bounds
                for (size_t i = 0; i < steering_start; i++) {
                    vars_lowerbound[i] = -1.0e19;
                    vars_upperbound[i] = 1.0e19;
                }

                // Steering angle bounds (rad)
                for (size_t i = steering_start; i < acceleration_start; i++) {
                    vars_lowerbound[i] = -constraints.max_steering_angle;
                    vars_upperbound[i] = constraints.max_steering_angle;
                }

                // Acceleration bounds (m/s^2)
                for (size_t i = acceleration_start; i < n_vars; i++) {
                    vars_lowerbound[i] = -constraints.max_linear_acceleration;
                    vars_upperbound[i] = constraints.max_linear_acceleration;
                }

                // Set constraint bounds
                Dvector constraints_lowerbound(n_constraints);
                Dvector constraints_upperbound(n_constraints);

                // All constraints should be 0 except initial state
                for (size_t i = 0; i < n_constraints; i++) {
                    constraints_lowerbound[i] = 0.0;
                    constraints_upperbound[i] = 0.0;
                }

                // Initial state constraints
                constraints_lowerbound[x_start] = current_state.pose.point.x;
                constraints_upperbound[x_start] = current_state.pose.point.x;

                constraints_lowerbound[y_start] = current_state.pose.point.y;
                constraints_upperbound[y_start] = current_state.pose.point.y;

                constraints_lowerbound[yaw_start] = current_state.pose.angle.yaw;
                constraints_upperbound[yaw_start] = current_state.pose.angle.yaw;

                constraints_lowerbound[v_start] = current_state.velocity.linear;
                constraints_upperbound[v_start] = current_state.velocity.linear;

                constraints_lowerbound[cte_start] = cte;
                constraints_upperbound[cte_start] = cte;

                constraints_lowerbound[epsi_start] = epsi;
                constraints_upperbound[epsi_start] = epsi;

                // Create FG_eval object
                FG_eval fg_eval(ref_trajectory, mpc_config_, constraints);

                // IPOPT options
                std::string options;
                options += "Integer print_level  " + std::to_string(mpc_config_.print_level) + "\n";
                options += "Sparse  true        forward\n";
                options += "Sparse  true        reverse\n";
                options += "Numeric max_cpu_time " + std::to_string(mpc_config_.max_solver_time) + "\n";

                // Solve the problem
                CppAD::ipopt::solve_result<Dvector> solution;
                CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound,
                                                      constraints_lowerbound, constraints_upperbound, fg_eval,
                                                      solution);

                // Check solution status
                result.success = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

                if (result.success) {
                    // Extract first control inputs
                    result.steering = solution.x[steering_start];
                    result.acceleration = solution.x[acceleration_start];

                    // Extract predicted trajectory
                    for (size_t i = 0; i < N + 1; i++) {
                        result.predicted_x.push_back(solution.x[x_start + i]);
                        result.predicted_y.push_back(solution.x[y_start + i]);
                    }

                    // Update warm start values
                    for (size_t i = 0; i < N - 1; i++) {
                        previous_steering_[i] = solution.x[steering_start + i + 1];
                        previous_acceleration_[i] = solution.x[acceleration_start + i + 1];
                    }
                    previous_steering_[N - 1] = solution.x[steering_start + N - 1];
                    previous_acceleration_[N - 1] = solution.x[acceleration_start + N - 1];
                }

                return result;
            }

            // FG_eval implementation
            FG_eval::FG_eval(const MPCFollower::ReferenceTrajectory &ref, const MPCFollower::MPCConfig &config,
                             const RobotConstraints &constraints)
                : ref_trajectory_(ref), mpc_config_(config), robot_constraints_(constraints) {
                // Set up variable indices
                size_t N = mpc_config_.horizon_steps;
                x_start_ = 0;
                y_start_ = x_start_ + N + 1;
                yaw_start_ = y_start_ + N + 1;
                v_start_ = yaw_start_ + N + 1;
                cte_start_ = v_start_ + N + 1;
                epsi_start_ = cte_start_ + N + 1;
                steering_start_ = epsi_start_ + N + 1;
                acceleration_start_ = steering_start_ + N;
            }

            void FG_eval::operator()(ADvector &fg, const ADvector &vars) {
                using CppAD::AD;

                size_t N = mpc_config_.horizon_steps;

                // Cost function
                fg[0] = 0.0;

                // Cost based on reference state
                for (size_t i = 0; i < N + 1; i++) {
                    fg[0] += mpc_config_.weight_cte * CppAD::pow(vars[cte_start_ + i], 2);
                    fg[0] += mpc_config_.weight_epsi * CppAD::pow(vars[epsi_start_ + i], 2);

                    if (i < ref_trajectory_.velocity.size()) {
                        fg[0] +=
                            mpc_config_.weight_vel * CppAD::pow(vars[v_start_ + i] - ref_trajectory_.velocity[i], 2);
                    }
                }

                // Cost based on control effort
                for (size_t i = 0; i < N; i++) {
                    fg[0] += mpc_config_.weight_steering * CppAD::pow(vars[steering_start_ + i], 2);
                    fg[0] += mpc_config_.weight_acceleration * CppAD::pow(vars[acceleration_start_ + i], 2);
                }

                // Cost based on control rate (smoothness)
                for (size_t i = 0; i < N - 1; i++) {
                    fg[0] += mpc_config_.weight_steering_rate *
                             CppAD::pow(vars[steering_start_ + i + 1] - vars[steering_start_ + i], 2);
                    fg[0] += mpc_config_.weight_acceleration_rate *
                             CppAD::pow(vars[acceleration_start_ + i + 1] - vars[acceleration_start_ + i], 2);
                }

                // Setup constraints
                // Initial state constraints (index 0 of fg is cost, so constraints start at index 1)
                fg[1 + x_start_] = vars[x_start_];
                fg[1 + y_start_] = vars[y_start_];
                fg[1 + yaw_start_] = vars[yaw_start_];
                fg[1 + v_start_] = vars[v_start_];
                fg[1 + cte_start_] = vars[cte_start_];
                fg[1 + epsi_start_] = vars[epsi_start_];

                // Dynamics constraints
                for (size_t i = 0; i < N; i++) {
                    // State at time t
                    AD<double> x0 = vars[x_start_ + i];
                    AD<double> y0 = vars[y_start_ + i];
                    AD<double> yaw0 = vars[yaw_start_ + i];
                    AD<double> v0 = vars[v_start_ + i];
                    AD<double> cte0 = vars[cte_start_ + i];
                    AD<double> epsi0 = vars[epsi_start_ + i];

                    // State at time t+1
                    AD<double> x1 = vars[x_start_ + i + 1];
                    AD<double> y1 = vars[y_start_ + i + 1];
                    AD<double> yaw1 = vars[yaw_start_ + i + 1];
                    AD<double> v1 = vars[v_start_ + i + 1];
                    AD<double> cte1 = vars[cte_start_ + i + 1];
                    AD<double> epsi1 = vars[epsi_start_ + i + 1];

                    // Actuators at time t
                    AD<double> steering = vars[steering_start_ + i];
                    AD<double> acceleration = vars[acceleration_start_ + i];

                    // Kinematic bicycle model
                    double dt = mpc_config_.dt;
                    double Lf = robot_constraints_.wheelbase;

                    // Reference values at this timestep
                    AD<double> ref_x = 0.0;
                    AD<double> ref_y = 0.0;
                    AD<double> ref_yaw = 0.0;

                    if (i < ref_trajectory_.x.size()) {
                        ref_x = ref_trajectory_.x[i];
                        ref_y = ref_trajectory_.y[i];
                        ref_yaw = ref_trajectory_.yaw[i];
                    }

                    // State transition constraints
                    // x(t+1) = x(t) + v(t) * cos(yaw(t)) * dt
                    fg[2 + x_start_ + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * dt);

                    // y(t+1) = y(t) + v(t) * sin(yaw(t)) * dt
                    fg[2 + y_start_ + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * dt);

                    // yaw(t+1) = yaw(t) + v(t) / Lf * tan(steering(t)) * dt
                    // Using small angle approximation: tan(x) ≈ x for small x
                    fg[2 + yaw_start_ + i] = yaw1 - (yaw0 + v0 * steering / Lf * dt);

                    // v(t+1) = v(t) + acceleration(t) * dt
                    fg[2 + v_start_ + i] = v1 - (v0 + acceleration * dt);

                    // cte(t+1) = (ref_y - y(t)) + v(t) * sin(epsi(t)) * dt
                    // Simplified: cte evolves based on velocity and heading error
                    fg[2 + cte_start_ + i] = cte1 - (cte0 + v0 * CppAD::sin(epsi0) * dt);

                    // epsi(t+1) = (yaw(t) - ref_yaw) + v(t) / Lf * steering(t) * dt
                    fg[2 + epsi_start_ + i] = epsi1 - (epsi0 + v0 * steering / Lf * dt);
                }
            }

        } // namespace path
    } // namespace tracking
} // namespace navcon
