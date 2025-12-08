#include "navcon/pred/mpc_trailer.hpp"
#include "navcon/types.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace navcon {
    namespace pred {

        MPCTrailerFollower::MPCTrailerFollower() : MPCTrailerFollower(MPCConfig{}) {}

        MPCTrailerFollower::MPCTrailerFollower(const MPCConfig &mpc_config) : mpc_config_(mpc_config) {
            previous_steering_.resize(mpc_config_.horizon_steps, 0.0);
            previous_acceleration_.resize(mpc_config_.horizon_steps, 0.0);
        }

        VelocityCommand MPCTrailerFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                            const RobotConstraints &constraints, double dt,
                                                            const WorldConstraints *world_constraints) {
            (void)world_constraints;
            (void)dt;

            // Use trailer state for path tracking when available
            const Pose &tracking_pose = current_state.has_trailer ? current_state.trailer_pose : current_state.pose;

            VelocityCommand cmd;
            cmd.valid = false;

            if (path_.waypoints.empty()) {
                return cmd;
            }

            if (is_goal_reached(tracking_pose, goal.target_pose)) {
                cmd.valid = true;
                cmd.status_message = "Goal reached";
                status_.goal_reached = true;
                status_.mode = "stopped";
                cmd.linear_velocity = 0.0;
                cmd.angular_velocity = 0.0;
                return cmd;
            }

            PathError error = calculate_path_error(current_state);
            ReferenceTrajectory ref_traj = calculate_reference_trajectory(error, current_state, constraints);

            MPCSolution solution = solve_mpc(current_state, ref_traj, constraints, error.cte, error.epsi);
            if (!solution.success) {
                cmd.valid = false;
                cmd.status_message = "MPCTrailer solver failed";
                return cmd;
            }

            // Debug logging: trailer vs reference path tracking
            {
                static int debug_counter = 0;
                if ((debug_counter++ % 10) == 0) {
                    const Pose &trailer_pose =
                        current_state.has_trailer ? current_state.trailer_pose : current_state.pose;

                    const double ref_x = !ref_traj.x.empty() ? ref_traj.x.front() : 0.0;
                    const double ref_y = !ref_traj.y.empty() ? ref_traj.y.front() : 0.0;
                    const double ref_yaw = !ref_traj.yaw.empty() ? ref_traj.yaw.front() : 0.0;

                    const double beta =
                        current_state.has_trailer
                            ? normalize_angle(current_state.pose.angle.yaw - current_state.trailer_pose.angle.yaw)
                            : 0.0;

                    std::cout << "[MPCTrailer] "
                              << "pose=(" << trailer_pose.point.x << "," << trailer_pose.point.y << "), "
                              << "yaw=" << trailer_pose.angle.yaw << ", "
                              << "ref=(" << ref_x << "," << ref_y << "), "
                              << "ref_yaw=" << ref_yaw << ", "
                              << "cte=" << error.cte << ", epsi=" << error.epsi << ", "
                              << "beta=" << beta << ", "
                              << "v=" << current_state.velocity.linear << ", "
                              << "u=(steer=" << solution.steering << ", acc=" << solution.acceleration << ")"
                              << std::endl;
                }
            }

            previous_steering_[0] = solution.steering;
            previous_acceleration_[0] = solution.acceleration;

            predicted_trajectory_.clear();
            for (size_t i = 0; i < solution.predicted_x.size(); ++i) {
                Point p;
                p.x = solution.predicted_x[i];
                p.y = solution.predicted_y[i];
                p.z = 0.0;
                predicted_trajectory_.push_back(p);
            }

            status_.distance_to_goal = tracking_pose.point.distance_to(goal.target_pose.point);
            status_.cross_track_error = std::abs(error.cte);
            status_.heading_error = std::abs(error.epsi);
            status_.goal_reached = false;
            status_.mode = "mpc_trailer_tracking";

            // Allow reverse: clamp to [min_linear_velocity, max_linear_velocity]
            double target_velocity = mpc_config_.ref_velocity + solution.acceleration * mpc_config_.dt;
            target_velocity =
                std::clamp(target_velocity, constraints.min_linear_velocity, constraints.max_linear_velocity);

            double steering_angle = solution.steering;
            steering_angle =
                std::clamp(steering_angle, -constraints.max_steering_angle, constraints.max_steering_angle);

            if (config_.output_units == OutputUnits::NORMALIZED) {
                // Map symmetric velocity range to [-1,1]
                double max_abs_vel =
                    std::max(std::abs(constraints.max_linear_velocity), std::abs(constraints.min_linear_velocity));
                if (max_abs_vel <= 0.0) {
                    cmd.linear_velocity = 0.0;
                } else {
                    cmd.linear_velocity = target_velocity / max_abs_vel;
                }
                cmd.angular_velocity = steering_angle / constraints.max_steering_angle;
            } else {
                cmd.linear_velocity = target_velocity;
                cmd.angular_velocity = steering_angle;
            }

            cmd.valid = true;
            cmd.status_message = "MPCTrailer tracking";
            return cmd;
        }

        std::string MPCTrailerFollower::get_type() const { return "mpc_trailer_follower"; }

        void MPCTrailerFollower::set_mpc_config(const MPCConfig &config) {
            mpc_config_ = config;
            previous_steering_.resize(config.horizon_steps, 0.0);
            previous_acceleration_.resize(config.horizon_steps, 0.0);
        }

        MPCTrailerFollower::MPCConfig MPCTrailerFollower::get_mpc_config() const { return mpc_config_; }

        MPCTrailerFollower::PathError MPCTrailerFollower::calculate_path_error(const RobotState &current_state) {
            PathError result{};

            // Use trailer pose when available, otherwise fall back to primary pose.
            const Pose &tracking_pose = current_state.has_trailer ? current_state.trailer_pose : current_state.pose;

            if (path_.waypoints.empty()) {
                result.nearest_index = 0;
                result.cte = 0.0;
                result.epsi = 0.0;
                result.nearest_point = tracking_pose.point;
                result.path_heading = tracking_pose.angle.yaw;
                return result;
            }

            double min_dist = std::numeric_limits<double>::max();
            size_t nearest_idx = 0;

            for (size_t i = 0; i < path_.waypoints.size(); ++i) {
                double dist = tracking_pose.point.distance_to(path_.waypoints[i].point);
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_idx = i;
                }
            }

            result.nearest_index = nearest_idx;
            result.nearest_point = path_.waypoints[nearest_idx].point;

            double path_heading = 0.0;
            if (nearest_idx < path_.waypoints.size() - 1) {
                Point next = path_.waypoints[nearest_idx + 1].point;
                Point curr = path_.waypoints[nearest_idx].point;
                path_heading = std::atan2(next.y - curr.y, next.x - curr.x);
            } else if (nearest_idx > 0) {
                Point next = path_.waypoints[nearest_idx].point;
                Point curr = path_.waypoints[nearest_idx - 1].point;
                path_heading = std::atan2(next.y - curr.y, next.x - curr.x);
            } else {
                path_heading = path_.waypoints[nearest_idx].angle.yaw;
            }
            // For reverse motion, treat the path direction as "backwards" so that
            // the trailer can align while moving in reverse along the path.
            if (mpc_config_.ref_velocity < 0.0) {
                path_heading = this->normalize_angle(path_heading + M_PI);
            }
            result.path_heading = path_heading;

            double dx = tracking_pose.point.x - result.nearest_point.x;
            double dy = tracking_pose.point.y - result.nearest_point.y;

            result.cte = -dx * std::sin(result.path_heading) + dy * std::cos(result.path_heading);
            const double heading =
                current_state.has_trailer ? current_state.trailer_pose.angle.yaw : current_state.pose.angle.yaw;
            result.epsi = normalize_angle(heading - result.path_heading);
            path_index_ = nearest_idx;

            return result;
        }

        MPCTrailerFollower::ReferenceTrajectory
        MPCTrailerFollower::calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
                                                           const RobotConstraints &constraints) {
            (void)current_state;
            (void)constraints;

            ReferenceTrajectory ref;
            size_t start_idx = error.nearest_index;

            for (size_t i = 0; i < mpc_config_.horizon_steps + 1; ++i) {
                // Use absolute reference speed so geometry of preview does not
                // depend on sign (forward vs reverse).
                double distance_ahead = std::abs(mpc_config_.ref_velocity) * mpc_config_.dt * i;

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
                // For reverse motion, define reference yaw as facing opposite to
                // the geometric path direction, so that backing along the path
                // produces small heading error.
                if (mpc_config_.ref_velocity < 0.0) {
                    yaw = this->normalize_angle(yaw + M_PI);
                }
                ref.yaw.push_back(yaw);
                ref.velocity.push_back(mpc_config_.ref_velocity);
            }

            return ref;
        }

        MPCTrailerFollower::MPCSolution MPCTrailerFollower::solve_mpc(const RobotState &current_state,
                                                                      const ReferenceTrajectory &ref_trajectory,
                                                                      const RobotConstraints &constraints, double cte,
                                                                      double epsi) {
            using CppAD::AD;
            using Dvector = CPPAD_TESTVECTOR(double);

            MPCSolution result;
            result.success = false;

            size_t N = mpc_config_.horizon_steps;
            // State vector per timestep:
            // [x_trailer, y_trailer, yaw_trailer, yaw_tractor, v_tractor, cte, epsi]
            size_t n_states = 7;
            size_t n_actuators = 2;

            size_t n_vars = n_states * (N + 1) + n_actuators * N;
            size_t n_constraints = n_states * (N + 1) + (N - 1);

            Dvector vars(n_vars);
            for (size_t i = 0; i < n_vars; i++) {
                vars[i] = 0.0;
            }

            size_t x_trailer_start = 0;
            size_t y_trailer_start = x_trailer_start + N + 1;
            size_t yaw_trailer_start = y_trailer_start + N + 1;
            size_t yaw_tractor_start = yaw_trailer_start + N + 1;
            size_t v_start = yaw_tractor_start + N + 1;
            size_t cte_start = v_start + N + 1;
            size_t epsi_start = cte_start + N + 1;
            size_t steering_start = epsi_start + N + 1;
            size_t acceleration_start = steering_start + N;
            size_t steering_rate_start = n_states * (N + 1);

            const Pose &trailer_pose = current_state.has_trailer ? current_state.trailer_pose : current_state.pose;

            vars[x_trailer_start] = trailer_pose.point.x;
            vars[y_trailer_start] = trailer_pose.point.y;
            vars[yaw_trailer_start] = trailer_pose.angle.yaw;
            vars[yaw_tractor_start] = current_state.pose.angle.yaw;
            vars[v_start] = current_state.velocity.linear;
            vars[cte_start] = cte;
            vars[epsi_start] = epsi;

            for (size_t i = 0; i < N; i++) {
                if (i < previous_steering_.size()) {
                    vars[steering_start + i] = previous_steering_[i];
                }
                if (i < previous_acceleration_.size()) {
                    vars[acceleration_start + i] = previous_acceleration_[i];
                }
            }

            Dvector vars_lowerbound(n_vars);
            Dvector vars_upperbound(n_vars);

            for (size_t i = 0; i < steering_start; i++) {
                vars_lowerbound[i] = -1.0e19;
                vars_upperbound[i] = 1.0e19;
            }

            for (size_t i = steering_start; i < acceleration_start; i++) {
                vars_lowerbound[i] = -constraints.max_steering_angle;
                vars_upperbound[i] = constraints.max_steering_angle;
            }

            for (size_t i = acceleration_start; i < n_vars; i++) {
                vars_lowerbound[i] = -constraints.max_linear_acceleration;
                vars_upperbound[i] = constraints.max_linear_acceleration;
            }

            Dvector constraints_lowerbound(n_constraints);
            Dvector constraints_upperbound(n_constraints);

            for (size_t i = 0; i < n_constraints; i++) {
                constraints_lowerbound[i] = 0.0;
                constraints_upperbound[i] = 0.0;
            }

            constraints_lowerbound[x_trailer_start] = trailer_pose.point.x;
            constraints_upperbound[x_trailer_start] = trailer_pose.point.x;

            constraints_lowerbound[y_trailer_start] = trailer_pose.point.y;
            constraints_upperbound[y_trailer_start] = trailer_pose.point.y;

            constraints_lowerbound[yaw_trailer_start] = trailer_pose.angle.yaw;
            constraints_upperbound[yaw_trailer_start] = trailer_pose.angle.yaw;

            constraints_lowerbound[yaw_tractor_start] = current_state.pose.angle.yaw;
            constraints_upperbound[yaw_tractor_start] = current_state.pose.angle.yaw;

            constraints_lowerbound[v_start] = current_state.velocity.linear;
            constraints_upperbound[v_start] = current_state.velocity.linear;

            constraints_lowerbound[cte_start] = cte;
            constraints_upperbound[cte_start] = cte;

            constraints_lowerbound[epsi_start] = epsi;
            constraints_upperbound[epsi_start] = epsi;

            double steer_rate_bound = constraints.max_steering_rate * mpc_config_.dt;
            for (size_t i = 0; i < N - 1; ++i) {
                constraints_lowerbound[steering_rate_start + i] = -steer_rate_bound;
                constraints_upperbound[steering_rate_start + i] = steer_rate_bound;
            }

            TrailerFGEval fg_eval(ref_trajectory, mpc_config_, constraints);

            std::string options;
            options += "Integer print_level  " + std::to_string(mpc_config_.print_level) + "\n";
            options += "Sparse  true        forward\n";
            options += "Sparse  true        reverse\n";
            options += "Numeric max_cpu_time " + std::to_string(mpc_config_.max_solver_time) + "\n";

            CppAD::ipopt::solve_result<Dvector> solution;
            CppAD::ipopt::solve<Dvector, TrailerFGEval>(options, vars, vars_lowerbound, vars_upperbound,
                                                        constraints_lowerbound, constraints_upperbound, fg_eval,
                                                        solution);

            result.success = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

            if (result.success) {
                result.steering = solution.x[steering_start];
                result.acceleration = solution.x[acceleration_start];

                for (size_t i = 0; i < N + 1; i++) {
                    result.predicted_x.push_back(solution.x[x_trailer_start + i]);
                    result.predicted_y.push_back(solution.x[y_trailer_start + i]);
                }

                for (size_t i = 0; i < N - 1; i++) {
                    previous_steering_[i] = solution.x[steering_start + i + 1];
                    previous_acceleration_[i] = solution.x[acceleration_start + i + 1];
                }
                previous_steering_[N - 1] = solution.x[steering_start + N - 1];
                previous_acceleration_[N - 1] = solution.x[acceleration_start + N - 1];
            }

            return result;
        }

        TrailerFGEval::TrailerFGEval(const MPCTrailerFollower::ReferenceTrajectory &ref,
                                     const MPCTrailerFollower::MPCConfig &config, const RobotConstraints &constraints)
            : ref_trajectory_(ref), mpc_config_(config), robot_constraints_(constraints) {
            size_t N = mpc_config_.horizon_steps;
            size_t n_states = 7;

            x_trailer_start_ = 0;
            y_trailer_start_ = x_trailer_start_ + N + 1;
            yaw_trailer_start_ = y_trailer_start_ + N + 1;
            yaw_tractor_start_ = yaw_trailer_start_ + N + 1;
            v_start_ = yaw_tractor_start_ + N + 1;
            cte_start_ = v_start_ + N + 1;
            epsi_start_ = cte_start_ + N + 1;
            steering_start_ = epsi_start_ + N + 1;
            acceleration_start_ = steering_start_ + N;
            steering_rate_start_ = n_states * (N + 1);
            steering_rate_limit_ = robot_constraints_.max_steering_rate * mpc_config_.dt;
        }

        void TrailerFGEval::operator()(ADvector &fg, const ADvector &vars) {
            using CppAD::AD;

            size_t N = mpc_config_.horizon_steps;

            fg[0] = 0.0;

            for (size_t i = 0; i < N; i++) {
                fg[0] += mpc_config_.weight_cte * CppAD::pow(vars[cte_start_ + i], 2);
                fg[0] += mpc_config_.weight_epsi * CppAD::pow(vars[epsi_start_ + i], 2);

                if (i < ref_trajectory_.velocity.size()) {
                    fg[0] += mpc_config_.weight_vel * CppAD::pow(vars[v_start_ + i] - ref_trajectory_.velocity[i], 2);
                }
            }

            for (size_t i = 0; i < N; i++) {
                fg[0] += mpc_config_.weight_steering * CppAD::pow(vars[steering_start_ + i], 2);
                fg[0] += mpc_config_.weight_acceleration * CppAD::pow(vars[acceleration_start_ + i], 2);
            }

            for (size_t i = 0; i < N - 1; i++) {
                fg[0] += mpc_config_.weight_steering_rate *
                         CppAD::pow(vars[steering_start_ + i + 1] - vars[steering_start_ + i], 2);
                fg[0] += mpc_config_.weight_acceleration_rate *
                         CppAD::pow(vars[acceleration_start_ + i + 1] - vars[acceleration_start_ + i], 2);
            }

            // Penalize articulation angle beta = theta_tractor - theta_trailer
            for (size_t i = 0; i < N + 1; ++i) {
                AD<double> theta_trailer = vars[yaw_trailer_start_ + i];
                AD<double> theta_tractor = vars[yaw_tractor_start_ + i];
                AD<double> beta = theta_tractor - theta_trailer;
                fg[0] += mpc_config_.weight_beta * CppAD::pow(beta, 2);
            }

            fg[1 + x_trailer_start_] = vars[x_trailer_start_];
            fg[1 + y_trailer_start_] = vars[y_trailer_start_];
            fg[1 + yaw_trailer_start_] = vars[yaw_trailer_start_];
            fg[1 + yaw_tractor_start_] = vars[yaw_tractor_start_];
            fg[1 + v_start_] = vars[v_start_];
            fg[1 + cte_start_] = vars[cte_start_];
            fg[1 + epsi_start_] = vars[epsi_start_];

            for (size_t i = 0; i < N; i++) {
                AD<double> x_trailer_0 = vars[x_trailer_start_ + i];
                AD<double> y_trailer_0 = vars[y_trailer_start_ + i];
                AD<double> theta_trailer_0 = vars[yaw_trailer_start_ + i];
                AD<double> theta_tractor_0 = vars[yaw_tractor_start_ + i];
                AD<double> v0 = vars[v_start_ + i];
                AD<double> cte0 = vars[cte_start_ + i];
                AD<double> epsi0 = vars[epsi_start_ + i];

                AD<double> x_trailer_1 = vars[x_trailer_start_ + i + 1];
                AD<double> y_trailer_1 = vars[y_trailer_start_ + i + 1];
                AD<double> theta_trailer_1 = vars[yaw_trailer_start_ + i + 1];
                AD<double> theta_tractor_1 = vars[yaw_tractor_start_ + i + 1];
                AD<double> v1 = vars[v_start_ + i + 1];
                AD<double> cte1 = vars[cte_start_ + i + 1];
                AD<double> epsi1 = vars[epsi_start_ + i + 1];

                AD<double> steering = vars[steering_start_ + i];
                AD<double> acceleration = vars[acceleration_start_ + i];

                const double dt = mpc_config_.dt;

                const double L0 = (mpc_config_.truck_wheelbase > 1e-3) ? mpc_config_.truck_wheelbase
                                                                       : std::max(robot_constraints_.wheelbase, 1e-3);
                const double M0 = mpc_config_.truck_hitch_offset;
                const double L1 = (mpc_config_.trailer_hitch_length > 1e-3)
                                      ? mpc_config_.trailer_hitch_length
                                      : std::max(robot_constraints_.robot_length, 1e-3);

                AD<double> ref_yaw_rate = 0.0;
                if (i + 1 < ref_trajectory_.yaw.size()) {
                    ref_yaw_rate = (ref_trajectory_.yaw[i + 1] - ref_trajectory_.yaw[i]) / dt;
                }

                // Articulated truck–trailer kinematics (single trailer) closely
                // mirroring the model used in truck_trailer.py:
                //   dtheta0 = v0/L0 * tan(delta0)
                //   dtheta1 = v0/L1 * sin(beta) - M0/L1 * cos(beta) * dtheta0
                //   v1      = v0*cos(beta) + M0*sin(beta)*dtheta0
                AD<double> beta = theta_tractor_0 - theta_trailer_0;
                AD<double> dtheta_tractor = v0 / L0 * CppAD::tan(steering);
                AD<double> dtheta_trailer = v0 / L1 * CppAD::sin(beta) - M0 / L1 * CppAD::cos(beta) * dtheta_tractor;
                AD<double> v_trailer = v0 * CppAD::cos(beta) + M0 * CppAD::sin(beta) * dtheta_tractor;

                // Trailer motion
                fg[2 + x_trailer_start_ + i] =
                    x_trailer_1 - (x_trailer_0 + v_trailer * CppAD::cos(theta_trailer_0) * dt);
                fg[2 + y_trailer_start_ + i] =
                    y_trailer_1 - (y_trailer_0 + v_trailer * CppAD::sin(theta_trailer_0) * dt);
                fg[2 + yaw_trailer_start_ + i] = theta_trailer_1 - (theta_trailer_0 + dtheta_trailer * dt);

                // Tractor yaw evolution
                fg[2 + yaw_tractor_start_ + i] = theta_tractor_1 - (theta_tractor_0 + dtheta_tractor * dt);

                // Longitudinal speed
                fg[2 + v_start_ + i] = v1 - (v0 + acceleration * dt);

                // Error dynamics for trailer relative to reference path
                fg[2 + cte_start_ + i] = cte1 - (cte0 + v_trailer * CppAD::sin(epsi0) * dt);
                fg[2 + epsi_start_ + i] = epsi1 - (epsi0 + (dtheta_trailer - ref_yaw_rate) * dt);
            }

            for (size_t i = 0; i < N - 1; ++i) {
                fg[1 + steering_rate_start_ + i] = vars[steering_start_ + i + 1] - vars[steering_start_ + i];
            }
        }

    } // namespace pred
} // namespace navcon
