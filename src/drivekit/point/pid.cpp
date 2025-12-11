#include "drivekit/point/pid.hpp"
#include <algorithm>
#include <cmath>

namespace drivekit {
    namespace point {

        PIDFollower::PIDFollower() { reset(); }

        VelocityCommand PIDFollower::compute_control(const RobotState &current_state, const Goal &goal,
                                                     const RobotConstraints &constraints, double dt,
                                                     const WorldConstraints *dynamic_constraints) {
            VelocityCommand cmd;

            // Check if goal is reached
            if (this->is_goal_reached(current_state.pose, goal.target_pose)) {
                cmd.valid = true;
                cmd.status_message = "Goal reached";
                status_.goal_reached = true;
                status_.mode = "stopped";
                return cmd;
            }

            // Determine if this is a diff/skid drive that supports turn_first
            const bool is_diff_drive = (constraints.steering_type == SteeringType::DIFFERENTIAL ||
                                        constraints.steering_type == SteeringType::SKID_STEER);

            // Calculate errors
            double dx = goal.target_pose.point.x - current_state.pose.point.x;
            double dy = goal.target_pose.point.y - current_state.pose.point.y;
            double distance_error = std::sqrt(dx * dx + dy * dy);
            double heading_error = this->calculate_heading_error(current_state.pose, goal.target_pose.point);

            // Update status
            status_.distance_to_goal = distance_error;
            status_.heading_error = heading_error;
            status_.goal_reached = false;

            // PID for angular control with stronger gains
            double angular_control = 0.0;
            const double heading_deadband = 0.1; // 0.1 radians (~6 degrees) deadband

            if (std::abs(heading_error) > heading_deadband) {
                angular_integral_ += heading_error * dt;
                double angular_derivative = (heading_error - last_heading_error_) / dt;
                last_heading_error_ = heading_error;

                // Use much stronger proportional gain for angular control
                angular_control = (config_.kp_angular * 3.0) * heading_error + config_.ki_angular * angular_integral_ +
                                  config_.kd_angular * angular_derivative;
            } else {
                // Within deadband - stop turning and reset integral
                angular_integral_ = 0.0;
                last_heading_error_ = 0.0;
                angular_control = 0.0;
            }

            // Check turn_first mode for diff/skid drive: rotate in place until aligned
            if (is_diff_drive && current_state.turn_first && std::abs(heading_error) > config_.angular_tolerance) {
                // Turn in place - zero linear velocity until aligned
                cmd.linear_velocity = 0.0;
                cmd.angular_velocity =
                    std::clamp(angular_control, -constraints.max_angular_velocity, constraints.max_angular_velocity);

                cmd.valid = true;
                cmd.status_message = "Turning to align";
                status_.mode = "turning";

                return cmd;
            }

            // PID for linear control
            linear_integral_ += distance_error * dt;
            double linear_derivative = (distance_error - last_distance_error_) / dt;
            last_distance_error_ = distance_error;

            double linear_control = config_.kp_linear * distance_error + config_.ki_linear * linear_integral_ +
                                    config_.kd_linear * linear_derivative;

            // Reduce linear speed when turning - very aggressive reduction
            double turn_reduction = 1.0 - std::min(std::abs(heading_error) / (M_PI / 6),
                                                   0.95);     // Start reducing at 30°, max 95% reduction
            linear_control *= std::max(turn_reduction, 0.05); // Keep minimum 5% speed

            // If heading error is large, prioritize turning over moving forward
            if (std::abs(heading_error) > M_PI / 3) { // 60 degrees
                linear_control *= 0.1;                // Reduce to 10% speed when heading error is large
            }

            // Apply control to velocity command
            cmd.linear_velocity =
                std::clamp(linear_control, constraints.min_linear_velocity, constraints.max_linear_velocity);
            cmd.angular_velocity =
                std::clamp(angular_control, -constraints.max_angular_velocity, constraints.max_angular_velocity);

            cmd.valid = true;
            cmd.status_message = "Tracking goal";
            status_.mode = "tracking";

            return cmd;
        }

        void PIDFollower::reset() {
            Base::reset();
            linear_integral_ = 0.0;
            angular_integral_ = 0.0;
            last_distance_error_ = 0.0;
            last_heading_error_ = 0.0;
        }

        std::string PIDFollower::get_type() const { return "pid_follower"; }

    } // namespace point
} // namespace drivekit
