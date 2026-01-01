#pragma once

#include "drivekit/controller.hpp"
#include <algorithm>
#include <cmath>

namespace drivekit {
    namespace point {

        /// Simple carrot chasing controller for basic navigation.
        /// Follows a "carrot" point ahead of the robot with proportional control.
        class CarrotFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::status_;

            inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                   const RobotConstraints &constraints, double dt,
                                                   const WorldConstraints *dynamic_constraints = nullptr) override {
                (void)dt;
                (void)dynamic_constraints;
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

                // Calculate distance and heading to goal
                double dx = goal.target_pose.point.x - current_state.pose.point.x;
                double dy = goal.target_pose.point.y - current_state.pose.point.y;
                double distance = std::sqrt(dx * dx + dy * dy);
                double heading_error = this->calculate_heading_error(current_state.pose, goal.target_pose.point);

                // Update status
                status_.distance_to_goal = distance;
                status_.heading_error = heading_error;
                status_.goal_reached = false;

                // Simple proportional control
                double angular_control = config_.kp_angular * heading_error;

                // Check turn_first mode for diff/skid drive: rotate in place until aligned
                if (is_diff_drive && current_state.turn_first && std::abs(heading_error) > config_.angular_tolerance) {
                    // Turn in place - zero linear velocity until aligned
                    cmd.linear_velocity = 0.0;
                    cmd.angular_velocity = std::clamp(angular_control, -constraints.max_angular_velocity,
                                                      constraints.max_angular_velocity);

                    cmd.valid = true;
                    cmd.status_message = "Turning to align";
                    status_.mode = "turning";

                    return cmd;
                }

                // Reduce speed when turning sharply
                double turn_reduction = 1.0 - std::min(std::abs(heading_error) / M_PI, 0.8);

                // Scale speed based on distance (slow down near goal)
                double distance_scale = std::min(distance / (config_.goal_tolerance * 5.0), 1.0);

                double linear_control = config_.kp_linear * distance * turn_reduction * distance_scale;

                // Apply carrot control for velocity command
                cmd.linear_velocity =
                    std::clamp(linear_control, 0.0, constraints.max_linear_velocity); // Only forward for carrot
                cmd.angular_velocity =
                    std::clamp(angular_control, -constraints.max_angular_velocity, constraints.max_angular_velocity);

                cmd.valid = true;
                cmd.status_message = "Chasing carrot";
                status_.mode = "carrot";

                return cmd;
            }

            inline std::string get_type() const override { return "carrot_follower"; }
        };

    } // namespace point
} // namespace drivekit
