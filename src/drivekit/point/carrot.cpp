#include "drivekit/point/carrot.hpp"
#include <algorithm>
#include <cmath>

namespace drivekit {
    namespace point {

        VelocityCommand CarrotFollower::compute_control(const RobotState &current_state, const Goal &goal,
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

        std::string CarrotFollower::get_type() const { return "carrot_follower"; }

    } // namespace point
} // namespace drivekit
