#pragma once

#include "drivekit/types.hpp"
#include <cmath>
#include <memory>

namespace drivekit {

    /// Abstract base controller interface for all path/point following controllers.
    class Controller {
      public:
        virtual ~Controller() = default;

        /// Main control computation.
        /// @param current_state Current robot state (pose, velocity, etc.)
        /// @param goal Target goal to reach
        /// @param constraints Robot physical constraints
        /// @param dt Time step in seconds
        /// @param dynamic_constraints Optional runtime constraints (obstacles, zones)
        /// @return Velocity command to execute
        virtual VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                const RobotConstraints &constraints, double dt,
                                                const WorldConstraints *dynamic_constraints = nullptr) = 0;

        /// Set path for path-following controllers.
        virtual void set_path(const Path &path) {
            path_ = path;
            path_index_ = 0;
        }

        /// Reset controller state.
        virtual void reset() {
            path_.drivekits.clear();
            path_index_ = 0;
            status_ = ControllerStatus{};
        }

        /// Get current controller status.
        virtual ControllerStatus get_status() const { return status_; }

        /// Configure controller parameters.
        virtual void set_config(const ControllerConfig &config) { config_ = config; }

        /// Get current controller configuration.
        virtual ControllerConfig get_config() const { return config_; }

        /// Get controller type name.
        virtual std::string get_type() const = 0;

        /// Get current path index (for visualization/debugging).
        virtual size_t get_path_index() const { return path_index_; }

      protected:
        ControllerConfig config_;
        ControllerStatus status_;
        Path path_;
        size_t path_index_ = 0;

        /// Normalize angle to [-pi, pi] range.
        inline double normalize_angle(double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        /// Calculate Euclidean distance between two points.
        inline double calculate_distance(const Point &p1, const Point &p2) { return p1.distance_to(p2); }

        /// Calculate heading error from current pose to target point.
        inline double calculate_heading_error(const Pose &current, const Point &target) {
            double dx = target.x - current.point.x;
            double dy = target.y - current.point.y;
            double desired_heading = std::atan2(dy, dx);
            return normalize_angle(desired_heading - current.rotation.to_euler().yaw);
        }

        /// Check if goal is reached within tolerance.
        inline bool is_goal_reached(const Pose &current, const Pose &goal, double tolerance = -1.0) {
            double dist = calculate_distance(current.point, goal.point);
            double angle_diff =
                std::abs(normalize_angle(goal.rotation.to_euler().yaw - current.rotation.to_euler().yaw));

            status_.distance_to_goal = dist;
            status_.heading_error = angle_diff;

            // Use provided tolerance if positive, otherwise use config default
            double pos_tol = (tolerance > 0.0) ? tolerance : config_.goal_tolerance;
            return dist < pos_tol && angle_diff < config_.angular_tolerance;
        }
    };

} // namespace drivekit