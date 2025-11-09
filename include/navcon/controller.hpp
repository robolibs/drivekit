#pragma once

#include "navcon/types.hpp"
#include <cmath>
#include <memory>

namespace navcon {

    // Abstract base controller interface
    class Controller {
      public:
        virtual ~Controller() = default;

        // Main control computation
        virtual VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                const RobotConstraints &constraints, double dt) = 0;

        // Optional path following
        virtual void set_path(const Path &path) {
            path_ = path;
            path_index_ = 0;
        }

        // Reset controller state
        virtual void reset() {
            path_.waypoints.clear();
            path_index_ = 0;
            status_ = ControllerStatus{};
        }

        // Get current status
        virtual ControllerStatus get_status() const { return status_; }

        // Configure controller
        virtual void set_config(const ControllerConfig &config) { config_ = config; }

        virtual ControllerConfig get_config() const { return config_; }

        // Get controller type name
        virtual std::string get_type() const = 0;

      protected:
        ControllerConfig config_;
        ControllerStatus status_;
        Path path_;
        size_t path_index_ = 0;

        // Helper functions
        double normalize_angle(double angle);

        double calculate_distance(const Point &p1, const Point &p2);

        double calculate_heading_error(const Pose &current, const Point &target);

        bool is_goal_reached(const Pose &current, const Pose &goal);
    };

} // namespace navcon