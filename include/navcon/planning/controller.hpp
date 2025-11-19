#pragma once

#include "navcon/planning/types.hpp"
#include <memory>
#include <string>

namespace navcon {
    namespace planning {

        // Abstract base planner interface
        class Controller {
          public:
            virtual ~Controller() = default;

            // Main planning computation
            virtual Path compute_path(const Pose &start, const Pose &goal, const RobotConstraints &constraints) = 0;

            // Reset planner state
            virtual void reset() { current_path_.waypoints.clear(); }

            // Get current path
            virtual const Path &get_path() const { return current_path_; }

            // Configure planner
            virtual void set_config(const PlannerConfig &config) { config_ = config; }

            virtual PlannerConfig get_config() const { return config_; }

            // Get planner type name
            virtual std::string get_type() const = 0;

          protected:
            PlannerConfig config_;
            Path current_path_;
        };

    } // namespace planning
} // namespace navcon
