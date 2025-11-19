#pragma once

#include "navcon/types.hpp"
#include <string>

namespace navcon {
    namespace planning {

        // Planner configuration
        struct PlannerConfig {
            // General parameters
            double grid_resolution = 0.1; // meters
            double robot_radius = 0.3;    // meters

            // A* parameters
            double heuristic_weight = 1.0;

            // RRT parameters
            double step_size = 0.5;
            int max_iterations = 1000;
            double goal_bias = 0.1;

            // PRM parameters
            int num_samples = 500;
            double connection_radius = 1.0;
        };

        // Planner status
        struct PlannerStatus {
            bool planning_complete = false;
            double path_length = 0.0;
            int nodes_expanded = 0;
            double planning_time = 0.0; // seconds
            std::string mode;           // e.g., "planning", "complete", "failed"
        };

    } // namespace planning
} // namespace navcon
