#pragma once

#include "concord/concord.hpp"
#include "navcon/planning/algorithms/astar.hpp"
#include "navcon/planning/controller.hpp"
#include "navcon/types.hpp"
#include <memory>
#include <optional>
#include <vector>

#ifdef HAS_RERUN
#include <rerun.hpp>
#else
// Forward declarations when rerun is not available
namespace rerun {
    class RecordingStream;
}
#endif

namespace navcon {

    // Planner types
    enum class PlannerType { ASTAR, RRT, PRM };

    // ============================================================================
    // Planner - Path planning
    // ============================================================================
    class Planner {
      private:
        PlannerType planner_type;
        std::unique_ptr<planning::Controller> controller;

        // Current planning state
        std::optional<Pose> start_pose;
        std::optional<Pose> goal_pose;
        bool planning_complete = false;

        // Robot constraints (set during initialization)
        RobotConstraints constraints_;

#ifdef HAS_RERUN
        // Recording stream for visualization
        std::shared_ptr<rerun::RecordingStream> rec;
#endif

        // Entity prefix for visualization
        std::string entity_prefix = "planning";

        // Planner parameters
        struct PlannerParams {
            // General parameters
            float grid_resolution = 0.1f; // meters
            float robot_radius = 0.3f;    // meters

            // A* parameters
            float heuristic_weight = 1.0f;

            // RRT parameters
            float step_size = 0.5f;
            int max_iterations = 1000;
            float goal_bias = 0.1f;

            // PRM parameters
            int num_samples = 500;
            float connection_radius = 1.0f;
        } params;

      public:
        Planner(PlannerType type = PlannerType::ASTAR);
        ~Planner() = default;

        // Initialize with robot constraints and recording stream
        void init(const RobotConstraints &robot_constraints,
                  std::shared_ptr<rerun::RecordingStream> recording_stream = nullptr,
                  const std::string &prefix = "planning");

        // Goal management
        void set_start(const Pose &start);
        void set_goal(const Pose &goal);
        void clear_start();
        void clear_goal();

        // Planning control - returns path
        Path plan();

        // Status
        bool is_planning_complete() const;

        // Planner configuration
        void set_planner_type(PlannerType type);
        void set_planner_params(const PlannerParams &new_params);
        const PlannerParams &get_planner_params() const;

        // Get results
        const Path &get_path() const;

        // Direct access to controller
        planning::Controller *get_controller();
        const planning::Controller *get_controller() const;

        // Clear planning state
        void reset();

        // Visualization
        void tock() const;

      private:
        // Internal helper methods
        void create_planner();
    };

} // namespace navcon
