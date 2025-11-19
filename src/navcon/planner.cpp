#include "navcon/planner.hpp"
#include <cmath>
#include <iostream>

namespace navcon {

    // Planner constructor
    Planner::Planner(PlannerType type) : planner_type(type) {}

    // Initialize with robot constraints and recording stream
    void Planner::init(const RobotConstraints &robot_constraints,
                       std::shared_ptr<rerun::RecordingStream> recording_stream, const std::string &prefix) {
        constraints_ = robot_constraints;
#ifdef HAS_RERUN
        rec = recording_stream;
        entity_prefix = prefix;
#else
        (void)recording_stream;
        (void)prefix;
#endif
        create_planner();
    }

    // Goal management
    void Planner::set_start(const Pose &start) {
        std::cout << "Planner: Setting start to (" << start.point.x << ", " << start.point.y << ")" << std::endl;
        start_pose = start;
        planning_complete = false;
    }

    void Planner::set_goal(const Pose &goal) {
        std::cout << "Planner: Setting goal to (" << goal.point.x << ", " << goal.point.y << ")" << std::endl;
        goal_pose = goal;
        planning_complete = false;
    }

    void Planner::clear_start() {
        start_pose.reset();
        planning_complete = false;
    }

    void Planner::clear_goal() {
        goal_pose.reset();
        planning_complete = false;
    }

    // Planning control - returns path
    Path Planner::plan() {
        planning_complete = false;

        if (!start_pose.has_value() || !goal_pose.has_value()) {
            std::cout << "Planner: Cannot plan without start and goal" << std::endl;
            return Path{};
        }

        Path result;
        if (controller) {
            result = controller->compute_path(start_pose.value(), goal_pose.value(), constraints_);
        }

        planning_complete = true;
        return result;
    }

    // Status
    bool Planner::is_planning_complete() const { return planning_complete; }

    // Planner configuration
    void Planner::set_planner_type(PlannerType type) {
        planner_type = type;
        create_planner();
    }

    void Planner::set_planner_params(const PlannerParams &new_params) { params = new_params; }

    const Planner::PlannerParams &Planner::get_planner_params() const { return params; }

    // Get results
    const Path &Planner::get_path() const {
        if (controller) {
            return controller->get_path();
        }
        static Path empty_path;
        return empty_path;
    }

    // Direct access to controller
    planning::Controller *Planner::get_controller() { return controller.get(); }

    const planning::Controller *Planner::get_controller() const { return controller.get(); }

    // Clear planning state
    void Planner::reset() {
        start_pose.reset();
        goal_pose.reset();
        planning_complete = false;
        if (controller) {
            controller->reset();
        }
    }

    // Visualization
    void Planner::tock() const {
#ifdef HAS_RERUN
        if (!rec) {
            return;
        }

        // Visualize start pose
        if (start_pose.has_value()) {
            rec->log_static(entity_prefix + "/start", rerun::Points3D({{static_cast<float>(start_pose->point.x),
                                                                        static_cast<float>(start_pose->point.y), 0.0f}})
                                                          .with_colors({{0, 255, 0}}) // Green for start
                                                          .with_radii({{0.15f}}));
        }

        // Visualize goal pose
        if (goal_pose.has_value()) {
            rec->log_static(entity_prefix + "/goal", rerun::Points3D({{static_cast<float>(goal_pose->point.x),
                                                                       static_cast<float>(goal_pose->point.y), 0.0f}})
                                                         .with_colors({{255, 0, 0}}) // Red for goal
                                                         .with_radii({{0.15f}}));
        }

        // Visualize planned path
        if (controller) {
            const auto &current_path = controller->get_path();
            if (!current_path.waypoints.empty()) {
                std::vector<std::array<float, 3>> path_points;
                for (const auto &waypoint : current_path.waypoints) {
                    path_points.push_back(
                        {static_cast<float>(waypoint.point.x), static_cast<float>(waypoint.point.y), 0.0f});
                }

                if (path_points.size() >= 2) {
                    auto path_line = rerun::components::LineStrip3D(path_points);
                    rec->log_static(entity_prefix + "/path",
                                    rerun::LineStrips3D(path_line)
                                        .with_colors({{0, 0, 255, 200}}) // Blue for planned path
                                        .with_radii({{0.05f}}));
                }

                // Visualize waypoints
                std::vector<rerun::components::Position3D> waypoint_positions;
                for (const auto &waypoint : current_path.waypoints) {
                    waypoint_positions.push_back(
                        {static_cast<float>(waypoint.point.x), static_cast<float>(waypoint.point.y), 0.0f});
                }

                if (!waypoint_positions.empty()) {
                    rec->log_static(entity_prefix + "/waypoints", rerun::Points3D(waypoint_positions)
                                                                      .with_colors({{0, 0, 255}}) // Blue waypoints
                                                                      .with_radii({{0.07f}}));
                }
            }
        }
#else
        (void)0;
#endif
    }

    // Internal helper methods
    void Planner::create_planner() {
        switch (planner_type) {
        case PlannerType::ASTAR:
            std::cout << "Creating A* planner..." << std::endl;
            controller = std::make_unique<planning::algorithms::AStarPlanner>();
            break;
        case PlannerType::RRT:
            std::cout << "Creating RRT planner (not yet implemented)..." << std::endl;
            controller = std::make_unique<planning::algorithms::AStarPlanner>(); // Fallback
            break;
        case PlannerType::PRM:
            std::cout << "Creating PRM planner (not yet implemented)..." << std::endl;
            controller = std::make_unique<planning::algorithms::AStarPlanner>(); // Fallback
            break;
        }
    }

} // namespace navcon
