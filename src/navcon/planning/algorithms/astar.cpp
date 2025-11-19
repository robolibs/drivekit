#include "navcon/planning/algorithms/astar.hpp"
#include <iostream>

namespace navcon {
    namespace planning {
        namespace algorithms {

            Path AStarPlanner::compute_path(const Pose &start, const Pose &goal, const RobotConstraints &constraints) {
                (void)constraints; // TODO: use constraints

                current_path_.waypoints.clear();

                // TODO: Implement A* algorithm
                std::cout << "A* planning not yet implemented - returning straight line" << std::endl;

                // For now, just return a straight line path
                current_path_.waypoints.push_back(start);
                current_path_.waypoints.push_back(goal);

                return current_path_;
            }

            std::string AStarPlanner::get_type() const { return "astar_planner"; }

        } // namespace algorithms
    } // namespace planning
} // namespace navcon
