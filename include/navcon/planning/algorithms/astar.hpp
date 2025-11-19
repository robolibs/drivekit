#pragma once

#include "navcon/planning/controller.hpp"

namespace navcon {
    namespace planning {
        namespace algorithms {

            // A* path planner
            class AStarPlanner : public Controller {
              public:
                Path compute_path(const Pose &start, const Pose &goal, const RobotConstraints &constraints) override;

                std::string get_type() const override;
            };

        } // namespace algorithms
    } // namespace planning
} // namespace navcon
