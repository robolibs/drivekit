#pragma once

#include "waypoint/controller.hpp"
#include <algorithm>

namespace waypoint {
    namespace point {

        // Simple carrot chasing controller for basic navigation
        class CarrotFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::status_;

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *dynamic_constraints = nullptr) override;

            std::string get_type() const override;
        };

    } // namespace point
} // namespace waypoint
