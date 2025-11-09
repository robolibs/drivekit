#pragma once

#include "navcon/controller.hpp"
#include <algorithm>

namespace navcon {
    namespace point {

        // Simple carrot chasing controller for basic navigation
        class CarrotFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::status_;

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt) override;

            std::string get_type() const override;
        };

    } // namespace point
} // namespace navcon