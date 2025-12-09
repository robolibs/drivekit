#pragma once

#include "drivekit/controller.hpp"
#include <algorithm>

namespace drivekit {
    namespace point {

        // PID controller for basic point-to-point navigation
        class PIDFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::status_;

            PIDFollower();

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *dynamic_constraints = nullptr) override;

            void reset() override;

            std::string get_type() const override;

          private:
            double linear_integral_ = 0.0;
            double angular_integral_ = 0.0;
            double last_distance_error_ = 0.0;
            double last_heading_error_ = 0.0;
        };

    } // namespace point
} // namespace drivekit
