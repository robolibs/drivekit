#pragma once

#include "navcon/controller.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace navcon {
    namespace path {

        // Stanley controller for path tracking (Stanford DARPA Grand Challenge)
        // Combines heading error and cross-track error correction
        class StanleyFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt) override;

            std::string get_type() const override;

          private:
            VelocityCommand compute_goal_control(const RobotState &current_state, const Goal &goal,
                                                 const RobotConstraints &constraints);

            void update_path_index(const Pose &current_pose);

            std::tuple<Point, double, double> find_closest_path_point(const Pose &current_pose);
        };

    } // namespace path
} // namespace navcon
