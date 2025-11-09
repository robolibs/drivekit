#pragma once

#include "navcon/controller.hpp"
#include <algorithm>
#include <iostream>

namespace navcon {
    namespace path {

        // Pure Pursuit controller for smooth path following
        class PurePursuitFollower : public Controller {
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
            std::optional<Point> find_lookahead_point(const Pose &current_pose, double lookahead_distance,
                                                      double progress_distance);

            double compute_progress_distance(double robot_length) const;
        };

    } // namespace path
} // namespace navcon
