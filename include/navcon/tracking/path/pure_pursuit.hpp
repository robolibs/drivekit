#pragma once

#include "navcon/tracking/controller.hpp"
#include <algorithm>
#include <iostream>

namespace navcon {
    namespace tracking {
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
                                                const RobotConstraints &constraints, double dt,
                                                const WorldConstraints *dynamic_constraints = nullptr) override;

                std::string get_type() const override;

              private:
                // Find lookahead point from rear axle position
                std::optional<Point> find_lookahead_point(const Point &rear_axle, double current_yaw,
                                                          double lookahead_distance, double progress_distance);

                // Helper: find intersection between lookahead circle and path segment
                std::optional<Point> find_circle_segment_intersection(const Point &circle_center, double radius,
                                                                      const Point &segment_start,
                                                                      const Point &segment_end);
            };

        } // namespace path
    } // namespace tracking
} // namespace navcon
