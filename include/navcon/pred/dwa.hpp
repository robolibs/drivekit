#pragma once
#include "navcon/controller.hpp"
#include <vector>

namespace navcon {
    namespace pred {
        // DWA (Dynamic Window Approach) - Fast reactive local planner
        // TODO: Full implementation later
        class DWAFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            struct DWAConfig {
                double predict_time = 1.0;
                double dt = 0.1;
                size_t v_samples = 10;
                size_t w_samples = 20;
                double max_accel = 2.0;
                double max_angular_accel = 3.0;
                double weight_heading = 1.0;
                double weight_distance = 2.0;
                double weight_velocity = 0.5;
                double weight_clearance = 1.0;
                double target_velocity = 1.0;
            };

            DWAFollower();
            explicit DWAFollower(const DWAConfig &dwa_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;
            void set_dwa_config(const DWAConfig &config);
            DWAConfig get_dwa_config() const;

          private:
            DWAConfig dwa_config_;
        };
    } // namespace pred
} // namespace navcon
