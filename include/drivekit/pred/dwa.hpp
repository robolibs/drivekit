#pragma once

#include "drivekit/controller.hpp"
#include <vector>

namespace drivekit {
    namespace pred {

        /// DWA (Dynamic Window Approach) - Fast reactive local planner.
        /// TODO: Full implementation later.
        class DWAFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            /// DWA-specific configuration.
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

            inline DWAFollower() : DWAFollower(DWAConfig{}) {}

            inline explicit DWAFollower(const DWAConfig &dwa_config) : dwa_config_(dwa_config) {}

            inline VelocityCommand compute_control(const RobotState &, const Goal &, const RobotConstraints &, double,
                                                   const WorldConstraints * = nullptr) override {
                VelocityCommand cmd;
                cmd.valid = false;
                cmd.status_message = "DWA not implemented";
                return cmd;
            }

            inline std::string get_type() const override { return "dwa_follower"; }

            inline void set_dwa_config(const DWAConfig &config) { dwa_config_ = config; }

            inline DWAConfig get_dwa_config() const { return dwa_config_; }

          private:
            DWAConfig dwa_config_;
        };

    } // namespace pred
} // namespace drivekit
