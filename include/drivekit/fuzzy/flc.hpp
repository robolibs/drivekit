#pragma once

#include "drivekit/controller.hpp"
#include <map>
#include <string>
#include <vector>

namespace drivekit {
    namespace fuzzy {

        /// FLC (Fuzzy Logic Controller) for path following.
        /// Uses linguistic variables and fuzzy rules for human-like control.
        /// TODO: Full implementation later.
        class FuzzyFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            /// Linguistic terms.
            enum class Term { NL, NM, NS, ZE, PS, PM, PL };

            /// FLC-specific configuration.
            struct FLCConfig {
                double cte_range = 2.0;
                double heading_range = 1.57;

                double max_steering = 0.7;
                double base_velocity = 0.6;
                double min_velocity = 0.2;

                double rule_weight = 1.0;
                bool use_cte_derivative = false;
            };

            inline FuzzyFollower() : FuzzyFollower(FLCConfig{}) {}

            inline explicit FuzzyFollower(const FLCConfig &flc_config) : flc_config_(flc_config) {}

            inline VelocityCommand compute_control(const RobotState &, const Goal &, const RobotConstraints &, double,
                                                   const WorldConstraints * = nullptr) override {
                VelocityCommand cmd;
                cmd.valid = false;
                cmd.status_message = "FLC not implemented";
                return cmd;
            }

            inline std::string get_type() const override { return "fuzzy_follower"; }

            inline void set_flc_config(const FLCConfig &config) { flc_config_ = config; }

            inline FLCConfig get_flc_config() const { return flc_config_; }

          private:
            FLCConfig flc_config_;
        };

    } // namespace fuzzy
} // namespace drivekit
