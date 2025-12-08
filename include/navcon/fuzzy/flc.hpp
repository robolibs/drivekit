#pragma once
#include "navcon/controller.hpp"
#include <map>
#include <string>
#include <vector>

namespace navcon {
    namespace fuzzy {

        // FLC (Fuzzy Logic Controller) for path following
        // Uses linguistic variables and fuzzy rules for human-like control
        class FuzzyFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            // Linguistic terms
            enum class Term { NL, NM, NS, ZE, PS, PM, PL }; // Negative/Zero/Positive Large/Medium/Small

            // FLC configuration
            struct FLCConfig {
                // Input ranges
                double cte_range = 2.0;      // Cross-track error range (m)
                double heading_range = 1.57; // Heading error range (rad)

                // Output parameters
                double max_steering = 0.7;  // Max steering output (normalized)
                double base_velocity = 0.6; // Base velocity when on track
                double min_velocity = 0.2;  // Minimum velocity

                // Fuzzy system tuning
                double rule_weight = 1.0;        // Global rule weight
                bool use_cte_derivative = false; // Use rate of CTE change
            };

            FuzzyFollower();
            explicit FuzzyFollower(const FLCConfig &flc_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;
            void set_flc_config(const FLCConfig &config);
            FLCConfig get_flc_config() const;

          private:
            FLCConfig flc_config_;

            // TODO: Implement fuzzy logic methods
            // - Fuzzification (crisp → fuzzy)
            // - Rule evaluation (IF-THEN)
            // - Defuzzification (fuzzy → crisp)
        };

    } // namespace fuzzy
} // namespace navcon
