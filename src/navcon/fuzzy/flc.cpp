#include "navcon/fuzzy/flc.hpp"

namespace navcon {
    namespace fuzzy {

        FuzzyFollower::FuzzyFollower() : FuzzyFollower(FLCConfig{}) {}

        FuzzyFollower::FuzzyFollower(const FLCConfig &flc_config) : flc_config_(flc_config) {}

        VelocityCommand FuzzyFollower::compute_control(const RobotState &, const Goal &, const RobotConstraints &,
                                                       double, const WorldConstraints *) {
            VelocityCommand cmd;
            cmd.valid = false;
            cmd.status_message = "FLC not implemented";

            // TODO: Implement fuzzy logic control
            // Example rule: IF cte is LARGE and heading_error is POSITIVE THEN steering is NEGATIVE_LARGE

            return cmd;
        }

        std::string FuzzyFollower::get_type() const { return "fuzzy_follower"; }

        void FuzzyFollower::set_flc_config(const FLCConfig &config) { flc_config_ = config; }

        FuzzyFollower::FLCConfig FuzzyFollower::get_flc_config() const { return flc_config_; }

    } // namespace fuzzy
} // namespace navcon
