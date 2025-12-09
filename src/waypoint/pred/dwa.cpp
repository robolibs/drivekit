#include "waypoint/pred/dwa.hpp"

namespace waypoint {
    namespace pred {
        DWAFollower::DWAFollower() : DWAFollower(DWAConfig{}) {}
        DWAFollower::DWAFollower(const DWAConfig &dwa_config) : dwa_config_(dwa_config) {}

        VelocityCommand DWAFollower::compute_control(const RobotState &, const Goal &, const RobotConstraints &, double,
                                                     const WorldConstraints *) {
            VelocityCommand cmd;
            cmd.valid = false;
            cmd.status_message = "DWA not implemented";
            return cmd;
        }

        std::string DWAFollower::get_type() const { return "dwa_follower"; }
        void DWAFollower::set_dwa_config(const DWAConfig &config) { dwa_config_ = config; }
        DWAFollower::DWAConfig DWAFollower::get_dwa_config() const { return dwa_config_; }
    } // namespace pred
} // namespace waypoint
