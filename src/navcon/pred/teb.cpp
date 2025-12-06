#include "navcon/pred/teb.hpp"

namespace navcon {
    namespace pred {
        TEBFollower::TEBFollower() : TEBFollower(TEBConfig{}) {}
        TEBFollower::TEBFollower(const TEBConfig &teb_config) : teb_config_(teb_config) {}
        
        VelocityCommand TEBFollower::compute_control(const RobotState &, const Goal &, const RobotConstraints &, double, const WorldConstraints *) {
            VelocityCommand cmd;
            cmd.valid = false;
            cmd.status_message = "TEB not implemented";
            return cmd;
        }
        
        std::string TEBFollower::get_type() const { return "teb_follower"; }
        void TEBFollower::set_teb_config(const TEBConfig &config) { teb_config_ = config; }
        TEBFollower::TEBConfig TEBFollower::get_teb_config() const { return teb_config_; }
    }
}
