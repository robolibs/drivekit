#include "navcon/pred/mppi.hpp"

namespace navcon {
    namespace pred {
        MPPIFollower::MPPIFollower() : MPPIFollower(MPPIConfig{}) {}
        MPPIFollower::MPPIFollower(const MPPIConfig &mppi_config) : mppi_config_(mppi_config), rng_(std::random_device{}()) {}
        
        VelocityCommand MPPIFollower::compute_control(const RobotState &, const Goal &, const RobotConstraints &, double, const WorldConstraints *) {
            VelocityCommand cmd;
            cmd.valid = false;
            cmd.status_message = "MPPI not implemented";
            return cmd;
        }
        
        std::string MPPIFollower::get_type() const { return "mppi_follower"; }
        void MPPIFollower::set_mppi_config(const MPPIConfig &config) { mppi_config_ = config; }
        MPPIFollower::MPPIConfig MPPIFollower::get_mppi_config() const { return mppi_config_; }
    }
}
