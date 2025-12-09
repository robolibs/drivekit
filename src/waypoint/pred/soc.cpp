#include "waypoint/pred/soc.hpp"

namespace waypoint {
    namespace pred {

        SOCFollower::SOCFollower() : Base() {}

        SOCFollower::SOCFollower(const SOCConfig &soc_config) : Base() { set_soc_config(soc_config); }

        std::string SOCFollower::get_type() const { return "soc_follower"; }

        void SOCFollower::set_soc_config(const SOCConfig &config) {
            SOCConfig cfg = config;

            // Map SOCConfig to underlying MPPIConfig.
            MPPIConfig mppi_cfg = get_mppi_config();
            mppi_cfg.horizon_steps = cfg.horizon_steps;
            mppi_cfg.dt = cfg.dt;
            mppi_cfg.num_samples = cfg.num_samples;
            mppi_cfg.temperature = cfg.temperature;
            mppi_cfg.steering_noise = cfg.steering_noise;
            mppi_cfg.acceleration_noise = cfg.acceleration_noise;
            mppi_cfg.weight_cte = cfg.weight_cte;
            mppi_cfg.weight_epsi = cfg.weight_epsi;
            mppi_cfg.weight_vel = cfg.weight_vel;
            mppi_cfg.weight_steering = cfg.weight_steering;
            mppi_cfg.weight_acceleration = cfg.weight_acceleration;
            mppi_cfg.ref_velocity = cfg.ref_velocity;

            set_mppi_config(mppi_cfg);
        }

        SOCFollower::SOCConfig SOCFollower::get_soc_config() const {
            SOCConfig cfg;
            MPPIConfig mppi_cfg = get_mppi_config();

            cfg.horizon_steps = mppi_cfg.horizon_steps;
            cfg.dt = mppi_cfg.dt;
            cfg.num_samples = mppi_cfg.num_samples;
            cfg.temperature = mppi_cfg.temperature;
            cfg.steering_noise = mppi_cfg.steering_noise;
            cfg.acceleration_noise = mppi_cfg.acceleration_noise;
            cfg.weight_cte = mppi_cfg.weight_cte;
            cfg.weight_epsi = mppi_cfg.weight_epsi;
            cfg.weight_vel = mppi_cfg.weight_vel;
            cfg.weight_steering = mppi_cfg.weight_steering;
            cfg.weight_acceleration = mppi_cfg.weight_acceleration;
            cfg.ref_velocity = mppi_cfg.ref_velocity;

            // Leave SOC-specific fields (guide_* and variances) as configured by user.
            return cfg;
        }

    } // namespace pred
} // namespace waypoint
