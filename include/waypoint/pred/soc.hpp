#pragma once

#include "waypoint/pred/mppi.hpp"

namespace waypoint {
    namespace pred {

        // SOC (Stochastic Optimal Control) controller.
        // For now this is a thin wrapper around MPPIFollower with its
        // own configuration struct and type name, so behavior matches
        // the existing MPPI implementation while exposing a separate
        // controller type for experimentation.
        class SOCFollower : public MPPIFollower {
          public:
            using Base = MPPIFollower;
            using MPPIConfig = MPPIFollower::MPPIConfig;

            struct SOCConfig {
                // Prediction horizon
                size_t horizon_steps = 20;
                double dt = 0.1;

                // Sampling settings
                size_t guide_samples = 64; // guide particle count
                size_t num_samples = 512;  // final MPPI sample count

                // Temperatures
                double temperature = 1.0;       // for MPPI weighting
                double guide_temperature = 1.0; // for guide weighting

                // Noise / covariance
                double steering_noise = 0.5;
                double acceleration_noise = 0.3;
                double initial_steer_variance = 0.25;
                double min_steer_variance = 1e-4;
                double max_steer_variance = 1.0;

                // Cost weights (mirrors MPPI/MPC as much as possible)
                double weight_cte = 100.0;        // Cross-track error
                double weight_epsi = 100.0;       // Heading error
                double weight_vel = 1.0;          // Velocity tracking
                double weight_steering = 10.0;    // Steering effort
                double weight_acceleration = 5.0; // Acceleration effort

                // Reference velocity along the path
                double ref_velocity = 1.0; // m/s

                // Guide update
                size_t guide_iterations = 2;
                double guide_step_size = 0.2; // simple gradient-descent step

                // Reserved for future parallelization
                size_t num_threads = 4;
            };

            SOCFollower();
            explicit SOCFollower(const SOCConfig &soc_config);

            std::string get_type() const override;

            // Configuration
            void set_soc_config(const SOCConfig &config);
            SOCConfig get_soc_config() const;
        };

    } // namespace pred
} // namespace waypoint
