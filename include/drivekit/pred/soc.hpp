#pragma once

#include "drivekit/pred/mppi.hpp"

namespace drivekit {
    namespace pred {

        // SOC (Stochastic Optimal Control) controller - SVG-MPPI
        // Stein Variational Guided Model Predictive Path Integral Control
        // Uses SVGD to guide sampling distribution for better exploration
        class SOCFollower : public MPPIFollower {
          public:
            using Base = MPPIFollower;
            using MPPIConfig = MPPIFollower::MPPIConfig;

            struct SOCConfig {
                // Prediction horizon
                size_t horizon_steps = 20;
                double dt = 0.1;

                // Sampling settings
                size_t guide_samples = 64; // guide particle count for SVGD
                size_t num_samples = 512;  // final MPPI sample count
                size_t grad_samples = 32;  // samples for gradient estimation

                // Temperatures
                double temperature = 1.0;       // for MPPI weighting
                double guide_temperature = 1.0; // for guide weighting

                // Noise / covariance
                double steering_noise = 0.5;
                double acceleration_noise = 0.3;
                double initial_steer_variance = 0.25;
                double min_steer_variance = 1e-4;
                double max_steer_variance = 1.0;

                // Cost weights
                double weight_cte = 100.0;
                double weight_epsi = 100.0;
                double weight_vel = 1.0;
                double weight_steering = 10.0;
                double weight_acceleration = 5.0;

                // Reference velocity
                double ref_velocity = 1.0;

                // SVGD parameters
                size_t svgd_iterations = 3;    // number of SVGD update steps
                double svgd_step_size = 0.1;   // SVGD gradient descent step
                double kernel_bandwidth = 1.0; // RBF kernel bandwidth

                // Covariance adaptation
                bool use_covariance_adaptation = true;
                double gaussian_fitting_lambda = 0.1;

                size_t num_threads = 4;
            };

            SOCFollower();
            explicit SOCFollower(const SOCConfig &soc_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;

            void set_soc_config(const SOCConfig &config);
            SOCConfig get_soc_config() const;

          private:
            SOCConfig soc_config_;

            // Guide particles for SVGD
            std::vector<std::vector<double>> guide_steering_;
            std::vector<std::vector<double>> guide_acceleration_;

            // Helper methods for SVG-MPPI
            double rbf_kernel(const std::vector<double> &x1, const std::vector<double> &x2, double bandwidth) const;
            std::vector<double> compute_svgd_gradient(size_t particle_idx,
                                                      const std::vector<std::vector<double>> &particles,
                                                      const std::vector<double> &costs, double bandwidth) const;
            double estimate_gradient_log_likelihood(size_t particle_idx, size_t time_idx, size_t control_idx,
                                                    const RobotState &current_state, const Goal &goal,
                                                    const RobotConstraints &constraints) const;
        };

    } // namespace pred
} // namespace drivekit
