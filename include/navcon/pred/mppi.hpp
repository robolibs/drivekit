#pragma once
#include "navcon/controller.hpp"
#include <random>
#include <vector>

namespace navcon {
    namespace pred {
        // MPPI (Model Predictive Path Integral) - Sampling-based optimal control
        // TODO: Full implementation later
        class MPPIFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            struct MPPIConfig {
                size_t horizon_steps = 20;
                double dt = 0.1;
                size_t num_samples = 1000;
                double temperature = 1.0;
                double steering_noise = 0.5;
                double acceleration_noise = 0.3;
                double weight_cte = 100.0;
                double weight_epsi = 100.0;
                double weight_vel = 1.0;
                double weight_steering = 10.0;
                double weight_acceleration = 5.0;
                double ref_velocity = 1.0;
                size_t num_threads = 4;
            };

            MPPIFollower();
            explicit MPPIFollower(const MPPIConfig &mppi_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;
            void set_mppi_config(const MPPIConfig &config);
            MPPIConfig get_mppi_config() const;

          private:
            MPPIConfig mppi_config_;
            std::mt19937 rng_;
        };
    } // namespace pred
} // namespace navcon
