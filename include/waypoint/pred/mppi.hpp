#pragma once

#include "waypoint/controller.hpp"
#include <random>
#include <vector>

namespace waypoint {
    namespace pred {

        // MPPI (Model Predictive Path Integral) - Sampling-based optimal control
        // Sampling-based analogue of MPCFollower, using a kinematic bicycle model
        // and path-tracking cost similar to the IPOPT-based MPC implementation.
        class MPPIFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            struct MPPIConfig {
                // Prediction horizon
                size_t horizon_steps = 20;
                double dt = 0.1;

                // Sampling settings
                size_t num_samples = 1000;
                double temperature = 1.0;
                double steering_noise = 0.5;
                double acceleration_noise = 0.3;

                // Cost weights (mirroring MPC as much as possible)
                double weight_cte = 100.0;        // Cross-track error
                double weight_epsi = 100.0;       // Heading error
                double weight_vel = 1.0;          // Velocity tracking
                double weight_steering = 10.0;    // Steering effort
                double weight_acceleration = 5.0; // Acceleration effort

                // Reference velocity along the path
                double ref_velocity = 1.0; // m/s

                // Reserved for future parallelization
                size_t num_threads = 4;
            };

            // Simple path error description (mirrors MPCFollower::PathError)
            struct PathError {
                size_t nearest_index = 0;
                double cte = 0.0;          // Cross-track error (m)
                double epsi = 0.0;         // Heading error (rad)
                Point nearest_point{};     // Nearest point on path
                double path_heading = 0.0; // Heading of path at nearest point
            };

            // Reference trajectory along the path for the horizon
            struct ReferenceTrajectory {
                std::vector<double> x;
                std::vector<double> y;
                std::vector<double> yaw;
                std::vector<double> velocity;
            };

            MPPIFollower();
            explicit MPPIFollower(const MPPIConfig &mppi_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;

            // Configuration
            void set_mppi_config(const MPPIConfig &config);
            MPPIConfig get_mppi_config() const;

            // Predicted trajectory from the best sample (for visualization/debugging)
            const std::vector<Point> &get_predicted_trajectory() const { return predicted_trajectory_; }

          private:
            MPPIConfig mppi_config_;

            // Random number generator for sampling
            std::mt19937 rng_;

            // Mean control sequence used as sampling center (receding horizon)
            std::vector<double> mean_steering_;
            std::vector<double> mean_acceleration_;

            // Predicted trajectory from the best-cost rollout
            std::vector<Point> predicted_trajectory_;

            // Helper methods
            PathError calculate_path_error(const RobotState &current_state);
            ReferenceTrajectory calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
                                                               const RobotConstraints &constraints);
        };

    } // namespace pred
} // namespace waypoint
