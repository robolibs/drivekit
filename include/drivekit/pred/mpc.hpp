#pragma once

#include "drivekit/controller.hpp"
#include <algorithm>
#include <optional>
#include <vector>

namespace drivekit {
    namespace pred {

        // MPC (Model Predictive Control) controller for optimal path tracking.
        // Original implementation used IPOPT + CppAD; this version uses a custom
        // lightweight optimizer (MPCOptimizer) implemented in the .cpp file so
        // that no nonlinear solver dependencies are required.
        class MPCFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            // MPC-specific configuration
            struct MPCConfig {
                // Prediction horizon
                size_t horizon_steps = 10; // Number of steps to predict
                double dt = 0.1;           // Time step (seconds)

                // Cost function weights
                // Balanced for smooth tracking without oscillation
                // Key ratio: steering_rate/cte should be ~3-5 for stability
                double weight_cte = 100.0;  // Cross-track error weight
                double weight_epsi = 100.0; // Heading error weight
                double weight_vel = 1.0;    // Velocity tracking weight
                // Control effort weights provide damping
                double weight_steering = 10.0;    // Steering effort weight
                double weight_acceleration = 5.0; // Acceleration effort weight
                // High smoothness weights prevent rapid control changes (key for stability)
                double weight_steering_rate = 500.0;    // Steering rate (ratio = 5.0)
                double weight_acceleration_rate = 50.0; // Acceleration rate

                // Reference velocity
                double ref_velocity = 1.0; // m/s

                // Solver settings (kept for future advanced optimizers; unused in
                // the current lightweight implementation)
                double max_solver_time = 0.5; // seconds (hint for future solvers)
                int print_level = 0;          // 0 = silent, 5 = verbose
            };

            MPCFollower();
            explicit MPCFollower(const MPCConfig &mpc_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;

            // Configuration
            void set_mpc_config(const MPCConfig &config);
            MPCConfig get_mpc_config() const;
            const std::vector<Point> &get_predicted_trajectory() const { return predicted_trajectory_; }

            // Make these public so FG_eval can access them
            struct PathError {
                size_t nearest_index;
                double cte;  // Cross-track error (m)
                double epsi; // Heading error (rad)
                Point nearest_point;
                double path_heading;
            };

            struct ReferenceTrajectory {
                std::vector<double> x;
                std::vector<double> y;
                std::vector<double> yaw;
                std::vector<double> velocity;
            };

          private:
            MPCConfig mpc_config_;

            // Previous control inputs for warm starting
            std::vector<double> previous_steering_;
            std::vector<double> previous_acceleration_;

            // Predicted trajectory (for visualization/debugging)
            std::vector<Point> predicted_trajectory_;

            // Helper methods
            PathError calculate_path_error(const RobotState &current_state);
            ReferenceTrajectory calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
                                                               const RobotConstraints &constraints);

            // MPC solver
            struct MPCSolution {
                bool success;
                double steering;
                double acceleration;
                std::vector<double> predicted_x;
                std::vector<double> predicted_y;
                // Full optimized control sequence (for warm starting)
                std::vector<double> steering_sequence;
                std::vector<double> acceleration_sequence;
            };
            MPCSolution solve_mpc(const RobotState &current_state, const ReferenceTrajectory &ref_trajectory,
                                  const RobotConstraints &constraints);
        };

    } // namespace pred
} // namespace drivekit
