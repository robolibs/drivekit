#pragma once

#include "navcon/controller.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <optional>
#include <vector>

namespace navcon {
    namespace pred {

        // MPC (Model Predictive Control) controller for optimal path tracking
        // Uses IPOPT solver with CppAD for automatic differentiation
        // Based on kinematic bicycle model
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
                double weight_cte = 2000.0;             // Cross-track error weight
                double weight_epsi = 2000.0;            // Heading error weight
                double weight_vel = 1.0;                // Velocity tracking weight
                double weight_steering = 5.0;           // Steering effort weight
                double weight_acceleration = 5.0;       // Acceleration effort weight
                double weight_steering_rate = 200.0;    // Steering rate (smoothness)
                double weight_acceleration_rate = 10.0; // Acceleration rate (smoothness)

                // Reference velocity
                double ref_velocity = 1.0; // m/s

                // Solver settings
                double max_solver_time = 0.5; // seconds
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
            };
            MPCSolution solve_mpc(const RobotState &current_state, const ReferenceTrajectory &ref_trajectory,
                                  const RobotConstraints &constraints, double cte, double epsi);
        };

        // FG_eval class for IPOPT cost function and constraints evaluation
        class FG_eval {
          public:
            using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);

            // Constructor
            FG_eval(const MPCFollower::ReferenceTrajectory &ref, const MPCFollower::MPCConfig &config,
                    const RobotConstraints &constraints);

            // Operator for IPOPT
            void operator()(ADvector &fg, const ADvector &vars);

          private:
            MPCFollower::ReferenceTrajectory ref_trajectory_;
            MPCFollower::MPCConfig mpc_config_;
            RobotConstraints robot_constraints_;

            // State and control indices in the vars vector
            size_t x_start_;
            size_t y_start_;
            size_t yaw_start_;
            size_t v_start_;
            size_t cte_start_;
            size_t epsi_start_;
            size_t steering_start_;
            size_t acceleration_start_;
            size_t steering_rate_start_;
            double steering_rate_limit_;
        };

    } // namespace pred
} // namespace navcon
