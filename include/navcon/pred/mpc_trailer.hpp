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

        // Experimental MPC controller intended for articulated vehicles (tractor + trailer).
        // Currently it reuses the same kinematic bicycle model as MPCFollower but is configured
        // to better support reverse motion by allowing negative velocities.
        class MPCTrailerFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            struct MPCConfig {
                // Prediction horizon
                size_t horizon_steps = 10;
                double dt = 0.1;

                // Cost function weights
                double weight_cte = 100.0;
                double weight_epsi = 100.0;
                double weight_vel = 1.0;

                double weight_steering = 10.0;
                double weight_acceleration = 5.0;
                double weight_steering_rate = 500.0;
                double weight_acceleration_rate = 50.0;

                // Articulation (trailer yaw difference) penalty
                double weight_beta = 100.0;

                // Reference velocity (can be negative for reverse)
                double ref_velocity = 1.0;

                // Simple truck–trailer geometry
                // L0: tractor wheelbase (rear axle to front axle)
                // M0: distance from tractor rear axle to hitch
                // L1: distance along trailer from axle to hitch
                double truck_wheelbase = 1.0;      // L0
                double truck_hitch_offset = 0.0;   // M0
                double trailer_hitch_length = 1.0; // L1

                // Solver settings
                double max_solver_time = 0.5;
                int print_level = 0;
            };

            MPCTrailerFollower();
            explicit MPCTrailerFollower(const MPCConfig &mpc_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;

            // Configuration
            void set_mpc_config(const MPCConfig &config);
            MPCConfig get_mpc_config() const;
            const std::vector<Point> &get_predicted_trajectory() const { return predicted_trajectory_; }

            struct PathError {
                size_t nearest_index;
                double cte;
                double epsi;
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

            std::vector<double> previous_steering_;
            std::vector<double> previous_acceleration_;
            std::vector<Point> predicted_trajectory_;

            PathError calculate_path_error(const RobotState &current_state);
            ReferenceTrajectory calculate_reference_trajectory(const PathError &error, const RobotState &current_state,
                                                               const RobotConstraints &constraints);

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

        class TrailerFGEval {
          public:
            using ADvector = CPPAD_TESTVECTOR(CppAD::AD<double>);

            TrailerFGEval(const MPCTrailerFollower::ReferenceTrajectory &ref,
                          const MPCTrailerFollower::MPCConfig &config, const RobotConstraints &constraints);

            void operator()(ADvector &fg, const ADvector &vars);

          private:
            MPCTrailerFollower::ReferenceTrajectory ref_trajectory_;
            MPCTrailerFollower::MPCConfig mpc_config_;
            RobotConstraints robot_constraints_;

            size_t x_trailer_start_;
            size_t y_trailer_start_;
            size_t yaw_trailer_start_;
            size_t yaw_tractor_start_;
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
