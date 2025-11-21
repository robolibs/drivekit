#pragma once

#include "navcon/tracking/controller.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <optional>

namespace navcon {
    namespace tracking {
        namespace path {

            // LQR (Linear Quadratic Regulator) controller for optimal path tracking
            // Based on "Path tracking simulation with LQR steering control" from PythonRobotics
            class LQRFollower : public Controller {
              public:
                using Base = Controller;
                using Base::config_;
                using Base::path_;
                using Base::path_index_;
                using Base::status_;

                VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                                const RobotConstraints &constraints, double dt,
                                                const WorldConstraints *world_constraints = nullptr) override;

                std::string get_type() const override;

              private:
                // Previous errors for derivative calculation
                double previous_lateral_error_ = 0.0;
                double previous_heading_error_ = 0.0;

                // Find nearest point on path and calculate errors
                struct PathError {
                    size_t nearest_index;
                    double lateral_error;  // Cross-track error (m)
                    double heading_error;  // Heading error (rad)
                    double path_curvature; // Curvature at nearest point
                    Point nearest_point;
                    double path_heading;
                };
                PathError calculate_path_error(const RobotState &current_state);

                // Solve Discrete Algebraic Riccati Equation (DARE)
                // Returns optimal LQR gain matrix K
                Eigen::MatrixXd solve_dare(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &Q,
                                           const Eigen::MatrixXd &R);

                // Compute LQR gain for current state
                Eigen::MatrixXd compute_lqr_gain(double velocity, const RobotConstraints &constraints, double dt);
            };

        } // namespace path
    } // namespace tracking
} // namespace navcon
