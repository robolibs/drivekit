#pragma once

#include "drivekit/pred/mppi.hpp"
#include <vector>

namespace drivekit {
    namespace pred {

        // MCA (Monte Carlo Approximation) controller - DRA-MPPI
        // Dynamic Risk-Aware Model Predictive Path Integral control
        // Incorporates uncertain future motions of dynamic obstacles via Monte Carlo sampling
        class MCAFollower : public MPPIFollower {
          public:
            using Base = MPPIFollower;
            using MPPIConfig = MPPIFollower::MPPIConfig;

            // Dynamic obstacle representation
            struct DynamicObstacle {
                size_t id = 0;
                double radius = 0.3; // Obstacle radius (m)

                // Predicted trajectory as Gaussian distributions per timestep
                // For multi-modal predictions, use mixture of Gaussians
                struct GaussianMode {
                    double weight = 1.0;        // Mode weight (sum to 1.0 across modes)
                    std::vector<double> mean_x; // Mean x position per timestep
                    std::vector<double> mean_y; // Mean y position per timestep
                    std::vector<double> std_x;  // Standard deviation x per timestep
                    std::vector<double> std_y;  // Standard deviation y per timestep
                };

                std::vector<GaussianMode> modes; // Multiple modes for multi-modal predictions

                // Simple constructor for single-mode (unimodal) prediction
                DynamicObstacle() = default;

                // Helper to add a unimodal prediction
                void set_unimodal_prediction(const std::vector<double> &mean_x, const std::vector<double> &mean_y,
                                             const std::vector<double> &std_x, const std::vector<double> &std_y) {
                    modes.clear();
                    GaussianMode mode;
                    mode.weight = 1.0;
                    mode.mean_x = mean_x;
                    mode.mean_y = mean_y;
                    mode.std_x = std_x;
                    mode.std_y = std_y;
                    modes.push_back(mode);
                }

                // Helper to add a mode to multi-modal prediction
                void add_mode(double weight, const std::vector<double> &mean_x, const std::vector<double> &mean_y,
                              const std::vector<double> &std_x, const std::vector<double> &std_y) {
                    GaussianMode mode;
                    mode.weight = weight;
                    mode.mean_x = mean_x;
                    mode.mean_y = mean_y;
                    mode.std_x = std_x;
                    mode.std_y = std_y;
                    modes.push_back(mode);
                }
            };

            struct MCAConfig {
                // Prediction horizon
                size_t horizon_steps = 20;
                double dt = 0.1;

                // Sampling settings
                size_t num_samples = 400;      // K - MPPI trajectory samples
                size_t num_mc_samples = 20000; // Nmc - Monte Carlo samples for collision probability
                double temperature = 1.0;
                double steering_noise = 0.5;
                double acceleration_noise = 0.3;

                // Cost weights (mirrors MPPI/MPC)
                double weight_cte = 100.0;        // Cross-track error
                double weight_epsi = 100.0;       // Heading error
                double weight_vel = 1.0;          // Velocity tracking
                double weight_steering = 10.0;    // Steering effort
                double weight_acceleration = 5.0; // Acceleration effort

                // Risk-aware cost weights
                double weight_soft_risk = 50.0; // ωsoft - Linear penalty on collision probability
                double weight_hard_risk = 1e6;  // ωhard - Hard constraint violation penalty

                // Risk threshold
                double risk_threshold = 0.05; // σ - Maximum allowable collision probability

                // Velocity scaling based on risk
                double min_velocity_scale = 0.1; // Minimum velocity scale when max risk
                double risk_slowdown_gain = 5.0; // How aggressively to slow down with risk

                // Safety margin added to robot radius for collision checking
                double robot_radius_margin = 0.5; // Extra margin in meters

                // Reference velocity along the path
                double ref_velocity = 1.0; // m/s

                // Turn-first behavior parameters
                double turn_first_activation_deg = 60.0;
                double turn_first_release_deg = 15.0;

                // Reserved for future parallelization
                size_t num_threads = 4;
            };

            MCAFollower();
            explicit MCAFollower(const MCAConfig &mca_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;

            // Configuration
            void set_mca_config(const MCAConfig &config);
            MCAConfig get_mca_config() const;

            // Set dynamic obstacles for risk-aware planning
            void set_dynamic_obstacles(const std::vector<DynamicObstacle> &obstacles);
            const std::vector<DynamicObstacle> &get_dynamic_obstacles() const { return dynamic_obstacles_; }

            // Get collision probabilities for the last computed trajectory samples
            const std::vector<std::vector<double>> &get_sample_collision_probs() const {
                return sample_collision_probs_;
            }

          protected:
            MCAConfig mca_config_;
            std::vector<DynamicObstacle> dynamic_obstacles_;

            // Collision probabilities per sample per timestep [K][T]
            std::vector<std::vector<double>> sample_collision_probs_;

            // Random number generator for Monte Carlo sampling
            std::mt19937 mca_rng_;

            // Monte Carlo approximation of joint collision probability
            // Returns collision probability for each sample at given timestep
            std::vector<double> compute_collision_probabilities(const std::vector<std::vector<Point>> &trajectories,
                                                                size_t timestep, double robot_radius);

            // Evaluate probability density at a point for a given obstacle and timestep
            double evaluate_obstacle_pdf(const DynamicObstacle &obstacle, size_t timestep, double x, double y) const;

            // Check if point is in collision region
            inline bool is_in_collision_region(double robot_x, double robot_y, double point_x, double point_y,
                                               double radius) const {
                double dx = robot_x - point_x;
                double dy = robot_y - point_y;
                return (dx * dx + dy * dy) <= (radius * radius);
            }
        };

    } // namespace pred
} // namespace drivekit
