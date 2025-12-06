#pragma once
#include "navcon/controller.hpp"
#include <vector>

namespace navcon {
    namespace pred {
        // TEB (Timed Elastic Band) - Trajectory optimization with time information
        // TODO: Full implementation later
        class TEBFollower : public Controller {
          public:
            using Base = Controller;
            using Base::config_;
            using Base::path_;
            using Base::path_index_;
            using Base::status_;

            struct TEBConfig {
                size_t num_poses = 15;
                double dt_ref = 0.3;
                double dt_hysteresis = 0.1;
                size_t max_iterations = 10;
                double optimization_tolerance = 0.01;
                double weight_kinematics = 1.0;
                double weight_time_optimal = 1.0;
                double weight_obstacle = 50.0;
                double weight_viapoint = 1.0;
                double weight_velocity = 1.0;
                double weight_acceleration = 1.0;
                double weight_jerk = 0.5;
                double max_vel_x = 1.0;
                double max_vel_theta = 1.0;
                double max_acc_x = 0.5;
                double max_acc_theta = 0.5;
                double min_obstacle_dist = 0.5;
                double inflation_dist = 0.6;
                bool enable_homotopy_class_planning = false;
                size_t max_number_classes = 3;
            };

            TEBFollower();
            explicit TEBFollower(const TEBConfig &teb_config);

            VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                            const RobotConstraints &constraints, double dt,
                                            const WorldConstraints *world_constraints = nullptr) override;

            std::string get_type() const override;
            void set_teb_config(const TEBConfig &config);
            TEBConfig get_teb_config() const;

          private:
            TEBConfig teb_config_;
        };
    } // namespace pred
} // namespace navcon
