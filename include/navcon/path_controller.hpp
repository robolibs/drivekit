#pragma once

#include "navcon/controller.hpp"
#include "navcon/controllers/pure_pursuit.hpp"
#include <cmath>
#include <memory>

namespace navcon {

    // Main controller that switches between followers and turners based on path analysis
    class PathController : public Controller {
      public:
        enum class ControllerType { FOLLOWER };

        enum class FollowerType { PURE_PURSUIT };

        inline PathController(FollowerType follower_type = FollowerType::PURE_PURSUIT, double min_turning_radius = 2.0)
            : min_turning_radius_(min_turning_radius) {
            // Create the follower
            switch (follower_type) {
            case FollowerType::PURE_PURSUIT:
                follower_ = std::make_unique<controllers::PurePursuitFollower>();
                break;
            }

            // Start with follower
            active_type_ = ControllerType::FOLLOWER;
            active_controller_ = follower_.get();
        }

        inline std::string get_type() const override { return "path_controller"; }

        inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal, double dt) {
            // Use stored constraints
            return compute_control(current_state, goal, constraints_, dt);
        }

        inline VelocityCommand compute_control(const RobotState &current_state, const Goal &goal,
                                               const RobotConstraints &constraints, double dt) override {

            // For now, just use the follower
            // TODO: Implement logic to switch to Reeds-Shepp turner for sharp turns
            return follower_->compute_control(current_state, goal, constraints, dt);
        }

        inline void set_path(const Path &path) override {
            Controller::set_path(path);
            // Set path on follower
            follower_->set_path(path);

            // Start with follower
            active_type_ = ControllerType::FOLLOWER;
            active_controller_ = follower_.get();
        }

        inline void reset() override {
            Controller::reset();
            follower_->reset();

            active_type_ = ControllerType::FOLLOWER;
            active_controller_ = follower_.get();
        }

        inline void set_config(const ControllerConfig &config) override {
            Controller::set_config(config);
            follower_->set_config(config);
        }

        // Configuration
        inline void set_sharp_turn_threshold(double angle_degrees) {
            sharp_turn_threshold_ = angle_degrees * M_PI / 180.0;
        }

        inline void set_u_turn_threshold(double angle_degrees) { u_turn_threshold_ = angle_degrees * M_PI / 180.0; }

        inline void set_lookahead_distance(double distance) { lookahead_distance_ = distance; }

        // Status
        inline ControllerType get_active_type() const { return active_type_; }
        inline std::string get_active_controller_name() const {
            switch (active_type_) {
            case ControllerType::FOLLOWER:
                return "follower";
            }
            return "unknown";
        }

        // Get references to sub-controllers for advanced configuration
        inline Controller *get_follower() const { return follower_.get(); }

        // Set robot constraints
        inline void set_constraints(const RobotConstraints &constraints) { constraints_ = constraints; }

      private:
        std::unique_ptr<Controller> follower_;

        ControllerType active_type_;
        Controller *active_controller_;

        // Configuration parameters
        double sharp_turn_threshold_ = 60.0 * M_PI / 180.0; // 60 degrees
        double u_turn_threshold_ = 120.0 * M_PI / 180.0;    // 120 degrees
        double lookahead_distance_ = 5.0;                   // meters
        double min_turning_radius_;

        // Stored constraints for compute_control overload
        RobotConstraints constraints_;
    };

} // namespace navcon