#pragma once

#include "concord/concord.hpp"
#include "drivekit/controller.hpp"
#include "drivekit/path/lqr.hpp"
#include "drivekit/path/pure_pursuit.hpp"
#include "drivekit/path/stanley.hpp"
#include "drivekit/point/carrot.hpp"
#include "drivekit/point/pid.hpp"
#include "drivekit/pred/mpc.hpp"
#include "drivekit/pred/mppi.hpp"
#include "drivekit/pred/soc.hpp"
#include "drivekit/types.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#ifdef HAS_RERUN
#include <rerun.hpp>
#else
// Forward declarations when rerun is not available
namespace rerun {
    class RecordingStream;
}
#endif

namespace drivekit {

    // Goal types for tracking
    struct NavigationGoal {
        concord::Point target;
        float tolerance = 1.0f; // meters
        float max_speed = 1.0f; // m/s

        NavigationGoal(concord::Point t, float tol = 1.0f, float speed = 1.0f);
    };

    struct PathGoal {
        std::vector<concord::Point> drivekits;
        float tolerance = 1.0f; // meters
        float max_speed = 1.0f; // m/s
        bool loop = false;      // whether to loop back to start

        PathGoal(std::vector<concord::Point> wp, float tol = 1.0f, float speed = 1.0f, bool l = false);
    };

    // Controller types for tracking
    enum class TrackerType { PID, PURE_PURSUIT, STANLEY, CARROT, LQR, MPC, MPC_TRAILER, MPPI, SOC };

    // ============================================================================
    // Tracker - High-level path/point tracking controller
    // ============================================================================
    class Tracker {
      private:
        TrackerType controller_type;
        std::unique_ptr<Controller> controller;

        // Current navigation state
        std::optional<NavigationGoal> current_goal;
        std::optional<PathGoal> current_path;
        size_t current_drivekit_index = 0;
        bool goal_reached = false;
        bool path_completed = false;

        // Robot constraints (set during initialization)
        RobotConstraints constraints_;

#ifdef HAS_RERUN
        // Recording stream for visualization
        std::shared_ptr<rerun::RecordingStream> rec;
#endif

        // Entity prefix for visualization (e.g., "robot_0")
        std::string entity_prefix = "tracking";

        // Current robot state (updated in tick)
        mutable RobotState current_state_;

        // Controller parameters
        struct ControllerParams {
            // PID parameters - increased gains for proper movement
            float linear_kp = 2.0f, linear_ki = 0.0f, linear_kd = 0.1f;
            float angular_kp = 1.5f, angular_ki = 0.0f, angular_kd = 0.1f;

            // Pure pursuit parameters
            float lookahead_distance = 2.5f; // Shorter distance to stay closer to path
            float lookahead_gain = 0.5f;

            // Stanley parameters
            float cross_track_gain = 2.5f; // Increased for better cross-track correction
            float softening_gain = 1.5f;   // Increased for better heading response

            // Carrot parameters
            float carrot_distance = 1.0f;

            // Path controller parameters
            float sharp_turn_threshold = 60.0f;   // degrees
            float u_turn_threshold = 120.0f;      // degrees
            float path_lookahead_distance = 5.0f; // meters
        } params;

      public:
        Tracker(TrackerType type = TrackerType::PID);
        ~Tracker() = default;

        // Initialize with robot constraints and recording stream
        void init(const RobotConstraints &robot_constraints,
                  std::shared_ptr<rerun::RecordingStream> recording_stream = nullptr,
                  const std::string &prefix = "tracking");

        // Goal management
        void set_goal(const NavigationGoal &goal);
        void set_path(const PathGoal &path);
        void clear_goal();
        void clear_path();

        // Navigation control - returns velocity command
        // dynamic_constraints: optional runtime constraints (obstacles, speed limits, etc.)
        VelocityCommand tick(const RobotState &current_state, float dt,
                             const WorldConstraints *dynamic_constraints = nullptr);

        // Status
        bool is_goal_reached() const;
        bool is_path_completed() const;

        // Controller configuration
        void set_controller_type(TrackerType type);
        void set_controller_params(const ControllerParams &new_params);
        const ControllerParams &get_controller_params() const;

        // Status information
        float get_distance_to_goal() const;
        float get_distance_to_current_drivekit() const;
        concord::Point get_current_target() const;

        // Emergency stop
        void emergency_stop();

        // Smoothen path by adding interpolated points between drivekits
        void smoothen(float interval_cm = 100.0f);

        // Direct access to controller
        Controller *get_controller();
        const Controller *get_controller() const;

        // Visualization
        void tock() const;

      private:
        // Internal helper methods
        Goal get_current_tracking_goal() const;
        void update_drivekit_progress(const RobotState &current_state);
        void create_controller();
        void update_goal_status(const RobotState &current_state);
    };

} // namespace drivekit
