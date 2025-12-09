#pragma once

#ifdef HAS_RERUN

#include "../path/pure_pursuit.hpp"
#include "../point/carrot.hpp"
#include "../point/pid.hpp"

#include "drivekit/types.hpp"
#include <array>
#include <memory>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>
#include <rerun/result.hpp>
#include <string>
#include <vector>

namespace drivekit {
    namespace visualize {

        // Color palette for visualization
        extern const std::vector<rerun::Color> palette;

        // Show a single path
        void show_path(std::shared_ptr<rerun::RecordingStream> rec, const Path &path,
                       const std::string &entity_path = "path", const rerun::Color &color = {0, 255, 0});

        // Show multiple paths with different colors
        void show_paths(std::shared_ptr<rerun::RecordingStream> rec, const std::vector<Path> &paths,
                        const std::string &base_entity_path = "paths");

        // Show robot pose as a rectangle
        void show_robot_pose(std::shared_ptr<rerun::RecordingStream> rec, const Pose &pose,
                             const std::string &entity_path = "robot", const rerun::Color &color = {255, 255, 0},
                             float scale = 1.0f);

        // Show robot state
        void show_robot_state(std::shared_ptr<rerun::RecordingStream> rec, const RobotState &state,
                              const std::string &entity_path = "robot_state", const rerun::Color &color = {255, 255, 0},
                              float scale = 1.0f);

        // Show controller status
        void show_controller_status(std::shared_ptr<rerun::RecordingStream> rec, const ControllerStatus &status,
                                    const std::string &entity_path = "controller_status");

        // Show goal as a simple point
        void show_goal(std::shared_ptr<rerun::RecordingStream> rec, const Goal &goal,
                       const std::string &entity_path = "goal", const rerun::Color &color = {0, 255, 0});

    } // namespace visualize
} // namespace drivekit

#endif // HAS_RERUN
