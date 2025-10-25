#pragma once

#ifdef HAS_RERUN

#include "../followers/carrot.hpp"
#include "../followers/pid.hpp"
#include "../followers/pure_pursuit.hpp"

#include "../types.hpp"
#include <array>
#include <memory>
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>
#include <rerun/result.hpp>
#include <string>
#include <vector>

namespace navcon {
    namespace visualize {

        // Color palette for visualization
        static const std::vector<rerun::Color> palette = {
            {255, 0, 0},     // Red
            {0, 255, 0},     // Green
            {0, 0, 255},     // Blue
            {255, 255, 0},   // Yellow
            {255, 0, 255},   // Magenta
            {0, 255, 255},   // Cyan
            {255, 165, 0},   // Orange
            {128, 0, 128},   // Purple
            {255, 192, 203}, // Pink
            {165, 42, 42}    // Brown
        };

        // Show a single path
        inline void show_path(std::shared_ptr<rerun::RecordingStream> rec, const Path &path,
                              const std::string &entity_path = "path", const rerun::Color &color = {0, 255, 0}) {
            if (path.waypoints.empty()) return;

            std::vector<rerun::Position3D> positions;
            for (const auto &waypoint : path.waypoints) {
                positions.emplace_back(waypoint.point.x, waypoint.point.y, 0.0f);
            }

            if (!positions.empty()) {
                rec->log_static(entity_path,
                                rerun::LineStrips3D(rerun::components::LineStrip3D(positions)).with_colors(color));
            }
        }

        // Show multiple paths with different colors
        inline void show_paths(std::shared_ptr<rerun::RecordingStream> rec, const std::vector<Path> &paths,
                               const std::string &base_entity_path = "paths") {
            for (size_t i = 0; i < paths.size(); ++i) {
                const auto &color = palette[i % palette.size()];
                show_path(rec, paths[i], base_entity_path + "/" + std::to_string(i), color);
            }
        }

        // Show robot pose
        inline void show_robot_pose(std::shared_ptr<rerun::RecordingStream> rec, const Pose &pose,
                                    const std::string &entity_path = "robot", const rerun::Color &color = {255, 255, 0},
                                    float scale = 1.0f) {
            // Robot position
            rec->log_static(
                entity_path + "/position",
                rerun::Points3D({{pose.point.x, pose.point.y, 0.0f}}).with_colors(color).with_radii(0.02f * scale));

            // Robot orientation (arrow)
            float arrow_length = 0.5f * scale;
            float end_x = pose.point.x + arrow_length * std::cos(pose.angle.yaw);
            float end_y = pose.point.y + arrow_length * std::sin(pose.angle.yaw);

            std::vector<rerun::Position3D> orientation_line = {{pose.point.x, pose.point.y, 0.0f},
                                                               {end_x, end_y, 0.0f}};
            rec->log_static(entity_path + "/orientation",
                            rerun::LineStrips3D(rerun::components::LineStrip3D(orientation_line)).with_colors(color));
        }

        // Show robot state with velocity vector
        inline void show_robot_state(std::shared_ptr<rerun::RecordingStream> rec, const RobotState &state,
                                     const std::string &entity_path = "robot_state",
                                     const rerun::Color &color = {255, 255, 0}, float scale = 1.0f) {
            // Show robot pose
            show_robot_pose(rec, state.pose, entity_path, color, scale);

            // Show velocity vector
            if (state.velocity.linear > 0.01f) {
                float vel_scale = 2.0f * scale;
                float end_x = state.pose.point.x + vel_scale * state.velocity.linear * std::cos(state.pose.angle.yaw);
                float end_y = state.pose.point.y + vel_scale * state.velocity.linear * std::sin(state.pose.angle.yaw);

                std::vector<rerun::Position3D> velocity_line = {{state.pose.point.x, state.pose.point.y, 0.0f},
                                                                {end_x, end_y, 0.0f}};
                rec->log_static(entity_path + "/velocity",
                                rerun::LineStrips3D(rerun::components::LineStrip3D(velocity_line))
                                    .with_colors({0, 255, 255})); // Cyan for velocity
            }
        }

        // Show controller status
        inline void show_controller_status(std::shared_ptr<rerun::RecordingStream> rec, const ControllerStatus &status,
                                           const std::string &entity_path = "controller_status") {
            std::string status_text = "Mode: " + status.mode + "\n";
            status_text += "Distance to goal: " + std::to_string(status.distance_to_goal) + "\n";
            status_text += "Heading error: " + std::to_string(status.heading_error) + "\n";
            status_text += "Goal reached: ";
            status_text += (status.goal_reached ? "Yes" : "No");

            rec->log_static(entity_path, rerun::TextLog(status_text).with_level(rerun::TextLogLevel::Info));
        }

        // Show goal
        inline void show_goal(std::shared_ptr<rerun::RecordingStream> rec, const Goal &goal,
                              const std::string &entity_path = "goal", const rerun::Color &color = {0, 255, 0}) {
            show_robot_pose(rec, goal.target_pose, entity_path, color, 1.5f);
        }

    } // namespace visualize
} // namespace navcon

#endif // HAS_RERUN
