#ifdef HAS_RERUN

#include "navcon/tracking/utils/visualize.hpp"
#include <array>
#include <cmath>
#include <string>

namespace navcon {
    namespace tracking {
        namespace visualize {

            // Color palette for visualization
            const std::vector<rerun::Color> palette = {
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
            void show_path(std::shared_ptr<rerun::RecordingStream> rec, const Path &path,
                           const std::string &entity_path, const rerun::Color &color) {
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
            void show_paths(std::shared_ptr<rerun::RecordingStream> rec, const std::vector<Path> &paths,
                            const std::string &base_entity_path) {
                for (size_t i = 0; i < paths.size(); ++i) {
                    const auto &color = palette[i % palette.size()];
                    show_path(rec, paths[i], base_entity_path + "/" + std::to_string(i), color);
                }
            }

            // Show robot pose as a rectangle
            void show_robot_pose(std::shared_ptr<rerun::RecordingStream> rec, const Pose &pose,
                                 const std::string &entity_path, const rerun::Color &color, float scale) {
                // Robot dimensions
                float width = 0.4f * scale;  // Robot width
                float length = 0.6f * scale; // Robot length (front to back)

                float cos_yaw = std::cos(pose.angle.yaw);
                float sin_yaw = std::sin(pose.angle.yaw);

                // Calculate rectangle corners in robot frame, then transform to world frame
                // Front-left, Front-right, Back-right, Back-left, Front-left (closed rectangle)
                std::vector<std::array<float, 2>> corners_local = {
                    {length / 2, width / 2},   // Front-left
                    {length / 2, -width / 2},  // Front-right
                    {-length / 2, -width / 2}, // Back-right
                    {-length / 2, width / 2},  // Back-left
                    {length / 2, width / 2}    // Close the rectangle
                };

                std::vector<rerun::Position3D> rectangle_points;
                for (const auto &corner : corners_local) {
                    float x_world = pose.point.x + cos_yaw * corner[0] - sin_yaw * corner[1];
                    float y_world = pose.point.y + sin_yaw * corner[0] + cos_yaw * corner[1];
                    rectangle_points.push_back({x_world, y_world, 0.0f});
                }

                // Draw rectangle as a line strip
                rec->log_static(entity_path + "/body",
                                rerun::LineStrips3D(rerun::components::LineStrip3D(rectangle_points))
                                    .with_colors(color)
                                    .with_radii(0.025f * scale));

                // Add a small arrow at the front to show orientation
                float arrow_length = length * 0.4f;
                float front_center_x = pose.point.x + cos_yaw * length / 2;
                float front_center_y = pose.point.y + sin_yaw * length / 2;
                float arrow_end_x = pose.point.x + cos_yaw * (length / 2 + arrow_length);
                float arrow_end_y = pose.point.y + sin_yaw * (length / 2 + arrow_length);

                std::vector<rerun::Position3D> arrow_line = {{front_center_x, front_center_y, 0.0f},
                                                             {arrow_end_x, arrow_end_y, 0.0f}};
                rec->log_static(entity_path + "/orientation",
                                rerun::LineStrips3D(rerun::components::LineStrip3D(arrow_line))
                                    .with_colors(color)
                                    .with_radii(0.03f * scale));
            }

            // Show robot state
            void show_robot_state(std::shared_ptr<rerun::RecordingStream> rec, const RobotState &state,
                                  const std::string &entity_path, const rerun::Color &color, float scale) {
                // Show robot pose as rectangle
                show_robot_pose(rec, state.pose, entity_path, color, scale);
            }

            // Show controller status
            void show_controller_status(std::shared_ptr<rerun::RecordingStream> rec, const ControllerStatus &status,
                                        const std::string &entity_path) {
                std::string status_text = "Mode: " + status.mode + "\n";
                status_text += "Distance to goal: " + std::to_string(status.distance_to_goal) + "\n";
                status_text += "Heading error: " + std::to_string(status.heading_error) + "\n";
                status_text += "Goal reached: ";
                status_text += (status.goal_reached ? "Yes" : "No");

                rec->log_static(entity_path, rerun::TextLog(status_text).with_level(rerun::TextLogLevel::Info));
            }

            // Show goal as a simple point
            void show_goal(std::shared_ptr<rerun::RecordingStream> rec, const Goal &goal,
                           const std::string &entity_path, const rerun::Color &color) {
                rec->log_static(entity_path,
                                rerun::Points3D({{goal.target_pose.point.x, goal.target_pose.point.y, 0.0f}})
                                    .with_colors(color)
                                    .with_radii(0.15f));
            }

        } // namespace visualize
    } // namespace tracking
} // namespace navcon

#endif // HAS_RERUN
