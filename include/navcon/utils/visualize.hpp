#pragma once

#ifdef HAS_RERUN

#include "../types.hpp"
#include "../turners/dubins.hpp"
#include "../turners/reeds_shepp.hpp"
#include "../followers/pure_pursuit.hpp"
#include "../followers/carrot.hpp"
#include "../followers/pid.hpp"
#include <rerun.hpp>
#include <rerun/recording_stream.hpp>
#include <rerun/result.hpp>
#include <vector>
#include <memory>
#include <string>
#include <array>

namespace navcon {
namespace visualize {

// Color palette for visualization
static const std::vector<rerun::Color> palette = {
    {255, 0, 0},   // Red
    {0, 255, 0},   // Green
    {0, 0, 255},   // Blue
    {255, 255, 0}, // Yellow
    {255, 0, 255}, // Magenta
    {0, 255, 255}, // Cyan
    {255, 165, 0}, // Orange
    {128, 0, 128}, // Purple
    {255, 192, 203}, // Pink
    {165, 42, 42}  // Brown
};

// Show a single path
inline void show_path(std::shared_ptr<rerun::RecordingStream> rec, 
                     const Path& path, 
                     const std::string& entity_path = "path",
                     const rerun::Color& color = {0, 255, 0}) {
    if (path.waypoints.empty()) return;
    
    std::vector<rerun::Position3D> positions;
    for (const auto& waypoint : path.waypoints) {
        positions.emplace_back(waypoint.point.x, waypoint.point.y, 0.0f);
    }
    
    if (!positions.empty()) {
        rec->log_static(entity_path, rerun::LineStrips3D(rerun::components::LineStrip3D(positions)).with_colors(color));
    }
}

// Show multiple paths with different colors
inline void show_paths(std::shared_ptr<rerun::RecordingStream> rec,
                      const std::vector<Path>& paths,
                      const std::string& base_entity_path = "paths") {
    for (size_t i = 0; i < paths.size(); ++i) {
        const auto& color = palette[i % palette.size()];
        show_path(rec, paths[i], base_entity_path + "/" + std::to_string(i), color);
    }
}

// Show Dubins path
inline void show_dubins_path(std::shared_ptr<rerun::RecordingStream> rec,
                            const turners::DubinsPath& dubins_path,
                            const std::string& entity_path = "dubins_path",
                            const rerun::Color& color = {255, 0, 0}) {
    if (dubins_path.waypoints.empty()) return;
    
    std::vector<rerun::Position3D> positions;
    for (const auto& waypoint : dubins_path.waypoints) {
        positions.emplace_back(waypoint.point.x, waypoint.point.y, 0.0f);
    }
    
    if (!positions.empty()) {
        rec->log_static(entity_path, rerun::LineStrips3D(rerun::components::LineStrip3D(positions)).with_colors(color));
    }
    
    // Show segment types as text annotations
    if (!dubins_path.segments.empty()) {
        std::string segment_info = dubins_path.name + " [";
        for (size_t i = 0; i < dubins_path.segments.size(); ++i) {
            if (i > 0) segment_info += ", ";
            char seg_char = static_cast<char>(dubins_path.segments[i].type);
            segment_info += seg_char;
        }
        segment_info += "]";
        
        rec->log_static(entity_path + "/info", 
                rerun::TextLog(segment_info).with_level(rerun::TextLogLevel::Info));
    }
}

// Show multiple Dubins paths (all in gray)
inline void show_dubins_paths(std::shared_ptr<rerun::RecordingStream> rec,
                             const std::vector<turners::DubinsPath>& dubins_paths,
                             const std::string& base_entity_path = "dubins_paths") {
    rerun::Color gray_color(128, 128, 128); // Gray color for all paths
    for (size_t i = 0; i < dubins_paths.size(); ++i) {
        show_dubins_path(rec, dubins_paths[i], 
                        base_entity_path + "/" + dubins_paths[i].name, gray_color);
    }
}

// Show Reeds-Shepp path
inline void show_reeds_shepp_path(std::shared_ptr<rerun::RecordingStream> rec,
                                 const turners::ReedsSheppPath& rs_path,
                                 const std::string& entity_path = "reeds_shepp_path",
                                 const rerun::Color& color = {0, 0, 255}) {
    if (rs_path.waypoints.empty()) return;
    
    std::vector<rerun::Position3D> positions;
    std::vector<rerun::Color> colors;
    
    for (size_t i = 0; i < rs_path.waypoints.size(); ++i) {
        const auto& waypoint = rs_path.waypoints[i];
        positions.emplace_back(waypoint.point.x, waypoint.point.y, 0.0f);
        
        // Color code based on forward/backward motion
        // Find which segment this waypoint belongs to
        bool is_forward = true;
        if (!rs_path.segments.empty()) {
            // Simple heuristic: assume equal distribution of waypoints across segments
            size_t seg_idx = (i * rs_path.segments.size()) / rs_path.waypoints.size();
            seg_idx = std::min(seg_idx, rs_path.segments.size() - 1);
            is_forward = rs_path.segments[seg_idx].forward;
        }
        
        colors.emplace_back(is_forward ? color.r() : color.r()/2, 
                           is_forward ? color.g() : color.g()/2, 
                           is_forward ? color.b() : color.b()/2);
    }
    
    rec->log_static(entity_path, rerun::LineStrips3D(rerun::components::LineStrip3D(positions)).with_colors(colors));
    
    // Show segment types as text annotations
    if (!rs_path.segments.empty()) {
        std::string segment_info = rs_path.name + " [";
        for (size_t i = 0; i < rs_path.segments.size(); ++i) {
            if (i > 0) segment_info += ", ";
            char seg_char = static_cast<char>(rs_path.segments[i].type);
            segment_info += seg_char;
            if (!rs_path.segments[i].forward) segment_info += "(rev)";
        }
        segment_info += "]";
        
        rec->log_static(entity_path + "/info", 
                rerun::TextLog(segment_info).with_level(rerun::TextLogLevel::Info));
    }
}

// Show multiple Reeds-Shepp paths (all in gray)
inline void show_reeds_shepp_paths(std::shared_ptr<rerun::RecordingStream> rec,
                                  const std::vector<turners::ReedsSheppPath>& rs_paths,
                                  const std::string& base_entity_path = "reeds_shepp_paths") {
    rerun::Color gray_color(128, 128, 128); // Gray color for all paths
    for (size_t i = 0; i < rs_paths.size(); ++i) {
        show_reeds_shepp_path(rec, rs_paths[i], 
                             base_entity_path + "/" + rs_paths[i].name, gray_color);
    }
}

// Show robot pose
inline void show_robot_pose(std::shared_ptr<rerun::RecordingStream> rec,
                           const Pose& pose,
                           const std::string& entity_path = "robot",
                           const rerun::Color& color = {255, 255, 0},
                           float scale = 1.0f) {
    // Robot position
    rec->log_static(entity_path + "/position", 
            rerun::Points3D({{pose.point.x, pose.point.y, 0.0f}})
                .with_colors(color)
                .with_radii(0.05f * scale));
    
    // Robot orientation (arrow)
    float arrow_length = 0.5f * scale;
    float end_x = pose.point.x + arrow_length * std::cos(pose.angle.yaw);
    float end_y = pose.point.y + arrow_length * std::sin(pose.angle.yaw);
    
    std::vector<rerun::Position3D> orientation_line = {{pose.point.x, pose.point.y, 0.0f}, 
                                                        {end_x, end_y, 0.0f}};
    rec->log_static(entity_path + "/orientation", 
            rerun::LineStrips3D(rerun::components::LineStrip3D(orientation_line))
                .with_colors(color));
}

// Show robot state with velocity vector
inline void show_robot_state(std::shared_ptr<rerun::RecordingStream> rec,
                            const RobotState& state,
                            const std::string& entity_path = "robot_state",
                            const rerun::Color& color = {255, 255, 0},
                            float scale = 1.0f) {
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
inline void show_controller_status(std::shared_ptr<rerun::RecordingStream> rec,
                                  const ControllerStatus& status,
                                  const std::string& entity_path = "controller_status") {
    std::string status_text = "Mode: " + status.mode + "\n";
    status_text += "Distance to goal: " + std::to_string(status.distance_to_goal) + "\n";
    status_text += "Heading error: " + std::to_string(status.heading_error) + "\n";
    status_text += "Goal reached: ";
    status_text += (status.goal_reached ? "Yes" : "No");
    
    rec->log_static(entity_path, 
            rerun::TextLog(status_text).with_level(rerun::TextLogLevel::Info));
}

// Show goal
inline void show_goal(std::shared_ptr<rerun::RecordingStream> rec,
                     const Goal& goal,
                     const std::string& entity_path = "goal",
                     const rerun::Color& color = {0, 255, 0}) {
    show_robot_pose(rec, goal.target_pose, entity_path, color, 1.5f);
}

// Show coordinate system
inline void show_coordinate_system(std::shared_ptr<rerun::RecordingStream> rec,
                                  const Point& origin = {0, 0},
                                  const std::string& entity_path = "coordinate_system",
                                  float scale = 1.0f) {
    // X-axis (red)
    std::vector<rerun::Position3D> x_axis = {{origin.x, origin.y, 0.0f}, 
                                              {origin.x + scale, origin.y, 0.0f}};
    rec->log_static(entity_path + "/x_axis", 
            rerun::LineStrips3D(rerun::components::LineStrip3D(x_axis))
                .with_colors({255, 0, 0}));
    
    // Y-axis (green)
    std::vector<rerun::Position3D> y_axis = {{origin.x, origin.y, 0.0f}, 
                                              {origin.x, origin.y + scale, 0.0f}};
    rec->log_static(entity_path + "/y_axis", 
            rerun::LineStrips3D(rerun::components::LineStrip3D(y_axis))
                .with_colors({0, 255, 0}));
    
    // Origin point
    std::vector<rerun::Position3D> origin_point = {{origin.x, origin.y, 0.0f}};
    rec->log_static(entity_path + "/origin", 
            rerun::Points3D(origin_point)
                .with_colors({255, 255, 255})
                .with_radii(0.1f));
}

// Show grid
inline void show_grid(std::shared_ptr<rerun::RecordingStream> rec,
                     const Point& center = {0, 0},
                     float size = 10.0f,
                     float spacing = 1.0f,
                     const std::string& entity_path = "grid") {
    std::vector<std::vector<rerun::Position3D>> grid_lines;
    
    // Vertical lines
    for (float x = center.x - size; x <= center.x + size; x += spacing) {
        grid_lines.push_back({
            {x, center.y - size, 0.0f},
            {x, center.y + size, 0.0f}
        });
    }
    
    // Horizontal lines
    for (float y = center.y - size; y <= center.y + size; y += spacing) {
        grid_lines.push_back({
            {center.x - size, y, 0.0f},
            {center.x + size, y, 0.0f}
        });
    }
    
    if (!grid_lines.empty()) {
        std::vector<rerun::components::LineStrip3D> strips;
        for (const auto& line : grid_lines) {
            strips.push_back(rerun::components::LineStrip3D(line));
        }
        rec->log_static(entity_path, 
                rerun::LineStrips3D(strips)
                    .with_colors({128, 128, 128})); // Gray
    }
}

// Initialize recording stream
inline std::shared_ptr<rerun::RecordingStream> init_recording(const std::string& app_name = "navcon") {
    auto rec = std::make_shared<rerun::RecordingStream>(app_name, "space");
    // Save session (ignore return value)
    auto result = rec->save("navcon_session.rrd");
    (void)result;
    return rec;
}

// Clear all entities
inline void clear_all(std::shared_ptr<rerun::RecordingStream> rec) {
    rec->log("", rerun::Clear(true));
}

} // namespace visualize
} // namespace navcon

#endif // HAS_RERUN