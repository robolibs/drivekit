#pragma once

#include "../types.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace navcon {
namespace turners {

/**
 * @brief Sharper - Custom sharp turn planner for agricultural vehicles
 * 
 * Creates safe turning patterns for vehicles that need to make sharp turns
 * while respecting machine dimensions. Generates waypoints at safe distances
 * (0.5x, 1x, 1.5x machine length) and connects them with proper arcs.
 * 
 * Inspired by agricultural headland turning patterns:
 * - Three-point turn (switch-back)
 * - Bulb turn
 * - Fishtail turn
 */
class Sharper {
public:
    struct SharpTurnPath {
        std::vector<Pose> waypoints;
        std::vector<std::string> segment_types;  // "approach", "arc", "exit", etc.
        double total_length;
        std::string pattern_name;  // "three_point", "bulb", "fishtail"
    };

    Sharper(double min_turning_radius, double machine_length, double machine_width = 0.0)
        : radius_(min_turning_radius)
        , machine_length_(machine_length)
        , machine_width_(machine_width > 0 ? machine_width : machine_length * 0.4) {}

    /**
     * @brief Generate a sharp turn path between two poses
     * 
     * @param start Starting pose
     * @param end Ending pose
     * @param turn_angle Angle between start and end directions (radians)
     * @param pattern Type of turn: "three_point", "bulb", "fishtail", "auto"
     * @return SharpTurnPath with waypoints and metadata
     */
    SharpTurnPath plan_sharp_turn(const Pose& start, const Pose& end, 
                                  const std::string& pattern = "auto") const {
        
        // Calculate turn angle
        double turn_angle = calculate_turn_angle(start, end);
        
        if (pattern == "auto") {
            return select_best_pattern(start, end, turn_angle);
        } else if (pattern == "three_point") {
            return generate_three_point_turn(start, end, turn_angle);
        } else if (pattern == "bulb") {
            return generate_bulb_turn(start, end, turn_angle);
        } else if (pattern == "fishtail") {
            return generate_fishtail_turn(start, end, turn_angle);
        }
        
        return SharpTurnPath(); // Empty path if pattern not recognized
    }

    /**
     * @brief Generate a three-point (switch-back) turn
     * Used for very tight spaces where vehicle must reverse
     */
    SharpTurnPath generate_three_point_turn(const Pose& start, const Pose& end,
                                           double turn_angle) const {
        SharpTurnPath path;
        path.pattern_name = "three_point";
        
        // Key distances based on machine length
        double approach_dist = 1.5 * machine_length_;  // Distance to drive forward
        double backup_dist = 1.0 * machine_length_;    // Distance to reverse
        
        // Start position
        path.waypoints.push_back(start);
        path.segment_types.push_back("start");
        
        // 1. Drive forward in starting direction
        Pose forward_point = start;
        forward_point.point.x += approach_dist * cos(start.angle.yaw);
        forward_point.point.y += approach_dist * sin(start.angle.yaw);
        forward_point.angle.yaw = start.angle.yaw;
        path.waypoints.push_back(forward_point);
        path.segment_types.push_back("approach_forward");
        
        // 2. Calculate intermediate angle (roughly halfway between start and end)
        double intermediate_angle = normalize_angle(start.angle.yaw + turn_angle * 0.5);
        
        // 3. Reverse with steering toward intermediate direction
        Pose reverse_point = forward_point;
        reverse_point.point.x -= backup_dist * cos(intermediate_angle);
        reverse_point.point.y -= backup_dist * sin(intermediate_angle);
        reverse_point.angle.yaw = intermediate_angle;
        path.waypoints.push_back(reverse_point);
        path.segment_types.push_back("reverse");
        
        // 4. Drive forward toward final direction
        Pose final_approach = reverse_point;
        final_approach.point.x += approach_dist * cos(end.angle.yaw);
        final_approach.point.y += approach_dist * sin(end.angle.yaw);
        final_approach.angle.yaw = end.angle.yaw;
        path.waypoints.push_back(final_approach);
        path.segment_types.push_back("final_approach");
        
        // 5. End position
        path.waypoints.push_back(end);
        path.segment_types.push_back("end");
        
        calculate_path_length(path);
        return path;
    }

    /**
     * @brief Generate a bulb turn
     * A wider turn that looks like a light bulb shape
     */
    SharpTurnPath generate_bulb_turn(const Pose& start, const Pose& end,
                                    double turn_angle) const {
        SharpTurnPath path;
        path.pattern_name = "bulb";
        
        double bulb_radius = machine_length_ * 0.8;  // Bulb radius based on machine length
        
        // Start
        path.waypoints.push_back(start);
        path.segment_types.push_back("start");
        
        // Create bulb shape with multiple waypoints that follow the turn angle
        int num_bulb_points = 5;
        for (int i = 1; i <= num_bulb_points; ++i) {
            double t = i / double(num_bulb_points + 1);
            double current_angle = normalize_angle(start.angle.yaw + turn_angle * t);
            
            // Bulb offset (peaks in the middle) - perpendicular to current direction
            double bulb_factor = sin(M_PI * t) * bulb_radius;
            double perpendicular_angle = current_angle + M_PI/2;
            
            Pose bulb_point;
            // Base position along the turn
            bulb_point.point.x = start.point.x + radius_ * t * cos(current_angle);
            bulb_point.point.y = start.point.y + radius_ * t * sin(current_angle);
            
            // Add bulb offset perpendicular to the turn direction
            bulb_point.point.x += bulb_factor * cos(perpendicular_angle);
            bulb_point.point.y += bulb_factor * sin(perpendicular_angle);
            bulb_point.angle.yaw = current_angle;
            
            path.waypoints.push_back(bulb_point);
            path.segment_types.push_back("bulb_arc");
        }
        
        // End
        path.waypoints.push_back(end);
        path.segment_types.push_back("end");
        
        calculate_path_length(path);
        return path;
    }

    /**
     * @brief Generate a fishtail turn
     * An S-shaped turn that extends beyond the corner
     */
    SharpTurnPath generate_fishtail_turn(const Pose& start, const Pose& end,
                                        double turn_angle) const {
        SharpTurnPath path;
        path.pattern_name = "fishtail";
        
        // Key points for fishtail
        double extend_dist = machine_length_ * 1.2;
        
        // Start
        path.waypoints.push_back(start);
        path.segment_types.push_back("start");
        
        // 1. Initial straight approach in starting direction
        Pose approach;
        approach.point.x = start.point.x + 0.5 * machine_length_ * cos(start.angle.yaw);
        approach.point.y = start.point.y + 0.5 * machine_length_ * sin(start.angle.yaw);
        approach.angle.yaw = start.angle.yaw;
        path.waypoints.push_back(approach);
        path.segment_types.push_back("approach");
        
        // 2. First curve (opposite direction - the "tail")
        // Go in opposite direction to create the fishtail effect
        double tail_angle = normalize_angle(start.angle.yaw - turn_angle * 0.4);
        Pose tail_point;
        tail_point.point.x = approach.point.x + extend_dist * cos(tail_angle);
        tail_point.point.y = approach.point.y + extend_dist * sin(tail_angle);
        tail_point.angle.yaw = tail_angle;
        path.waypoints.push_back(tail_point);
        path.segment_types.push_back("tail_out");
        
        // 3. Transition toward final angle
        double transition_angle = normalize_angle(start.angle.yaw + turn_angle * 0.7);
        Pose transition_point;
        transition_point.point.x = tail_point.point.x + machine_length_ * cos(transition_angle);
        transition_point.point.y = tail_point.point.y + machine_length_ * sin(transition_angle);
        transition_point.angle.yaw = transition_angle;
        path.waypoints.push_back(transition_point);
        path.segment_types.push_back("transition");
        
        // 4. Final approach to end position and angle
        Pose final_approach;
        final_approach.point.x = end.point.x - 0.5 * machine_length_ * cos(end.angle.yaw);
        final_approach.point.y = end.point.y - 0.5 * machine_length_ * sin(end.angle.yaw);
        final_approach.angle.yaw = end.angle.yaw;
        path.waypoints.push_back(final_approach);
        path.segment_types.push_back("final_approach");
        
        // 5. End position
        path.waypoints.push_back(end);
        path.segment_types.push_back("end");
        
        calculate_path_length(path);
        return path;
    }

private:
    double radius_;
    double machine_length_;
    double machine_width_;
    
    double calculate_turn_angle(const Pose& start, const Pose& end) const {
        return normalize_angle(end.angle.yaw - start.angle.yaw);
    }
    
    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    void calculate_path_length(SharpTurnPath& path) const {
        path.total_length = 0.0;
        for (size_t i = 1; i < path.waypoints.size(); ++i) {
            double dx = path.waypoints[i].point.x - path.waypoints[i-1].point.x;
            double dy = path.waypoints[i].point.y - path.waypoints[i-1].point.y;
            path.total_length += sqrt(dx*dx + dy*dy);
        }
    }
    
    SharpTurnPath select_best_pattern(const Pose& start, const Pose& end,
                                     double turn_angle) const {
        double angle_deg = fabs(turn_angle) * 180.0 / M_PI;
        
        // Decision logic based on turn angle and available space
        if (angle_deg > 150) {
            // Very sharp turn - need three-point
            return generate_three_point_turn(start, end, turn_angle);
        } else if (angle_deg > 90) {
            // Moderate sharp turn - bulb is efficient
            return generate_bulb_turn(start, end, turn_angle);
        } else {
            // Wider turn - fishtail gives smooth path
            return generate_fishtail_turn(start, end, turn_angle);
        }
    }
};

} // namespace turners
} // namespace navcon