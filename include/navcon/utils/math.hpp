#pragma once

#include <cmath>
#include <algorithm>
#include "navcon/types.hpp"

namespace navcon {
namespace utils {

class Math {
public:
    // Angle utilities
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    static double angle_difference(double angle1, double angle2) {
        return normalize_angle(angle1 - angle2);
    }
    
    // Linear interpolation
    static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }
    
    // Pose interpolation
    static Pose lerp_pose(const Pose& p1, const Pose& p2, double t) {
        Pose result;
        result.point.x = lerp(p1.point.x, p2.point.x, t);
        result.point.y = lerp(p1.point.y, p2.point.y, t);
        result.point.z = lerp(p1.point.z, p2.point.z, t);
        
        // Angle interpolation (shortest path)
        double angle_diff = angle_difference(p2.angle.yaw, p1.angle.yaw);
        result.angle.yaw = p1.angle.yaw + t * angle_diff;
        result.angle.pitch = lerp(p1.angle.pitch, p2.angle.pitch, t);
        result.angle.roll = lerp(p1.angle.roll, p2.angle.roll, t);
        
        return result;
    }
    
    // Distance calculations
    static double distance_2d(const Point& p1, const Point& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    static double distance_3d(const Point& p1, const Point& p2) {
        return p1.distance_to(p2);
    }
    
    // Path length calculation
    static double path_length(const std::vector<Pose>& waypoints) {
        if (waypoints.size() < 2) return 0.0;
        
        double total = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            total += distance_2d(waypoints[i-1].point, waypoints[i].point);
        }
        return total;
    }
    
    // Find closest point on line segment
    static Point closest_point_on_segment(const Point& p, const Point& a, const Point& b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double length_sq = dx * dx + dy * dy;
        
        if (length_sq < 1e-6) {
            return a;  // Degenerate segment
        }
        
        // Project point onto line
        double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq;
        t = std::clamp(t, 0.0, 1.0);
        
        Point closest;
        closest.x = a.x + t * dx;
        closest.y = a.y + t * dy;
        closest.z = lerp(a.z, b.z, t);
        
        return closest;
    }
    
    // Cross track error (signed distance from line)
    static double cross_track_error(const Point& p, const Point& a, const Point& b) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double length = std::sqrt(dx * dx + dy * dy);
        
        if (length < 1e-6) return 0.0;
        
        // Normal vector (perpendicular to line)
        double nx = -dy / length;
        double ny = dx / length;
        
        // Signed distance
        return (p.x - a.x) * nx + (p.y - a.y) * ny;
    }
    
    // Curvature calculation from three points
    static double curvature_from_points(const Point& p1, const Point& p2, const Point& p3) {
        // Menger curvature formula
        double area = std::abs(
            (p2.x - p1.x) * (p3.y - p1.y) - 
            (p3.x - p1.x) * (p2.y - p1.y)
        ) / 2.0;
        
        double d12 = distance_2d(p1, p2);
        double d23 = distance_2d(p2, p3);
        double d13 = distance_2d(p1, p3);
        
        if (d12 * d23 * d13 < 1e-6) return 0.0;
        
        return 4.0 * area / (d12 * d23 * d13);
    }
    
    // Smooth a value with rate limiting
    static double rate_limit(double current, double target, double max_rate, double dt) {
        double diff = target - current;
        double max_change = max_rate * dt;
        
        if (std::abs(diff) <= max_change) {
            return target;
        }
        
        return current + std::copysign(max_change, diff);
    }
    
    // Dead zone application
    static double apply_deadzone(double value, double deadzone) {
        if (std::abs(value) < deadzone) return 0.0;
        
        // Scale remaining range
        double sign = value > 0 ? 1.0 : -1.0;
        return sign * (std::abs(value) - deadzone) / (1.0 - deadzone);
    }
    
    // Sigmoid function for smooth transitions
    static double sigmoid(double x, double steepness = 1.0) {
        return 1.0 / (1.0 + std::exp(-steepness * x));
    }
    
    // Map value from one range to another
    static double map_range(double value, double in_min, double in_max, double out_min, double out_max) {
        double t = (value - in_min) / (in_max - in_min);
        t = std::clamp(t, 0.0, 1.0);
        return out_min + t * (out_max - out_min);
    }
};

} // namespace utils
} // namespace navcon