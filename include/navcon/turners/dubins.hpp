#pragma once

#include <vector>
#include <tuple>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include "navcon/types.hpp"

namespace navcon {
namespace turners {

enum class DubinsSegmentType {
    LEFT = 'L',
    STRAIGHT = 'S', 
    RIGHT = 'R'
};

struct DubinsSegment {
    DubinsSegmentType type;
    double length;
};

struct DubinsPath {
    std::vector<DubinsSegment> segments;
    std::vector<Pose> waypoints;
    double total_length;
    std::string name;
    
    DubinsPath() : total_length(0.0) {}
};

class Dubins {
public:
    Dubins(double min_turning_radius) : radius_(min_turning_radius) {}
    
    // Generate shortest Dubins path
    inline DubinsPath plan_path(const Pose& start, const Pose& end, double step_size = 0.1) const {
        auto all_paths = get_all_paths(start, end, step_size);
        
        if (all_paths.empty()) {
            return DubinsPath(); // Return empty path if no valid paths found
        }
        
        // Find shortest path
        auto shortest = std::min_element(all_paths.begin(), all_paths.end(),
            [](const DubinsPath& a, const DubinsPath& b) {
                return a.total_length < b.total_length;
            });
        
        return *shortest;
    }
    
    // Get all possible Dubins paths (6 types)
    inline std::vector<DubinsPath> get_all_paths(const Pose& start, const Pose& end, double step_size = 0.1) const {
        std::vector<DubinsPath> paths;
        
        // Transform to local coordinates
        auto [x, y, phi] = transform_to_local(start, end);
        
        // Try all 6 Dubins path types
        auto try_path = [&](auto compute_func, const std::string& name) {
            auto [valid, t, p, q] = compute_func(x, y, phi);
            if (valid && !std::isnan(t) && !std::isnan(p) && !std::isnan(q)) {
                std::vector<DubinsSegment> segments;
                
                // Create segments based on path type
                if (name == "LSL") {
                    segments = {{DubinsSegmentType::LEFT, t}, {DubinsSegmentType::STRAIGHT, p}, {DubinsSegmentType::LEFT, q}};
                } else if (name == "RSR") {
                    segments = {{DubinsSegmentType::RIGHT, t}, {DubinsSegmentType::STRAIGHT, p}, {DubinsSegmentType::RIGHT, q}};
                } else if (name == "LSR") {
                    segments = {{DubinsSegmentType::LEFT, t}, {DubinsSegmentType::STRAIGHT, p}, {DubinsSegmentType::RIGHT, q}};
                } else if (name == "RSL") {
                    segments = {{DubinsSegmentType::RIGHT, t}, {DubinsSegmentType::STRAIGHT, p}, {DubinsSegmentType::LEFT, q}};
                } else if (name == "RLR") {
                    segments = {{DubinsSegmentType::RIGHT, t}, {DubinsSegmentType::LEFT, p}, {DubinsSegmentType::RIGHT, q}};
                } else if (name == "LRL") {
                    segments = {{DubinsSegmentType::LEFT, t}, {DubinsSegmentType::RIGHT, p}, {DubinsSegmentType::LEFT, q}};
                }
                
                auto path = generate_waypoints(segments, start, name, step_size);
                if (!path.waypoints.empty()) {
                    paths.push_back(path);
                }
            }
        };
        
        try_path([this](double x, double y, double phi) { return compute_lsl(x, y, phi); }, "LSL");
        try_path([this](double x, double y, double phi) { return compute_rsr(x, y, phi); }, "RSR");
        try_path([this](double x, double y, double phi) { return compute_lsr(x, y, phi); }, "LSR");
        try_path([this](double x, double y, double phi) { return compute_rsl(x, y, phi); }, "RSL");
        try_path([this](double x, double y, double phi) { return compute_rlr(x, y, phi); }, "RLR");
        try_path([this](double x, double y, double phi) { return compute_lrl(x, y, phi); }, "LRL");
        
        return paths;
    }

protected:
    double radius_;
    
    // Utility functions
    inline double mod2pi(double theta) const {
        return theta - 2.0 * M_PI * std::floor(theta / (2.0 * M_PI));
    }
    
    inline double pi_2_pi(double theta) const {
        while (theta > M_PI) theta -= 2.0 * M_PI;
        while (theta < -M_PI) theta += 2.0 * M_PI;
        return theta;
    }
    
    // Transform poses to local coordinate system
    inline std::tuple<double, double, double> transform_to_local(const Pose& start, const Pose& end) const {
        double dx = end.point.x - start.point.x;
        double dy = end.point.y - start.point.y;
        double dth = end.angle.yaw - start.angle.yaw;
        
        // Transform to local coordinates (start pose becomes origin)
        double x = (dx * cos(start.angle.yaw) + dy * sin(start.angle.yaw)) / radius_;
        double y = (-dx * sin(start.angle.yaw) + dy * cos(start.angle.yaw)) / radius_;
        double phi = pi_2_pi(dth);
        
        return {x, y, phi};
    }
    
    // Six Dubins path types
    inline std::tuple<bool, double, double, double> compute_lsl(double x, double y, double phi) const {
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);
        
        double p_sq = 2 + x*x + y*y - 2*cos_phi + 2*x*(sin_phi - sin(0)) - 2*y*(cos_phi - cos(0));
        
        if (p_sq < 0) return {false, 0, 0, 0};
        
        double p = sqrt(p_sq);
        double theta = atan2(y - sin_phi, x - cos_phi + 1);
        double t = mod2pi(-theta);
        double q = mod2pi(phi - theta);
        
        return {true, t, p, q};
    }
    
    inline std::tuple<bool, double, double, double> compute_rsr(double x, double y, double phi) const {
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);
        
        double p_sq = 2 + x*x + y*y - 2*cos_phi - 2*x*(sin_phi - sin(0)) + 2*y*(cos_phi - cos(0));
        
        if (p_sq < 0) return {false, 0, 0, 0};
        
        double p = sqrt(p_sq);
        double theta = atan2(y - sin_phi, x - cos_phi - 1);
        double t = mod2pi(theta);
        double q = mod2pi(theta - phi);
        
        return {true, t, p, q};
    }
    
    inline std::tuple<bool, double, double, double> compute_lsr(double x, double y, double phi) const {
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);
        
        double p_sq = x*x + y*y - 2 + 2*cos_phi + 2*x*(sin_phi + sin(0)) + 2*y*(cos_phi + cos(0));
        
        if (p_sq < 0) return {false, 0, 0, 0};
        
        double p = sqrt(p_sq);
        double theta = atan2(y - sin_phi, x - cos_phi + 1) - atan2(2, p);
        double t = mod2pi(-theta);
        double q = mod2pi(-phi + theta);
        
        return {true, t, p, q};
    }
    
    inline std::tuple<bool, double, double, double> compute_rsl(double x, double y, double phi) const {
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);
        
        double p_sq = x*x + y*y - 2 + 2*cos_phi - 2*x*(sin_phi + sin(0)) - 2*y*(cos_phi + cos(0));
        
        if (p_sq < 0) return {false, 0, 0, 0};
        
        double p = sqrt(p_sq);
        double theta = atan2(y - sin_phi, x - cos_phi - 1) + atan2(2, p);
        double t = mod2pi(theta);
        double q = mod2pi(phi - theta);
        
        return {true, t, p, q};
    }
    
    inline std::tuple<bool, double, double, double> compute_rlr(double x, double y, double phi) const {
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);
        
        double xi = x - sin_phi;
        double eta = y - 1 + cos_phi;
        double rho = 0.25 * (2 + sqrt(xi*xi + eta*eta));
        
        if (rho > 1) return {false, 0, 0, 0};
        
        double u = acos(rho);
        double theta = atan2(eta, xi);
        double t = mod2pi(theta + u);
        double q = mod2pi(phi - t + u);
        
        return {true, t, 2*u, q};
    }
    
    inline std::tuple<bool, double, double, double> compute_lrl(double x, double y, double phi) const {
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);
        
        double xi = x - sin_phi;
        double eta = y - 1 + cos_phi;
        double rho = 0.25 * (2 + sqrt(xi*xi + eta*eta));
        
        if (rho > 1) return {false, 0, 0, 0};
        
        double u = acos(rho);
        double theta = atan2(eta, xi);
        double t = mod2pi(-theta + u);
        double q = mod2pi(-phi + t + u);
        
        return {true, t, 2*u, q};
    }
    
    // Generate waypoints from path parameters
    inline DubinsPath generate_waypoints(const std::vector<DubinsSegment>& segments, 
                                         const Pose& start, 
                                         const std::string& name,
                                         double step_size) const {
        DubinsPath path;
        path.segments = segments;
        path.name = name;
        path.total_length = 0;
        
        // Calculate total length
        for (const auto& seg : segments) {
            path.total_length += seg.length * radius_;
        }
        
        // Generate waypoints
        path.waypoints.push_back(start);
        Pose current_pose = start;
        
        for (const auto& segment : segments) {
            auto segment_waypoints = interpolate_segment(current_pose, segment, step_size);
            
            // Add waypoints (skip first one to avoid duplicates)
            for (size_t i = 1; i < segment_waypoints.size(); ++i) {
                path.waypoints.push_back(segment_waypoints[i]);
            }
            
            // Update current pose
            if (!segment_waypoints.empty()) {
                current_pose = segment_waypoints.back();
            }
        }
        
        return path;
    }
    
    // Interpolate along a single segment
    inline std::vector<Pose> interpolate_segment(const Pose& start_pose, 
                                                const DubinsSegment& segment,
                                                double step_size) const {
        std::vector<Pose> waypoints;
        double total_distance = segment.length * radius_;
        int num_steps = std::max(1, static_cast<int>(total_distance / step_size));
        double actual_step = total_distance / num_steps;
        
        waypoints.push_back(start_pose);
        
        for (int i = 1; i <= num_steps; ++i) {
            double distance = i * actual_step;
            Pose new_pose = apply_motion(start_pose, segment.type, distance);
            waypoints.push_back(new_pose);
        }
        
        return waypoints;
    }
    
    // Apply motion along a segment
    inline Pose apply_motion(const Pose& start, DubinsSegmentType type, double distance) const {
        Pose result = start;
        
        switch (type) {
            case DubinsSegmentType::STRAIGHT:
                result.point.x += distance * cos(start.angle.yaw);
                result.point.y += distance * sin(start.angle.yaw);
                break;
                
            case DubinsSegmentType::LEFT:
                {
                    double angle_change = distance / radius_;
                    result.angle.yaw += angle_change;
                    result.point.x += radius_ * (sin(result.angle.yaw) - sin(start.angle.yaw));
                    result.point.y += radius_ * (cos(start.angle.yaw) - cos(result.angle.yaw));
                }
                break;
                
            case DubinsSegmentType::RIGHT:
                {
                    double angle_change = distance / radius_;
                    result.angle.yaw -= angle_change;
                    result.point.x += radius_ * (sin(start.angle.yaw) - sin(result.angle.yaw));
                    result.point.y += radius_ * (cos(result.angle.yaw) - cos(start.angle.yaw));
                }
                break;
        }
        
        result.angle.yaw = pi_2_pi(result.angle.yaw);
        return result;
    }
};

} // namespace turners
} // namespace navcon