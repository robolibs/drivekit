#pragma once

#include "dubins.hpp"
#include <array>

namespace navcon {
namespace turners {

enum class ReedsSheppSegmentType {
    LEFT_FORWARD = 'L',
    STRAIGHT_FORWARD = 'S',
    RIGHT_FORWARD = 'R',
    LEFT_BACKWARD = 'l',
    STRAIGHT_BACKWARD = 's',
    RIGHT_BACKWARD = 'r'
};

struct ReedsSheppSegment {
    ReedsSheppSegmentType type;
    double length;
    bool forward;
    
    ReedsSheppSegment(ReedsSheppSegmentType t, double l, bool fwd = true) 
        : type(t), length(l), forward(fwd) {}
};

struct ReedsSheppPath {
    std::vector<ReedsSheppSegment> segments;
    std::vector<Pose> waypoints;
    double total_length;
    std::string name;
    
    ReedsSheppPath() : total_length(0.0) {}
};

class ReedsShepp : public Dubins {
public:
    ReedsShepp(double min_turning_radius) : Dubins(min_turning_radius) {}
    
    // Generate shortest Reeds-Shepp path
    inline ReedsSheppPath plan_path(const Pose& start, const Pose& end, double step_size = 0.1) const {
        auto all_paths = get_all_paths(start, end, step_size);
        
        if (all_paths.empty()) {
            return ReedsSheppPath();
        }
        
        auto shortest = std::min_element(all_paths.begin(), all_paths.end(),
            [](const ReedsSheppPath& a, const ReedsSheppPath& b) {
                return a.total_length < b.total_length;
            });
        
        return *shortest;
    }
    
    // Get all possible Reeds-Shepp paths (48 types)
    inline std::vector<ReedsSheppPath> get_all_paths(const Pose& start, const Pose& end, double step_size = 0.1) const {
        std::vector<ReedsSheppPath> paths;
        
        // Transform to local coordinates
        auto [x, y, phi] = transform_to_local(start, end);
        
        // Generate all path families
        auto csc_paths = generate_csc_paths(x, y, phi, start, step_size);
        auto ccc_paths = generate_ccc_paths(x, y, phi, start, step_size);
        auto cccc_paths = generate_cccc_paths(x, y, phi, start, step_size);
        auto ccsc_paths = generate_ccsc_paths(x, y, phi, start, step_size);
        auto ccscc_paths = generate_ccscc_paths(x, y, phi, start, step_size);
        
        // Combine all paths
        paths.insert(paths.end(), csc_paths.begin(), csc_paths.end());
        paths.insert(paths.end(), ccc_paths.begin(), ccc_paths.end());
        paths.insert(paths.end(), cccc_paths.begin(), cccc_paths.end());
        paths.insert(paths.end(), ccsc_paths.begin(), ccsc_paths.end());
        paths.insert(paths.end(), ccscc_paths.begin(), ccscc_paths.end());
        
        return paths;
    }

private:
    // Generate CSC (Curve-Straight-Curve) paths
    inline std::vector<ReedsSheppPath> generate_csc_paths(double x, double y, double phi, 
                                                         const Pose& start, double step_size) const {
        std::vector<ReedsSheppPath> paths;
        
        // LSL, RSR, LSR, RSL (forward)
        auto try_csc = [&](auto compute_func, const std::string& name, 
                          ReedsSheppSegmentType seg1, ReedsSheppSegmentType seg2, ReedsSheppSegmentType seg3) {
            auto [valid, t, p, q] = compute_func(x, y, phi);
            if (valid && !std::isnan(t) && !std::isnan(p) && !std::isnan(q)) {
                std::vector<ReedsSheppSegment> segments = {
                    {seg1, t, true}, {seg2, p, true}, {seg3, q, true}
                };
                paths.push_back(generate_rs_waypoints(segments, start, name, step_size));
            }
        };
        
        try_csc([this](double x, double y, double phi) { return compute_lsl(x, y, phi); }, 
                "LSL", ReedsSheppSegmentType::LEFT_FORWARD, ReedsSheppSegmentType::STRAIGHT_FORWARD, ReedsSheppSegmentType::LEFT_FORWARD);
        try_csc([this](double x, double y, double phi) { return compute_rsr(x, y, phi); }, 
                "RSR", ReedsSheppSegmentType::RIGHT_FORWARD, ReedsSheppSegmentType::STRAIGHT_FORWARD, ReedsSheppSegmentType::RIGHT_FORWARD);
        try_csc([this](double x, double y, double phi) { return compute_lsr(x, y, phi); }, 
                "LSR", ReedsSheppSegmentType::LEFT_FORWARD, ReedsSheppSegmentType::STRAIGHT_FORWARD, ReedsSheppSegmentType::RIGHT_FORWARD);
        try_csc([this](double x, double y, double phi) { return compute_rsl(x, y, phi); }, 
                "RSL", ReedsSheppSegmentType::RIGHT_FORWARD, ReedsSheppSegmentType::STRAIGHT_FORWARD, ReedsSheppSegmentType::LEFT_FORWARD);
        
        // Add reverse variants
        try_csc([this](double x, double y, double phi) { return compute_lsl(-x, y, -phi); }, 
                "LsL", ReedsSheppSegmentType::LEFT_FORWARD, ReedsSheppSegmentType::STRAIGHT_BACKWARD, ReedsSheppSegmentType::LEFT_FORWARD);
        try_csc([this](double x, double y, double phi) { return compute_rsr(-x, y, -phi); }, 
                "RsR", ReedsSheppSegmentType::RIGHT_FORWARD, ReedsSheppSegmentType::STRAIGHT_BACKWARD, ReedsSheppSegmentType::RIGHT_FORWARD);
        try_csc([this](double x, double y, double phi) { return compute_lsr(-x, y, -phi); }, 
                "LsR", ReedsSheppSegmentType::LEFT_FORWARD, ReedsSheppSegmentType::STRAIGHT_BACKWARD, ReedsSheppSegmentType::RIGHT_FORWARD);
        try_csc([this](double x, double y, double phi) { return compute_rsl(-x, y, -phi); }, 
                "RsL", ReedsSheppSegmentType::RIGHT_FORWARD, ReedsSheppSegmentType::STRAIGHT_BACKWARD, ReedsSheppSegmentType::LEFT_FORWARD);
        
        return paths;
    }
    
    // Generate CCC (Curve-Curve-Curve) paths
    inline std::vector<ReedsSheppPath> generate_ccc_paths(double x, double y, double phi, 
                                                         const Pose& start, double step_size) const {
        std::vector<ReedsSheppPath> paths;
        
        // LRL, RLR
        auto try_ccc = [&](auto compute_func, const std::string& name, 
                          ReedsSheppSegmentType seg1, ReedsSheppSegmentType seg2, ReedsSheppSegmentType seg3) {
            auto [valid, t, p, q] = compute_func(x, y, phi);
            if (valid && !std::isnan(t) && !std::isnan(p) && !std::isnan(q)) {
                std::vector<ReedsSheppSegment> segments = {
                    {seg1, t, true}, {seg2, p, true}, {seg3, q, true}
                };
                paths.push_back(generate_rs_waypoints(segments, start, name, step_size));
            }
        };
        
        try_ccc([this](double x, double y, double phi) { return compute_lrl(x, y, phi); }, 
                "LRL", ReedsSheppSegmentType::LEFT_FORWARD, ReedsSheppSegmentType::RIGHT_FORWARD, ReedsSheppSegmentType::LEFT_FORWARD);
        try_ccc([this](double x, double y, double phi) { return compute_rlr(x, y, phi); }, 
                "RLR", ReedsSheppSegmentType::RIGHT_FORWARD, ReedsSheppSegmentType::LEFT_FORWARD, ReedsSheppSegmentType::RIGHT_FORWARD);
        
        // Add reverse variants
        try_ccc([this](double x, double y, double phi) { return compute_lrl(-x, y, -phi); }, 
                "lrl", ReedsSheppSegmentType::LEFT_BACKWARD, ReedsSheppSegmentType::RIGHT_BACKWARD, ReedsSheppSegmentType::LEFT_BACKWARD);
        try_ccc([this](double x, double y, double phi) { return compute_rlr(-x, y, -phi); }, 
                "rlr", ReedsSheppSegmentType::RIGHT_BACKWARD, ReedsSheppSegmentType::LEFT_BACKWARD, ReedsSheppSegmentType::RIGHT_BACKWARD);
        
        return paths;
    }
    
    // Generate CCCC (Curve-Curve-Curve-Curve) paths
    inline std::vector<ReedsSheppPath> generate_cccc_paths(double x, double y, double phi, 
                                                          const Pose& start, double step_size) const {
        std::vector<ReedsSheppPath> paths;
        
        // LRLR variants
        auto [valid, t, u, v] = compute_lrlr(x, y, phi);
        if (valid && !std::isnan(t) && !std::isnan(u) && !std::isnan(v)) {
            std::vector<ReedsSheppSegment> segments = {
                {ReedsSheppSegmentType::LEFT_FORWARD, t, true}, 
                {ReedsSheppSegmentType::RIGHT_FORWARD, u, true}, 
                {ReedsSheppSegmentType::LEFT_FORWARD, u, true}, 
                {ReedsSheppSegmentType::RIGHT_FORWARD, v, true}
            };
            paths.push_back(generate_rs_waypoints(segments, start, "LRLR", step_size));
        }
        
        // Add more CCCC variants as needed
        
        return paths;
    }
    
    // Generate CCSC (Curve-Curve-Straight-Curve) paths
    inline std::vector<ReedsSheppPath> generate_ccsc_paths(double x, double y, double phi, 
                                                          const Pose& start, double step_size) const {
        std::vector<ReedsSheppPath> paths;
        
        // LRSL variants
        auto [valid, t, u, v] = compute_lrsl(x, y, phi);
        if (valid && !std::isnan(t) && !std::isnan(u) && !std::isnan(v)) {
            std::vector<ReedsSheppSegment> segments = {
                {ReedsSheppSegmentType::LEFT_FORWARD, t, true}, 
                {ReedsSheppSegmentType::RIGHT_FORWARD, M_PI_2, true}, 
                {ReedsSheppSegmentType::STRAIGHT_FORWARD, u, true}, 
                {ReedsSheppSegmentType::LEFT_FORWARD, v, true}
            };
            paths.push_back(generate_rs_waypoints(segments, start, "LRSL", step_size));
        }
        
        return paths;
    }
    
    // Generate CCSCC (Curve-Curve-Straight-Curve-Curve) paths
    inline std::vector<ReedsSheppPath> generate_ccscc_paths(double x, double y, double phi, 
                                                           const Pose& start, double step_size) const {
        std::vector<ReedsSheppPath> paths;
        
        // LRSLR variants
        auto [valid, t, u, v] = compute_lrslr(x, y, phi);
        if (valid && !std::isnan(t) && !std::isnan(u) && !std::isnan(v)) {
            std::vector<ReedsSheppSegment> segments = {
                {ReedsSheppSegmentType::LEFT_FORWARD, t, true}, 
                {ReedsSheppSegmentType::RIGHT_FORWARD, M_PI_2, true}, 
                {ReedsSheppSegmentType::STRAIGHT_FORWARD, u, true}, 
                {ReedsSheppSegmentType::LEFT_FORWARD, M_PI_2, true}, 
                {ReedsSheppSegmentType::RIGHT_FORWARD, v, true}
            };
            paths.push_back(generate_rs_waypoints(segments, start, "LRSLR", step_size));
        }
        
        return paths;
    }
    
    // Additional RS computation functions
    inline std::tuple<bool, double, double, double> compute_lrlr(double x, double y, double phi) const {
        double xi = x + sin(phi);
        double eta = y - 1.0 - cos(phi);
        double rho = 0.25 * (2.0 + sqrt(xi*xi + eta*eta));
        
        if (rho <= 1.0) {
            double u = acos(rho);
            double alpha = atan2(eta, xi);
            double t = mod2pi(alpha + u);
            double v = mod2pi(t - phi);
            return {true, t, u, v};
        }
        return {false, 0, 0, 0};
    }
    
    inline std::tuple<bool, double, double, double> compute_lrsl(double x, double y, double phi) const {
        double xi = x - sin(phi);
        double eta = y - 1.0 + cos(phi);
        double rho = sqrt(xi*xi + eta*eta);
        
        if (rho >= 2.0) {
            double r = sqrt(rho*rho - 4.0);
            double u = 2.0 - r;
            double theta = atan2(eta, xi);
            double t = mod2pi(theta + atan2(r, -2.0));
            double v = mod2pi(phi - M_PI_2 - t);
            return {true, t, u, v};
        }
        return {false, 0, 0, 0};
    }
    
    inline std::tuple<bool, double, double, double> compute_lrslr(double x, double y, double phi) const {
        double xi = x + sin(phi);
        double eta = y - 1.0 - cos(phi);
        double rho = sqrt(xi*xi + eta*eta);
        
        if (rho >= 2.0) {
            double u = 4.0 - sqrt(rho*rho - 4.0);
            if (u <= 0.0) {
                double theta = atan2((4.0 - u)*xi - 2.0*eta, -2.0*xi + (u - 4.0)*eta);
                double t = mod2pi(theta);
                double v = mod2pi(t - phi);
                return {true, t, u, v};
            }
        }
        return {false, 0, 0, 0};
    }
    
    // Generate waypoints for Reeds-Shepp path
    inline ReedsSheppPath generate_rs_waypoints(const std::vector<ReedsSheppSegment>& segments, 
                                               const Pose& start, 
                                               const std::string& name,
                                               double step_size) const {
        ReedsSheppPath path;
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
            auto segment_waypoints = interpolate_rs_segment(current_pose, segment, step_size);
            
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
    
    // Interpolate along a single Reeds-Shepp segment
    inline std::vector<Pose> interpolate_rs_segment(const Pose& start_pose, 
                                                   const ReedsSheppSegment& segment,
                                                   double step_size) const {
        std::vector<Pose> waypoints;
        double total_distance = segment.length * radius_;
        int num_steps = std::max(1, static_cast<int>(total_distance / step_size));
        double actual_step = total_distance / num_steps;
        
        waypoints.push_back(start_pose);
        
        for (int i = 1; i <= num_steps; ++i) {
            double distance = i * actual_step;
            Pose new_pose = apply_rs_motion(start_pose, segment.type, distance, segment.forward);
            waypoints.push_back(new_pose);
        }
        
        return waypoints;
    }
    
    // Apply motion along a Reeds-Shepp segment
    inline Pose apply_rs_motion(const Pose& start, ReedsSheppSegmentType type, double distance, bool forward) const {
        Pose result = start;
        double direction = forward ? 1.0 : -1.0;
        
        switch (type) {
            case ReedsSheppSegmentType::STRAIGHT_FORWARD:
            case ReedsSheppSegmentType::STRAIGHT_BACKWARD:
                result.point.x += direction * distance * cos(start.angle.yaw);
                result.point.y += direction * distance * sin(start.angle.yaw);
                break;
                
            case ReedsSheppSegmentType::LEFT_FORWARD:
            case ReedsSheppSegmentType::LEFT_BACKWARD:
                {
                    double angle_change = direction * distance / radius_;
                    result.angle.yaw += angle_change;
                    result.point.x += radius_ * (sin(result.angle.yaw) - sin(start.angle.yaw));
                    result.point.y += radius_ * (cos(start.angle.yaw) - cos(result.angle.yaw));
                }
                break;
                
            case ReedsSheppSegmentType::RIGHT_FORWARD:
            case ReedsSheppSegmentType::RIGHT_BACKWARD:
                {
                    double angle_change = direction * distance / radius_;
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