#include "drivekit/controller.hpp"
#include <cmath>

namespace drivekit {

    double Controller::normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double Controller::calculate_distance(const Point &p1, const Point &p2) { return p1.distance_to(p2); }

    double Controller::calculate_heading_error(const Pose &current, const Point &target) {
        double dx = target.x - current.point.x;
        double dy = target.y - current.point.y;
        double desired_heading = std::atan2(dy, dx);
        return normalize_angle(desired_heading - current.rotation.to_euler().yaw);
    }

    bool Controller::is_goal_reached(const Pose &current, const Pose &goal, double tolerance) {
        double dist = calculate_distance(current.point, goal.point);
        double angle_diff = std::abs(normalize_angle(goal.rotation.to_euler().yaw - current.rotation.to_euler().yaw));

        status_.distance_to_goal = dist;
        status_.heading_error = angle_diff;

        // Use provided tolerance if positive, otherwise use config default
        double pos_tol = (tolerance > 0.0) ? tolerance : config_.goal_tolerance;
        return dist < pos_tol && angle_diff < config_.angular_tolerance;
    }

} // namespace drivekit
