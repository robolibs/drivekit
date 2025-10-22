#pragma once

#include "dubins.hpp"
#include <algorithm>
#include <array>
#include <cassert>

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

            inline ReedsSheppSegment(ReedsSheppSegmentType t, double l, bool fwd = true) : type(t), length(l), forward(fwd) {}
        };

        struct ReedsSheppPath {
            std::vector<ReedsSheppSegment> segments;
            std::vector<Pose> waypoints;
            double total_length;
            std::string name;

            inline ReedsSheppPath() : total_length(0.0) {}
        };

        class ReedsSheppStateSpace {
          public:
            enum ReedsSheppPathSegmentType { RS_NOP = 0, RS_LEFT = 1, RS_STRAIGHT = 2, RS_RIGHT = 3 };

            static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

            class ReedsSheppPath {
              public:
                ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                               double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                               double w = 0., double x = 0.)
                    : type_(type), totalLength_(fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x)) {
                    length_[0] = t;
                    length_[1] = u;
                    length_[2] = v;
                    length_[3] = w;
                    length_[4] = x;
                }

                inline double length() const { return totalLength_; }

                const ReedsSheppPathSegmentType *type_;
                double length_[5];
                double totalLength_;
            };

            inline ReedsSheppStateSpace(double turningRadius) : rho_(turningRadius) {}

            inline double distance(double q0[3], double q1[3]) const;
            inline std::vector<ReedsSheppPathSegmentType> type(double q0[3], double q1[3]) const;
            inline void sample(double q0[3], double q1[3], double step_size, double &length,
                               std::vector<std::vector<double>> &points) const;
            inline ReedsSheppPath reedsShepp(double q0[3], double q1[3]) const;
            inline std::vector<ReedsSheppPath> getAllReedsSheppPaths(double q0[3], double q1[3]) const;
            inline void interpolate(double q0[3], ReedsSheppPath &path, double seg, double s[3]) const;

          protected:
            double rho_;
        };

        class ReedsShepp : public Dubins {
          public:
            inline ReedsShepp(double min_turning_radius) : Dubins(min_turning_radius), rs_(min_turning_radius) {}

            inline ReedsSheppPath plan_path(const Pose &start, const Pose &end, double step_size = 0.1) const {
                auto all_paths = get_all_paths(start, end, step_size);

                if (all_paths.empty()) {
                    return ReedsSheppPath();
                }

                auto shortest = std::min_element(
                    all_paths.begin(), all_paths.end(),
                    [](const ReedsSheppPath &a, const ReedsSheppPath &b) { return a.total_length < b.total_length; });

                return *shortest;
            }

            inline std::vector<ReedsSheppPath> get_all_paths(const Pose &start, const Pose &end,
                                                             double step_size = 0.1) const {
                std::vector<ReedsSheppPath> paths;

                // Convert to ReedsSheppStateSpace format
                double q0[3] = {start.point.x, start.point.y, start.angle.yaw};
                double q1[3] = {end.point.x, end.point.y, end.angle.yaw};

                // Get all possible Reeds-Shepp paths using the proper implementation
                auto rs_paths = rs_.getAllReedsSheppPaths(q0, q1);

                // Convert to our format
                for (const auto &rs_path : rs_paths) {
                    if (rs_path.length() < std::numeric_limits<double>::max()) {
                        ReedsSheppPath path;
                        path.total_length = rs_path.length() * radius_;
                        path.name = getPathName(rs_path);

                        // Convert segments
                        for (int i = 0; i < 5; ++i) {
                            if (rs_path.type_[i] == ReedsSheppStateSpace::RS_NOP) break;

                            ReedsSheppSegment segment(ReedsSheppSegmentType::LEFT_FORWARD, 0.0, true);
                            bool forward = rs_path.length_[i] >= 0;

                            switch (rs_path.type_[i]) {
                            case ReedsSheppStateSpace::RS_LEFT:
                                segment.type = forward ? ReedsSheppSegmentType::LEFT_FORWARD
                                                       : ReedsSheppSegmentType::LEFT_BACKWARD;
                                break;
                            case ReedsSheppStateSpace::RS_RIGHT:
                                segment.type = forward ? ReedsSheppSegmentType::RIGHT_FORWARD
                                                       : ReedsSheppSegmentType::RIGHT_BACKWARD;
                                break;
                            case ReedsSheppStateSpace::RS_STRAIGHT:
                                segment.type = forward ? ReedsSheppSegmentType::STRAIGHT_FORWARD
                                                       : ReedsSheppSegmentType::STRAIGHT_BACKWARD;
                                break;
                            case ReedsSheppStateSpace::RS_NOP:
                                break;
                            }
                            segment.length = fabs(rs_path.length_[i]);
                            segment.forward = forward;
                            path.segments.push_back(segment);
                        }

                        // Generate waypoints for this specific path
                        sampleSpecificPath(rs_path, q0, step_size, path.waypoints);

                        paths.push_back(path);
                    }
                }

                return paths;
            }

          private:
            ReedsSheppStateSpace rs_;

            inline void sampleSpecificPath(const ReedsSheppStateSpace::ReedsSheppPath &rs_path, double q0[3], double step_size,
                                    std::vector<Pose> &waypoints) const {
                waypoints.clear();

                double length = rs_path.length() * radius_;

                for (double seg = 0.0; seg <= length; seg += step_size) {
                    double qnew[3] = {};
                    rs_.interpolate(q0, const_cast<ReedsSheppStateSpace::ReedsSheppPath &>(rs_path), seg / radius_,
                                    qnew);

                    Pose pose;
                    pose.point.x = qnew[0];
                    pose.point.y = qnew[1];
                    pose.angle.yaw = qnew[2];
                    waypoints.push_back(pose);
                }
            }

            inline std::string getPathName(const ReedsSheppStateSpace::ReedsSheppPath &path) const {
                std::string name = "RS_";
                for (int i = 0; i < 5; ++i) {
                    if (path.type_[i] == ReedsSheppStateSpace::RS_NOP) break;

                    char seg = 'X';
                    bool backward = path.length_[i] < 0;

                    switch (path.type_[i]) {
                    case ReedsSheppStateSpace::RS_LEFT:
                        seg = backward ? 'l' : 'L';
                        break;
                    case ReedsSheppStateSpace::RS_RIGHT:
                        seg = backward ? 'r' : 'R';
                        break;
                    case ReedsSheppStateSpace::RS_STRAIGHT:
                        seg = backward ? 's' : 'S';
                        break;
                    case ReedsSheppStateSpace::RS_NOP:
                        seg = 'N';
                        break;
                    }
                    name += seg;
                }
                return name;
            }
        };

        // Static path types
        inline const ReedsSheppStateSpace::ReedsSheppPathSegmentType ReedsSheppStateSpace::reedsSheppPathType[18][5] = {
            {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},        // 0
            {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},       // 1
            {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},      // 2
            {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},      // 3
            {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},   // 4
            {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},  // 5
            {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 6
            {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},  // 7
            {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},  // 8
            {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},   // 9
            {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},  // 10
            {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 11
            {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 12
            {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},    // 13
            {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 14
            {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},   // 15
            {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT}, // 16
            {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}  // 17
        };

        // Complete ReedsSheppStateSpace implementation with all 18 path types
        inline double ReedsSheppStateSpace::distance(double q0[3], double q1[3]) const {
            return rho_ * reedsShepp(q0, q1).length();
        }

        inline std::vector<ReedsSheppStateSpace::ReedsSheppPathSegmentType>
        ReedsSheppStateSpace::type(double q0[3], double q1[3]) const {
            ReedsSheppPath path = reedsShepp(q0, q1);
            std::vector<ReedsSheppStateSpace::ReedsSheppPathSegmentType> type_list;
            for (int i = 0; i < 5; ++i) type_list.push_back(path.type_[i]);
            return type_list;
        }

        inline void ReedsSheppStateSpace::sample(double q0[3], double q1[3], double step_size, double &length,
                                                 std::vector<std::vector<double>> &points) const {
            ReedsSheppPath path = reedsShepp(q0, q1);
            length = rho_ * path.length();

            for (double seg = 0.0; seg <= length; seg += step_size) {
                double qnew[3] = {};
                interpolate(q0, const_cast<ReedsSheppPath &>(path), seg / rho_, qnew);
                std::vector<double> v(qnew, qnew + sizeof qnew / sizeof qnew[0]);
                points.push_back(v);
            }
            return;
        }

        inline ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::reedsShepp(double q0[3], double q1[3]) const {
            auto all_paths = getAllReedsSheppPaths(q0, q1);

            if (all_paths.empty()) {
                return ReedsSheppPath();
            }

            // Return the shortest path
            auto shortest = std::min_element(
                all_paths.begin(), all_paths.end(),
                [](const ReedsSheppPath &a, const ReedsSheppPath &b) { return a.length() < b.length(); });

            return *shortest;
        }

        inline std::vector<ReedsSheppStateSpace::ReedsSheppPath>
        ReedsSheppStateSpace::getAllReedsSheppPaths(double q0[3], double q1[3]) const {
            std::vector<ReedsSheppPath> paths;

            // Transform to local coordinates
            double dx = q1[0] - q0[0], dy = q1[1] - q0[1], dth = q1[2] - q0[2];
            double c = cos(q0[2]), s = sin(q0[2]);
            double x = c * dx + s * dy, y = -s * dx + c * dy;
            double phi = dth;

            // Scale by turning radius
            x /= rho_;
            y /= rho_;

            // Implementation of the full Reeds-Shepp algorithm
            const double pi = M_PI;
            const double twopi = 2. * pi;
            const double RS_EPS = 1e-6;
            const double ZERO = 10 * std::numeric_limits<double>::epsilon();

            auto mod2pi = [&](double x) {
                double v = fmod(x, twopi);
                if (v < -pi)
                    v += twopi;
                else if (v > pi)
                    v -= twopi;
                return v;
            };

            auto polar = [&](double x, double y, double &r, double &theta) {
                r = sqrt(x * x + y * y);
                theta = atan2(y, x);
            };

            auto tauOmega = [&](double u, double v, double xi, double eta, double phi, double &tau, double &omega) {
                double delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
                double t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
                tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
                omega = mod2pi(tau - u + v - phi);
            };

            // All the path generation functions
            auto LpSpLp = [&](double x, double y, double phi, double &t, double &u, double &v) {
                polar(x - sin(phi), y - 1. + cos(phi), u, t);
                if (t >= -ZERO) {
                    v = mod2pi(phi - t);
                    if (v >= -ZERO) {
                        return true;
                    }
                }
                return false;
            };

            auto LpSpRp = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double t1, u1;
                polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
                u1 = u1 * u1;
                if (u1 >= 4.) {
                    double theta;
                    u = sqrt(u1 - 4.);
                    theta = atan2(2., u);
                    t = mod2pi(t1 + theta);
                    v = mod2pi(t - phi);
                    return t >= -ZERO && v >= -ZERO;
                }
                return false;
            };

            auto LpRmL = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
                polar(xi, eta, u1, theta);
                if (u1 <= 4.) {
                    u = -2. * asin(.25 * u1);
                    t = mod2pi(theta + .5 * u + pi);
                    v = mod2pi(phi - t + u);
                    return t >= -ZERO && u <= ZERO;
                }
                return false;
            };

            auto LpRupLumRm = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
                if (rho <= 1.) {
                    u = acos(rho);
                    tauOmega(u, -u, xi, eta, phi, t, v);
                    return t >= -ZERO && v <= ZERO;
                }
                return false;
            };

            auto LpRumLumRp = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
                if (rho >= 0 && rho <= 1) {
                    u = -acos(rho);
                    if (u >= -.5 * pi) {
                        tauOmega(u, u, xi, eta, phi, t, v);
                        return t >= -ZERO && v >= -ZERO;
                    }
                }
                return false;
            };

            auto LpRmSmLm = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
                polar(xi, eta, rho, theta);
                if (rho >= 2.) {
                    double r = sqrt(rho * rho - 4.);
                    u = 2. - r;
                    t = mod2pi(theta + atan2(r, -2.));
                    v = mod2pi(phi - .5 * pi - t);
                    return t >= -ZERO && u <= ZERO && v <= ZERO;
                }
                return false;
            };

            auto LpRmSmRm = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
                polar(-eta, xi, rho, theta);
                if (rho >= 2.) {
                    t = theta;
                    u = 2. - rho;
                    v = mod2pi(t + .5 * pi - phi);
                    return t >= -ZERO && u <= ZERO && v <= ZERO;
                }
                return false;
            };

            auto LpRmSLmRp = [&](double x, double y, double phi, double &t, double &u, double &v) {
                double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
                polar(xi, eta, rho, theta);
                if (rho >= 2.) {
                    u = 4. - sqrt(rho * rho - 4.);
                    if (u <= ZERO) {
                        t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
                        v = mod2pi(t - phi);
                        return t >= -ZERO && v >= -ZERO;
                    }
                }
                return false;
            };

            // Now generate all possible paths
            double t, u, v, L;

            // CSC paths
            if (LpSpLp(x, y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[14], t, u, v));
            }
            if (LpSpLp(-x, y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[14], -t, -u, -v));
            }
            if (LpSpLp(x, -y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[15], t, u, v));
            }
            if (LpSpLp(-x, -y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[15], -t, -u, -v));
            }
            if (LpSpRp(x, y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[12], t, u, v));
            }
            if (LpSpRp(-x, y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[12], -t, -u, -v));
            }
            if (LpSpRp(x, -y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[13], t, u, v));
            }
            if (LpSpRp(-x, -y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[13], -t, -u, -v));
            }

            // CCC paths
            if (LpRmL(x, y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[0], t, u, v));
            }
            if (LpRmL(-x, y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[0], -t, -u, -v));
            }
            if (LpRmL(x, -y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[1], t, u, v));
            }
            if (LpRmL(-x, -y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[1], -t, -u, -v));
            }

            // Backwards CCC
            double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
            if (LpRmL(xb, yb, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[0], v, u, t));
            }
            if (LpRmL(-xb, yb, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[0], -v, -u, -t));
            }
            if (LpRmL(xb, -yb, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[1], v, u, t));
            }
            if (LpRmL(-xb, -yb, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[1], -v, -u, -t));
            }

            // CCCC paths
            if (LpRupLumRm(x, y, phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[2], t, u, -u, v));
            }
            if (LpRupLumRm(-x, y, -phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[2], -t, -u, u, -v));
            }
            if (LpRupLumRm(x, -y, -phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[3], t, u, -u, v));
            }
            if (LpRupLumRm(-x, -y, phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[3], -t, -u, u, -v));
            }

            if (LpRumLumRp(x, y, phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[2], t, u, u, v));
            }
            if (LpRumLumRp(-x, y, -phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[2], -t, -u, -u, -v));
            }
            if (LpRumLumRp(x, -y, -phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[3], t, u, u, v));
            }
            if (LpRumLumRp(-x, -y, phi, t, u, v)) {
                L = fabs(t) + 2. * fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[3], -t, -u, -u, -v));
            }

            // CCSC paths
            if (LpRmSmLm(x, y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[4], t, -.5 * pi, u, v));
            }
            if (LpRmSmLm(-x, y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[4], -t, .5 * pi, -u, -v));
            }
            if (LpRmSmLm(x, -y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[5], t, -.5 * pi, u, v));
            }
            if (LpRmSmLm(-x, -y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[5], -t, .5 * pi, -u, -v));
            }

            if (LpRmSmRm(x, y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[8], t, -.5 * pi, u, v));
            }
            if (LpRmSmRm(-x, y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[8], -t, .5 * pi, -u, -v));
            }
            if (LpRmSmRm(x, -y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[9], t, -.5 * pi, u, v));
            }
            if (LpRmSmRm(-x, -y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[9], -t, .5 * pi, -u, -v));
            }

            // Backwards CCSC
            if (LpRmSmLm(xb, yb, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[6], v, u, -.5 * pi, t));
            }
            if (LpRmSmLm(-xb, yb, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[6], -v, -u, .5 * pi, -t));
            }
            if (LpRmSmLm(xb, -yb, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[7], v, u, -.5 * pi, t));
            }
            if (LpRmSmLm(-xb, -yb, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[7], -v, -u, .5 * pi, -t));
            }

            if (LpRmSmRm(xb, yb, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[10], v, u, -.5 * pi, t));
            }
            if (LpRmSmRm(-xb, yb, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[10], -v, -u, .5 * pi, -t));
            }
            if (LpRmSmRm(xb, -yb, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[11], v, u, -.5 * pi, t));
            }
            if (LpRmSmRm(-xb, -yb, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[11], -v, -u, .5 * pi, -t));
            }

            // CCSCC paths
            if (LpRmSLmRp(x, y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[16], t, -.5 * pi, u, -.5 * pi, v));
            }
            if (LpRmSLmRp(-x, y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[16], -t, .5 * pi, -u, .5 * pi, -v));
            }
            if (LpRmSLmRp(x, -y, -phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[17], t, -.5 * pi, u, -.5 * pi, v));
            }
            if (LpRmSLmRp(-x, -y, phi, t, u, v)) {
                L = fabs(t) + fabs(u) + fabs(v);
                paths.push_back(ReedsSheppPath(reedsSheppPathType[17], -t, .5 * pi, -u, .5 * pi, -v));
            }

            return paths;
        }

        inline void ReedsSheppStateSpace::interpolate(double q0[3], ReedsSheppPath &path, double seg,
                                                      double s[3]) const {
            if (seg < 0.0) seg = 0.0;
            if (seg > path.length()) seg = path.length();

            double phi, v;

            s[0] = s[1] = 0.0;
            s[2] = q0[2];

            for (unsigned int i = 0; i < 5 && seg > 0; ++i) {
                if (path.length_[i] < 0) {
                    v = std::max(-seg, path.length_[i]);
                    seg += v;
                } else {
                    v = std::min(seg, path.length_[i]);
                    seg -= v;
                }
                phi = s[2];
                switch (path.type_[i]) {
                case RS_LEFT:
                    s[0] += (sin(phi + v) - sin(phi));
                    s[1] += (-cos(phi + v) + cos(phi));
                    s[2] = phi + v;
                    break;
                case RS_RIGHT:
                    s[0] += (-sin(phi - v) + sin(phi));
                    s[1] += (cos(phi - v) - cos(phi));
                    s[2] = phi - v;
                    break;
                case RS_STRAIGHT:
                    s[0] += (v * cos(phi));
                    s[1] += (v * sin(phi));
                    break;
                case RS_NOP:
                    break;
                }
            }

            s[0] = s[0] * rho_ + q0[0];
            s[1] = s[1] * rho_ + q0[1];
        }

    } // namespace turners
} // namespace navcon
