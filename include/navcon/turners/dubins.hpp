#pragma once

#include "navcon/types.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace navcon {
    namespace turners {

        enum class DubinsSegmentType { LEFT = 'L', STRAIGHT = 'S', RIGHT = 'R' };

        struct DubinsSegment {
            DubinsSegmentType type;
            double length;
        };

        struct DubinsPath {
            std::vector<DubinsSegment> segments;
            std::vector<Pose> waypoints;
            double total_length;
            std::string name;

            inline DubinsPath() : total_length(0.0) {}
        };

        class DubinsStateSpace {
          public:
            enum DubinsPathSegmentType { DUBINS_LEFT = 0, DUBINS_STRAIGHT = 1, DUBINS_RIGHT = 2 };

            static const DubinsPathSegmentType dubinsPathType[6][3];

            class DubinsPath {
              public:
                DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
                           double p = std::numeric_limits<double>::max(), double q = 0.)
                    : type_(type) {
                    length_[0] = t;
                    length_[1] = p;
                    length_[2] = q;
                    assert(t >= 0.);
                    assert(p >= 0.);
                    assert(q >= 0.);
                }

                inline double length() const { return length_[0] + length_[1] + length_[2]; }

                const DubinsPathSegmentType *type_;
                double length_[3];
            };

            inline DubinsStateSpace(double turningRadius = 1.0) : rho_(turningRadius) {}

            inline void sample(double q0[3], double q1[3], double step_size, double &length,
                        std::vector<std::vector<double>> &points) const;
            inline double distance(double q0[3], double q1[3]);
            inline DubinsPath dubins(double q0[3], double q1[3]) const;

          public:
            inline void interpolate(double q0[3], DubinsPath &path, double seg, double s[3]) const;

          protected:
            double rho_;
        };

        class Dubins {
          public:
            inline Dubins(double min_turning_radius) : radius_(min_turning_radius), ds_(min_turning_radius) {}

            inline DubinsPath plan_path(const Pose &start, const Pose &end, double step_size = 0.1) const {
                auto all_paths = get_all_paths(start, end, step_size);

                if (all_paths.empty()) {
                    return DubinsPath();
                }

                auto shortest =
                    std::min_element(all_paths.begin(), all_paths.end(), [](const DubinsPath &a, const DubinsPath &b) {
                        return a.total_length < b.total_length;
                    });

                return *shortest;
            }

            inline std::vector<DubinsPath> get_all_paths(const Pose &start, const Pose &end,
                                                         double step_size = 0.1) const {
                std::vector<DubinsPath> paths;

                // Convert to DubinsStateSpace format
                double q0[3] = {start.point.x, start.point.y, start.angle.yaw};
                double q1[3] = {end.point.x, end.point.y, end.angle.yaw};

                // Generate all 6 possible Dubins paths
                std::vector<std::function<DubinsStateSpace::DubinsPath(double, double, double)>> planners = {
                    [this](double d, double alpha, double beta) { return dubinsLSL(d, alpha, beta); },
                    [this](double d, double alpha, double beta) { return dubinsRSR(d, alpha, beta); },
                    [this](double d, double alpha, double beta) { return dubinsRSL(d, alpha, beta); },
                    [this](double d, double alpha, double beta) { return dubinsLSR(d, alpha, beta); },
                    [this](double d, double alpha, double beta) { return dubinsRLR(d, alpha, beta); },
                    [this](double d, double alpha, double beta) { return dubinsLRL(d, alpha, beta); }};

                std::vector<std::string> names = {"LSL", "RSR", "RSL", "LSR", "RLR", "LRL"};

                double x1 = q0[0], y1 = q0[1], th1 = q0[2];
                double x2 = q1[0], y2 = q1[1], th2 = q1[2];
                double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / radius_, th = atan2(dy, dx);
                double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);

                for (size_t i = 0; i < planners.size(); ++i) {
                    auto dubins_path = planners[i](d, alpha, beta);

                    if (dubins_path.length() < std::numeric_limits<double>::max()) {
                        DubinsPath path;
                        path.name = names[i];
                        path.total_length = dubins_path.length() * radius_;

                        // Convert segments
                        for (int j = 0; j < 3; ++j) {
                            DubinsSegment segment;
                            switch (dubins_path.type_[j]) {
                            case DubinsStateSpace::DUBINS_LEFT:
                                segment.type = DubinsSegmentType::LEFT;
                                break;
                            case DubinsStateSpace::DUBINS_RIGHT:
                                segment.type = DubinsSegmentType::RIGHT;
                                break;
                            case DubinsStateSpace::DUBINS_STRAIGHT:
                                segment.type = DubinsSegmentType::STRAIGHT;
                                break;
                            }
                            segment.length = dubins_path.length_[j];
                            path.segments.push_back(segment);
                        }

                        // Generate waypoints for this specific path
                        double total_length = dubins_path.length() * radius_;
                        for (double seg = 0.0; seg <= total_length; seg += step_size) {
                            double qnew[3] = {0.0, 0.0, 0.0};
                            ds_.interpolate(q0, dubins_path, seg / radius_, qnew);

                            Pose pose;
                            pose.point.x = qnew[0];
                            pose.point.y = qnew[1];
                            pose.angle.yaw = qnew[2];
                            path.waypoints.push_back(pose);
                        }

                        paths.push_back(path);
                    }
                }

                return paths;
            }

          protected:
            double radius_;
            DubinsStateSpace ds_;

            static const double twopi;
            static const double DUBINS_EPS;
            static const double DUBINS_ZERO;

            inline double mod2pi(double x) const {
                if (x < 0 && x > DUBINS_ZERO) return 0;
                double xm = x - twopi * floor(x / twopi);
                if (twopi - xm < .5 * DUBINS_EPS) xm = 0.;
                return xm;
            }

            inline DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta) const {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
                if (tmp >= DUBINS_ZERO) {
                    double theta = atan2(cb - ca, d + sa - sb);
                    double t = mod2pi(-alpha + theta);
                    double p = sqrt(std::max(tmp, 0.));
                    double q = mod2pi(beta - theta);
                    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[0], t, p, q);
                }
                return DubinsStateSpace::DubinsPath();
            }

            inline DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta) const {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
                if (tmp >= DUBINS_ZERO) {
                    double theta = atan2(ca - cb, d - sa + sb);
                    double t = mod2pi(alpha - theta);
                    double p = sqrt(std::max(tmp, 0.));
                    double q = mod2pi(-beta + theta);
                    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[1], t, p, q);
                }
                return DubinsStateSpace::DubinsPath();
            }

            inline DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta) const {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
                if (tmp >= DUBINS_ZERO) {
                    double p = sqrt(std::max(tmp, 0.));
                    double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
                    double t = mod2pi(alpha - theta);
                    double q = mod2pi(beta - theta);
                    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[2], t, p, q);
                }
                return DubinsStateSpace::DubinsPath();
            }

            inline DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta) const {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
                if (tmp >= DUBINS_ZERO) {
                    double p = sqrt(std::max(tmp, 0.));
                    double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
                    double t = mod2pi(-alpha + theta);
                    double q = mod2pi(-beta + theta);
                    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[3], t, p, q);
                }
                return DubinsStateSpace::DubinsPath();
            }

            inline DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta) const {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
                if (fabs(tmp) < 1.) {
                    double p = twopi - acos(tmp);
                    double theta = atan2(ca - cb, d - sa + sb);
                    double t = mod2pi(alpha - theta + .5 * p);
                    double q = mod2pi(alpha - beta - t + p);
                    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[4], t, p, q);
                }
                return DubinsStateSpace::DubinsPath();
            }

            inline DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta) const {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
                if (fabs(tmp) < 1.) {
                    double p = twopi - acos(tmp);
                    double theta = atan2(-ca + cb, d + sa - sb);
                    double t = mod2pi(-alpha + theta + .5 * p);
                    double q = mod2pi(beta - alpha - t + p);
                    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubinsPathType[5], t, p, q);
                }
                return DubinsStateSpace::DubinsPath();
            }
        };

        // Static constants
        inline const double Dubins::twopi = 2. * M_PI;
        inline const double Dubins::DUBINS_EPS = 1e-6;
        inline const double Dubins::DUBINS_ZERO = -1e-7;

        // Static path types
        inline const DubinsStateSpace::DubinsPathSegmentType DubinsStateSpace::dubinsPathType[6][3] = {
            {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},  {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
            {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT}, {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
            {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},    {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}};

        // DubinsStateSpace implementation
        inline double DubinsStateSpace::distance(double q0[3], double q1[3]) { return rho_ * dubins(q0, q1).length(); }

        inline DubinsStateSpace::DubinsPath DubinsStateSpace::dubins(double q0[3], double q1[3]) const {
            double x1 = q0[0], y1 = q0[1], th1 = q0[2];
            double x2 = q1[0], y2 = q1[1], th2 = q1[2];
            double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
            double alpha = fmod(th1 - th, 2 * M_PI), beta = fmod(th2 - th, 2 * M_PI);

            if (alpha < 0) alpha += 2 * M_PI;
            if (beta < 0) beta += 2 * M_PI;

            // Use the working implementation
            const double twopi = 2. * M_PI;
            const double DUBINS_EPS = 1e-6;
            const double DUBINS_ZERO = -1e-7;

            auto mod2pi = [&](double x) {
                if (x < 0 && x > DUBINS_ZERO) return 0.0;
                double xm = x - twopi * floor(x / twopi);
                if (twopi - xm < .5 * DUBINS_EPS) xm = 0.;
                return xm;
            };

            auto dubinsLSL = [&](double d, double alpha, double beta) {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
                if (tmp >= DUBINS_ZERO) {
                    double theta = atan2(cb - ca, d + sa - sb);
                    double t = mod2pi(-alpha + theta);
                    double p = sqrt(std::max(tmp, 0.));
                    double q = mod2pi(beta - theta);
                    return DubinsPath(dubinsPathType[0], t, p, q);
                }
                return DubinsPath();
            };

            auto dubinsRSR = [&](double d, double alpha, double beta) {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
                if (tmp >= DUBINS_ZERO) {
                    double theta = atan2(ca - cb, d - sa + sb);
                    double t = mod2pi(alpha - theta);
                    double p = sqrt(std::max(tmp, 0.));
                    double q = mod2pi(-beta + theta);
                    return DubinsPath(dubinsPathType[1], t, p, q);
                }
                return DubinsPath();
            };

            auto dubinsRSL = [&](double d, double alpha, double beta) {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
                if (tmp >= DUBINS_ZERO) {
                    double p = sqrt(std::max(tmp, 0.));
                    double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
                    double t = mod2pi(alpha - theta);
                    double q = mod2pi(beta - theta);
                    return DubinsPath(dubinsPathType[2], t, p, q);
                }
                return DubinsPath();
            };

            auto dubinsLSR = [&](double d, double alpha, double beta) {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
                if (tmp >= DUBINS_ZERO) {
                    double p = sqrt(std::max(tmp, 0.));
                    double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
                    double t = mod2pi(-alpha + theta);
                    double q = mod2pi(-beta + theta);
                    return DubinsPath(dubinsPathType[3], t, p, q);
                }
                return DubinsPath();
            };

            auto dubinsRLR = [&](double d, double alpha, double beta) {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
                if (fabs(tmp) < 1.) {
                    double p = twopi - acos(tmp);
                    double theta = atan2(ca - cb, d - sa + sb);
                    double t = mod2pi(alpha - theta + .5 * p);
                    double q = mod2pi(alpha - beta - t + p);
                    return DubinsPath(dubinsPathType[4], t, p, q);
                }
                return DubinsPath();
            };

            auto dubinsLRL = [&](double d, double alpha, double beta) {
                double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
                double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
                if (fabs(tmp) < 1.) {
                    double p = twopi - acos(tmp);
                    double theta = atan2(-ca + cb, d + sa - sb);
                    double t = mod2pi(-alpha + theta + .5 * p);
                    double q = mod2pi(beta - alpha - t + p);
                    return DubinsPath(dubinsPathType[5], t, p, q);
                }
                return DubinsPath();
            };

            if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS) return DubinsPath(dubinsPathType[0], 0, d, 0);

            DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
            double len, minLength = path.length();

            if ((len = tmp.length()) < minLength) {
                minLength = len;
                path = tmp;
            }
            tmp = dubinsRSL(d, alpha, beta);
            if ((len = tmp.length()) < minLength) {
                minLength = len;
                path = tmp;
            }
            tmp = dubinsLSR(d, alpha, beta);
            if ((len = tmp.length()) < minLength) {
                minLength = len;
                path = tmp;
            }
            tmp = dubinsRLR(d, alpha, beta);
            if ((len = tmp.length()) < minLength) {
                minLength = len;
                path = tmp;
            }
            tmp = dubinsLRL(d, alpha, beta);
            if ((len = tmp.length()) < minLength) path = tmp;
            return path;
        }

        inline void DubinsStateSpace::interpolate(double q0[3], DubinsPath &path, double seg, double s[3]) const {
            if (seg < 0.0) seg = 0.0;
            if (seg > path.length()) seg = path.length();

            double phi, v;

            s[0] = s[1] = 0.0;
            s[2] = q0[2];

            for (unsigned int i = 0; i < 3 && seg > 0; ++i) {
                v = std::min(seg, path.length_[i]);
                seg -= v;
                phi = s[2];
                switch (path.type_[i]) {
                case DUBINS_LEFT:
                    s[0] += (sin(phi + v) - sin(phi));
                    s[1] += (-cos(phi + v) + cos(phi));
                    s[2] = phi + v;
                    break;
                case DUBINS_RIGHT:
                    s[0] += (-sin(phi - v) + sin(phi));
                    s[1] += (cos(phi - v) - cos(phi));
                    s[2] = phi - v;
                    break;
                case DUBINS_STRAIGHT:
                    s[0] += (v * cos(phi));
                    s[1] += (v * sin(phi));
                    break;
                }
            }

            s[0] = s[0] * rho_ + q0[0];
            s[1] = s[1] * rho_ + q0[1];
        }

        inline void DubinsStateSpace::sample(double q0[3], double q1[3], double step_size, double &length,
                                             std::vector<std::vector<double>> &points) const {
            DubinsPath path = dubins(q0, q1);
            length = rho_ * path.length();

            for (double seg = 0.0; seg <= length; seg += step_size) {
                double qnew[3] = {};
                interpolate(q0, path, seg / rho_, qnew);
                std::vector<double> v(qnew, qnew + sizeof qnew / sizeof qnew[0]);
                points.push_back(v);
            }
            return;
        }

    } // namespace turners
} // namespace navcon
