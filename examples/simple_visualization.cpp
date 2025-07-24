#include "navcon/navcon.hpp"
#include "navcon/turners/dubins.hpp"
#include "navcon/turners/reeds_shepp.hpp"
#include "navcon/utils/visualize.hpp"
#include <cstdlib>
#include <ctime>
#include <iostream>

int main() {

#ifdef HAS_RERUN
    // Initialize visualization
    auto rec = std::make_shared<rerun::RecordingStream>("name", "space");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    // Clear previous visualization data (allows running multiple times without restarting rerun)
    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    // No grid or coordinate system - just the essential elements

    // Generate random start and end poses
    std::srand(std::time(nullptr));

    // Random start pose
    navcon::Pose start;
    start.point.x = 1.0 + (std::rand() / (double)RAND_MAX) * 2.0;           // Random x between 1 and 3
    start.point.y = 1.0 + (std::rand() / (double)RAND_MAX) * 2.0;           // Random y between 1 and 3
    start.angle.yaw = (std::rand() / (double)RAND_MAX) * 2.0 * M_PI - M_PI; // Random angle between -π and π

    // Random end pose further from start
    navcon::Pose end = start;
    end.point.x = start.point.x + (std::rand() / (double)RAND_MAX) * 6.0 - 3.0; // Within ±3 of start
    end.point.y = start.point.y + (std::rand() / (double)RAND_MAX) * 6.0 - 3.0; // Within ±3 of start
    end.angle.yaw = (std::rand() / (double)RAND_MAX) * 2.0 * M_PI - M_PI; // Random angle between -π and π

    // Visualize poses
    navcon::visualize::show_robot_pose(rec, start, "start", rerun::Color(0, 255, 0)); // Green start
    navcon::visualize::show_robot_pose(rec, end, "end", rerun::Color(255, 0, 0));     // Red end

    // Create Dubins planner and find all paths
    navcon::turners::Dubins dubins(0.2);
    auto all_dubins_paths = dubins.get_all_paths(start, end, 0.05);
    auto shortest_dubins = dubins.plan_path(start, end, 0.05);

    // Show all Dubins paths in gray
    navcon::visualize::show_dubins_paths(rec, all_dubins_paths, "dubins_paths");

    // Show shortest Dubins path highlighted
    navcon::visualize::show_dubins_path(rec, shortest_dubins, "shortest_dubins", rerun::Color(255, 255, 0));

    // Create Reeds-Shepp planner and find paths
    navcon::turners::ReedsShepp reeds_shepp(0.2);
    auto all_rs_paths = reeds_shepp.get_all_paths(start, end, 0.05);
    auto shortest_rs = reeds_shepp.plan_path(start, end, 0.05);

    // Show all Reeds-Shepp paths in gray
    navcon::visualize::show_reeds_shepp_paths(rec, all_rs_paths, "reeds_shepp_paths");

    // Show shortest Reeds-Shepp path highlighted
    navcon::visualize::show_reeds_shepp_path(rec, shortest_rs, "shortest_reeds_shepp", rerun::Color(255, 0, 255));

#else
#endif

    return 0;
}
