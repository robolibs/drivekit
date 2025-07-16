#include "navcon/navcon.hpp"
#include "navcon/turners/dubins.hpp"
#include "navcon/turners/reeds_shepp.hpp"
#include "navcon/utils/visualize.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "NavCon Dubins and Reeds-Shepp Path Planning Example" << std::endl;
    
#ifdef HAS_RERUN
    // Initialize visualization
    auto rec = navcon::visualize::init_recording("navcon_demo");
    navcon::visualize::show_coordinate_system(rec);
    navcon::visualize::show_grid(rec);
    
    // Define start and end poses
    navcon::Pose start_pose;
    start_pose.point = {0.0, 0.0};
    start_pose.angle.yaw = 0.0;
    
    navcon::Pose end_pose;
    end_pose.point = {5.0, 3.0};
    end_pose.angle.yaw = M_PI / 2;
    
    // Show start and end poses
    navcon::visualize::show_robot_pose(rec, start_pose, "start_pose", {255, 0, 0});
    navcon::visualize::show_robot_pose(rec, end_pose, "end_pose", {0, 255, 0});
    
    // Create Dubins path planner
    double min_turning_radius = 1.0;
    navcon::turners::Dubins dubins(min_turning_radius);
    
    std::cout << "Computing Dubins paths..." << std::endl;
    auto dubins_paths = dubins.get_all_paths(start_pose, end_pose, 0.1);
    
    std::cout << "Found " << dubins_paths.size() << " Dubins paths:" << std::endl;
    for (const auto& path : dubins_paths) {
        std::cout << "  " << path.name << ": length = " << path.total_length << std::endl;
    }
    
    // Visualize all Dubins paths
    navcon::visualize::show_dubins_paths(rec, dubins_paths, "dubins_paths");
    
    // Create Reeds-Shepp path planner
    navcon::turners::ReedsShepp reeds_shepp(min_turning_radius);
    
    std::cout << "\nComputing Reeds-Shepp paths..." << std::endl;
    auto rs_paths = reeds_shepp.get_all_paths(start_pose, end_pose, 0.1);
    
    std::cout << "Found " << rs_paths.size() << " Reeds-Shepp paths:" << std::endl;
    for (const auto& path : rs_paths) {
        std::cout << "  " << path.name << ": length = " << path.total_length << std::endl;
    }
    
    // Visualize all Reeds-Shepp paths
    navcon::visualize::show_reeds_shepp_paths(rec, rs_paths, "reeds_shepp_paths");
    
    // Find and highlight the shortest paths
    if (!dubins_paths.empty()) {
        auto shortest_dubins = dubins.plan_path(start_pose, end_pose, 0.1);
        std::cout << "\nShortest Dubins path: " << shortest_dubins.name 
                  << " (length: " << shortest_dubins.total_length << ")" << std::endl;
        navcon::visualize::show_dubins_path(rec, shortest_dubins, "shortest_dubins", {255, 255, 0});
    }
    
    if (!rs_paths.empty()) {
        auto shortest_rs = reeds_shepp.plan_path(start_pose, end_pose, 0.1);
        std::cout << "Shortest Reeds-Shepp path: " << shortest_rs.name 
                  << " (length: " << shortest_rs.total_length << ")" << std::endl;
        navcon::visualize::show_reeds_shepp_path(rec, shortest_rs, "shortest_reeds_shepp", {255, 0, 255});
    }
    
    std::cout << "\nVisualization is running. Check the Rerun viewer!" << std::endl;
    std::cout << "Press Ctrl+C to exit." << std::endl;
    
    // Keep the program running so visualization stays active
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
#else
    std::cout << "Visualization not enabled. Rebuild with -DNAVCON_ENABLE_VISUALIZATION=ON to see paths." << std::endl;
    
    // Still demonstrate the path planning without visualization
    navcon::Pose start_pose;
    start_pose.point = {0.0, 0.0};
    start_pose.angle.yaw = 0.0;
    
    navcon::Pose end_pose;
    end_pose.point = {5.0, 3.0};
    end_pose.angle.yaw = M_PI / 2;
    
    double min_turning_radius = 1.0;
    navcon::turners::Dubins dubins(min_turning_radius);
    navcon::turners::ReedsShepp reeds_shepp(min_turning_radius);
    
    auto dubins_paths = dubins.get_all_paths(start_pose, end_pose, 0.1);
    auto rs_paths = reeds_shepp.get_all_paths(start_pose, end_pose, 0.1);
    
    std::cout << "Found " << dubins_paths.size() << " Dubins paths and " 
              << rs_paths.size() << " Reeds-Shepp paths." << std::endl;
    
    if (!dubins_paths.empty()) {
        auto shortest_dubins = dubins.plan_path(start_pose, end_pose, 0.1);
        std::cout << "Shortest Dubins path: " << shortest_dubins.name 
                  << " (length: " << shortest_dubins.total_length << ")" << std::endl;
    }
    
    if (!rs_paths.empty()) {
        auto shortest_rs = reeds_shepp.plan_path(start_pose, end_pose, 0.1);
        std::cout << "Shortest Reeds-Shepp path: " << shortest_rs.name 
                  << " (length: " << shortest_rs.total_length << ")" << std::endl;
    }
#endif
    
    return 0;
}
