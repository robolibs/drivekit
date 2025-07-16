#include "navcon/navcon.hpp"
#include "navcon/turners/dubins.hpp"
#include "navcon/turners/reeds_shepp.hpp"
#include "navcon/utils/visualize.hpp"
#include <iostream>
#include <cstdlib>
#include <ctime>

int main() {
    std::cout << "Dubins and Reeds-Shepp Path Visualization Example" << std::endl;
    
#ifdef HAS_RERUN
    // Initialize visualization
    auto rec = navcon::visualize::init_recording("navcon_paths");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }
    
    // Clear previous visualization data (allows running multiple times without restarting rerun)
    rec->log("", rerun::Clear::RECURSIVE);
    
    // No grid or coordinate system - just the essential elements
    
    // Generate random start and end poses
    std::srand(std::time(nullptr));
    
    // Random start pose
    navcon::Pose start;
    start.point.x = 1.0 + (std::rand() / (double)RAND_MAX) * 2.0; // Random x between 1 and 3
    start.point.y = 1.0 + (std::rand() / (double)RAND_MAX) * 2.0; // Random y between 1 and 3
    start.angle.yaw = (std::rand() / (double)RAND_MAX) * 2.0 * M_PI - M_PI; // Random angle between -π and π
    
    // Random end pose further from start
    navcon::Pose end;
    end.point.x = start.point.x + (std::rand() / (double)RAND_MAX) * 6.0 - 3.0; // Within ±3 of start
    end.point.y = start.point.y + (std::rand() / (double)RAND_MAX) * 6.0 - 3.0; // Within ±3 of start  
    end.angle.yaw = (std::rand() / (double)RAND_MAX) * 2.0 * M_PI - M_PI; // Random angle between -π and π
    
    std::cout << "Start: (" << start.point.x << ", " << start.point.y << ") heading " << start.angle.yaw << " rad" << std::endl;
    std::cout << "End: (" << end.point.x << ", " << end.point.y << ") heading " << end.angle.yaw << " rad" << std::endl;
    
    // Visualize poses
    navcon::visualize::show_robot_pose(rec, start, "start", rerun::Color(0, 255, 0));  // Green start
    navcon::visualize::show_robot_pose(rec, end, "end", rerun::Color(255, 0, 0));      // Red end
    
    // Create Dubins planner and find all paths
    navcon::turners::Dubins dubins(0.5);
    auto all_dubins_paths = dubins.get_all_paths(start, end, 0.05);
    auto shortest_dubins = dubins.plan_path(start, end, 0.05);
    
    std::cout << "Dubins paths found: " << all_dubins_paths.size() << std::endl;
    std::cout << "Shortest Dubins path: " << shortest_dubins.name 
              << " (length: " << shortest_dubins.total_length << ")" << std::endl;
    
    // Debug: Print start and end of shortest Dubins path
    if (!shortest_dubins.waypoints.empty()) {
        auto& first = shortest_dubins.waypoints.front();
        auto& last = shortest_dubins.waypoints.back();
        std::cout << "Dubins start: (" << first.point.x << ", " << first.point.y << ", " << first.angle.yaw << ")" << std::endl;
        std::cout << "Dubins end: (" << last.point.x << ", " << last.point.y << ", " << last.angle.yaw << ")" << std::endl;
        std::cout << "Expected end: (" << end.point.x << ", " << end.point.y << ", " << end.angle.yaw << ")" << std::endl;
        
        // Check heading error
        double heading_error = std::abs(last.angle.yaw - end.angle.yaw);
        if (heading_error > M_PI) heading_error = 2*M_PI - heading_error; // Wrap around
        std::cout << "Heading error: " << heading_error << " radians (" << heading_error * 180.0 / M_PI << " degrees)" << std::endl;
    }
    
    // Show all Dubins paths in gray
    navcon::visualize::show_dubins_paths(rec, all_dubins_paths, "dubins_paths");
    
    // Show shortest Dubins path highlighted
    navcon::visualize::show_dubins_path(rec, shortest_dubins, "shortest_dubins", rerun::Color(255, 255, 0));
    
    // Create Reeds-Shepp planner and find paths
    navcon::turners::ReedsShepp reeds_shepp(0.5);
    auto all_rs_paths = reeds_shepp.get_all_paths(start, end, 0.05);
    auto shortest_rs = reeds_shepp.plan_path(start, end, 0.05);
    
    std::cout << "Reeds-Shepp paths found: " << all_rs_paths.size() << std::endl;
    std::cout << "Shortest Reeds-Shepp path: " << shortest_rs.name 
              << " (length: " << shortest_rs.total_length << ")" << std::endl;
    
    // Debug: Print start and end of shortest Reeds-Shepp path
    if (!shortest_rs.waypoints.empty()) {
        auto& first_rs = shortest_rs.waypoints.front();
        auto& last_rs = shortest_rs.waypoints.back();
        std::cout << "RS start: (" << first_rs.point.x << ", " << first_rs.point.y << ", " << first_rs.angle.yaw << ")" << std::endl;
        std::cout << "RS end: (" << last_rs.point.x << ", " << last_rs.point.y << ", " << last_rs.angle.yaw << ")" << std::endl;
        std::cout << "RS expected end: (" << end.point.x << ", " << end.point.y << ", " << end.angle.yaw << ")" << std::endl;
        
        // Check heading error
        double rs_heading_error = std::abs(last_rs.angle.yaw - end.angle.yaw);
        if (rs_heading_error > M_PI) rs_heading_error = 2*M_PI - rs_heading_error; // Wrap around
        std::cout << "RS heading error: " << rs_heading_error << " radians (" << rs_heading_error * 180.0 / M_PI << " degrees)" << std::endl;
    }
    
    // Show all Reeds-Shepp paths in gray
    navcon::visualize::show_reeds_shepp_paths(rec, all_rs_paths, "reeds_shepp_paths");
    
    // Show shortest Reeds-Shepp path highlighted  
    navcon::visualize::show_reeds_shepp_path(rec, shortest_rs, "shortest_reeds_shepp", rerun::Color(255, 0, 255));
    
    std::cout << "Check the Rerun viewer at http://localhost:9876" << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();
    
#else
    std::cout << "Visualization not enabled. Rebuild with -DNAVCON_ENABLE_VISUALIZATION=ON" << std::endl;
#endif
    
    return 0;
}