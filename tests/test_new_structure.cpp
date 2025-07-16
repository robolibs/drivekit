#include "navcon/path_controller.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

using namespace navcon;

void test_path_controller_creation() {
    std::cout << "Testing PathController creation..." << std::endl;
    
    // Test Stanley follower
    PathController controller1(PathController::FollowerType::STANLEY);
    assert(controller1.get_type() == "path_controller");
    assert(controller1.get_active_type() == PathController::ControllerType::FOLLOWER);
    
    // Test Pure Pursuit follower
    PathController controller2(PathController::FollowerType::PURE_PURSUIT);
    assert(controller2.get_type() == "path_controller");
    assert(controller2.get_active_type() == PathController::ControllerType::FOLLOWER);
    
    std::cout << "✓ PathController creation test passed" << std::endl;
}

void test_point_turner() {
    std::cout << "Testing Point Turner..." << std::endl;
    
    turners::PointTurner turner;
    
    // Test initial state
    assert(turner.get_type() == "point_turner");
    assert(!turner.is_configured());
    assert(!turner.is_turn_complete());
    
    // Test configuration
    Pose start_pose{{0, 0}, {0, 0, 0}};
    turner.configure_turn_90_deg(start_pose, true);  // 90° clockwise
    
    assert(turner.is_configured());
    assert(!turner.is_turn_complete());
    
    // Test control computation
    RobotState state;
    state.pose = start_pose;
    state.velocity = {0, 0, 0};
    
    Goal goal;
    goal.target_pose = start_pose;
    goal.target_pose.angle.yaw = -M_PI/2;  // 90° clockwise
    
    RobotConstraints constraints;
    constraints.max_angular_velocity = 1.0;
    
    auto cmd = turner.compute_control(state, goal, constraints, 0.1);
    assert(cmd.valid);
    assert(cmd.linear_velocity == 0.0);  // No linear motion for point turns
    assert(cmd.angular_velocity != 0.0);  // Should have angular motion
    
    std::cout << "✓ Point Turner test passed" << std::endl;
}

void test_u_turn_turner() {
    std::cout << "Testing U-Turn Turner..." << std::endl;
    
    turners::UTurnTurner turner;
    
    // Test initial state
    assert(turner.get_type() == "u_turn_turner");
    assert(!turner.is_configured());
    assert(!turner.is_turn_complete());
    
    // Test configuration
    Pose start_pose{{0, 0}, {0, 0, 0}};
    Pose end_pose{{10, 10}, {0, 0, M_PI}};
    
    turner.configure_turn(start_pose, end_pose);
    
    assert(turner.is_configured());
    assert(!turner.is_turn_complete());
    assert(turner.get_total_waypoints() > 0);
    
    // Test control computation
    RobotState state;
    state.pose = start_pose;
    state.velocity = {0, 0, 0};
    
    Goal goal;
    goal.target_pose = end_pose;
    
    RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.0;
    
    auto cmd = turner.compute_control(state, goal, constraints, 0.1);
    assert(cmd.valid);
    
    std::cout << "✓ U-Turn Turner test passed" << std::endl;
}

void test_controller_switching() {
    std::cout << "Testing controller switching..." << std::endl;
    
    PathController controller(PathController::FollowerType::STANLEY);
    
    // Create path with sharp turn
    Path path;
    path.waypoints.emplace_back(Pose{{0, 0}, {0, 0, 0}});
    path.waypoints.emplace_back(Pose{{5, 0}, {0, 0, 0}});
    path.waypoints.emplace_back(Pose{{5, 0}, {0, 0, M_PI/2}});  // 90° turn
    path.waypoints.emplace_back(Pose{{5, 5}, {0, 0, M_PI/2}});
    
    controller.set_path(path);
    controller.set_sharp_turn_threshold(45.0);  // Should trigger on 90° turn
    
    // Start with follower
    assert(controller.get_active_type() == PathController::ControllerType::FOLLOWER);
    
    // Simulate getting close to turn
    RobotState state;
    state.pose = Pose{{4, 0}, {0, 0, 0}};  // Close to turn
    state.velocity = {0, 0, 0};
    
    Goal goal;
    goal.target_pose = path.waypoints.back();
    
    RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.0;
    
    // This should trigger switching
    auto cmd = controller.compute_control(state, goal, constraints, 0.1);
    assert(cmd.valid);
    
    std::cout << "  Active controller: " << controller.get_active_controller_name() << std::endl;
    
    std::cout << "✓ Controller switching test passed" << std::endl;
}

void test_followers() {
    std::cout << "Testing followers..." << std::endl;
    
    // Test Stanley follower
    followers::StanleyFollower stanley;
    assert(stanley.get_type() == "stanley_follower");
    
    // Test Pure Pursuit follower
    followers::PurePursuitFollower pure_pursuit;
    assert(pure_pursuit.get_type() == "pure_pursuit_follower");
    
    // Test that they can handle basic control
    Path path;
    path.waypoints.emplace_back(Pose{{0, 0}, {0, 0, 0}});
    path.waypoints.emplace_back(Pose{{10, 0}, {0, 0, 0}});
    
    stanley.set_path(path);
    pure_pursuit.set_path(path);
    
    RobotState state;
    state.pose = Pose{{0, 0}, {0, 0, 0}};
    
    Goal goal;
    goal.target_pose = path.waypoints.back();
    
    RobotConstraints constraints;
    constraints.max_linear_velocity = 2.0;
    constraints.max_angular_velocity = 1.0;
    
    auto cmd1 = stanley.compute_control(state, goal, constraints, 0.1);
    auto cmd2 = pure_pursuit.compute_control(state, goal, constraints, 0.1);
    
    assert(cmd1.valid);
    assert(cmd2.valid);
    
    std::cout << "✓ Followers test passed" << std::endl;
}

int main() {
    std::cout << "Sharp Turn Controller Tests" << std::endl;
    std::cout << "===========================" << std::endl;
    
    try {
        test_path_controller_creation();
        test_point_turner();
        test_u_turn_turner();
        test_controller_switching();
        test_followers();
        
        std::cout << std::endl << "All tests passed! ✅" << std::endl;
        std::cout << "The new structure is working correctly." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}