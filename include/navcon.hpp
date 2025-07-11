#pragma once

// NavCon - Navigation Controller Library
// A simulator-agnostic, zero-dependency navigation controller framework

// Core interfaces
#include "navcon/types.hpp"
#include "navcon/controller.hpp"
#include "navcon/robot_config.hpp"
#include "navcon/wheel_controller.hpp"

// Kinematic models
#include "navcon/kinematics/differential_drive.hpp"
#include "navcon/kinematics/ackermann.hpp"
#include "navcon/kinematics/multi_axle.hpp"

// Control algorithms
#include "navcon/controllers/pid.hpp"
#include "navcon/controllers/pure_pursuit.hpp"
#include "navcon/controllers/stanley.hpp"
#include "navcon/controllers/carrot.hpp"

// Output adapters
#include "navcon/adapters/velocity_command.hpp"
#include "navcon/adapters/normalized_command.hpp"

// Utilities
#include "navcon/utils/math.hpp"
#include "navcon/factory.hpp"