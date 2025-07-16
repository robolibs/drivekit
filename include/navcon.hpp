#pragma once

// NavCon - Navigation Controller Library
// A simulator-agnostic, zero-dependency navigation controller framework

// Core interfaces
#include "navcon/types.hpp"
#include "navcon/controller.hpp"

// Control algorithms
#include "navcon/controllers/pid.hpp"
#include "navcon/controllers/pure_pursuit.hpp"
#include "navcon/controllers/stanley.hpp"
#include "navcon/controllers/carrot.hpp"

// Advanced controllers
#include "navcon/path_controller.hpp"

// High-level interface
#include "navcon/navcon.hpp"

// Utilities
#include "navcon/factory.hpp"