#pragma once

// NavCon - Navigation Controller Library
// A simulator-agnostic, zero-dependency navigation controller framework

// Core interfaces
#include "navcon/controller.hpp"
#include "navcon/types.hpp"

// Control algorithms
#include "navcon/followers/carrot.hpp"
#include "navcon/followers/pid.hpp"
#include "navcon/followers/pure_pursuit.hpp"

// Advanced controllers
#include "navcon/path_controller.hpp"

// High-level interface
#include "navcon/navcon.hpp"

// Utilities
#include "navcon/factory.hpp"
