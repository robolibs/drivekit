# Sharp Turn Controller Implementation

## Overview

The Sharp Turn Controller system has been implemented with a clean, simplified architecture that separates path followers from turn maneuvers. The main `PathController` automatically switches between followers and turners based on path analysis.

## Architecture

### Directory Structure
```
include/navcon/
├── controller.hpp              # Base controller interface
├── types.hpp                   # Data structures
├── path_controller.hpp         # Main orchestrator
├── followers/                  # Path following controllers
│   ├── stanley.hpp            # Stanley follower
│   ├── pure_pursuit.hpp       # Pure pursuit follower
│   ├── carrot.hpp             # Carrot follower
│   └── pid.hpp                # PID follower
└── turners/                    # Turn maneuver controllers
    ├── point_turner.hpp       # Same position, different heading
    └── u_turn_turner.hpp      # Position-to-position turns
```

### Core Components

#### 1. PathController (Main Orchestrator)
- **Purpose**: Automatically switches between followers and turners
- **Key Features**:
  - Analyzes path ahead for sharp turns
  - Configurable turn thresholds (45°, 120°, etc.)
  - Seamless switching between controller types
  - Lookahead distance configuration

#### 2. Followers (Path Following)
Located in `followers/` directory:
- **StanleyFollower**: Stanley path tracking algorithm
- **PurePursuitFollower**: Pure pursuit path following
- **CarrotFollower**: Simple carrot chasing
- **PIDFollower**: Basic PID control

#### 3. Turners (Turn Maneuvers)
Located in `turners/` directory:

##### Point Turner
- **Purpose**: Same position, different heading (street corners)
- **Use Cases**: 90°, 180° turns at intersections
- **Features**:
  - In-place rotation
  - Configurable target heading
  - Quick setup methods (90°, 180°)

##### U-Turn Turner
- **Purpose**: Position-to-position turns (headlands)
- **Use Cases**: Agricultural headlands, parking maneuvers
- **Features**:
  - Multiple turn patterns (bulb, omega, square)
  - Configurable headland width
  - Smooth transition between positions

## Usage

### Basic Usage
```cpp
#include "navcon/path_controller.hpp"

// Create controller with Stanley follower
PathController controller(PathController::FollowerType::STANLEY);

// Configure turn thresholds
controller.set_sharp_turn_threshold(45.0);   // Point turn threshold
controller.set_u_turn_threshold(120.0);      // U-turn threshold
controller.set_lookahead_distance(3.0);      // Lookahead distance

// Set path and run control loop
controller.set_path(path);
auto cmd = controller.compute_control(state, goal, constraints, dt);
```

### Controller Types

#### PathController::FollowerType
- `STANLEY`: Stanley path tracking
- `PURE_PURSUIT`: Pure pursuit following

#### Turn Detection
- **Sharp Turn Threshold**: Angle above which point turner activates
- **U-Turn Threshold**: Angle above which U-turn turner activates
- **Lookahead Distance**: How far ahead to analyze path

### Turn Types

#### Point Turner (Street Corners)
```cpp
// Direct usage for 90° turn
auto point_turner = std::make_unique<turners::PointTurner>();
point_turner->configure_turn_90_deg(current_pose, true);  // clockwise
```

#### U-Turn Turner (Headlands)
```cpp
// Direct usage for headland turn
auto u_turner = std::make_unique<turners::UTurnTurner>();
u_turner->configure_headland_turn(start_pose, end_pose, 5.0);  // 5m width
```

## Configuration

### Turn Thresholds
- **Sharp Turn**: 45° (default) - triggers point turner
- **U-Turn**: 120° (default) - triggers U-turn turner
- **Lookahead**: 5.0m (default) - distance to analyze ahead

### Turn Patterns (U-Turn)
- **BULB**: Simple circular arc
- **OMEGA**: Teardrop shape
- **SQUARE**: Rectangular pattern

## Example

See `examples/simple_sharp_turn_example.cpp` for a complete demonstration showing:
1. Path following with Stanley controller
2. Automatic switching to point turner for 90° turns
3. Automatic switching to U-turn turner for headland turns
4. Seamless return to path following

## Benefits

1. **Separation of Concerns**: Followers handle path tracking, turners handle maneuvers
2. **Automatic Switching**: No manual intervention required
3. **Configurable**: Adjustable thresholds and parameters
4. **Extensible**: Easy to add new followers or turners
5. **Clean Architecture**: Simple, understandable code structure

## Key Features

- **Unified Interface**: All controllers inherit from base `Controller`
- **Automatic Detection**: Path analysis determines when to switch
- **Two Turner Types**: Point (same position) vs U-turn (position change)
- **Seamless Handoff**: Smooth transitions between controllers
- **Configurable Thresholds**: Adjustable angle and distance parameters