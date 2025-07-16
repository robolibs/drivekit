
<img align="right" width="26%" src="./misc/logo.png">

# Navcon

A C++ library for waypoint navigation controller for heterogeneous robots with differential drives, holonomic robots, and multi-axle robots.

## Features

- **Path Following**: Stanley, Pure Pursuit, Carrot, and PID controllers
- **Path Planning**: Dubins paths and Reeds-Shepp curves for optimal vehicle maneuvers
- **Visualization**: Optional Rerun integration for real-time path visualization
- **Header-Only**: Easy integration without separate compilation

## Usage

This is a header-only library. Simply include the headers you need:

```cpp
#include "navcon/navcon.hpp"
#include "navcon/turners/dubins.hpp"
#include "navcon/turners/reeds_shepp.hpp"
#include "navcon/utils/visualize.hpp"  // Only if visualization is enabled
```

## Building

### Basic Build (No Visualization)
```bash
mkdir build && cd build
cmake ..
make
```

### With Examples and Visualization
```bash
make compile  # Automatically enables visualization with examples
make build
make run
```

Or manually:
```bash
mkdir build && cd build
cmake -DNAVCON_BUILD_EXAMPLES=ON ..
make
```

## Visualization

The library includes optional visualization using [Rerun](https://rerun.io/). Visualization is automatically enabled when building examples.

When enabled, you can:
- Visualize Dubins and Reeds-Shepp paths
- See robot poses and trajectories
- Display coordinate systems and grids
- Monitor controller status in real-time

To enable visualization:
1. Build with examples: `make compile` or `cmake -DNAVCON_BUILD_EXAMPLES=ON`
2. Use the `navcon::visualize` namespace in your code
3. View results in the Rerun viewer at http://localhost:9090

## Path Planning

### Dubins Paths
Forward-only paths with 6 types: LSL, RSR, LSR, RSL, RLR, LRL

```cpp
navcon::turners::Dubins dubins(min_turning_radius);
auto path = dubins.plan_path(start_pose, end_pose);
```

### Reeds-Shepp Curves
Paths with reverse motion capability for shorter maneuvers

```cpp
navcon::turners::ReedsShepp reeds_shepp(min_turning_radius);
auto path = reeds_shepp.plan_path(start_pose, end_pose);
```
