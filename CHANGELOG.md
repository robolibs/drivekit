# Changelog

## [0.0.6] - 2026-01-18

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Migrate build system to using PROJECT file
- Update dependency versions

## [0.0.3] - 2026-01-03

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Update datapod and optinum dependency versions

## [0.0.2] - 2026-01-01

### <!-- 0 -->⛰️  Features

- Simplify core and prediction controller implementations

### <!-- 2 -->🚜 Refactor

- Standardize dependencies and build configuration

## [0.2.1] - 2025-12-15

### <!-- 1 -->🐛 Bug Fixes

- Revert project version and correct Git tag typo

### <!-- 7 -->⚙️ Miscellaneous Tasks

- `compat: Update concord to 2.5.0`

## [0.2.1] - 2025-12-15

### <!-- 7 -->⚙️ Miscellaneous Tasks

- `compat: Update concord to 2.5.0`

## [0.2.0] - 2025-12-15

### <!-- 0 -->⛰️  Features

- Refactor motion planning for improved path following
- Implement and integrate the MCA controller with risk-based scaling
- Feat: Enable turn-first behavior for differential drive robots
- Refactor SOCFollower for improved obstacle avoidance and control
- Introduce Monte Carlo Approximation (MCA) controller
- Refactor turn-first logic with hysteresis for MPC/MPPI controllers
- Implement turn-first velocity suppression in MPC and MPPI
- Implement in-place rotation for differential robots
- Add support for custom build paths
- Rename project from navcon to waypoint
- Support differential drive steering types
- Introduce Stochastic Optimal Control (SOC) follower
- MPC and LQR to remove external solver dependencies
- Add MPPI controller and refactor existing trackers
- Implement articulated vehicle MPC controller
- Implement a barebone fuzzy logic
- Introduce other predictive control algorithms
- Refine MPC for enhanced path tracking and stability
- Implement Ackermann steering with rate limits
- Refactor and enhance path-based controller configurations
- Feat: Add LQR and MPC controllers for path tracking
- Add Model Predictive Control (MPC) for path tracking
- Implement LQR path following controller
- Feat: Add dynamic world constraints to control system
- Introduce planning and tracking modules
- Allow reverse maneuvers in tight turns
- Refactor controllers and introduce example suite
- Refactor controllers and integrate Stanley for path tracking
- Add pure pursuit controller and S-path example

### <!-- 2 -->🚜 Refactor

- Migrate from spdlog to iostream for logging
- Rename project from Waypoint to DriveKit
- Rename project from `navcon` to `waypoint`
- Refactor: Remove LQR and MPC conditional compilation and dependencies
- Isolate and generalize control and tracking components
- Refactor pure pursuit and Stanley controller for improved control
- Refactor to move `allow_reverse` to `RobotState`
- Refactor Pure Pursuit controller for enhanced accuracy
- Refactor navigation to planning and tracking subsystems
- Rename `controller` and `visualization` to `tracking`
- Rework recording stream initialization and Rerun integration
- Extract class method implementations to source files
- Refactor pure pursuit for improved lookahead calculation

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor build system to use modern CMake and FetchContent
- Remove unused build flags and dependencies

### Build

- Improve xmake project detection and cleanup
- Migrate build system to xmake in addition to cmake
- Integrate Rerun SDK conditionally and fix build issues

## [0.1.6] - 2025-11-01

### <!-- 2 -->🚜 Refactor

- Refactor: Remove `waypoint::factory` and update includes

## [0.1.5] - 2025-11-01

### <!-- 0 -->⛰️  Features

- Refactor `waypoint` for modular control and visualization
- Rename project, add logging, and new example

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor config targets to use local dependencies

## [0.1.5] - 2025-11-01

### <!-- 0 -->⛰️  Features

- Refactor `waypoint` for modular control and visualization
- Rename project, add logging, and new example

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Refactor config targets to use local dependencies

## [0.1.3] - 2025-10-25

### <!-- 2 -->🚜 Refactor

- Remove unused path planners and simplify visualizations

## [0.1.2] - 2025-10-23

### <!-- 2 -->🚜 Refactor

- Make static constants and path types inline

## [0.1.1] - 2025-10-22

### <!-- 2 -->🚜 Refactor

- Add `inline` specifiers to functions

## [0.1.0] - 2025-10-15

### <!-- 0 -->⛰️  Features

- Introduce the "Sharper" agricultural sharp-turn planner
- Refactor example to skip prompt and use public interpolation
- Adopt Rerun-based rebuild of GUI and path planning
- Replace classic steering with Reeds-Shepp maneuvers
- Insert densified waypoints into NavCon paths
- Unify autonomous controllers under modular Navcon framework
- Refactor Stanley controller for improved path tracking
- Refactor motion control for improved differential drive maneuvers
- Init

### <!-- 1 -->🐛 Bug Fixes

- Reset visualization state on every program run

### <!-- 2 -->🚜 Refactor

- Modernize build system and export style settings
- Streamline C++ demo into silent gRPC visualization
- Migrate rerun calls to new static-logging API
- Refactor controllers to directly produce velocity commands

### <!-- 3 -->📚 Documentation

- Polish acknowledgements typography
- Add third-party attribution document

### <!-- 5 -->🎨 Styling

- Factor: Add `inline` specifier to appropriate functions

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Purge obsolete docs and binary asset


