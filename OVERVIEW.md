# NavCon - Navigation Controller Library

## Executive Summary

NavCon is a **simulator-agnostic, zero-dependency C++20 navigation controller library** designed for autonomous robots with diverse kinematic models. It provides a clean abstraction layer between high-level navigation algorithms and low-level robot control, supporting differential drive, Ackermann steering, and multi-axle robots.

## Architecture Philosophy

### Core Design Principles

1. **Simulator Agnostic**: Works with any robotics framework (ROS, Gazebo, flatsim, real hardware)
2. **Zero Dependencies**: Only depends on standard C++ libraries and the lightweight Concord geometry library
3. **Template-Based Flexibility**: Supports multiple input states and output command types
4. **Modular Design**: Separate kinematic models, control algorithms, and output adapters
5. **Header-Only Implementation**: Easy integration without complex build dependencies

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           APPLICATION LAYER                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │     ROS     │  │   Flatsim   │  │   Gazebo    │  │  Real Robot │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           NAVCON INTERFACE                                  │
│                                                                             │
│    ┌─────────────────────────────────────────────────────────────────────┐ │
│    │                       ROBOT CONFIGURATION                           │ │
│    │  ┌─────────────────────────────────────────────────────────────────┐│ │
│    │  │ struct RobotConfig {                                           ││ │
│    │  │     std::vector<Wheel> wheels;  // Physical wheel setup        ││ │
│    │  │     Size dimensions;           // Robot size                   ││ │
│    │  │     // Auto-computed properties:                               ││ │
│    │  │     double wheelbase;          // Front-rear distance          ││ │
│    │  │     double track_width;        // Left-right distance          ││ │
│    │  │     DriveType detect_drive_type();                             ││ │
│    │  │ };                                                             ││ │
│    │  │                                                                ││ │
│    │  │ struct Wheel {                                                 ││ │
│    │  │     Point position;            // Relative to robot center     ││ │
│    │  │     double max_steer_angle;    // Steering capability          ││ │
│    │  │     double max_throttle;       // Power capability             ││ │
│    │  │     double steer_differential; // Differential steering        ││ │
│    │  │     double throttle_differential;                              ││ │
│    │  │ };                                                             ││ │
│    │  └─────────────────────────────────────────────────────────────────┘│ │
│    └─────────────────────────────────────────────────────────────────────┘ │
│                                    │                                        │
│                                    ▼                                        │
│    ┌─────────────────────────────────────────────────────────────────────┐ │
│    │                    CONTROLLER FACTORY                               │ │
│    │  ┌─────────────────────────────────────────────────────────────────┐│ │
│    │  │              Template Controller Base                           ││ │
│    │  │  Controller<InputState, OutputCommand>                          ││ │
│    │  │  ┌─────────────────────────────────────────────────────────────┐││ │
│    │  │  │ virtual OutputCommand compute_control(                      │││ │
│    │  │  │     const InputState& current_state,                        │││ │
│    │  │  │     const Goal& goal,                                       │││ │
│    │  │  │     const RobotConstraints& constraints,                    │││ │
│    │  │  │     double dt)                                              │││ │
│    │  │  └─────────────────────────────────────────────────────────────┘││ │
│    │  └─────────────────────────────────────────────────────────────────┘│ │
│    └─────────────────────────────────────────────────────────────────────┘ │
│                                    │                                        │
│                                    ▼                                        │
│    ┌─────────────────────────────────────────────────────────────────────┐ │
│    │                      WHEEL CONTROLLER                               │ │
│    │  ┌─────────────────────────────────────────────────────────────────┐│ │
│    │  │ WheelController(const RobotConfig& config)                     ││ │
│    │  │                                                                ││ │
│    │  │ WheelCommands velocity_to_wheels(VelocityCommand)              ││ │
│    │  │ WheelCommands normalized_to_wheels(NormalizedCommand)          ││ │
│    │  │                                                                ││ │
│    │  │ // Output: Individual wheel steering & throttle                ││ │
│    │  │ struct WheelCommand {                                          ││ │
│    │  │     double steering_angle;                                     ││ │
│    │  │     double throttle;                                           ││ │
│    │  │ };                                                             ││ │
│    │  └─────────────────────────────────────────────────────────────────┘│ │
│    └─────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        CONTROL ALGORITHMS LAYER                            │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │     PID     │  │Pure Pursuit │  │   Stanley   │  │   Carrot    │        │
│  │ Controller  │  │ Controller  │  │ Controller  │  │ Controller  │        │
│  │             │  │             │  │             │  │             │        │
│  │ • Linear    │  │ • Geometric │  │ • Cross-    │  │ • Simple    │        │
│  │   Control   │  │   Path      │  │   Track     │  │   Point     │        │
│  │ • Angular   │  │   Following │  │   Error     │  │   Following │        │
│  │   Control   │  │ • Lookahead │  │ • Heading   │  │ • Propor-   │        │
│  │ • Velocity  │  │   Distance  │  │   Error     │  │   tional    │        │
│  │   Based     │  │ • Curvature │  │ • Ackermann │  │   Control   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        KINEMATIC MODELS LAYER                              │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │Differential │  │  Ackermann  │  │ Multi-Axle  │  │  Holonomic  │        │
│  │    Drive    │  │   Steering  │  │  Steering   │  │   (Future)  │        │
│  │             │  │             │  │             │  │             │        │
│  │ • Can       │  │ • Fixed     │  │ • Front +   │  │ • Omni-     │        │
│  │   Rotate    │  │   Wheelbase │  │   Rear      │  │   directional│        │
│  │   in Place  │  │ • Min Turn  │  │   Steering  │  │ • X/Y/Theta │        │
│  │ • Left/Right│  │   Radius    │  │ • Complex   │  │   Control   │        │
│  │   Wheels    │  │ • Speed +   │  │   Kinematics│  │ • Decoupled │        │
│  │ • Simple    │  │   Steering  │  │ • Multiple  │  │   Motion    │        │
│  │   Kinematics│  │   Angle     │  │   Modes     │  │             │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                         OUTPUT ADAPTERS LAYER                               │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  Velocity   │  │  Ackermann  │  │Differential │  │ Normalized  │         │
│  │  Command    │  │   Command   │  │   Command   │  │   Command   │         │
│  │             │  │             │  │             │  │             │         │
│  │ • Linear    │  │ • Speed     │  │ • Left      │  │ • Throttle  │         │
│  │   Velocity  │  │   (m/s)     │  │   Speed     │  │   (-1 to 1) │         │
│  │ • Angular   │  │ • Steering  │  │ • Right     │  │ • Steering  │         │
│  │   Velocity  │  │   Angle     │  │   Speed     │  │   (-1 to 1) │         │
│  │ • Lateral   │  │   (rad)     │  │ • Wheel     │  │ • Gamepad   │         │
│  │   (Holonomic│  │             │  │   Based     │  │   Style     │         │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Data Flow Architecture

### Input Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              INPUT SOURCES                                  │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            STATE AGGREGATION                                │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ Robot State │  │    Goal     │  │ Constraints │  │ Time Delta  │         │
│  │             │  │             │  │             │  │             │         │
│  │ • Pose      │  │ • Target    │  │ • Kinematic │  │ • dt        │         │
│  │   - Pose    │  │   Pose      │  │   Limits    │  │ • Update    │         │
│  │   - Euler   │  │  • Path     │  │ • Dynamic   │  │   Rate      │         │
│  │ • Velocity  │  │   Waypoints │  │   Limits    │  │ • Timing    │         │
│  │   - Linear  │  │ • Tolerance │  │ • Physical  │  │   Info      │         │
│  │   - Angular │  │   Limits    │  │   Dimensions│  │             │         │
│  │   - Lateral │  │ • Speed     │  │ • Safety    │  │             │         │
│  │ • Timestamp │  │   Profile   │  │   Margins   │  │             │         │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Control Processing Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           CONTROL COMPUTATION                               │
│                                                                             │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                          GOAL ANALYSIS                                 │ │
│  │  ┌────────────────────────────────────────────────────────────────────┐│ │
│  │  │ • Distance to Goal     • Heading Error                             ││ │
│  │  │ • Path Availability    • Cross-Track Error                         ││ │
│  │  │ • Goal Tolerance       • Lookahead Distance                        ││ │
│  │  │ • Completion Status    • Waypoint Management                       ││ │
│  │  └────────────────────────────────────────────────────────────────────┘│ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                        │
│                                    ▼                                        │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                      ALGORITHM SELECTION                               │ │
│  │                                                                        │ │
│  │  PID Controller:           Pure Pursuit:        Stanley Controller:    │ │
│  │  ┌─────────────────┐      ┌─────────────────┐   ┌─────────────────┐    │ │
│  │  │ • Linear Error  │      │ • Lookahead     │   │ • Cross-Track   │    │ │
│  │  │ • Angular Error │      │   Point         │   │   Error         │    │ │
│  │  │ • Proportional  │      │ • Curvature     │   │ • Heading Error │    │ │
│  │  │ • Integral      │      │   Calculation   │   │ • Stanley Law   │    │ │
│  │  │ • Derivative    │      │ • Path Segment  │   │ • Speed Adapt   │    │ │
│  │  └─────────────────┘      └─────────────────┘   └─────────────────┘    │ │
│  └────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                        │
│                                    ▼                                        │
│  ┌────────────────────────────────────────────────────────────────────────┐ │
│  │                        CONSTRAINT CHECKING                             │ │
│  │  ┌────────────────────────────────────────────────────────────────────┐│ │
│  │  │ • Velocity Limits      • Acceleration Limits                       ││ │
│  │  │ • Steering Limits      • Turning Radius                            ││ │
│  │  │ • Physical Constraints • Safety Margins                            ││ │
│  │  │ • Kinematic Feasibility• Dynamic Stability                         ││ │
│  │  └────────────────────────────────────────────────────────────────────┘│ │
│  └────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Output Generation Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          OUTPUT GENERATION                                 │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                      KINEMATIC TRANSFORMATION                          │ │
│  │                                                                         │ │
│  │  Control Signal → Kinematic Model → Platform-Specific Command           │ │
│  │                                                                         │ │
│  │  ┌─────────────────┐      ┌─────────────────┐   ┌─────────────────┐   │ │
│  │  │ Linear Velocity │      │ Differential:   │   │ Velocity Cmd    │   │ │
│  │  │ Angular Velocity│  →   │ • Left/Right    │ → │ Ackermann Cmd   │   │ │
│  │  │ Lateral Velocity│      │   Wheel Speed   │   │ Differential Cmd│   │ │
│  │  │ (if applicable) │      │ Ackermann:      │   │ Normalized Cmd  │   │ │
│  │  └─────────────────┘      │ • Speed +       │   │ Multi-Axle Cmd  │   │ │
│  │                           │   Steering      │   └─────────────────┘   │ │
│  │                           │ Multi-Axle:     │                         │ │
│  │                           │ • Front/Rear    │                         │ │
│  │                           │   Steering      │                         │ │
│  │                           └─────────────────┘                         │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    │                                       │
│                                    ▼                                       │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                        STATUS REPORTING                                │ │
│  │  ┌─────────────────────────────────────────────────────────────────────┐│ │
│  │  │ • Command Validity     • Distance to Goal                          ││ │
│  │  │ • Status Message       • Cross-Track Error                         ││ │
│  │  │ • Control Mode         • Heading Error                             ││ │
│  │  │ • Goal Reached Flag    • Controller Type                           ││ │
│  │  └─────────────────────────────────────────────────────────────────────┘│ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Controller Algorithms

### 1. PID Controller
**Use Case**: Basic point-to-point navigation with direct control
**Characteristics**:
- Simple proportional-integral-derivative control
- Separate gains for linear and angular control
- Good for stable, predictable environments
- Minimal computational overhead

**Algorithm Flow**:
```
Current State → Goal Analysis → Error Calculation → PID Computation → Command Output
     ↓              ↓               ↓                    ↓                ↓
  Position      Distance        Linear Error      P+I+D Terms      Velocity Command
  Velocity      Heading         Angular Error     Clamping         Status Update
  Timestamp     Tolerance       Time Delta        Saturation       
```

### 2. Pure Pursuit Controller
**Use Case**: Smooth path following with predictive lookahead
**Characteristics**:
- Geometric path following algorithm
- Lookahead distance determines smoothness
- Excellent for curved paths
- Widely used in autonomous vehicles

**Algorithm Flow**:
```
Current State → Path Search → Lookahead Point → Curvature Calc → Kinematic Transform
     ↓              ↓             ↓                ↓                    ↓
  Position      Find Closest   Interpolate      Pure Pursuit        Command Output
  Heading       Waypoint      Future Point      Formula             (Speed + Steering)
  Path Data     Path Index    Lookahead Dist    Local Transform     Status Update
```

### 3. Stanley Controller
**Use Case**: Path following with cross-track error correction
**Characteristics**:
- Combines heading and cross-track error
- Excellent for lane-keeping and precise tracking
- Speed-adaptive control gains
- Used in autonomous driving systems

**Algorithm Flow**:
```
Current State → Path Analysis → Error Calculation → Stanley Formula → Command Output
     ↓              ↓               ↓                   ↓                ↓
  Position      Nearest Point   Heading Error      Control Law       Steering Angle
  Heading       Path Tangent    Cross-Track Err    Speed Adaptation  Speed Command
  Velocity      Normal Vector   Distance Error     Gain Scheduling   Status Update
```

### 4. Carrot Controller
**Use Case**: Simple goal-chasing with basic obstacle avoidance
**Characteristics**:
- Simplest algorithm for basic navigation
- Direct point-to-point control
- Good for open environments
- Minimal configuration required

**Algorithm Flow**:
```
Current State → Goal Vector → Proportional Control → Speed Adaptation → Command Output
     ↓              ↓               ↓                     ↓                ↓
  Position      Distance        Linear Control        Turn Reduction     Velocity Command
  Heading       Bearing         Angular Control       Distance Scaling   Status Update
  Goal          Error           P-only Gains          Safety Limits      
```

## Kinematic Models

### Differential Drive Kinematics

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        DIFFERENTIAL DRIVE MODEL                            │
│                                                                             │
│              Left Wheel                    Right Wheel                      │
│                  │                            │                            │
│                  ▼                            ▼                            │
│            ┌─────────┐                  ┌─────────┐                        │
│            │    ●    │                  │    ●    │                        │
│            └─────────┘                  └─────────┘                        │
│                  │                            │                            │
│                  └──────────┬─────────────────┘                            │
│                            │                                               │
│                     Track Width (W)                                        │
│                                                                             │
│  Forward Kinematics:                                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │ Linear Velocity  = (V_left + V_right) / 2                              │ │
│  │ Angular Velocity = (V_right - V_left) / W                              │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
│  Inverse Kinematics:                                                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │ V_left  = Linear - (Angular * W/2)                                     │ │
│  │ V_right = Linear + (Angular * W/2)                                     │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
│  Characteristics:                                                           │
│  • Can rotate in place (zero turning radius)                               │
│  • Simple kinematics and control                                           │
│  • Good for indoor navigation                                              │
│  • Limited by wheel slip at high speeds                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Ackermann Steering Kinematics

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ACKERMANN STEERING MODEL                           │
│                                                                             │
│           Front Axle                    Rear Axle                           │
│              │                            │                                │
│              ▼                            ▼                                │
│        ┌─────────┐                  ┌─────────┐                            │
│        │    ●    │ ← Steering       │    ●    │                            │
│        └─────────┘                  └─────────┘                            │
│              │                            │                                │
│              └──────────┬─────────────────┘                                │
│                        │                                                   │
│                 Wheelbase (L)                                              │
│                                                                             │
│  Kinematic Relationships:                                                   │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │ Angular Velocity = (Speed / Wheelbase) * tan(Steering Angle)           │ │
│  │ Turning Radius   = Wheelbase / tan(Steering Angle)                     │ │
│  │ Steering Angle   = atan((Angular Velocity * Wheelbase) / Speed)        │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
│  Instantaneous Center of Rotation (ICR):                                    │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │      │                                                                 │ │
│  │      ▼                                                                 │ │
│  │     ICR ●                                                              │ │
│  │      │   \                                                             │ │
│  │      │    \                                                            │ │
│  │      │     \                                                           │ │
│  │      R      \                                                          │ │
│  │      │       \                                                         │ │
│  │      │        \ Vehicle                                                │ │
│  │      │         ┌─────────┐                                             │ │
│  │      │         │    ●    │                                             │ │
│  │      │         └─────────┘                                             │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
│  Characteristics:                                                           │
│  • Cannot rotate in place (minimum turning radius)                         │
│  • Speed affects stability (higher speed = larger turning radius)          │
│  • Realistic car-like behavior                                             │
│  • Complex path planning required                                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Multi-Axle Steering Model

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       MULTI-AXLE STEERING MODEL                            │
│                                                                             │
│     Front Axle              Middle Axle(s)             Rear Axle           │
│         │                        │                        │                │
│         ▼                        ▼                        ▼                │
│   ┌─────────┐               ┌─────────┐               ┌─────────┐           │
│   │    ●    │ ← Front       │    ●    │               │    ●    │ ← Rear    │
│   └─────────┘   Steering    └─────────┘               └─────────┘   Steering│
│         │                        │                        │                │
│         └──────────┬─────────────┬─────────────┬─────────┘                │
│                    │             │             │                          │
│                Front L1        Middle L2     Rear L3                      │
│                                                                             │
│  Steering Modes:                                                            │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │ 1. Coordinated Steering (Minimum Turning Radius)                       │ │
│  │    • All wheels follow same ICR                                        │ │
│  │    • Front and rear steering angles coordinated                        │ │
│  │    • Tightest possible turns                                           │ │
│  │                                                                         │ │
│  │ 2. Counter Steering (High Speed Stability)                             │ │
│  │    • Rear wheels steer opposite to front                               │ │
│  │    • Reduces side forces at high speeds                                │ │
│  │    • Improved stability and comfort                                    │ │
│  │                                                                         │ │
│  │ 3. Crab Steering (Lateral Movement)                                    │ │
│  │    • All wheels point in same direction                                │ │
│  │    • Vehicle moves sideways                                            │ │
│  │    • Useful for parking and tight spaces                              │ │
│  │                                                                         │ │
│  │ 4. Front-Only Steering (Conventional)                                  │ │
│  │    • Only front axle steers                                            │ │
│  │    • Rear axles follow passively                                       │ │
│  │    • Simplest control mode                                             │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
│  Characteristics:                                                           │
│  • Complex kinematics requiring coordination                                │
│  • Multiple operating modes for different scenarios                        │
│  • Excellent maneuverability when properly controlled                      │
│  • Requires sophisticated control algorithms                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Type System & Data Structures

### Core Data Types

```cpp
// Robot State Information
struct RobotState {
    Pose pose;              // Position and orientation
    Velocity velocity;      // Current motion state
    double timestamp;       // Time reference
};

// Goal Specification
struct Goal {
    Pose target_pose;                    // Where to go
    std::optional<Velocity> target_velocity;  // How fast to arrive
    double tolerance_position;           // Acceptable error
    double tolerance_orientation;        // Acceptable heading error
};

// Path Following
struct Path {
    std::vector<Pose> waypoints;         // Sequence of poses
    std::vector<double> speeds;          // Speed at each waypoint
    bool is_closed;                      // Loop back to start
};

// Physical Constraints
struct RobotConstraints {
    // Kinematic Parameters
    double wheelbase;                    // Distance between axles
    double track_width;                  // Distance between wheels
    double wheel_radius;                 // Wheel radius
    
    // Dynamic Limits
    double max_linear_velocity;          // Maximum forward speed
    double min_linear_velocity;          // Minimum/reverse speed
    double max_angular_velocity;         // Maximum turning rate
    double max_linear_acceleration;      // Acceleration limits
    double max_angular_acceleration;     // Angular acceleration limits
    
    // Steering Limits (Ackermann)
    double max_steering_angle;           // Maximum steering angle
    double max_steering_rate;            // Steering rate limit
    double min_turning_radius;           // Minimum turning radius
    
    // Multi-axle Specific
    double rear_wheelbase;               // Distance to rear axle
    double max_rear_steering_angle;      // Rear steering limits
};
```

### Output Command Types

```cpp
// Base Command Interface
struct ControlOutput {
    bool valid;                          // Command validity
    std::string status_message;          // Status description
    OutputType type;                     // Command type identifier
};

// Velocity Command (Universal)
struct VelocityCommand : public ControlOutput {
    double linear_velocity;              // Forward/backward speed
    double angular_velocity;             // Turning rate
    double lateral_velocity;             // Side-to-side (holonomic)
};

// Ackermann Command (Car-like)
struct AckermannCommand : public ControlOutput {
    double speed;                        // Vehicle speed
    double steering_angle;               // Steering wheel angle
};

// Differential Command (Tank-like)
struct DifferentialCommand : public ControlOutput {
    double left_speed;                   // Left wheel speed
    double right_speed;                  // Right wheel speed
};

// Normalized Command (Gamepad-like)
struct NormalizedCommand : public ControlOutput {
    double throttle;                     // -1 (reverse) to 1 (forward)
    double steering;                     // -1 (left) to 1 (right)
};
```

## Factory Pattern & Extensibility

### Controller Factory System

```cpp
// Template-based Factory
template<typename InputState = RobotState, typename OutputCommand = VelocityCommand>
class ControllerFactory {
public:
    using ControllerType = Controller<InputState, OutputCommand>;
    using ControllerPtr = std::unique_ptr<ControllerType>;
    using CreatorFunc = std::function<ControllerPtr(const ControllerConfig&)>;
    
    // Register new controller types
    void register_controller(const std::string& name, CreatorFunc creator);
    
    // Create controller instance
    ControllerPtr create(const std::string& name, const ControllerConfig& config = {});
    
    // Query available types
    std::vector<std::string> get_available_types() const;
    bool has_controller(const std::string& name) const;
    
private:
    std::unordered_map<std::string, CreatorFunc> creators_;
};

// Convenience Factory Functions
std::unique_ptr<VelocityController> create_velocity_controller(
    const std::string& type, const ControllerConfig& config = {});
    
std::unique_ptr<AckermannController> create_ackermann_controller(
    const std::string& type, const ControllerConfig& config = {});
```

### Extension Points

```cpp
// Custom Controller Implementation
class CustomController : public Controller<RobotState, VelocityCommand> {
public:
    VelocityCommand compute_control(
        const RobotState& current_state,
        const Goal& goal,
        const RobotConstraints& constraints,
        double dt
    ) override {
        // Custom control logic here
        VelocityCommand cmd;
        // ... implementation
        return cmd;
    }
    
    std::string get_type() const override { return "custom"; }
};

// Registration with Factory
ControllerFactory<RobotState, VelocityCommand> factory;
factory.register_controller("custom", [](const ControllerConfig& config) {
    return std::make_unique<CustomController>();
});
```

## Integration Examples

### ROS Integration

```cpp
// ROS Node Example
class NavConROS {
private:
    navcon::RobotConfig robot_config_;
    std::unique_ptr<navcon::VelocityController> controller_;
    navcon::WheelController wheel_controller_;
    
public:
    NavConROS() : wheel_controller_(robot_config_) {
        // Build robot config from ROS parameters
        robot_config_ = build_config_from_ros_params();
        
        // Create controller based on robot type
        std::string controller_type = "pure_pursuit";
        if (robot_config_.detect_drive_type() == navcon::RobotConfig::DriveType::DIFFERENTIAL) {
            controller_type = "pid";
        }
        
        controller_ = navcon::create_velocity_controller(controller_type);
        
        // Configure
        navcon::ControllerConfig config;
        config.lookahead_distance = 2.0;
        controller_->set_config(config);
    }
    
    navcon::RobotConfig build_config_from_ros_params() {
        navcon::RobotConfigBuilder builder;
        
        // ROS node fills the struct - navcon doesn't know about ROS params
        builder.with_name("turtlebot3");
        builder.with_dimensions(0.3, 0.3);
        
        // Differential drive robot - ROS provides these values
        builder.add_wheel({
            .name = "left_wheel",
            .position = navcon::Point(-0.15, 0, 0),
            .size = navcon::Size(0.066, 0.066, 0),
            .max_steer_angle = 0.0,  // No steering
            .max_throttle = 1.0
        });
        
        builder.add_wheel({
            .name = "right_wheel", 
            .position = navcon::Point(0.15, 0, 0),
            .size = navcon::Size(0.066, 0.066, 0),
            .max_steer_angle = 0.0,  // No steering
            .max_throttle = 1.0
        });
        
        return builder.build();
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Convert ROS message to NavCon types
        navcon::RobotState state;
        state.pose.point.x = msg->pose.pose.position.x;
        state.pose.point.y = msg->pose.pose.position.y;
        // ... more conversions
        
        // Compute control
        auto cmd = controller_->compute_control(
            state, goal_, robot_config_.to_constraints(), dt
        );
        
        // Option 1: Publish as Twist
        geometry_msgs::Twist twist;
        twist.linear.x = cmd.linear_velocity;
        twist.angular.z = cmd.angular_velocity;
        cmd_pub_.publish(twist);
        
        // Option 2: Publish individual wheel commands
        auto wheel_cmds = wheel_controller_.velocity_to_wheels(cmd);
        publish_wheel_commands(wheel_cmds);
    }
};
```

### Gazebo Integration

```cpp
// Gazebo Plugin Example
class NavConGazebo : public gazebo::ModelPlugin {
private:
    navcon::RobotConfig robot_config_;
    std::unique_ptr<navcon::AckermannController> controller_;
    navcon::WheelController wheel_controller_;
    std::map<std::string, gazebo::physics::JointPtr> wheel_joints_;
    
public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
        // Build robot config from SDF model
        robot_config_ = build_config_from_sdf(model, sdf);
        wheel_controller_ = navcon::WheelController(robot_config_);
        
        // Initialize controller based on detected type
        if (robot_config_.detect_drive_type() == navcon::RobotConfig::DriveType::ACKERMANN) {
            controller_ = navcon::create_ackermann_controller("stanley");
        }
        
        // Map wheel names to Gazebo joints
        for (const auto& wheel : robot_config_.wheels) {
            wheel_joints_[wheel.name] = model->GetJoint(wheel.name + "_joint");
        }
    }
    
    navcon::RobotConfig build_config_from_sdf(
        gazebo::physics::ModelPtr model, 
        sdf::ElementPtr sdf
    ) {
        navcon::RobotConfigBuilder builder;
        
        // Gazebo plugin fills the struct - navcon doesn't know about SDF
        builder.with_name("ackermann_vehicle");
        builder.with_dimensions(2.0, 4.0);
        
        // Car-like robot - Gazebo provides these values
        builder.add_wheel({
            .name = "front_left_wheel",
            .position = navcon::Point(-1.0, 1.5, 0),
            .size = navcon::Size(0.3, 0.6, 0),
            .max_steer_angle = 0.6,  // ~35 degrees
            .max_throttle = 1.0
        });
        
        builder.add_wheel({
            .name = "front_right_wheel",
            .position = navcon::Point(1.0, 1.5, 0),
            .size = navcon::Size(0.3, 0.6, 0),
            .max_steer_angle = 0.6,
            .max_throttle = 1.0
        });
        
        // Rear wheels - no steering
        builder.add_wheel({
            .name = "rear_left_wheel",
            .position = navcon::Point(-1.0, -1.5, 0),
            .size = navcon::Size(0.3, 0.6, 0),
            .max_steer_angle = 0.0,
            .max_throttle = 1.0
        });
        
        builder.add_wheel({
            .name = "rear_right_wheel",
            .position = navcon::Point(1.0, -1.5, 0),
            .size = navcon::Size(0.3, 0.6, 0),
            .max_steer_angle = 0.0,
            .max_throttle = 1.0
        });
        
        return builder.build();
    }
    
    void OnUpdate() {
        // Get current state from Gazebo
        navcon::RobotState state = get_gazebo_state();
        
        // Compute control
        auto cmd = controller_->compute_control(
            state, goal_, robot_config_.to_constraints(), dt
        );
        
        // Convert to wheel commands
        auto wheel_cmds = wheel_controller_.velocity_to_wheels(cmd);
        
        // Apply to Gazebo joints
        for (const auto& [wheel_name, command] : wheel_cmds) {
            if (auto joint = wheel_joints_[wheel_name]) {
                // Set steering angle
                if (command.steering_angle != 0) {
                    joint->SetPosition(0, command.steering_angle);
                }
                // Set wheel velocity
                joint->SetVelocity(1, command.throttle * max_wheel_velocity_);
            }
        }
    }
};
```

### Flatsim Integration

```cpp
// Flatsim fills the RobotConfig struct - navcon knows NOTHING about JSON
navcon::RobotConfig create_truck_config() {
    navcon::RobotConfigBuilder builder;
    
    // Flatsim fills the struct however it wants - navcon doesn't care
    builder.with_name("8x8 Truck")
           .with_dimensions(2.5, 8.0)
           .with_mass(15000.0);
    
    // Add wheels - flatsim provides these values
    builder.add_wheel({
        .name = "front_left",
        .position = navcon::Point(-1.25, 5.0, 0),
        .size = navcon::Size(0.45, 0.96, 0),
        .max_steer_angle = 0.523,  // 30 degrees in radians
        .steer_differential = 0.052,
        .max_throttle = 0.25,
        .throttle_differential = 0.0
    });
    
    // ... more wheels
    
    return builder.build();
}

// Usage in flatsim
void setup_navigation() {
    // Create config - flatsim's responsibility
    navcon::RobotConfig config = create_truck_config();
    
    // Pass to navcon - this is the ONLY interface
    navcon::WheelController wheel_ctrl(config);
    auto controller = navcon::create_velocity_controller("pure_pursuit");
}
```

## Robot Configuration System

### Overview

NavCon uses a generic `RobotConfig` struct as its primary interface for describing robot physical properties and control capabilities. This design ensures complete simulator independence - any robotics framework can populate this struct from its own configuration format.

### Configuration Flow

```
Simulator-Specific Config → RobotConfig → NavCon Controllers
        (JSON, XML, etc.)      (Struct)      (Algorithms)
```

### Key Features

1. **Wheel-Based Description**: Each wheel has position, size, steering, and throttle parameters
2. **Auto-Detection**: The system automatically detects robot type (differential, Ackermann, etc.)
3. **Individual Wheel Control**: The `WheelController` converts high-level commands to per-wheel actions
4. **Flexible Mapping**: Supports complex multi-axle vehicles with differential steering/throttle

### Example Usage

```cpp
// Build robot configuration
navcon::RobotConfig config = navcon::RobotConfigBuilder()
    .with_name("8x8 Truck")
    .with_dimensions(2.5, 8.0)
    .add_wheel({
        .name = "front_left",
        .position = Point(-1.25, 5.0, 0),
        .max_steer_angle = 30 * M_PI/180,
        .steer_differential = 3 * M_PI/180,
        .max_throttle = 0.25
    })
    // ... add more wheels
    .build();

// Auto-detect drive type
auto drive_type = config.detect_drive_type(); // Returns MULTI_AXLE

// Create wheel controller
navcon::WheelController wheel_ctrl(config);

// Convert high-level command to wheel commands
navcon::VelocityCommand vel_cmd{.linear_velocity = 5.0, .angular_velocity = 0.5};
auto wheel_commands = wheel_ctrl.velocity_to_wheels(vel_cmd);

// Apply to individual wheels
for (const auto& [wheel_name, command] : wheel_commands) {
    // simulator applies command.steering_angle and command.throttle
}
```

## Architecture Validation

### Strengths

1. **Modularity**: Clean separation between algorithms, kinematics, and output formats
2. **Flexibility**: Template-based design supports diverse robot types and output formats
3. **Extensibility**: Factory pattern allows easy addition of new controllers
4. **Performance**: Header-only implementation minimizes overhead
5. **Portability**: Zero dependencies on specific frameworks (only Concord for geometry)
6. **Maintainability**: Clear interfaces and single responsibility principle
7. **Simulator Independence**: Generic `RobotConfig` struct interface allows any simulator integration

### Potential Concerns

1. **Template Complexity**: Heavy use of templates may increase compilation time
2. **Memory Management**: Factory pattern uses dynamic allocation
3. **Configuration Complexity**: Many parameters to tune for different robots
4. **Testing Coverage**: Template instantiation requires comprehensive testing

### Recommendations

1. **Documentation**: Comprehensive examples for each robot type
2. **Testing**: Unit tests for all controller/kinematic combinations
3. **Performance**: Benchmarking for real-time applications
4. **Validation**: Field testing with actual robot hardware
5. **Examples**: Reference implementations for common platforms

## Conclusion

NavCon represents a well-architected, flexible navigation controller library that successfully abstracts robot-specific details while providing powerful control algorithms. The design follows solid software engineering principles and provides clear extension points for future development.

The architecture is **sound and correct** for its intended purpose, with proper separation of concerns, flexible interfaces, and comprehensive coverage of common robotic platforms. The template-based design enables type safety while maintaining performance, and the factory pattern provides excellent extensibility.

The library is ready for production use and can be easily integrated into existing robotics frameworks while maintaining complete independence from any specific simulator or middleware.
