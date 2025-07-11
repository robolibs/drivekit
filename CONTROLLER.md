# Standalone Controller Architecture

## Overview

This document outlines the architecture for a completely decoupled, simulator-agnostic controller library that can work with any robotics framework (ROS, Gazebo, flatsim, or real hardware).

## Core Requirements for Abstraction

### 1. Essential State Information

#### Robot State (What the controller needs to know)
- **Position**: Current (x, y, z) in some reference frame
- **Orientation**: Current heading/rotation (quaternion or yaw angle minimum)
- **Velocities**: 
  - Linear velocity (forward/backward speed)
  - Angular velocity (turning rate)
  - Lateral velocity (for holonomic robots)
- **Timestamp**: For time-based calculations and derivatives

#### Goal/Target Information
- **Target Pose**: Where to go (position + orientation)
- **Path/Waypoints**: Series of poses for path following
- **Velocity Constraints**: Desired speeds at different points
- **Tolerance Parameters**: How close is "good enough"

#### Robot Physical Constraints
- **Kinematic Model Parameters**:
  - Wheelbase (distance between axles)
  - Track width (distance between wheels)
  - Wheel radius
  - Number of wheels/axles
  - Steering configuration
- **Dynamic Limits**:
  - Max/min velocities
  - Max accelerations
  - Max steering angles
  - Max steering rate
  - Turning radius constraints

### 2. Output Interface

The controller must output commands that any system can interpret:

#### Primary Outputs
- **Velocity Commands**:
  - Linear velocity (m/s)
  - Angular velocity (rad/s)
  
#### Alternative Outputs (system-specific)
- **Ackermann**: Steering angle + speed
- **Differential**: Left/right wheel speeds
- **Holonomic**: X/Y/theta velocities
- **Multi-axle**: Front/rear steering angles

### 3. Architecture Separation by Robot Type

#### Differential Drive
**Needs**:
- Current pose (x, y, theta)
- Current velocities
- Track width parameter
- Wheel constraints

**Control Strategy**:
- Direct angular velocity control
- Can rotate in place
- Simple heading-to-goal calculations

#### Ackermann Steering
**Needs**:
- Current pose and velocities
- Wheelbase parameter
- Max steering angle
- Min turning radius

**Control Strategies**:
- Pure Pursuit (geometric path following)
- Stanley (cross-track error correction)
- Model Predictive Control

**Special Considerations**:
- Cannot rotate in place
- Must consider turning radius
- Speed affects stability

#### Multi-Axle Steering
**Needs**:
- All Ackermann requirements PLUS:
- Rear axle steering limits
- Coordination parameters
- Center of rotation calculations

**Control Strategies**:
- Coordinated steering (reduce turning radius)
- Counter-steering (high-speed stability)
- Crab steering (lateral movement)

**Special Considerations**:
- Complex kinematics
- Multiple control modes
- Stability vs maneuverability trade-offs

### 4. Abstraction Layers

```
Application Layer (ROS, flatsim, etc.)
    ↓
Controller Interface (Pure Virtual)
    ↓
Kinematic Model Layer
    ├── Differential Model
    ├── Ackermann Model
    ├── Multi-Axle Model
    └── Holonomic Model
    ↓
Control Algorithm Layer
    ├── PID Control
    ├── Pure Pursuit
    ├── Stanley
    ├── MPC
    └── Custom Algorithms
    ↓
Output Adapter Layer
    ├── Twist Message (ROS)
    ├── Joint Commands
    ├── Normalized Controls (-1 to 1)
    └── Direct Velocities
```

### 5. Key Design Principles

#### 1. Zero Dependencies
- No ROS headers
- No simulator-specific types
- Standard C++ only
- Template-based for flexibility

#### 2. Data Flow
```
Input: State + Goal → Controller → Output: Commands
                          ↑
                    Parameters/Config
```

#### 3. Time Handling
- Controller receives dt (time step)
- No internal timing assumptions
- Caller responsible for update rate

#### 4. Coordinate Frames
- Controller works in local robot frame
- Transformations handled externally
- Option for global frame operation
