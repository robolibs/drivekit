# Coarse-to-Fine Trajectory Planning: DP + CILQR

## Repository Analysis Summary

### Current Architecture

DriveKit is a modular C++ path tracking library with a clean hierarchy:

```
Controller (abstract base)
├── point/   → PID, Carrot (point-to-point)
├── path/    → Pure Pursuit, Stanley, LQR (path-following)
├── pred/    → MPC, MPPI, MCA, SOC (predictive/sampling-based)
└── fuzzy/   → FLC (fuzzy logic)
```

### Key Existing Components

| Component | Location | Relevance to CILQR |
|-----------|----------|-------------------|
| `LQRFollower` | `path/lqr.hpp` | Base LQR with DARE solver - can be extended |
| `MPCFollower` | `pred/mpc.hpp` | Single-shooting optimizer, cost functions |
| `MPPIFollower` | `pred/mppi.hpp` | Sampling-based, warm-starting patterns |
| `RobotConstraints` | `types.hpp` | Kinematic limits (steering, velocity, acceleration) |
| `WorldConstraints` | `types.hpp` | Obstacles with Gaussian predictions |
| `Path` | `types.hpp` | Waypoint representation |

### Vehicle Models Used
- **Ackermann**: Bicycle model with steering angle
- **Differential**: Direct angular velocity control
- Both use kinematic (not dynamic) models

### Constraint Handling
- Current MPC uses soft constraints via cost penalties
- No explicit barrier functions or augmented Lagrangian
- Constraints: velocity limits, steering limits, steering rate limits

---

## Proposed Framework: DP → CILQR

### Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    COARSE-TO-FINE PLANNER                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐         ┌─────────────┐         ┌───────────┐ │
│  │   PHASE 1   │         │   PHASE 2   │         │  OUTPUT   │ │
│  │     DP      │ ──────► │   CILQR     │ ──────► │ Trajectory│ │
│  │  (Coarse)   │  tube   │   (Fine)    │         │           │ │
│  └─────────────┘         └─────────────┘         └───────────┘ │
│                                                                 │
│  • Discretized search    • Continuous optim     • Smooth       │
│  • Obstacle avoidance    • Constraint handling  • Driveable    │
│  • Global feasibility    • Local refinement     • Optimal      │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Dynamic Programming (DP) Planner

### Purpose
Find a coarse, collision-free reference trajectory through discretized state-space.

### State Space Discretization

```cpp
struct DPConfig {
    // Spatial discretization
    double s_resolution = 0.5;      // Along-path resolution (m)
    double l_resolution = 0.2;      // Lateral offset resolution (m)
    double l_max = 3.0;             // Max lateral deviation (m)
    
    // Temporal discretization (optional for speed planning)
    double t_resolution = 0.1;      // Time step (s)
    int num_time_steps = 50;        // Planning horizon
    
    // Cost weights
    double weight_obstacle = 1000.0;
    double weight_lateral = 10.0;
    double weight_curvature = 5.0;
    double weight_deviation = 1.0;
};
```

### Algorithm: Spatial DP (Frenet Frame)

```
Input: Reference path, obstacles, start/goal
Output: Coarse trajectory "tube"

1. Convert to Frenet frame (s, l) along reference path
2. Discretize lateral space: l ∈ [-l_max, l_max] with resolution
3. For each station s_i along path:
   a. For each lateral position l_j:
      - Check collision with obstacles
      - Compute transition costs from all (s_{i-1}, l_k)
      - Store minimum cost and parent pointer
4. Backtrack from goal to extract optimal path
5. Convert back to Cartesian coordinates
```

### DP Cost Function

```cpp
double compute_dp_cost(const DPNode& from, const DPNode& to, 
                       const std::vector<Obstacle>& obstacles) {
    double cost = 0.0;
    
    // 1. Obstacle cost (collision check + clearance)
    double clearance = compute_min_clearance(to.pose, obstacles);
    if (clearance < robot_radius) return INFINITY;  // Hard constraint
    cost += weight_obstacle * exp(-clearance / safety_margin);
    
    // 2. Lateral deviation from reference
    cost += weight_lateral * to.l * to.l;
    
    // 3. Curvature (smoothness)
    double curvature = compute_curvature(from, to);
    cost += weight_curvature * curvature * curvature;
    
    // 4. Lateral change (consistency)
    double dl = to.l - from.l;
    cost += weight_deviation * dl * dl;
    
    return cost;
}
```

### Output: Trajectory Tube

```cpp
struct TrajectoryTube {
    std::vector<Pose> reference;      // Coarse waypoints
    std::vector<double> l_bounds_min; // Min lateral bound at each point
    std::vector<double> l_bounds_max; // Max lateral bound at each point
    std::vector<double> velocities;   // Reference velocities
};
```

---

## Phase 2: Constrained Iterative LQR (CILQR)

### Purpose
Refine the coarse trajectory with:
- Strict kinematic constraint satisfaction
- Smooth, continuous control inputs
- Local obstacle avoidance within the tube

### CILQR Algorithm Overview

```
Input: Initial trajectory from DP, constraints
Output: Optimal trajectory and controls

1. Initialize with DP trajectory
2. Repeat until convergence:
   a. Forward pass: Rollout with current controls
   b. Linearize dynamics around trajectory
   c. Quadraticize cost (including barrier functions)
   d. Backward pass: Compute optimal gains K, k
   e. Forward pass: Line search with new controls
   f. Check convergence
3. Return optimized trajectory
```

### State and Control Vectors

```cpp
// State: [x, y, θ, v, δ] for Ackermann
//        [x, y, θ, v, ω] for Differential
struct CILQRState {
    double x, y;        // Position
    double theta;       // Heading
    double v;           // Velocity
    double delta;       // Steering angle (Ackermann) or omega (Diff)
};

// Control: [a, δ_dot] for Ackermann
//          [a, α]     for Differential (linear accel, angular accel)
struct CILQRControl {
    double acceleration;
    double steering_rate;  // or angular_acceleration
};
```

### Kinematic Model (Bicycle)

```cpp
CILQRState dynamics(const CILQRState& x, const CILQRControl& u, double dt) {
    CILQRState x_next;
    
    // Ackermann (bicycle model)
    x_next.x = x.x + x.v * cos(x.theta) * dt;
    x_next.y = x.y + x.v * sin(x.theta) * dt;
    x_next.theta = x.theta + x.v * tan(x.delta) / wheelbase * dt;
    x_next.v = x.v + u.acceleration * dt;
    x_next.delta = x.delta + u.steering_rate * dt;
    
    return x_next;
}
```

### Jacobians (for linearization)

```cpp
// A = ∂f/∂x
Matrix5d compute_A(const CILQRState& x, double dt) {
    Matrix5d A = Matrix5d::Identity();
    A(0, 2) = -x.v * sin(x.theta) * dt;
    A(0, 3) = cos(x.theta) * dt;
    A(1, 2) = x.v * cos(x.theta) * dt;
    A(1, 3) = sin(x.theta) * dt;
    A(2, 3) = tan(x.delta) / wheelbase * dt;
    A(2, 4) = x.v / (wheelbase * cos(x.delta) * cos(x.delta)) * dt;
    return A;
}

// B = ∂f/∂u
Matrix52d compute_B(const CILQRState& x, double dt) {
    Matrix52d B = Matrix52d::Zero();
    B(3, 0) = dt;  // acceleration -> velocity
    B(4, 1) = dt;  // steering_rate -> steering
    return B;
}
```

### Constraint Handling: Log-Barrier Method

```cpp
struct Constraints {
    // Velocity constraints
    double v_min, v_max;
    
    // Steering constraints
    double delta_min, delta_max;
    
    // Control constraints
    double a_min, a_max;
    double delta_dot_min, delta_dot_max;
    
    // Obstacle constraints (from tube)
    std::vector<double> l_min, l_max;  // Lateral bounds
};

double barrier_cost(double x, double x_min, double x_max, double mu) {
    // Log barrier: -mu * (log(x - x_min) + log(x_max - x))
    if (x <= x_min || x >= x_max) return INFINITY;
    return -mu * (log(x - x_min) + log(x_max - x));
}

// Relaxed log barrier (smooth near boundary)
double relaxed_barrier(double x, double x_min, double x_max, double mu, double delta) {
    double cost = 0.0;
    
    // Lower bound
    if (x - x_min < delta) {
        cost += mu * (pow((x - x_min - delta) / delta, 2) - 1) - mu * log(delta);
    } else {
        cost += -mu * log(x - x_min);
    }
    
    // Upper bound (similar)
    // ...
    
    return cost;
}
```

### CILQR Cost Function

```cpp
struct CILQRCost {
    // Tracking cost (from DP reference)
    double weight_position = 10.0;
    double weight_heading = 5.0;
    double weight_velocity = 1.0;
    
    // Control effort
    double weight_acceleration = 0.1;
    double weight_steering_rate = 0.1;
    
    // Smoothness
    double weight_jerk = 0.01;
    
    // Barrier parameter (decreases over iterations)
    double mu = 1.0;
};

double stage_cost(const CILQRState& x, const CILQRControl& u,
                  const CILQRState& x_ref, int k) {
    double cost = 0.0;
    
    // Tracking
    cost += weight_position * (pow(x.x - x_ref.x, 2) + pow(x.y - x_ref.y, 2));
    cost += weight_heading * pow(normalize_angle(x.theta - x_ref.theta), 2);
    cost += weight_velocity * pow(x.v - x_ref.v, 2);
    
    // Control effort
    cost += weight_acceleration * u.acceleration * u.acceleration;
    cost += weight_steering_rate * u.steering_rate * u.steering_rate;
    
    // Barrier costs for constraints
    cost += barrier_cost(x.v, v_min, v_max, mu);
    cost += barrier_cost(x.delta, delta_min, delta_max, mu);
    cost += barrier_cost(u.acceleration, a_min, a_max, mu);
    cost += barrier_cost(u.steering_rate, delta_dot_min, delta_dot_max, mu);
    
    // Tube constraints (lateral bounds)
    double l = compute_lateral_offset(x, reference_path, k);
    cost += barrier_cost(l, l_bounds_min[k], l_bounds_max[k], mu);
    
    return cost;
}
```

### Backward Pass (Value Function Approximation)

```cpp
void backward_pass(const Trajectory& traj, const std::vector<CILQRControl>& controls) {
    int N = traj.size();
    
    // Terminal cost
    Vx[N-1] = compute_terminal_cost_gradient(traj[N-1]);
    Vxx[N-1] = compute_terminal_cost_hessian(traj[N-1]);
    
    for (int k = N-2; k >= 0; k--) {
        // Linearize dynamics
        auto A = compute_A(traj[k], dt);
        auto B = compute_B(traj[k], dt);
        
        // Quadraticize cost
        auto [lx, lu, lxx, luu, lux] = quadraticize_cost(traj[k], controls[k], k);
        
        // Q-function coefficients
        Qx = lx + A.transpose() * Vx[k+1];
        Qu = lu + B.transpose() * Vx[k+1];
        Qxx = lxx + A.transpose() * Vxx[k+1] * A;
        Quu = luu + B.transpose() * Vxx[k+1] * B;
        Qux = lux + B.transpose() * Vxx[k+1] * A;
        
        // Regularization for positive definiteness
        Quu_reg = Quu + reg * Matrix2d::Identity();
        
        // Optimal gains
        K[k] = -Quu_reg.inverse() * Qux;
        k_ff[k] = -Quu_reg.inverse() * Qu;
        
        // Value function update
        Vx[k] = Qx + K[k].transpose() * Quu * k_ff[k] + K[k].transpose() * Qu + Qux.transpose() * k_ff[k];
        Vxx[k] = Qxx + K[k].transpose() * Quu * K[k] + K[k].transpose() * Qux + Qux.transpose() * K[k];
    }
}
```

### Forward Pass with Line Search

```cpp
Trajectory forward_pass(const Trajectory& traj_old, 
                        const std::vector<CILQRControl>& controls_old,
                        double alpha) {
    Trajectory traj_new;
    std::vector<CILQRControl> controls_new;
    
    traj_new[0] = traj_old[0];  // Fixed initial state
    
    for (int k = 0; k < N-1; k++) {
        // State deviation
        auto dx = traj_new[k] - traj_old[k];
        
        // Updated control
        controls_new[k] = controls_old[k] + alpha * k_ff[k] + K[k] * dx;
        
        // Clamp controls (projection)
        controls_new[k] = clamp_controls(controls_new[k]);
        
        // Rollout dynamics
        traj_new[k+1] = dynamics(traj_new[k], controls_new[k], dt);
    }
    
    return traj_new;
}
```

---

## Implementation Plan

### File Structure

```
include/drivekit/pred/
├── dp.hpp           # DP planner header
├── cilqr.hpp        # CILQR optimizer header
└── dp_cilqr.hpp     # Combined coarse-to-fine planner

src/drivekit/pred/
├── dp.cpp           # DP implementation
├── cilqr.cpp        # CILQR implementation
└── dp_cilqr.cpp     # Integration

examples/
└── dp_cilqr.cpp     # Example usage
```

### Class Design

```cpp
// ============ DP Planner ============
namespace drivekit::pred {

struct DPConfig {
    double s_resolution = 0.5;
    double l_resolution = 0.2;
    double l_max = 3.0;
    // ... weights
};

class DPPlanner {
public:
    DPPlanner(const DPConfig& config = DPConfig());
    
    TrajectoryTube plan(
        const Path& reference,
        const RobotState& start,
        const Pose& goal,
        const WorldConstraints& world
    );
    
private:
    DPConfig config_;
    // Internal grid, cost-to-go, etc.
};

// ============ CILQR Optimizer ============
struct CILQRConfig {
    int max_iterations = 50;
    double convergence_threshold = 1e-4;
    double mu_init = 1.0;           // Initial barrier parameter
    double mu_decay = 0.5;          // Barrier decay rate
    double reg_init = 1e-6;         // Regularization
    double reg_max = 1e6;
    double alpha_min = 1e-4;        // Min line search step
    // ... cost weights
};

class CILQROptimizer {
public:
    CILQROptimizer(const CILQRConfig& config = CILQRConfig());
    
    Trajectory optimize(
        const TrajectoryTube& tube,
        const RobotState& start,
        const RobotConstraints& constraints
    );
    
private:
    CILQRConfig config_;
    
    void backward_pass();
    Trajectory forward_pass(double alpha);
    double compute_cost(const Trajectory& traj);
    bool check_convergence();
};

// ============ Combined Planner ============
struct DPCILQRConfig {
    DPConfig dp_config;
    CILQRConfig cilqr_config;
    bool replan_on_deviation = true;
    double replan_threshold = 0.5;  // meters
};

class DPCILQRFollower : public Controller {
public:
    DPCILQRFollower(const DPCILQRConfig& config = DPCILQRConfig());
    
    VelocityCommand compute_control(
        const RobotState& current_state,
        const Goal& goal,
        const RobotConstraints& constraints,
        double dt,
        const WorldConstraints* world = nullptr
    ) override;
    
    void set_path(const Path& path) override;
    std::string get_type() const override { return "DP_CILQR"; }
    
    // Access planned trajectory
    const Trajectory& get_planned_trajectory() const;
    const TrajectoryTube& get_tube() const;
    
private:
    DPCILQRConfig config_;
    DPPlanner dp_planner_;
    CILQROptimizer cilqr_optimizer_;
    
    TrajectoryTube current_tube_;
    Trajectory current_trajectory_;
    size_t trajectory_index_ = 0;
    
    void replan(const RobotState& state, const WorldConstraints& world);
    bool needs_replan(const RobotState& state) const;
};

} // namespace drivekit::pred
```

### Integration with Tracker

```cpp
// In tracker.hpp
enum class TrackerType { 
    PID, PURE_PURSUIT, STANLEY, CARROT, LQR, 
    MPC, MPC_TRAILER, MPPI, SOC, MCA,
    DP_CILQR  // NEW
};

// In tracker.cpp
case TrackerType::DP_CILQR:
    controller_ = std::make_unique<pred::DPCILQRFollower>();
    break;
```

---

## Implementation Steps

### Step 1: Core Data Structures
- [ ] Add `TrajectoryTube` to `types.hpp`
- [ ] Add `CILQRState`, `CILQRControl` structs
- [ ] Add Frenet frame utilities

### Step 2: DP Planner
- [ ] Implement Frenet frame conversion
- [ ] Implement discretized state-space grid
- [ ] Implement DP cost function with obstacles
- [ ] Implement backtracking for path extraction
- [ ] Add tube width computation

### Step 3: CILQR Optimizer
- [ ] Implement kinematic model and Jacobians
- [ ] Implement log-barrier constraint handling
- [ ] Implement backward pass (Riccati recursion)
- [ ] Implement forward pass with line search
- [ ] Add regularization and convergence checks

### Step 4: Integration
- [ ] Create `DPCILQRFollower` controller
- [ ] Implement replanning logic
- [ ] Add to `Tracker` enum and factory
- [ ] Create example file

### Step 5: Testing & Tuning
- [ ] Unit tests for DP planner
- [ ] Unit tests for CILQR optimizer
- [ ] Integration tests with obstacles
- [ ] Parameter tuning for different scenarios

---

## References

1. **iLQR**: Tassa et al., "Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization"
2. **CILQR**: Chen et al., "Constrained Iterative LQR for On-Road Autonomous Driving Motion Planning"
3. **Log-Barrier**: Boyd & Vandenberghe, "Convex Optimization" (Interior Point Methods)
4. **Frenet Frame**: Werling et al., "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame"
5. **DP Planning**: Ziegler et al., "Trajectory Planning for Bertha - A Local, Continuous Method"

---

## Notes

### Why This Approach?

| Challenge | DP Solution | CILQR Solution |
|-----------|-------------|----------------|
| Obstacle avoidance | Global search finds collision-free path | Stays within safe tube |
| Local minima | Discretization explores all options | Warm-started from DP |
| Kinematic constraints | Coarse feasibility check | Strict satisfaction via barriers |
| Smoothness | Not guaranteed | Optimized control inputs |
| Computation | O(N × L²) per planning | O(N × iter) refinement |

### Comparison with Existing Controllers

| Controller | Global Planning | Constraint Handling | Smoothness |
|------------|-----------------|---------------------|------------|
| MPC | No (local) | Soft penalties | Good |
| MPPI | No (sampling) | Implicit | Moderate |
| LQR | No (tracking) | None | Good |
| **DP+CILQR** | **Yes (DP)** | **Hard (barriers)** | **Optimal** |

### Potential Extensions

1. **Speed Planning**: Add temporal dimension to DP for velocity profiles
2. **Dynamic Obstacles**: Predict obstacle motion in DP cost
3. **Multiple Homotopies**: Keep top-K DP solutions for CILQR
4. **Warm Starting**: Use previous CILQR solution when replanning
5. **Adaptive Horizon**: Extend horizon in complex scenarios
