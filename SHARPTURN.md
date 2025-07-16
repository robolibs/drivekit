# Sharp Turn Controller Design Document

## Overview

The Sharp Turn Controller implements the new **Turner** class hierarchy to handle complex turning maneuvers. It operates in two distinct modes: **Reactive** (automatic detection based on path constraints) and **Directive** (intentional segment replacement or heading adjustments). This design creates a clean separation between path-following (**Followers**) and maneuver-executing (**Turners**) while maintaining a unified **Controller** interface.

## Turner Operating Modes

### Reactive Mode (Automatic Detection)
The traditional approach where the turner automatically detects when it's needed:

- **Path Analysis**: Turner examines upcoming path geometry
- **Constraint Checking**: Determines if follower can handle the path segment
- **Automatic Handoff**: Switches when turning radius or angle constraints are violated

**Example**: Sharp 90° turn that exceeds vehicle's turning radius capability.

### Directive Mode (Intentional Replacement)
New capability where the turner executes explicit instructions regardless of geometric necessity:

#### 1. **Heading Adjustment Mode**
- **Purpose**: Change vehicle heading at the same location
- **Use Case**: Stationary pivot, orientation correction
- **Input**: Current pose + target heading
- **Output**: In-place rotation maneuver

**Example**: Agricultural vehicle needs to rotate 180° at field end before entering next row.

#### 2. **Segment Replacement Mode**  
- **Purpose**: Replace a path segment with an optimized turner maneuver
- **Use Case**: Headland turns, parking maneuvers, optimized field patterns
- **Input**: Start pose + end pose + preferred pattern
- **Output**: Custom maneuver replacing intermediate waypoints

**Example**: Agricultural headland with waypoints A→B→C→D gets replaced with A→[Turner Maneuver]→D.

## Waypoint Annotations

To support directive mode, waypoints can be annotated with turner instructions:

```cpp
struct Waypoint {
    Pose pose;
    std::map<std::string, std::string> annotations;
    
    // Example annotations:
    // annotations["turner"] = "sharp_turn"           // Which turner to use
    // annotations["pattern"] = "three_point"         // Preferred turn pattern  
    // annotations["mode"] = "heading_adjustment"     // Turn mode
    // annotations["segment_start"] = "true"          // Mark segment boundaries
    // annotations["segment_end"] = "true"            // Mark segment boundaries
};
```

### Annotation Examples

**Heading Adjustment:**
```cpp
waypoint.annotations["turner"] = "sharp_turn";
waypoint.annotations["mode"] = "heading_adjustment";
waypoint.annotations["target_heading"] = "1.57"; // 90 degrees in radians
```

**Segment Replacement:**
```cpp
// Start waypoint
start_wp.annotations["turner"] = "sharp_turn";
start_wp.annotations["mode"] = "segment_start";
start_wp.annotations["pattern"] = "bulb_omega";

// End waypoint  
end_wp.annotations["mode"] = "segment_end";
```

## Class Hierarchy Design

```cpp
// Top-level interface for all navigation controllers
class Controller {
public:
    virtual VelocityCommand compute_control(const RobotState& state,
                                           const Goal& goal,
                                           const RobotConstraints& constraints,
                                           double dt) = 0;
    virtual void set_path(const Path& path) = 0;
    virtual void reset() = 0;
    virtual ControllerStatus get_status() const = 0;
    virtual std::string get_type() const = 0;
    
    // Controller coordination interface
    virtual bool should_handoff() const { return false; }
    virtual std::string preferred_handoff_target() const { return ""; }
    virtual Path get_remaining_path() const = 0;
    virtual bool can_accept_handoff_from(const std::string& controller_type) const { return false; }
};

// Base class for path-following controllers (current Controller becomes this)
class Follower : public Controller {
public:
    // Follower-specific interface
    virtual bool can_handle_path_segment(const Path& path, size_t start_index, 
                                        const RobotConstraints& constraints) const = 0;
    virtual size_t find_handoff_point(const Path& path, 
                                     const RobotConstraints& constraints) const = 0;
    
protected:
    // Common follower functionality (from current Controller class)
    Path path_;
    size_t path_index_ = 0;
    ControllerConfig config_;
    ControllerStatus status_;
    
    // Helper methods (moved from current Controller)
    double normalize_angle(double angle);
    bool is_goal_reached(const Pose& current, const Pose& goal);
    // ... other helper methods
};

// Base class for maneuver controllers (new)
class Turner : public Controller {
public:
    enum class TurnerState {
        IDLE,           // Not executing a maneuver
        ANALYZING,      // Checking if maneuver is needed
        PLANNING,       // Computing maneuver trajectory
        EXECUTING,      // Performing the maneuver
        COMPLETED       // Maneuver finished
    };
    
    enum class TurnMode {
        REACTIVE,       // Automatic detection based on path constraints
        DIRECTIVE       // Explicit instruction execution
    };
    
    enum class DirectiveType {
        HEADING_ADJUSTMENT,  // Same position, change heading only
        SEGMENT_REPLACEMENT  // Replace path segment with maneuver
    };
    
    // Reactive mode interface (existing)
    virtual bool can_handle_maneuver(const Path& path, const RobotState& state,
                                    const RobotConstraints& constraints) const = 0;
    virtual void plan_maneuver(const Path& path, const RobotState& state,
                              const RobotConstraints& constraints) = 0;
    virtual bool is_maneuver_complete() const = 0;
    
    // Directive mode interface (new)
    virtual void configure_directive(DirectiveType type,
                                   const Pose& start_pose, 
                                   const Pose& end_pose,
                                   const std::string& pattern_hint = "") = 0;
    virtual void configure_heading_adjustment(const Pose& position, 
                                             double target_heading) = 0;
    virtual void configure_segment_replacement(const Pose& start_pose,
                                              const Pose& end_pose,
                                              const std::string& pattern_hint = "") = 0;
    
    // Handoff interface
    bool can_accept_handoff_from(const std::string& controller_type) const override {
        return controller_type.find("follower") != std::string::npos;
    }
    
protected:
    TurnerState state_ = TurnerState::IDLE;
    Path original_path_;
    Path remaining_path_;
    size_t handoff_point_;
};
```

## Concrete Implementations

### Follower Examples (Existing controllers become these)

```cpp
class PurePursuitFollower : public Follower {
public:
    std::string get_type() const override { return "pure_pursuit_follower"; }
    
    bool can_handle_path_segment(const Path& path, size_t start_index,
                                const RobotConstraints& constraints) const override {
        // Check if upcoming path segment can be handled with pure pursuit
        // Return false if sharp turns detected
        return !detect_sharp_turn_in_segment(path, start_index, constraints);
    }
    
    bool should_handoff() const override {
        size_t handoff_point = find_handoff_point(path_, constraints_);
        return handoff_point != path_.waypoints.size();
    }
    
    std::string preferred_handoff_target() const override {
        return "sharp_turn_turner";
    }
    
    size_t find_handoff_point(const Path& path,
                             const RobotConstraints& constraints) const override {
        // Look ahead for sharp turns that require turner
        for (size_t i = path_index_; i < path.waypoints.size() - 1; ++i) {
            if (requires_sharp_turn(path, i, constraints)) {
                return i;
            }
        }
        return path.waypoints.size(); // No handoff needed
    }
};

class StanleyFollower : public Follower {
public:
    std::string get_type() const override { return "stanley_follower"; }
    // Similar implementation to PurePursuitFollower
};
```

### Turner Implementation

```cpp
class SharpTurnTurner : public Turner {
public:
    enum class TurnPattern {
        THREE_POINT,      // Forward → Reverse → Forward
        FISHTAIL,         // S-shaped forward-only
        BULB_OMEGA,       // Circular loop
        T_TURN,           // Forward → Reverse arch → Forward
        PI_TURN,          // Rectangular forward pattern
        AUTO_SELECT       // Choose best pattern
    };
    
    std::string get_type() const override { return "sharp_turn_turner"; }
    
    VelocityCommand compute_control(const RobotState& state,
                                   const Goal& goal,
                                   const RobotConstraints& constraints,
                                   double dt) override {
        switch (state_) {
            case TurnerState::IDLE:
                if (can_handle_maneuver(original_path_, state, constraints)) {
                    state_ = TurnerState::PLANNING;
                }
                return VelocityCommand{0.0, 0.0, true, "Turner idle"};
                
            case TurnerState::PLANNING:
                plan_maneuver(original_path_, state, constraints);
                state_ = TurnerState::EXECUTING;
                return VelocityCommand{0.0, 0.0, true, "Planning maneuver"};
                
            case TurnerState::EXECUTING:
                auto cmd = execute_maneuver(state, constraints, dt);
                if (is_maneuver_complete()) {
                    update_remaining_path(state);
                    state_ = TurnerState::COMPLETED;
                }
                return cmd;
                
            case TurnerState::COMPLETED:
                return VelocityCommand{0.0, 0.0, true, "Maneuver complete, ready for handoff"};
        }
        
        return VelocityCommand{0.0, 0.0, false, "Invalid state"};
    }
    
    bool can_handle_maneuver(const Path& path, const RobotState& state,
                            const RobotConstraints& constraints) const override {
        // Check if sharp turn is needed at current position
        return detect_sharp_turn_needed(path, state, constraints);
    }
    
    void plan_maneuver(const Path& path, const RobotState& state,
                      const RobotConstraints& constraints) override {
        // Analyze turn requirements
        auto turn_analysis = analyze_required_turn(path, state);
        
        // Select best pattern
        selected_pattern_ = select_turn_pattern(turn_analysis, constraints);
        
        // Generate maneuver waypoints
        maneuver_waypoints_ = generate_turn_waypoints(selected_pattern_, 
                                                     turn_analysis, 
                                                     constraints);
        
        // Find where to resume original path
        handoff_point_ = find_path_resume_point(path, turn_analysis);
    }
    
    bool should_handoff() const override {
        return state_ == TurnerState::COMPLETED && !remaining_path_.waypoints.empty();
    }
    
    std::string preferred_handoff_target() const override {
        return "pure_pursuit_follower"; // or "stanley_follower"
    }
    
    Path get_remaining_path() const override {
        return remaining_path_;
    }

private:
    TurnPattern selected_pattern_;
    std::vector<Pose> maneuver_waypoints_;
    size_t current_maneuver_waypoint_ = 0;
    
    VelocityCommand execute_maneuver(const RobotState& state,
                                    const RobotConstraints& constraints,
                                    double dt);
    
    std::vector<Pose> generate_turn_waypoints(TurnPattern pattern,
                                             const TurnAnalysis& analysis,
                                             const RobotConstraints& constraints);
    
    void update_remaining_path(const RobotState& state);
};
```

## Controller Orchestration

### Simple Controller Manager

```cpp
class ControllerManager {
private:
    std::unique_ptr<Controller> active_controller_;
    std::unordered_map<std::string, std::unique_ptr<Controller>> controllers_;
    
public:
    void register_controller(const std::string& name, std::unique_ptr<Controller> controller) {
        controllers_[name] = std::move(controller);
    }
    
    void set_active_controller(const std::string& name) {
        auto it = controllers_.find(name);
        if (it != controllers_.end()) {
            active_controller_ = it->second.get();
        }
    }
    
    VelocityCommand compute_control(const RobotState& state,
                                   const Goal& goal,
                                   const RobotConstraints& constraints,
                                   double dt) {
        
        // Check if current controller wants to handoff
        if (active_controller_->should_handoff()) {
            std::string target = active_controller_->preferred_handoff_target();
            
            auto it = controllers_.find(target);
            if (it != controllers_.end() && 
                it->second->can_accept_handoff_from(active_controller_->get_type())) {
                
                // Perform handoff
                Path remaining = active_controller_->get_remaining_path();
                it->second->set_path(remaining);
                active_controller_ = it->second.get();
            }
        }
        
        return active_controller_->compute_control(state, goal, constraints, dt);
    }
};
```

## Usage Examples

### Example 1: Reactive Mode (Automatic Detection)

```cpp
int main() {
    ControllerManager manager;
    
    // Register followers
    manager.register_controller("pure_pursuit", 
        std::make_unique<PurePursuitFollower>());
    manager.register_controller("stanley", 
        std::make_unique<StanleyFollower>());
    
    // Register turners
    manager.register_controller("sharp_turn", 
        std::make_unique<SharpTurnTurner>());
    
    // Start with a follower
    manager.set_active_controller("pure_pursuit");
    
    // Set path with sharp turns that will trigger automatic handoff
    Path field_path = generate_agricultural_path();
    manager.set_path(field_path);
    
    // Main loop - automatic handoffs based on path constraints
    while (!goal_reached) {
        auto cmd = manager.compute_control(state, goal, constraints, dt);
        robot.execute(cmd);
    }
}
```

### Example 2: Directive Mode - Heading Adjustment

```cpp
int main() {
    // Direct usage for stationary pivot
    SharpTurnTurner turner;
    
    // Configure for 180-degree in-place turn
    Pose current_position;
    current_position.point.x = 10.0;
    current_position.point.y = 5.0;
    current_position.angle.yaw = 0.0;  // Facing east
    
    double target_heading = M_PI;      // Turn to face west
    
    turner.configure_heading_adjustment(current_position, target_heading);
    
    // Execute turn
    while (!turner.is_maneuver_complete()) {
        auto cmd = turner.compute_control(state, goal, constraints, dt);
        robot.execute(cmd);
    }
}
```

### Example 3: Directive Mode - Segment Replacement  

```cpp
int main() {
    // Agricultural headland turn optimization
    Path original_path = generate_field_rows();
    
    // Mark headland segment for turner replacement
    size_t row_end_index = find_row_end(original_path);
    size_t next_row_start = find_next_row_start(original_path);
    
    // Annotate waypoints for segment replacement
    original_path.waypoints[row_end_index].annotations["turner"] = "sharp_turn";
    original_path.waypoints[row_end_index].annotations["mode"] = "segment_start";
    original_path.waypoints[row_end_index].annotations["pattern"] = "bulb_omega";
    
    original_path.waypoints[next_row_start].annotations["mode"] = "segment_end";
    
    // Use ControllerManager with annotation-aware follower
    ControllerManager manager;
    manager.register_controller("stanley", std::make_unique<StanleyFollower>());
    manager.register_controller("sharp_turn", std::make_unique<SharpTurnTurner>());
    
    manager.set_active_controller("stanley");
    manager.set_path(original_path);
    
    // Follower will detect annotations and handoff to turner for optimized headland turn
    while (!goal_reached) {
        auto cmd = manager.compute_control(state, goal, constraints, dt);
        robot.execute(cmd);
    }
}
```

### Example 4: Programmatic Segment Replacement

```cpp
int main() {
    SharpTurnTurner turner;
    
    // Define start and end of segment to replace
    Pose segment_start;
    segment_start.point.x = 50.0;  // End of crop row
    segment_start.point.y = 10.0;
    segment_start.angle.yaw = 0.0; // Facing east
    
    Pose segment_end;
    segment_end.point.x = 50.0;    // Start of next row
    segment_end.point.y = 15.0;    // 5m headland
    segment_end.angle.yaw = M_PI;  // Facing west
    
    // Configure turner to replace intermediate waypoints with optimized turn
    turner.configure_segment_replacement(segment_start, segment_end, "bulb_omega");
    
    // Execute optimized headland turn
    while (!turner.is_maneuver_complete()) {
        auto cmd = turner.compute_control(state, goal, constraints, dt);
        robot.execute(cmd);
    }
    
    // Get remaining path to continue with follower
    Path remaining = turner.get_remaining_path();
    follower.set_path(remaining);
}
```

### Example 2: Custom Hybrid Controller

```cpp
class AgricultureController : public Controller {
private:
    std::unique_ptr<Follower> row_follower_;
    std::unique_ptr<Turner> headland_turner_;
    Controller* active_;
    
public:
    AgricultureController() {
        row_follower_ = std::make_unique<PurePursuitFollower>();
        headland_turner_ = std::make_unique<SharpTurnTurner>();
        active_ = row_follower_.get();
    }
    
    std::string get_type() const override { return "agriculture_hybrid"; }
    
    VelocityCommand compute_control(const RobotState& state,
                                   const Goal& goal,
                                   const RobotConstraints& constraints,
                                   double dt) override {
        
        // Auto-switch based on controller capabilities
        if (active_ == row_follower_.get() && row_follower_->should_handoff()) {
            headland_turner_->set_path(row_follower_->get_remaining_path());
            active_ = headland_turner_.get();
        }
        else if (active_ == headland_turner_.get() && headland_turner_->should_handoff()) {
            row_follower_->set_path(headland_turner_->get_remaining_path());
            active_ = row_follower_.get();
        }
        
        return active_->compute_control(state, goal, constraints, dt);
    }
};
```

## Migration Path

### Phase 1: Rename Current Controller
```cpp
// Current controller.hpp becomes follower.hpp
class Follower : public Controller {
    // All existing Controller functionality moves here
};

// Update existing controllers
class StanleyController : public Follower { /* ... */ };
class PurePursuitController : public Follower { /* ... */ };
```

### Phase 2: Add Turner Base Class
```cpp
// New turner.hpp
class Turner : public Controller {
    // Turner-specific functionality
};
```

### Phase 3: Implement SharpTurnTurner
```cpp
class SharpTurnTurner : public Turner {
    // Sharp turn implementation
};
```

## Benefits of This Design

1. **Clear Separation**: Followers vs Turners have distinct responsibilities
2. **Proper Inheritance**: Each category has appropriate base class
3. **Unified Interface**: Controller remains the universal interface
4. **Backward Compatible**: Existing controllers become Followers with minimal changes
5. **Extensible**: Easy to add new Followers or Turners
6. **Composable**: Can build complex behaviors from simple components

This hierarchy respects the naming you wanted while providing a clean, extensible architecture for navigation control.