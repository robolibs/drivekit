#pragma once

#include "navcon/types.hpp"
#include <algorithm>

namespace navcon {
namespace adapters {

// Adapter for normalized commands (-1 to 1 range)
class NormalizedCommandAdapter {
public:
    // Scale factors for fine-tuning normalized output
    struct ScaleFactors {
        double throttle_deadzone = 0.05;  // Below this, output is zero
        double steering_deadzone = 0.05;
        double throttle_curve = 1.0;      // 1.0 = linear, >1.0 = more aggressive
        double steering_curve = 1.0;
    };
    
    // Apply scaling and dead zones to normalized command
    static NormalizedCommand apply_scaling(
        const NormalizedCommand& cmd,
        const ScaleFactors& factors = ScaleFactors{}
    ) {
        NormalizedCommand output = cmd;
        
        // Apply deadzone to throttle
        if (std::abs(output.throttle) < factors.throttle_deadzone) {
            output.throttle = 0.0;
        } else {
            // Apply curve
            double sign = output.throttle > 0 ? 1.0 : -1.0;
            output.throttle = sign * std::pow(std::abs(output.throttle), factors.throttle_curve);
        }
        
        // Apply deadzone to steering
        if (std::abs(output.steering) < factors.steering_deadzone) {
            output.steering = 0.0;
        } else {
            // Apply curve
            double sign = output.steering > 0 ? 1.0 : -1.0;
            output.steering = sign * std::pow(std::abs(output.steering), factors.steering_curve);
        }
        
        return output;
    }
    
    // Convert normalized to percentage (0-100)
    static std::pair<double, double> to_percentage(const NormalizedCommand& cmd) {
        double throttle_percent = (cmd.throttle + 1.0) * 50.0;  // -1 to 1 -> 0 to 100
        double steering_percent = (cmd.steering + 1.0) * 50.0;
        
        return {
            std::clamp(throttle_percent, 0.0, 100.0),
            std::clamp(steering_percent, 0.0, 100.0)
        };
    }
    
    // Convert from joystick input (typically -1 to 1)
    static NormalizedCommand from_joystick(
        double x_axis,  // Left/right
        double y_axis,  // Forward/back
        bool tank_drive = false
    ) {
        NormalizedCommand cmd;
        cmd.valid = true;
        
        if (tank_drive) {
            // Tank drive: y_axis controls both, x_axis adds differential
            cmd.throttle = std::clamp(y_axis, -1.0, 1.0);
            cmd.steering = std::clamp(x_axis, -1.0, 1.0);
        } else {
            // Car-like: y_axis is throttle, x_axis is steering
            cmd.throttle = std::clamp(y_axis, -1.0, 1.0);
            cmd.steering = std::clamp(x_axis, -1.0, 1.0);
        }
        
        cmd.status_message = "Joystick input";
        return cmd;
    }
    
    // Safety limits
    static NormalizedCommand apply_safety_limits(
        const NormalizedCommand& cmd,
        double max_throttle = 1.0,
        double max_steering = 1.0
    ) {
        NormalizedCommand output = cmd;
        
        // Apply throttle limits
        if (output.throttle > 0) {
            output.throttle = std::min(output.throttle, max_throttle);
        } else {
            output.throttle = std::max(output.throttle, -max_throttle);
        }
        
        // Apply steering limits
        if (output.steering > 0) {
            output.steering = std::min(output.steering, max_steering);
        } else {
            output.steering = std::max(output.steering, -max_steering);
        }
        
        return output;
    }
};

} // namespace adapters
} // namespace navcon