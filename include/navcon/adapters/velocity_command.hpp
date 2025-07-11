#pragma once

#include "navcon/types.hpp"
#include "navcon/kinematics/differential_drive.hpp"
#include "navcon/kinematics/ackermann.hpp"

namespace navcon {
namespace adapters {

// Adapter to convert between different command types
class VelocityCommandAdapter {
public:
    // Convert any command type to VelocityCommand
    static VelocityCommand to_velocity(const ControlOutput& cmd, const RobotConstraints& constraints) {
        switch (cmd.type) {
            case OutputType::VELOCITY_COMMAND:
                return static_cast<const VelocityCommand&>(cmd);
                
            case OutputType::DIFFERENTIAL_COMMAND:
                return kinematics::DifferentialDrive::wheels_to_velocity(
                    static_cast<const DifferentialCommand&>(cmd), constraints);
                
            case OutputType::ACKERMANN_COMMAND:
                return kinematics::Ackermann::ackermann_to_velocity(
                    static_cast<const AckermannCommand&>(cmd), constraints);
                
            case OutputType::NORMALIZED_COMMAND:
                return kinematics::DifferentialDrive::normalized_to_velocity(
                    static_cast<const NormalizedCommand&>(cmd), constraints);
                
            default:
                VelocityCommand invalid;
                invalid.valid = false;
                invalid.status_message = "Unsupported command type";
                return invalid;
        }
    }
    
    // Convert VelocityCommand to other types
    static DifferentialCommand to_differential(const VelocityCommand& cmd, const RobotConstraints& constraints) {
        return kinematics::DifferentialDrive::velocity_to_wheels(cmd, constraints);
    }
    
    static AckermannCommand to_ackermann(const VelocityCommand& cmd, const RobotConstraints& constraints) {
        return kinematics::Ackermann::velocity_to_ackermann(cmd, constraints);
    }
    
    static NormalizedCommand to_normalized(const VelocityCommand& cmd, const RobotConstraints& constraints) {
        NormalizedCommand output;
        output.valid = cmd.valid;
        output.status_message = cmd.status_message;
        
        // Normalize linear velocity
        if (cmd.linear_velocity >= 0) {
            output.throttle = cmd.linear_velocity / constraints.max_linear_velocity;
        } else {
            output.throttle = cmd.linear_velocity / std::abs(constraints.min_linear_velocity);
        }
        
        // Normalize angular velocity
        output.steering = cmd.angular_velocity / constraints.max_angular_velocity;
        
        // Clamp to [-1, 1]
        output.throttle = std::clamp(output.throttle, -1.0, 1.0);
        output.steering = std::clamp(output.steering, -1.0, 1.0);
        
        return output;
    }
};

} // namespace adapters
} // namespace navcon