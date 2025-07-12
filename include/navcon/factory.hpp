#pragma once

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include "navcon/controller.hpp"
#include "navcon/controllers/pid.hpp"
#include "navcon/controllers/pure_pursuit.hpp"
#include "navcon/controllers/stanley.hpp"
#include "navcon/controllers/carrot.hpp"

namespace navcon {

// Factory for creating controllers
class ControllerFactory {
public:
    using ControllerType = Controller;
    using ControllerPtr = std::unique_ptr<ControllerType>;
    using CreatorFunc = std::function<ControllerPtr(const ControllerConfig&)>;
    
    // Register a controller type
    void register_controller(const std::string& name, CreatorFunc creator) {
        creators_[name] = std::move(creator);
    }
    
    // Create a controller by name
    ControllerPtr create(const std::string& name, const ControllerConfig& config = {}) const {
        auto it = creators_.find(name);
        if (it != creators_.end()) {
            auto controller = it->second(config);
            if (controller) {
                controller->set_config(config);
            }
            return controller;
        }
        return nullptr;
    }
    
    // Get list of available controller types
    std::vector<std::string> get_available_types() const {
        std::vector<std::string> types;
        types.reserve(creators_.size());
        for (const auto& [name, _] : creators_) {
            types.push_back(name);
        }
        return types;
    }
    
    // Check if a controller type is available
    bool has_controller(const std::string& name) const {
        return creators_.find(name) != creators_.end();
    }
    
    // Register default controllers
    void register_defaults() {
        using namespace controllers;
        
        // PID controller
        register_controller("pid", [](const ControllerConfig& config) {
            return std::make_unique<PIDController>();
        });
        
        // Pure Pursuit controller
        register_controller("pure_pursuit", [](const ControllerConfig& config) {
            return std::make_unique<PurePursuitController>();
        });
        
        // Stanley controller
        register_controller("stanley", [](const ControllerConfig& config) {
            return std::make_unique<StanleyController>();
        });
        
        // Carrot controller
        register_controller("carrot", [](const ControllerConfig& config) {
            return std::make_unique<CarrotController>();
        });
    }
    
private:
    std::unordered_map<std::string, CreatorFunc> creators_;
};

// Convenience factory function
inline std::unique_ptr<Controller> create_controller(
    const std::string& type,
    const ControllerConfig& config = {}
) {
    ControllerFactory factory;
    factory.register_defaults();
    return factory.create(type, config);
}

} // namespace navcon