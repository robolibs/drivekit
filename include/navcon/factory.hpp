#pragma once

#include "navcon/controller.hpp"
#include "navcon/followers/carrot.hpp"
#include "navcon/followers/pid.hpp"
#include "navcon/followers/pure_pursuit.hpp"
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace navcon {

    // Factory for creating controllers
    class ControllerFactory {
      public:
        using ControllerType = Controller;
        using ControllerPtr = std::unique_ptr<ControllerType>;
        using CreatorFunc = std::function<ControllerPtr(const ControllerConfig &)>;

        // Register a controller type
        inline void register_controller(const std::string &name, CreatorFunc creator) {
            creators_[name] = std::move(creator);
        }

        // Create a controller by name
        inline ControllerPtr create(const std::string &name, const ControllerConfig &config = {}) const {
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
        inline std::vector<std::string> get_available_types() const {
            std::vector<std::string> types;
            types.reserve(creators_.size());
            for (const auto &[name, _] : creators_) {
                types.push_back(name);
            }
            return types;
        }

        // Check if a controller type is available
        inline bool has_controller(const std::string &name) const { return creators_.find(name) != creators_.end(); }

        // Register default controllers
        inline void register_defaults() {
            using namespace followers;

            // PID controller
            register_controller("pid", [](const ControllerConfig &config) { return std::make_unique<PIDFollower>(); });

            // Pure Pursuit controller
            register_controller("pure_pursuit",
                                [](const ControllerConfig &config) { return std::make_unique<PurePursuitFollower>(); });

            // Carrot controller
            register_controller("carrot",
                                [](const ControllerConfig &config) { return std::make_unique<CarrotFollower>(); });
        }

      private:
        std::unordered_map<std::string, CreatorFunc> creators_;
    };

    // Convenience factory function
    inline std::unique_ptr<Controller> create_controller(const std::string &type, const ControllerConfig &config = {}) {
        ControllerFactory factory;
        factory.register_defaults();
        return factory.create(type, config);
    }

} // namespace navcon