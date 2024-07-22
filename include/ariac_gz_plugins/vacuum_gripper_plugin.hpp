#ifndef VACUUM_GRIPPER_PLUGIN_HPP_
#define VACUUM_GRIPPER_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

namespace ariac_plugins {
  // Forward declaration of private data class.
  class VacuumGripperPluginPrivate;

  class VacuumGripperPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate
  {
    public:
      VacuumGripperPlugin();
      ~VacuumGripperPlugin();

      void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) override;

    private:
      std::unique_ptr<VacuumGripperPluginPrivate> impl_;
  };
}

#endif