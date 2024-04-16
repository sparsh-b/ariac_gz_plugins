#ifndef ARIAC_CAMERA_PLUGIN_HPP
#define ARIAC_CAMERA_PLUGIN_HPP

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

namespace ariac_sensors{

  // Forward declaration of a class to hold the member data about the plugin
  class AriacCameraPluginPrivate;

  class AriacCameraPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
    public:
      AriacCameraPlugin();

      void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) override;

    private:
      std::unique_ptr<AriacCameraPluginPrivate> impl_;
  };
}

#endif