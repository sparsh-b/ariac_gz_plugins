#ifndef ARIAC_LOGICAL_CAMERA_PLUGIN_HPP
#define ARIAC_LOGICAL_CAMERA_PLUGIN_HPP

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

namespace ariac_sensors{
  // Forward declaration for a class to hold private members of the plugin class.
  class AriacLogicalCameraPluginPrivate;

  /// Plugin to attach to a gz LogicalCamera sensor in Ariac & convert its output msg to ROS & publish
  class AriacLogicalCameraPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
    public:
      AriacLogicalCameraPlugin();
      ~AriacLogicalCameraPlugin();

      void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) override;
  
    private:
      std::unique_ptr<AriacLogicalCameraPluginPrivate> impl_;
  };
}

#endif