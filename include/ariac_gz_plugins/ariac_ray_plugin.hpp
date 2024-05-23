#ifndef ARIAC_RAY_PLUGIN_HPP
#define ARIAC_RAY_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/break_beam_status.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>


namespace ariac_sensors{
  
  // Forward Declaration of a class to hold the private members of the AriacRayPlugin class.
  class AriacRayPluginPrivate;

  /// Plugin for the Ray sensors
  class AriacRayPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
    public:
      AriacRayPlugin();
      ~AriacRayPlugin();

      void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) override;

    private:
      std::unique_ptr<AriacRayPluginPrivate> impl_;
  };
}


#endif