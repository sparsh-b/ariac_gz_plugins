#ifndef ARIAC_LOGICAL_CAMERA_PLUGIN_HPP
#define ARIAC_LOGICAL_CAMERA_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/logical_camera_image.pb.h>
#include <ros_gz_bridge/convert.hpp>

#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/basic_logical_camera_image.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>


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

      void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);
  
    private:
      std::unique_ptr<AriacLogicalCameraPluginPrivate> impl_;
  };
}

#endif