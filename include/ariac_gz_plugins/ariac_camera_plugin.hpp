#ifndef ARIAC_CAMERA_PLUGIN_HPP
#define ARIAC_CAMERA_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>

#include <ros_gz_bridge/convert.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace ariac_sensors{
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

      void OnImage(const gz::msgs::Image & _gz_msg);
    
    private:
      std::shared_ptr<gz::transport::Node> gz_node_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  };
}

#endif