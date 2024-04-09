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
      /// The gz topic onto which Logical camera Image messages are published
      std::string gztopic_;
      
      /// gz node to subscribe to gztopic_ & callback the `OnNewLogicalFrame` method
      std::shared_ptr<gz::transport::Node> gz_node_;
      

      /// Type of the logical camera (Basic or Advanced)
      std::string camera_type_;

      /// Node for ros communication
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;
      bool publish_sensor_data_;

      /// List of models that the logical camera will publish
      std::vector<std::string> parts_to_publish_;
      std::vector<std::string> colors_;
      std::map<std::string, int> part_types_;
      std::map<std::string, int> part_colors_;

      /// Ariac messages & ROS Publishsers for Basic & Advanced logical camera images
      rclcpp::Publisher<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr basic_image_pub_;
      std::shared_ptr<ariac_msgs::msg::BasicLogicalCameraImage> basic_image_msg_;
      rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr advanced_image_pub_;
      std::shared_ptr<ariac_msgs::msg::AdvancedLogicalCameraImage> advanced_image_msg_;

      /// Sensor Health Subscription
      rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;
      void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);

      /// Publish latest logical camera data to ROS
      void OnNewLogicalFrame(const gz::msgs::LogicalCameraImage & _gz_msg);
  };
}

#endif