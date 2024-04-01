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
// #include <sensor_msgs/msg/camera_info.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <ariac_msgs/msg/sensors.hpp>

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

      void OnNewImageFrame(const gz::msgs::Image & _gz_msg);
      void OnNewDepthFrame(const gz::msgs::Image & _gz_msg);

      void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);
    
    private:
      std::string camera_type_;
      std::shared_ptr<gz::transport::Node> gz_node_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
      sensor_msgs::msg::CameraInfo camera_info_msg_;
      // std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
      std::string gz_topic_;
      std::string gz_topic_depth_;
      std::string cam_info_gz_topic_;

      bool publish_sensor_data_;
      sensor_msgs::msg::Image ros_msg;
      rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;

      void FillCameraInfoMsg(const gz::msgs::CameraInfo &_info_msg);
  };
}

#endif