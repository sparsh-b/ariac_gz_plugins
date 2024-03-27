#include <gz/plugin/Register.hh>
#include <typeinfo>


#include "ariac_camera_plugin/ariac_camera_plugin.hpp"

namespace ariac_sensors{

  AriacCameraPlugin::AriacCameraPlugin() {}

  void AriacCameraPlugin::Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) 
  {
    // Set up ros publisher
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    ros_node_ = rclcpp::Node::make_shared("rgb_camera_plugin_node");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ros_node_);
    
    auto spin = [this]()
      {
        while (rclcpp::ok()) {
          executor_->spin_once();
        }
      };
  
    thread_executor_spin_ = std::thread(spin);

    image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>(
      _sdf->Get<std::string>("ros_topic"), 10);

    // Set up gz subscriber
    gz_node_ = std::make_shared<gz::transport::Node>();

    gz_node_->Subscribe(
      _sdf->Get<std::string>("gz_topic"), &AriacCameraPlugin::OnImage, this);

  }

  void AriacCameraPlugin::OnImage(const gz::msgs::Image & _gz_msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
    // gzmsg << "callback received" << std::endl;
    image_pub_->publish(ros_msg);
  }

}

GZ_ADD_PLUGIN(
  ariac_sensors::AriacCameraPlugin,
  gz::sim::System,
  ariac_sensors::AriacCameraPlugin::ISystemConfigure
)
