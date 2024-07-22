#include <rclcpp/rclcpp.hpp>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>


#include "ariac_gz_plugins/vacuum_gripper_plugin.hpp"

namespace ariac_plugins {

class VacuumGripperPluginPrivate {
  public:
      std::shared_ptr<gz::transport::Node> gz_node_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;
};

VacuumGripperPlugin::VacuumGripperPlugin() : impl_(std::make_unique<VacuumGripperPluginPrivate>()) {}

VacuumGripperPlugin::~VacuumGripperPlugin() {}

void VacuumGripperPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr)
{
  // Set up ros publisher
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  impl_->ros_node_ = rclcpp::Node::make_shared("ariac_camera_plugin_node");
  impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  impl_->executor_->add_node(impl_->ros_node_);
  auto spin = [this]()
    {
      while (rclcpp::ok()) {
        impl_->executor_->spin_once();
      }
    };
  impl_->thread_executor_spin_ = std::thread(spin);

  
}

}