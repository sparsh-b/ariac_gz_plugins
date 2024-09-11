#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/logical_camera_image.pb.h>
#include <ros_gz_bridge/convert.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/basic_logical_camera_image.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>

#include "ariac_gz_plugins/ariac_logical_camera_plugin.hpp"

namespace ariac_sensors{

  class AriacLogicalCameraPluginPrivate {
    public:
      /// gz node to subscribe to gztopic_ & callback the `OnNewLogicalFrame` method
      std::shared_ptr<gz::transport::Node> gz_node_;      

      /// Type of the logical camera (Basic or Advanced)
      std::string camera_type_;
      std::string frame_name_;

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

      /// Publish latest logical camera data to ROS
      void OnNewLogicalFrame(const gz::msgs::LogicalCameraImage & _gz_msg);

      void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);
  };

  AriacLogicalCameraPlugin::AriacLogicalCameraPlugin() : impl_(std::make_unique<AriacLogicalCameraPluginPrivate>()) {}
  AriacLogicalCameraPlugin::~AriacLogicalCameraPlugin() {}

  void AriacLogicalCameraPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) {

    // Set up ros publisher
    std::vector<std::string> arguments = {"--ros-args"};
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(ament_index_cpp::get_package_share_directory("aprs_description")+"/config/robot_controllers.yaml");
    std::vector<const char *> argv;
    for (const auto & arg : arguments) {
      argv.push_back(reinterpret_cast<const char *>(arg.data()));
    }
    
    if (!rclcpp::ok()){
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }

    impl_->ros_node_ = rclcpp::Node::make_shared(_sdf->Get<std::string>("camera_name")+"_node");
    impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    impl_->executor_->add_node(impl_->ros_node_);
    auto spin = [this]()
      {
        while (rclcpp::ok()) {
          impl_->executor_->spin_once();
        }
      };
    impl_->thread_executor_spin_ = std::thread(spin);

    impl_->publish_sensor_data_ = true;
    impl_->camera_type_ = _sdf->Get<std::string>("camera_type");
    impl_->frame_name_ = _sdf->Get<std::string>("frame_name");

    if (impl_->camera_type_ == "basic") {
      impl_->basic_image_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BasicLogicalCameraImage>(
      _sdf->Get<std::string>("rostopic"), rclcpp::SensorDataQoS());
      impl_->basic_image_msg_ = std::make_shared<ariac_msgs::msg::BasicLogicalCameraImage>();
    } else if (impl_->camera_type_ == "advanced") {
      impl_->advanced_image_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      _sdf->Get<std::string>("rostopic"), rclcpp::SensorDataQoS());
      impl_->advanced_image_msg_ = std::make_shared<ariac_msgs::msg::AdvancedLogicalCameraImage>();
    }

    // Set list of models to publish
    impl_->parts_to_publish_ = {"pump", "battery", "regulator", "sensor"};
    impl_->colors_ = {"red", "green", "blue", "orange", "purple"};

    impl_->part_types_ = {
      {"battery", ariac_msgs::msg::Part::BATTERY},
      {"pump", ariac_msgs::msg::Part::PUMP},
      {"regulator", ariac_msgs::msg::Part::REGULATOR},
      {"sensor", ariac_msgs::msg::Part::SENSOR},
    };

    impl_->part_colors_ = {
      {"red", ariac_msgs::msg::Part::RED},
      {"green", ariac_msgs::msg::Part::GREEN},
      {"blue", ariac_msgs::msg::Part::BLUE},
      {"purple", ariac_msgs::msg::Part::PURPLE},
      {"orange", ariac_msgs::msg::Part::ORANGE},
    };

    // Subscribe to sensor health topic
    impl_->sensor_health_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 
      10, std::bind(&AriacLogicalCameraPluginPrivate::SensorHealthCallback, impl_.get(), std::placeholders::_1));

    // Set up gz subscriber
    impl_->gz_node_ = std::make_shared<gz::transport::Node>();
    std::string gztopic_ = _sdf->Get<std::string>("gztopic");

    RCLCPP_INFO_STREAM(impl_->ros_node_->get_logger(), "GZ_TOPIC_NAME: " << gztopic_);

    impl_->gz_node_->Subscribe(gztopic_, &AriacLogicalCameraPluginPrivate::OnNewLogicalFrame, impl_.get());
  }

  void AriacLogicalCameraPluginPrivate::OnNewLogicalFrame(const gz::msgs::LogicalCameraImage &_gz_msg) {    
    if (!publish_sensor_data_) {
      return;
    }

    geometry_msgs::msg::Pose sensor_pose;
    ros_gz_bridge::convert_gz_to_ros(_gz_msg.pose(), sensor_pose);

    std::vector<ariac_msgs::msg::PartPose> parts;
    std::vector<ariac_msgs::msg::KitTrayPose> trays;

    for (int i = 0; i < _gz_msg.model_size(); i++) {
      const auto & lc_model = _gz_msg.model(i);
      std::string name = lc_model.name();

      if (name.find("kit_tray") != std::string::npos) {
          ariac_msgs::msg::KitTrayPose kit_tray;

          std::string id_string = name.substr(9, 2);
          kit_tray.id = std::stoi(id_string);
          ros_gz_bridge::convert_gz_to_ros(lc_model.pose(), kit_tray.pose);

          trays.push_back(kit_tray);
          continue;
      }

      for(std::string part_type : parts_to_publish_){
        if (name.find(part_type) != std::string::npos) {
          ariac_msgs::msg::PartPose part;

          part.part.type = part_types_[part_type];
          
          for(std::string color : colors_){
            if (name.find(color) != std::string::npos) {
              part.part.color = part_colors_[color];
            }
          }
          ros_gz_bridge::convert_gz_to_ros(lc_model.pose(), part.pose);
          parts.push_back(part);

          break;
        }
      }
    }

    if (camera_type_ == "basic") {
      basic_image_msg_->sensor_pose = sensor_pose;

      basic_image_msg_->part_poses.clear();
      basic_image_msg_->tray_poses.clear();

      for (ariac_msgs::msg::PartPose &part : parts) {
        basic_image_msg_->part_poses.push_back(part.pose);
      }

      for (ariac_msgs::msg::KitTrayPose &tray : trays) {
        basic_image_msg_->tray_poses.push_back(tray.pose);
      }

      basic_image_msg_->header.frame_id = frame_name_;
      basic_image_msg_->header.stamp.sec = _gz_msg.header().stamp().sec();
      basic_image_msg_->header.stamp.nanosec = _gz_msg.header().stamp().nsec();

      basic_image_pub_->publish(*basic_image_msg_);
    }
    else if (camera_type_ == "advanced") {
      advanced_image_msg_->sensor_pose = sensor_pose;

      advanced_image_msg_->part_poses = parts;
      advanced_image_msg_->tray_poses = trays;

      advanced_image_msg_->header.frame_id = frame_name_;
      advanced_image_msg_->header.stamp.sec = _gz_msg.header().stamp().sec();
      advanced_image_msg_->header.stamp.nanosec = _gz_msg.header().stamp().nsec();

      advanced_image_pub_->publish(*advanced_image_msg_);
    }
  }

  void AriacLogicalCameraPluginPrivate::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg) {
    publish_sensor_data_ = msg->logical_camera;
  }

}

GZ_ADD_PLUGIN(
  ariac_sensors::AriacLogicalCameraPlugin,
  gz::sim::System,
  ariac_sensors::AriacLogicalCameraPlugin::ISystemConfigure
)