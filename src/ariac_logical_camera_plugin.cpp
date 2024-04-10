#include <gz/plugin/Register.hh>

#include "ariac_gz_plugins/ariac_logical_camera_plugin.hpp"

namespace ariac_sensors{
  AriacLogicalCameraPlugin::AriacLogicalCameraPlugin() {}
  AriacLogicalCameraPlugin::~AriacLogicalCameraPlugin() {}

  void AriacLogicalCameraPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) {

    // Set up ros publisher
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    ros_node_ = rclcpp::Node::make_shared("ariac_logical_camera_plugin_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ros_node_);
    auto spin = [this]()
      {
        while (rclcpp::ok()) {
          executor_->spin_once();
        }
      };
    thread_executor_spin_ = std::thread(spin);

    publish_sensor_data_ = false;
    camera_type_ = _sdf->Get<std::string>("camera_type");    

    if (camera_type_ == "basic") {
      basic_image_pub_ = ros_node_->create_publisher<ariac_msgs::msg::BasicLogicalCameraImage>(
      _sdf->Get<std::string>("rostopic"), rclcpp::SensorDataQoS());
      basic_image_msg_ = std::make_shared<ariac_msgs::msg::BasicLogicalCameraImage>();
    } else if (camera_type_ == "advanced") {
      advanced_image_pub_ = ros_node_->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      _sdf->Get<std::string>("rostopic"), rclcpp::SensorDataQoS());
      advanced_image_msg_ = std::make_shared<ariac_msgs::msg::AdvancedLogicalCameraImage>();
    }

    // Set list of models to publish
    parts_to_publish_ = {"pump", "battery", "regulator", "sensor"};
    colors_ = {"red", "green", "blue", "orange", "purple"};

    part_types_ = {
      {"battery", ariac_msgs::msg::Part::BATTERY},
      {"pump", ariac_msgs::msg::Part::PUMP},
      {"regulator", ariac_msgs::msg::Part::REGULATOR},
      {"sensor", ariac_msgs::msg::Part::SENSOR},
    };

    part_colors_ = {
      {"red", ariac_msgs::msg::Part::RED},
      {"green", ariac_msgs::msg::Part::GREEN},
      {"blue", ariac_msgs::msg::Part::BLUE},
      {"purple", ariac_msgs::msg::Part::PURPLE},
      {"orange", ariac_msgs::msg::Part::ORANGE},
    };

    // Subscribe to sensor health topic
    sensor_health_sub_ = ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 
      10, std::bind(&AriacLogicalCameraPlugin::SensorHealthCallback, this, std::placeholders::_1));

    // Set up gz subscriber
    gz_node_ = std::make_shared<gz::transport::Node>();
    gztopic_ = _sdf->Get<std::string>("gztopic");
    gz_node_->Subscribe(gztopic_, &AriacLogicalCameraPlugin::OnNewLogicalFrame, this);

  }

  void AriacLogicalCameraPlugin::OnNewLogicalFrame(const gz::msgs::LogicalCameraImage &_gz_msg) {

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

      basic_image_msg_->header.frame_id = _gz_msg.header().data(0).value(0);
      basic_image_msg_->header.stamp.sec = _gz_msg.header().stamp().sec();
      basic_image_msg_->header.stamp.nanosec = _gz_msg.header().stamp().nsec();

      basic_image_pub_->publish(*basic_image_msg_);
    }
    else if (camera_type_ == "advanced") {
      advanced_image_msg_->sensor_pose = sensor_pose;

      advanced_image_msg_->part_poses = parts;
      advanced_image_msg_->tray_poses = trays;

      advanced_image_msg_->header.frame_id = _gz_msg.header().data(0).value(0);
      advanced_image_msg_->header.stamp.sec = _gz_msg.header().stamp().sec();
      advanced_image_msg_->header.stamp.nanosec = _gz_msg.header().stamp().nsec();

      advanced_image_pub_->publish(*advanced_image_msg_);
    }
  }

  void AriacLogicalCameraPlugin::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg) {
    publish_sensor_data_ = msg->logical_camera;
  }

}

GZ_ADD_PLUGIN(
  ariac_sensors::AriacLogicalCameraPlugin,
  gz::sim::System,
  ariac_sensors::AriacLogicalCameraPlugin::ISystemConfigure
)
