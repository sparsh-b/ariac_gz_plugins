#include <gz/plugin/Register.hh>
#include <ros_gz_bridge/convert.hpp>

#include "ariac_gz_plugins/ariac_ray_plugin.hpp"

namespace ariac_sensors {

  class AriacRayPluginPrivate {
    public:
      /// gz node to subscribe to gztopic_ & callback the `OnNewLogicalFrame` method
      std::shared_ptr<gz::transport::Node> gz_node_;      

      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;

      /// Type of the sensor
      std::string sensor_type_;
      std::string sensor_name_;
      std::string frame_name_;


      /// Node for ros communication
      rclcpp::Node::SharedPtr ros_node_;
      ariac_msgs::msg::Sensors sensor_health_;

      ariac_msgs::msg::BreakBeamStatus status_msg_;
      rclcpp::Publisher<ariac_msgs::msg::BreakBeamStatus>::SharedPtr status_pub_;
      rclcpp::Publisher<ariac_msgs::msg::BreakBeamStatus>::SharedPtr change_pub_;

      rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;


      /// Sensor Health Subscription
      rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;
      void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);

      void ReadLaserScan(const gz::msgs::LaserScan &_gz_msg);
      void ReadPointCloudPacked(const gz::msgs::PointCloudPacked &_gz_msg);

      void PublishRange(const gz::msgs::LaserScan &_gz_msg);
      void PublishLaserScan(const gz::msgs::LaserScan &_gz_msg);
      void PublishPointCloud(const gz::msgs::PointCloudPacked &_gz_msg);
      void PublishBreakBeamStatus(const gz::msgs::LaserScan &_gz_msg);
  };

  AriacRayPlugin::AriacRayPlugin() : impl_(std::make_unique<AriacRayPluginPrivate>()) {}
  
  AriacRayPlugin::~AriacRayPlugin() {}

  // TODO: See if you can organize the code better
  void AriacRayPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) {

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

    impl_->sensor_type_ = _sdf->Get<std::string>("sensor_type");
    impl_->sensor_name_ = _sdf->Get<std::string>("sensor_name");
    impl_->frame_name_ = _sdf->Get<std::string>("frame_name");

    if (impl_->sensor_type_ == "break_beam")
    {  
      impl_->status_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(
        "ariac/sensors/" + impl_->sensor_name_ + "/status", rclcpp::SensorDataQoS());
      impl_->change_pub_ = impl_->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(
        "ariac/sensors/" + impl_->sensor_name_ + "/change", rclcpp::SensorDataQoS());
    }
    else if (impl_->sensor_type_ == "proximity")
    {
      impl_->range_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Range>(
        "ariac/sensors/" + impl_->sensor_name_ + "/scan", rclcpp::SensorDataQoS());
    }
    else if (impl_->sensor_type_ == "laser_profiler")
    {  
      impl_->laser_scan_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "ariac/sensors/" + impl_->sensor_name_ + "/scan", rclcpp::SensorDataQoS());
    }
    else if (impl_->sensor_type_ == "lidar")
    {      
      impl_->point_cloud_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "ariac/sensors/" + impl_->sensor_name_ + "/scan", rclcpp::SensorDataQoS());
    }
    else
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Sensor type not valid");
      return;
    }

    impl_->gz_node_ = std::make_shared<gz::transport::Node>();
    std::string sensor_topic_ = _sdf->Get<std::string>("gz_topic");
    impl_->gz_node_->Subscribe(sensor_topic_, &AriacRayPluginPrivate::ReadLaserScan, impl_.get());
    impl_->gz_node_->Subscribe(sensor_topic_+"/points", &AriacRayPluginPrivate::ReadPointCloudPacked, impl_.get());


    // Subscribe to sensor health topic
    impl_->sensor_health_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health",
      10, std::bind(&AriacRayPluginPrivate::SensorHealthCallback, impl_.get(), std::placeholders::_1));
  }


  void AriacRayPluginPrivate::ReadLaserScan(const gz::msgs::LaserScan &_gz_msg) {
    if (this->sensor_type_ == "proximity") {
      this->PublishRange(_gz_msg);
    } else if (this->sensor_type_ == "laser_profiler") {
      this->PublishLaserScan(_gz_msg);
    } else if (this->sensor_type_ == "break_beam") {
      this->PublishBreakBeamStatus(_gz_msg);
    } else {
      RCLCPP_ERROR(this->ros_node_->get_logger(), "Sensor type not valid");
      return;
    }
  }

  void AriacRayPluginPrivate::ReadPointCloudPacked(const gz::msgs::PointCloudPacked &_gz_msg) {
    if (this->sensor_type_ == "lidar") {
      this->PublishPointCloud(_gz_msg);
    }
  }

  void AriacRayPluginPrivate::PublishLaserScan(const gz::msgs::LaserScan &_gz_msg) {
    if (this->sensor_health_.laser_profiler) {
      sensor_msgs::msg::LaserScan ls;
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ls);
      ls.header.frame_id = this->frame_name_;
      laser_scan_pub_->publish(ls);
    }
  }

  void AriacRayPluginPrivate::PublishBreakBeamStatus(const gz::msgs::LaserScan &_gz_msg) {
    if (1){//this->sensor_health_.break_beam) {
      sensor_msgs::msg::LaserScan ls;
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ls);

      this->status_msg_.header.frame_id = this->frame_name_;
      this->status_msg_.header.stamp = ls.header.stamp;

      bool object_detected = false;
      bool publish_change = false;

      for(float distance : ls.ranges) {
        if (distance > 0.0 && distance < 1.0) {
            object_detected = true;
            break;
        }
      }

      if (this->status_msg_.object_detected != object_detected) {
        publish_change = true;
      }

      this->status_msg_.object_detected = object_detected;

      // Publish output
      if (publish_change) {
        this->change_pub_->publish(this->status_msg_);
      }
      this->status_pub_->publish(this->status_msg_);
    }
  }

  void AriacRayPluginPrivate::PublishRange(const gz::msgs::LaserScan &_gz_msg) {
    if (this->sensor_health_.proximity) {
      
      // Convert Laser scan to range
      sensor_msgs::msg::Range range_msg;
      auto range_max = _gz_msg.range_max();
      auto range_min = _gz_msg.range_min();
      auto fov = _gz_msg.angle_max() - _gz_msg.angle_min();
      double range = 0;
      for (int i=0; i<_gz_msg.ranges_size(); i++) {range += _gz_msg.ranges(i);}
      range /= _gz_msg.ranges_size();

      range_msg.max_range = range_max;      
      range_msg.min_range = range_min;
      range_msg.range = range;
      range_msg.field_of_view = fov;
      range_msg.header.stamp.sec = _gz_msg.header().stamp().sec();
      range_msg.header.stamp.nanosec = _gz_msg.header().stamp().nsec();
      range_msg.header.frame_id = this->frame_name_;
      range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      this->range_pub_->publish(range_msg);
    }
  }


  void AriacRayPluginPrivate::PublishPointCloud(const gz::msgs::PointCloudPacked &_gz_msg) {
    if (this->sensor_health_.lidar) {
      sensor_msgs::msg::PointCloud2 ros_msg;
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
      ros_msg.header.frame_id = this->frame_name_;
      this->point_cloud_pub_->publish(ros_msg);
    }
  }

  void AriacRayPluginPrivate::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg){
    this->sensor_health_ = *msg;
  }

}


GZ_ADD_PLUGIN(
  ariac_sensors::AriacRayPlugin,
  gz::sim::System,
  ariac_sensors::AriacRayPlugin::ISystemConfigure
)
