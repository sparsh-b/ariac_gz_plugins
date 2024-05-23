#include <gz/plugin/Register.hh>
#include <ros_gz_bridge/convert.hpp>

#include "ariac_gz_plugins/ariac_ray_plugin.hpp"

namespace ariac_sensors {
  AriacRayPlugin::AriacRayPlugin() {}
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
    this->ros_node_ = rclcpp::Node::make_shared("ariac_camera_plugin_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(this->ros_node_);
    auto spin = [this]()
      {
        while (rclcpp::ok()) {
          executor_->spin_once();
        }
      };
    thread_executor_spin_ = std::thread(spin);

    this->sensor_type_ = _sdf->Get<std::string>("sensor_type");
    this->sensor_name_ = _sdf->Get<std::string>("sensor_name");
    this->frame_name_ = _sdf->Get<std::string>("frame_name");

    if (this->sensor_type_ == "break_beam")
    {  
      this->status_pub_ = this->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(
        "ariac/sensors/" + this->sensor_name_ + "/status", rclcpp::SensorDataQoS());
      this->change_pub_ = this->ros_node_->create_publisher<ariac_msgs::msg::BreakBeamStatus>(
        "ariac/sensors/" + this->sensor_name_ + "/change", rclcpp::SensorDataQoS());
    }
    else if (this->sensor_type_ == "proximity")
    {
      this->range_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Range>(
        "ariac/sensors/" + this->sensor_name_ + "/scan", rclcpp::SensorDataQoS());
    }
    else if (this->sensor_type_ == "laser_profiler")
    {  
      this->laser_scan_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "ariac/sensors/" + this->sensor_name_ + "/scan", rclcpp::SensorDataQoS());
    }
    else if (this->sensor_type_ == "lidar")
    {      
      this->point_cloud_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "ariac/sensors/" + this->sensor_name_ + "/scan", rclcpp::SensorDataQoS());
    }
    else
    {
      RCLCPP_ERROR(this->ros_node_->get_logger(), "Sensor type not valid");
      return;
    }

    gz_node_ = std::make_shared<gz::transport::Node>();
    std::string sensor_topic_ = _sdf->Get<std::string>("gz_topic");
    gz_node_->Subscribe(sensor_topic_, &AriacRayPlugin::ReadLaserScan, this);
    gz_node_->Subscribe(sensor_topic_+"/points", &AriacRayPlugin::ReadPointCloudPacked, this);


    // Subscribe to sensor health topic
    sensor_health_sub_ = this->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health",
      10, std::bind(&AriacRayPlugin::SensorHealthCallback, this, std::placeholders::_1));
  }


  void AriacRayPlugin::ReadLaserScan(const gz::msgs::LaserScan &_gz_msg) {
    if (this->sensor_type_ == "proximity") {
      this->PublishRange(_gz_msg);
    } else if (this->sensor_type_ == "laser_profiler") {
      this->PublishLaserScan(_gz_msg);
    } else if (this->sensor_type_ == "break_beam") {
      this->PublishBreakBeamStatus(_gz_msg);
    }
  }

  void AriacRayPlugin::ReadPointCloudPacked(const gz::msgs::PointCloudPacked &_gz_msg) {
    if (this->sensor_type_ == "lidar") {
      this->PublishPointCloud(_gz_msg);
    }
  }

  void AriacRayPlugin::PublishLaserScan(const gz::msgs::LaserScan &_gz_msg) {
    if (this->sensor_health_.laser_profiler) {
      sensor_msgs::msg::LaserScan ls;
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ls);
      ls.header.frame_id = this->frame_name_;
      laser_scan_pub_->publish(ls);
    }
  }

  void AriacRayPlugin::PublishBreakBeamStatus(const gz::msgs::LaserScan &_gz_msg) {
    if (this->sensor_health_.break_beam) {
      sensor_msgs::msg::LaserScan ls;
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ls);
      
    }
  }

  void AriacRayPlugin::PublishRange(const gz::msgs::LaserScan &_gz_msg) {
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


  void AriacRayPlugin::PublishPointCloud(const gz::msgs::PointCloudPacked &_gz_msg) {
    if (this->sensor_health_.lidar) {
      sensor_msgs::msg::PointCloud2 ros_msg;
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
      ros_msg.header.frame_id = this->frame_name_;
      this->point_cloud_pub_->publish(ros_msg);
    }
  }


  void AriacRayPlugin::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg){
    this->sensor_health_ = *msg;
  }

}


GZ_ADD_PLUGIN(
  ariac_sensors::AriacRayPlugin,
  gz::sim::System,
  ariac_sensors::AriacRayPlugin::ISystemConfigure
)

      // int num_intensities = _gz_msg.intensities_size();
      // int num_ranges = _gz_msg.ranges_size();
      // assert (num_ranges == num_intensities);

      // sensor_msgs::msg::PointCloud pc;
      // pc.points = std::vector<geometry_msgs::msg::Point32>{};
      // sensor_msgs::msg::ChannelFloat32 cf32;
      // pc.channels = std::vector<sensor_msgs::msg::ChannelFloat32> {cf32};
      // pc.channels[0].name = "intensities";
      // pc.channels[0].values = std::vector<float>{};

      // for (int i=0; i<num_intensities; i++) {
      //   pc.points.push_back(_gz_msg.ranges(i));
      //   pc.channels[0].values.push_back(_gz_msg.intensities(i));
      // }