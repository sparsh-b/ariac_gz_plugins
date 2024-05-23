#ifndef ARIAC_RAY_PLUGIN_HPP
#define ARIAC_RAY_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>

#include <ariac_msgs/msg/sensors.hpp>
#include <ariac_msgs/msg/break_beam_status.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>


namespace ariac_sensors{
  /// Plugin to
  class AriacRayPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure
  {
    public:
      AriacRayPlugin();
      ~AriacRayPlugin();

      void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) override;

    private:
      /// gz node to subscribe to gztopic_ & callback the `OnNewLogicalFrame` method
      std::shared_ptr<gz::transport::Node> gz_node_;      

      /// Type of the sensor
      std::string sensor_type_;
      std::string sensor_name_;
      std::string frame_name_;


      /// Node for ros communication
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;
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
}


#endif