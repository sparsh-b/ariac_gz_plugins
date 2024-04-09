#include <gz/plugin/Register.hh>
#include <typeinfo>

#include <ariac_gz_plugins/ariac_camera_plugin.hpp>

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
    ros_node_ = rclcpp::Node::make_shared("ariac_camera_plugin_node");
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
    image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>(
      _sdf->Get<std::string>("rgb_img_ros_topic"), 10);
    depth_image_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>(
      _sdf->Get<std::string>("depth_img_ros_topic"), 10);

    camera_type_ = _sdf->Get<std::string>("camera_type");
    if (camera_type_ == "rgb") {
      gz_topic_ = _sdf->Get<std::string>("gz_topic");
      cam_info_gz_topic_ = _sdf->Get<std::string>("cam_info_gz_topic");
    } else if (camera_type_ == "rgbd") {
      gz_topic_ = _sdf->Get<std::string>("gz_topic") + "/image";
      gz_topic_depth_ = _sdf->Get<std::string>("gz_topic") + "/depth_image]";
      cam_info_gz_topic_ = _sdf->Get<std::string>("gz_topic") + "/camera_info";
    }

    // Set up gz subscriber
    gz_node_ = std::make_shared<gz::transport::Node>();
    gz_node_->Subscribe(gz_topic_, &AriacCameraPlugin::OnNewImageFrame, this);
    gz_node_->Subscribe(cam_info_gz_topic_, &AriacCameraPlugin::FillCameraInfoMsg, this);
    if (camera_type_ == "rgbd") {
      gz_node_->Subscribe(gz_topic_depth_, &AriacCameraPlugin::OnNewDepthFrame, this);
    }

    // Subscribe to sensor health topic
    sensor_health_sub_ = ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 
      10, std::bind(&AriacCameraPlugin::SensorHealthCallback, this, std::placeholders::_1));
    
    camera_info_pub_ = ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      _sdf->Get<std::string>("cam_info_ros_topic"), 10);
  }


  void AriacCameraPlugin::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg) {
    publish_sensor_data_ = msg->camera;
  }


  void AriacCameraPlugin::OnNewImageFrame(const gz::msgs::Image & _gz_msg)
  {
    if (publish_sensor_data_) {
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
      image_pub_->publish(ros_msg);

      camera_info_msg_.header.stamp.sec = _gz_msg.header().stamp().sec();
      camera_info_msg_.header.stamp.nanosec = _gz_msg.header().stamp().nsec();      
      camera_info_pub_->publish(camera_info_msg_);
    }
  }

  void AriacCameraPlugin::OnNewDepthFrame(const gz::msgs::Image & _gz_msg) {
    if (publish_sensor_data_) {
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
      depth_image_pub_->publish(ros_msg);
    }
  }

  void AriacCameraPlugin::FillCameraInfoMsg(const gz::msgs::CameraInfo &_info_msg) {
    // Assuming a particular camera's CameraInfo (except timestamp) doesn't change once it is spawned, 
    // fill the ROS msg only once at startup (but publish it along with every new image frame).
    
    camera_info_msg_.header.frame_id = _info_msg.header().data(0).value(0);
    // camera_info_msg_.header's time is updated while publishing

    camera_info_msg_.height = _info_msg.height();
    camera_info_msg_.width = _info_msg.width();

    if (_info_msg.distortion().model() == 0) {
      camera_info_msg_.distortion_model = "plumb_bob";
    } else if (_info_msg.distortion().model() == 1) {
      camera_info_msg_.distortion_model = "rational_polynomial";
    } else if (_info_msg.distortion().model() == 2) {
      camera_info_msg_.distortion_model = "equidistant";
    }
    camera_info_msg_.d.resize(5);
    camera_info_msg_.d[0] = _info_msg.distortion().k(0);
    camera_info_msg_.d[1] = _info_msg.distortion().k(1);
    camera_info_msg_.d[2] = _info_msg.distortion().k(2);
    camera_info_msg_.d[3] = _info_msg.distortion().k(3);
    camera_info_msg_.d[4] = _info_msg.distortion().k(4);

    camera_info_msg_.k[0] = _info_msg.intrinsics().k(0);
    camera_info_msg_.k[1] = _info_msg.intrinsics().k(1);
    camera_info_msg_.k[2] = _info_msg.intrinsics().k(2);
    camera_info_msg_.k[3] = _info_msg.intrinsics().k(3);
    camera_info_msg_.k[4] = _info_msg.intrinsics().k(4);
    camera_info_msg_.k[5] = _info_msg.intrinsics().k(5);
    camera_info_msg_.k[6] = _info_msg.intrinsics().k(6);
    camera_info_msg_.k[7] = _info_msg.intrinsics().k(7);
    camera_info_msg_.k[8] = _info_msg.intrinsics().k(8);

    camera_info_msg_.r[0] = _info_msg.rectification_matrix(0);
    camera_info_msg_.r[1] = _info_msg.rectification_matrix(1);
    camera_info_msg_.r[2] = _info_msg.rectification_matrix(2);
    camera_info_msg_.r[3] = _info_msg.rectification_matrix(3);
    camera_info_msg_.r[4] = _info_msg.rectification_matrix(4);
    camera_info_msg_.r[5] = _info_msg.rectification_matrix(5);
    camera_info_msg_.r[6] = _info_msg.rectification_matrix(6);
    camera_info_msg_.r[7] = _info_msg.rectification_matrix(7);
    camera_info_msg_.r[8] = _info_msg.rectification_matrix(8);

    camera_info_msg_.p[0] = _info_msg.projection().p(0);
    camera_info_msg_.p[1] = _info_msg.projection().p(1);
    camera_info_msg_.p[2] = _info_msg.projection().p(2);
    camera_info_msg_.p[3] = _info_msg.projection().p(3);
    camera_info_msg_.p[4] = _info_msg.projection().p(4);
    camera_info_msg_.p[5] = _info_msg.projection().p(5);
    camera_info_msg_.p[6] = _info_msg.projection().p(6);
    camera_info_msg_.p[7] = _info_msg.projection().p(7);
    camera_info_msg_.p[8] = _info_msg.projection().p(8);
    camera_info_msg_.p[9] = _info_msg.projection().p(9);
    camera_info_msg_.p[10] = _info_msg.projection().p(10);
    camera_info_msg_.p[11] = _info_msg.projection().p(11);

    // After CameraIfo message got filled once, unsubscribe the callback to prevent future callbacks
    // As an attempt to make the plugin lighter.
    gz_node_->Unsubscribe(cam_info_gz_topic_);
  }
}

GZ_ADD_PLUGIN(
  ariac_sensors::AriacCameraPlugin,
  gz::sim::System,
  ariac_sensors::AriacCameraPlugin::ISystemConfigure
)
