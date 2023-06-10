#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <boost/asio.hpp>
#include "geometry_msgs/PoseStamped.h"
// #include <tf2/LinearMath/Quaternion.h>
//  #include "sensor_msgs/MagneticField.h"
//  #include "sensor_msgs/BatteryState.h"
//  #include "sensor_msgs/NavSatFix.h"
//  #include "sensor_msgs/NavSatStatus.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class SensorLogBridge
{
public:
  SensorLogBridge(ros::NodeHandle nh)
      : nh_(nh), socket_(io_service_)
  {
    // Load parameters from the parameter server
    nh_.param<int>("port", udp_port_, 12000);
    nh_.param<int>("buffer_length", buffer_length_, 1024);
    nh_.param<std::string>("device_id", device_id_, "iphone");
    nh_.param<std::string>("frame_id", frame_id_, "iphone_link");
    nh_.param<std::string>("imu_topic_name", imu_topic_name_, "/iphone/imu");
    nh_.param<std::string>("pose_topic_name", pose_topic_name_, "/iphone/pose");
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_name_, 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_name_, 10);

    // Open the socket
    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::udp::v4(), udp_port_);
    socket_.open(endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(endpoint);
    if (socket_.is_open())
    {
      ROS_INFO("Socket bind successful");
    }
    else
    {
      ROS_ERROR("Socket bind failed");
    }

    // Start receiving UDP packets
    ROS_INFO("start recieve");
    startReceive();
  }

  void startReceive()
  {
    // Clear the receive buffer
    receive_buffer_.resize(buffer_length_);
    boost::system::error_code error;
    std::size_t bytes_transferred = socket_.receive_from(boost::asio::buffer(receive_buffer_), remote_endpoint_, 0, error);

    // Check for errors
    if (!error)
    {
      // Parse received data and publish IMU information
      std::string received_data(receive_buffer_.begin(), receive_buffer_.begin() + bytes_transferred);
      json_buffer_ += received_data;
      if (receive_buffer_[bytes_transferred - 2] == '}')
      {
        parseAndPublish(json_buffer_);
        json_buffer_.clear();
      }
    }
    else
    {
      ROS_ERROR("UDP receive error: %s", error.message().c_str());
    }
    ros::spinOnce();
    // Start receiving process again
    startReceive();
  }

  void parseAndPublish(std::string json)
  {
    boost::property_tree::ptree pt;
    std::istringstream iss(json);
    boost::property_tree::read_json(iss, pt);

    // std::string loggingTime = pt.get<std::string>("loggingTime");
    int loggingSample = pt.get<int>("loggingSample");
    double motionUserAccelerationX = pt.get<double>("motionUserAccelerationX");
    double motionUserAccelerationY = pt.get<double>("motionUserAccelerationY");
    double motionUserAccelerationZ = pt.get<double>("motionUserAccelerationZ");
    double motionRotationRateX = pt.get<double>("motionRotationRateX");
    double motionRotationRateY = pt.get<double>("motionRotationRateY");
    double motionRotationRateZ = pt.get<double>("motionRotationRateZ");
    double motionQuaternionW = pt.get<double>("motionQuaternionW");
    double motionQuaternionX = pt.get<double>("motionQuaternionX");
    double motionQuaternionY = pt.get<double>("motionQuaternionY");
    double motionQuaternionZ = pt.get<double>("motionQuaternionZ");

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.seq = loggingSample;
    imu_msg.header.frame_id = frame_id_;
    imu_msg.orientation.w = motionQuaternionW;
    imu_msg.orientation.x = motionQuaternionX;
    imu_msg.orientation.y = motionQuaternionY;
    imu_msg.orientation.z = motionQuaternionZ;
    imu_msg.orientation_covariance = {0.00872665, 0.0, 0.0, 0.0, 0.00872665, 0.0, 0.0, 0.0, 0.00872665};
    imu_msg.angular_velocity.x = motionRotationRateX;
    imu_msg.angular_velocity.y = motionRotationRateY;
    imu_msg.angular_velocity.z = motionRotationRateZ;
    imu_msg.angular_velocity_covariance = {0.000872665, 0.0, 0.0, 0.0, 0.000872665, 0.0, 0.0, 0.0, 0.000872665};
    imu_msg.linear_acceleration.x = motionUserAccelerationX;
    imu_msg.linear_acceleration.y = motionUserAccelerationY;
    imu_msg.linear_acceleration.z = motionUserAccelerationZ;
    imu_msg.linear_acceleration_covariance = {0.003, 0.0, 0.0, 0.0, 0.003, 0.0, 0.0, 0.0, 0.003};
    // Publish IMU information
    imu_pub_.publish(imu_msg);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = imu_msg.header;
    pose_msg.pose.orientation = imu_msg.orientation;
    pose_pub_.publish(pose_msg);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = imu_msg.header.stamp;
    transformStamped.header.frame_id = "iphone_pose_link";
    transformStamped.child_frame_id = frame_id_;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = motionQuaternionX;
    transformStamped.transform.rotation.y = motionQuaternionY;
    transformStamped.transform.rotation.z = motionQuaternionZ;
    transformStamped.transform.rotation.w = motionQuaternionW;
    dynamic_br_.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Publisher pose_pub_;
  // ros::Publisher mag_pub_;
  // ros::Publisher navsatFix_pub_;
  // ros::Publisher navsatStatus_pub_;
  // ros::Publisher bat_pub_;
  tf2_ros::TransformBroadcaster dynamic_br_;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;

  std::vector<char> receive_buffer_;
  std::string json_buffer_;
  int buffer_length_ = 1024;    // Maximum length of the receive buffer
  int udp_port_;                // UDP port number
  std::string device_id_;       // Device ID for the IMU
  std::string frame_id_;        // Frame ID for the IMU data
  std::string imu_topic_name_;  // IMU Topic name
  std::string pose_topic_name_; // Pose Topic name
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "sensorlog_bridge_node");
  ros::NodeHandle nh("~");

  SensorLogBridge sensorLogBridge(nh);
  ros::spin();

  return 0;
}
