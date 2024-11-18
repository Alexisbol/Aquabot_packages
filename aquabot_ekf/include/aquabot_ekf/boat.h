#ifndef BOAT_H
#define BOAT_H

#include <rclcpp/node.hpp>
#include <aquabot_ekf/gps2enu.h>
#include <aquabot_ekf/covariances.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ros_gz_interfaces/msg/param_vec.hpp>

using ros_gz_interfaces::msg::ParamVec;
using namespace std::chrono_literals;

namespace aquabot_ekf
{

struct Boat
{
  rclcpp::Node* node;
  toENU* enu;

  // forwarded messages
  PoseWithCovarianceStamped pose;
  Imu imu;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub;

  rclcpp::TimerBase::SharedPtr pose_timer;

  bool unify;

  Boat(rclcpp::Node* node, toENU* enu, bool unify_output = false) : node{node}, enu{enu}
  {
    // init pose and imu messages
    pose.header.frame_id = "world";
    imu.header.frame_id = "aquabot/wamv/imu_wamv_link/imu_wamv_sensor";

    for(auto i: {0, 7})
      pose.pose.covariance[i] = Covariances::xy;
    pose.pose.covariance[14] = Covariances::z;
    for(auto i: {21, 28, 35})
      pose.pose.covariance[i] = Covariances::rpy;

    for(auto i: {0, 4, 8})
    {
      imu.linear_acceleration_covariance[i] = Covariances::a;
      imu.angular_velocity_covariance[i] = Covariances::w;
      imu.orientation_covariance[i] = Covariances::rpy;
    }

    if(unify_output)
    {
      pose_timer = node->create_wall_timer(30ms, [this, node]()
                                           {
                                             pose.header.stamp = node->get_clock()->now();
                                             pose_pub->publish(pose);
                                           });
      pose_pub = node->create_publisher<PoseWithCovarianceStamped>("/aquabot/fused_pose", rclcpp::SensorDataQoS());
    }
    else
    {
      pose_pub = node->create_publisher<PoseWithCovarianceStamped>("/aquabot/gps_pose", rclcpp::SensorDataQoS());
      imu_pub = node->create_publisher<Imu>("/aquabot/imu", rclcpp::SensorDataQoS());
    }

  }

  inline void updatePose(const NavSatFix &gps)
  {
    if(!enu->isInit())
      enu->setReference(gps);

    enu->transform(gps.latitude, gps.longitude, gps.altitude,
                   pose.pose.pose.position);

    pose.pose.pose.position.z -= 1.62;  // not quite

    if(!pose_timer)
    {
      pose.header.stamp = node->get_clock()->now();
      pose_pub->publish(pose);
    }
  }

  inline void updateImu(const Imu &imu)
  {

    if(!pose_timer)
    {
      this->imu.orientation = imu.orientation;
      this->imu.angular_velocity = imu.angular_velocity;
      //this->imu.linear_acceleration = imu.linear_acceleration;

      this->imu.header.stamp = node->get_clock()->now();
      imu_pub->publish(this->imu);
    }
    else
    {
      this->pose.pose.pose.orientation = imu.orientation;
    }
  }
};
}


#endif // BOAT_H
