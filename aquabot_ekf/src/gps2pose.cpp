#include <aquabot_ekf/boat.h>
#include <aquabot_ekf/gps2enu.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

namespace aquabot_ekf
{

// main node
class GPS2Pose : public rclcpp::Node
{
public:
  GPS2Pose() : Node("gps2pose"), boat{this, &enu, declare_parameter("unify", false)}
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    static auto gps_sub = create_subscription<NavSatFix>("/aquabot/sensors/gps/gps/fix", 5, [this](NavSatFix::UniquePtr msg)
    {boat.updatePose(*msg);});

    static auto imu_sub = create_subscription<Imu>("/aquabot/sensors/imu/imu/data", 1, [this](Imu::UniquePtr msg)
    {boat.updateImu(*msg);});

    obs_pub = create_publisher<PoseArray>("/aquabot/turbines", 1);
    obs_sub = create_subscription<PoseArray>("/aquabot/ais_sensor/windturbines_positions", 1, [this](PoseArray::UniquePtr msg)
    {toPoses(*msg);});
  }


private:    

  toENU enu;
  Boat boat;

  rclcpp::Publisher<PoseArray>::SharedPtr obs_pub;
  rclcpp::Subscription<PoseArray>::SharedPtr obs_sub;

  void toPoses(const PoseArray &msg)
  {
    if(!enu.isInit())
      return;

    PoseArray poses;
    poses.poses.reserve(msg.poses.size());

    for(auto &gps: msg.poses)
    {
      auto &p{poses.poses.emplace_back()};
      p.orientation = gps.orientation;
      enu.transform(gps.position.x, gps.position.y, 1., p.position);
    }
    obs_pub->publish(poses);
  }

};

}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aquabot_ekf::GPS2Pose>());
  rclcpp::shutdown();
  return 0;
}
