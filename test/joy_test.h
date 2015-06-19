#include <gtest/gtest.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


const double EPS = 0.0001;

class TeleopTwistJoyTest : public ::testing::Test
{
public:

  TeleopTwistJoyTest():
    gotFirst_(false)
  {
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &TeleopTwistJoyTest::cmdVelCallback, this);
    joy_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1000);

    if (ros::param::get("/teleop_twist_joy/axis_linear", axis_linear_map))
    {
      ros::param::get("/teleop_twist_joy/axis_linear", axis_linear_map);
      ros::param::get("/teleop_twist_joy/scale_linear", scale_linear_map);
      ros::param::get("/teleop_twist_joy/scale_linear_turbo", scale_linear_turbo_map);
    }
    else
    {
      ros::param::get("/teleop_twist_joy/axis_linear", axis_linear_map["x"]);
      ros::param::get("/teleop_twist_joy/scale_linear", scale_linear_map["x"]);
      ros::param::get("/teleop_twist_joy/scale_linear_turbo", scale_linear_turbo_map["x"]);
    }

    if (ros::param::get("/teleop_twist_joy/axis_angular", axis_angular_map))
    {
      ros::param::get("/teleop_twist_joy/axis_angular", axis_angular_map);
      ros::param::get("/teleop_twist_joy/scale_angular", scale_angular_map);
    }
    else
    {
      ros::param::get("/teleop_twist_joy/axis_angular", axis_angular_map["yaw"]);
      ros::param::get("/teleop_twist_joy/scale_angular", scale_angular_map["yaw"]);
    }

    ros::param::get("/teleop_twist_joy/enable_button", enable_button);
    ros::param::get("/teleop_twist_joy/enable_turbo_button", enable_turbo_button);

  }

  ~TeleopTwistJoyTest()
  {
    cmd_vel_sub_.shutdown();
  }

  geometry_msgs::Twist recvCmdVel()
  {
    return recv_cmd_vel_;
  }

  void pubJoy(sensor_msgs::Joy joy_msg)
  {
    joy_pub_.publish(joy_msg);
  }

  bool isReady()
  {
    return (cmd_vel_sub_.getNumPublishers() > 0
      && joy_pub_.getNumSubscribers() > 0
      && gotFirst_);
  }

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, double> scale_linear_map;
  std::map<std::string, double> scale_linear_turbo_map;

  std::map<std::string, int> axis_angular_map;
  std::map<std::string, double> scale_angular_map;

  int enable_button;
  int enable_turbo_button;




private:
  ros::NodeHandle nh_;
  ros::Publisher joy_pub_;
  ros::Subscriber cmd_vel_sub_;

  bool gotFirst_;
  geometry_msgs::Twist recv_cmd_vel_;

  void cmdVelCallback(const geometry_msgs::Twist& vel_msg)
  {
    gotFirst_ = true;
    recv_cmd_vel_ = vel_msg;
  }

};
