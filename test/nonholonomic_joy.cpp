#include "joy_test.h"

// Test Cases
TEST_F(TeleopTwistJoyTest, testNonholonomicJoy)
{
  double cmd = 1;
  sensor_msgs::Joy cmd_joy;
  cmd_joy.buttons.resize(20);
  cmd_joy.axes.resize(5);
  cmd_joy.buttons[enable_button] = 1;
  cmd_joy.axes[axis_linear_map["x"]] = cmd;
  cmd_joy.axes[axis_angular_map["yaw"]] = cmd;

  ros::Duration(1).sleep();

  do
  {
    pubJoy(cmd_joy);
    ros::Duration(0.1).sleep();
  } while (!isReady());

  geometry_msgs::Twist cmd_vel = recvCmdVel();
  EXPECT_NEAR(cmd_vel.linear.x, cmd * scale_linear_map["x"], EPS);
  EXPECT_NEAR(cmd_vel.linear.y, 0.0, EPS);
  EXPECT_NEAR(cmd_vel.angular.z, cmd * scale_angular_map["yaw"], EPS);
}

TEST_F(TeleopTwistJoyTest, testNonholonomicJoyNotEnabled)
{
  double cmd = 1;
  sensor_msgs::Joy cmd_joy;
  cmd_joy.buttons.resize(20);
  cmd_joy.axes.resize(5);
  cmd_joy.buttons[enable_button] = 0;
  cmd_joy.axes[axis_linear_map["x"]] = cmd;
  cmd_joy.axes[axis_linear_map["y"]] = cmd;
  cmd_joy.axes[axis_angular_map["yaw"]] = cmd;

  ros::Duration(1).sleep();

  do
  {
    pubJoy(cmd_joy);
    ros::Duration(0.1).sleep();
  } while (!isReady());

  geometry_msgs::Twist cmd_vel = recvCmdVel();
  EXPECT_NEAR(cmd_vel.linear.x, 0.0, EPS);
  EXPECT_NEAR(cmd_vel.linear.y, 0.0, EPS);
  EXPECT_NEAR(cmd_vel.angular.z, 0.0, EPS);
}

TEST_F(TeleopTwistJoyTest, testNonholonomicJoyTurbo)
{
  double cmd = 1;
  sensor_msgs::Joy cmd_joy;
  cmd_joy.buttons.resize(20);
  cmd_joy.axes.resize(5);
  cmd_joy.buttons[enable_turbo_button] = 1;
  cmd_joy.buttons[enable_button] = 1;
  cmd_joy.axes[axis_linear_map["x"]] = cmd;
  cmd_joy.axes[axis_linear_map["y"]] = cmd;
  cmd_joy.axes[axis_angular_map["yaw"]] = cmd;

  ros::Duration(1).sleep();

  do
  {
    pubJoy(cmd_joy);
    ros::Duration(0.1).sleep();
  } while (!isReady());

  geometry_msgs::Twist cmd_vel = recvCmdVel();
  EXPECT_NEAR(cmd_vel.linear.x, cmd * scale_linear_turbo_map["x"], EPS);
  EXPECT_NEAR(cmd_vel.linear.y, 0.0, EPS);
  EXPECT_NEAR(cmd_vel.angular.z, cmd * scale_angular_map["yaw"], EPS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "nonholonomic_joy_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
