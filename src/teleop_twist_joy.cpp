/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, double> scale_linear_map;
  std::map<std::string, double> scale_linear_turbo_map;

  std::map<std::string, int> axis_angular_map;
  std::map<std::string, double> scale_angular_map;
  std::map<std::string, double> scale_angular_turbo_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->getParam("axis_linear", pimpl_->axis_linear_map);
    nh_param->getParam("scale_linear", pimpl_->scale_linear_map);
    nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_turbo_map);
  }
  else
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["x"], 0.5);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_turbo_map["x"], 1.0);
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->getParam("axis_angular", pimpl_->axis_angular_map);
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map);
    nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_turbo_map);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["yaw"], 0.5);
    nh_param->param<double>("scale_angular_turbo",
        pimpl_->scale_angular_turbo_map["yaw"], pimpl_->scale_angular_map["yaw"]);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map[it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_turbo_map[it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map[it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_turbo_map[it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  if (enable_turbo_button >= 0 && joy_msg->buttons[enable_turbo_button])
  {
    if (axis_linear_map.find("x") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_turbo_map["x"];
    }
    if (axis_linear_map.find("y") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_turbo_map["y"];
    }
    if  (axis_linear_map.find("z") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_turbo_map["z"];
    }
    if  (axis_angular_map.find("yaw") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_turbo_map["yaw"];
    }
    if  (axis_angular_map.find("pitch") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_turbo_map["pitch"];
    }
    if  (axis_angular_map.find("roll") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_turbo_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else if (joy_msg->buttons[enable_button])
  {
    if  (axis_linear_map.find("x") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_map["x"];
    }
    if  (axis_linear_map.find("y") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_map["y"];
    }
    if  (axis_linear_map.find("z") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_map["z"];
    }
    if  (axis_angular_map.find("yaw") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_map["yaw"];
    }
    if  (axis_angular_map.find("pitch") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_map["pitch"];
    }
    if  (axis_angular_map.find("roll") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
