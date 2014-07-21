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
#include "teleop_twist_joy/teleop_twist_joy.h"


namespace teleop_twist_joy
{

TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh) : nh_(nh)
{
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::joyCallback, this);
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  ros::param::param("~enable_button", enable_button_, 0);
  ros::param::param("~enable_turbo_button", enable_turbo_button_, -1);

  ros::param::param("~axis_linear", axis_linear_, 1);
  ros::param::param("~scale_linear", scale_linear_, 0.5f);
  ros::param::param("~scale_linear_turbo", scale_linear_turbo_, 1.0f);

  ros::param::param("~axis_angular", axis_angular_, 0);
  ros::param::param("~scale_angular", scale_angular_, 1.0f);

  sent_disable_msg_ = false;
}

void TeleopTwistJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  if (enable_turbo_button_ >= 0 && joy_msg->buttons[enable_turbo_button_])
  {
    cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_] * scale_linear_turbo_;
    cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_] * scale_angular_;
    cmd_vel_pub_.publish(cmd_vel_msg);
    sent_disable_msg_ = false;
  }
  else if (joy_msg->buttons[enable_button_])
  {
    cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_] * scale_linear_;
    cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_] * scale_angular_;
    cmd_vel_pub_.publish(cmd_vel_msg);
    sent_disable_msg_ = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg_)
    {
      cmd_vel_pub_.publish(cmd_vel_msg);
      sent_disable_msg_ = true;
    }
  }
}

}  // namespace teleop_twist_joy
