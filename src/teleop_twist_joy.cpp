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
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void speed_control(const sensor_msgs::Joy::ConstPtr& joy_msg);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  int enable_button;
  int enable_turbo_button;
  int linear_increase_button;
  int linear_decrease_button;
  int angular_increase_button;
  int angular_decrease_button;
  double increase_value;
  double decrease_value;
  // These variables will be used to control only when the button is pressed.
  bool speed_control_state[3] = {false,false,false};

  std::map<std::string, int> axis_linear_map;
  std::map< std::string, std::map<std::string, double> > scale_linear_map;

  std::map<std::string, int> axis_angular_map;
  std::map< std::string, std::map<std::string, double> > scale_angular_map;

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
  nh_param->param<int>("linear_increase_button", pimpl_->linear_increase_button, -1);
  nh_param->param<int>("linear_decrease_button", pimpl_->linear_decrease_button, -1);
  nh_param->param<int>("angular_increase_button", pimpl_->angular_increase_button, -1);
  nh_param->param<int>("angular_decrease_button", pimpl_->angular_decrease_button, -1);
  nh_param->param<double>("increase_value", pimpl_->increase_value, 0.01);
  nh_param->param<double>("decrease_value", pimpl_->decrease_value, 0.01);

  if(pimpl_->increase_value<0.0)
  {
    ROS_ERROR("İncrease value cannot be negative");
    pimpl_->increase_value*=-1;
    ROS_WARN("Increment value set to %f",pimpl_->increase_value);
  }
  if(pimpl_->decrease_value<0.0)
  {
    ROS_ERROR("Decrease value cannot be negative");
    pimpl_->decrease_value*=-1;
    ROS_WARN("Increment value set to %f",pimpl_->decrease_value);
  }
  

  if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
    nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["normal"]["x"], 0.5);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]["x"], 1.0);
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
    nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["normal"]["yaw"], 0.5);
    nh_param->param<double>("scale_angular_turbo",
        pimpl_->scale_angular_map["turbo"]["yaw"], pimpl_->scale_angular_map["normal"]["yaw"]);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y");
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z");
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch");
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll");

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::speed_control(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if(linear_increase_button>0  && joy_msg->buttons[linear_increase_button] )
  {
    if(!speed_control_state[0])
    {
      speed_control_state[0]=true;
      if( scale_linear_map["normal"]["x"]< scale_linear_map["turbo"]["x"])
      {
        scale_linear_map["normal"]["x"] += increase_value; 
      }
      ROS_INFO("linear max speed %f",scale_linear_map["normal"]["x"]);
    }
  }else{
    speed_control_state[0]=false;
  }

  if(linear_decrease_button>0 && joy_msg->buttons[linear_decrease_button] )
  {
    if(!speed_control_state[1])
    {
      speed_control_state[1]=true;
      
      scale_linear_map["normal"]["x"] -= decrease_value; 
      if(scale_linear_map["normal"]["x"]<0.01)
      {
        scale_linear_map["normal"]["x"] =0.01;
      }
      ROS_INFO("Linear max speed %f",scale_linear_map["normal"]["x"]);
    }
  }else{
    speed_control_state[1]=false;
  }

  if(angular_increase_button>0  && joy_msg->buttons[angular_increase_button] )
  {
    if(!speed_control_state[2])
    {
      speed_control_state[2]=true;
      if( scale_angular_map["normal"]["yaw"]< scale_angular_map["turbo"]["yaw"])
      {
        scale_angular_map["normal"]["yaw"] += increase_value; 
      }
      ROS_INFO("Angular max speed %f",scale_angular_map["normal"]["yaw"]);
    }
  }else{
    speed_control_state[2]=false;
  }

  if(linear_increase_button>0 && joy_msg->buttons[angular_decrease_button] )
  {
    if(!speed_control_state[3])
    {
      speed_control_state[3]=true;
      scale_angular_map["normal"]["yaw"] -= decrease_value; 
      if(scale_angular_map["normal"]["yaw"]<0.01)
      {
        scale_angular_map["normal"]["yaw"] =0.01;
      }
      ROS_INFO("Angular max speed %f",scale_angular_map["normal"]["yaw"]);
    }
  }else{
    speed_control_state[3]=false;
  }
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{

  speed_control(joy_msg);

  if (enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      geometry_msgs::Twist cmd_vel_msg;
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
