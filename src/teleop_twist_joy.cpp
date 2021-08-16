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
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"
#include "gfoe_j1939_blue_arrow_interface/BlueArrowHelmMode.h"

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
  void sendCmdVelMsgTwistStamped(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void sendCmdVelMsgTwist(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void sendModeCmdMsg(const std::string& mode);
  void sendXciControlEnable(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void sendBlueArrowModeMsg();
  
  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher mode_cmd_pub;
  ros::Publisher xci_control_pub;
  ros::Publisher bluearrow_mode_pub;

  int enable_button;
  int enable_turbo_button;
  int manual_button;
  int auto_button;
  int xci_control_button;
  int open_loop_button;
  int station_keep_button;
  int virtual_anchor_button;

  // Store state
  gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode bluearrow_mode;
  
  std::string joy_vel_output;

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

  nh_param->param<std::string>("joy_vel_output",
			       pimpl_->joy_vel_output, "TwistStamped");

  // Initialize state
  pimpl_->bluearrow_mode = gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode();
  pimpl_->bluearrow_mode.mode =
    gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode::OPEN_LOOP;

  if (pimpl_->joy_vel_output == "Twist")
    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  else
    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::TwistStamped>("cmd_vel", 1, true);

  pimpl_->mode_cmd_pub = nh->advertise<std_msgs::String>("send_command", 1, true);
  pimpl_->bluearrow_mode_pub = nh->advertise
    <gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode>("bluearrow_mode_cmd",
							 1, true);
  pimpl_->xci_control_pub = nh->advertise<std_msgs::Bool>("xci_control_enable", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);
  nh_param->param<int>("manual_button", pimpl_->manual_button, -1);
  nh_param->param<int>("auto_button", pimpl_->auto_button, -1);
  nh_param->param<int>("xci_control_button", pimpl_->xci_control_button, -1);
  nh_param->param<int>("open_loop_button", pimpl_->open_loop_button, -1);
  nh_param->param<int>("virtual_anchor_button", pimpl_->virtual_anchor_button, -1);
  nh_param->param<int>("station_keep_button", pimpl_->station_keep_button, -1);
  

  if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
    nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
  }
  else
  {
    // Set default the same as annie config yaml
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear",
			    pimpl_->scale_linear_map["normal"]["x"], 2.0);
    nh_param->param<double>("scale_linear_turbo",
			    pimpl_->scale_linear_map["turbo"]["x"],
			    pimpl_->scale_linear_map["normal"]["x"]);
    
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["y"], 3);
    nh_param->param<double>("scale_linear",
			    pimpl_->scale_linear_map["normal"]["y"], -0.56);
    nh_param->param<double>("scale_linear_turbo",
			    pimpl_->scale_linear_map["turbo"]["y"],
			    pimpl_->scale_linear_map["normal"]["y"]);
 
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
    nh_param->getParam("scale_angular_turbo",
		       pimpl_->scale_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular",
			    pimpl_->scale_angular_map["normal"]["yaw"], 0.211);
    nh_param->param<double>("scale_angular_turbo",
			    pimpl_->scale_angular_map["turbo"]["yaw"],
			    pimpl_->scale_angular_map["normal"]["yaw"]);
  }

  // output current setup to screen
  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);
  ROS_INFO_COND_NAMED(pimpl_->manual_button >= 0, "TeleopTwistJoy",
      "Manual on button %i.", pimpl_->manual_button);
  ROS_INFO_COND_NAMED(pimpl_->auto_button >= 0, "TeleopTwistJoy",
      "Autonomous on button %i.", pimpl_->auto_button);
  ROS_INFO_COND_NAMED(pimpl_->xci_control_button >= 0, "TeleopTwistJoy",
      "XCI control on button %i.", pimpl_->xci_control_button);

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

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg,
	      const std::map<std::string, int>& axis_map,
              const std::map<std::string,
	      double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

// function to make it backwards compatible
// but still support multiple types for joy_vel outputs
void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  if (joy_vel_output == "Twist")
    sendCmdVelMsgTwist(joy_msg, which_map);
  else
    sendCmdVelMsgTwistStamped(joy_msg, which_map);
}

void TeleopTwistJoy::Impl::sendCmdVelMsgTwistStamped(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::TwistStamped cmd_vel_msg;

  cmd_vel_msg.header.stamp = ros::Time::now();
  cmd_vel_msg.header.frame_id = "teleop_twist_joy";

  cmd_vel_msg.twist.linear.x = getVal(joy_msg, axis_linear_map,
				      scale_linear_map[which_map], "x");
  cmd_vel_msg.twist.linear.y = getVal(joy_msg, axis_linear_map,
				      scale_linear_map[which_map], "y");
  cmd_vel_msg.twist.linear.z = getVal(joy_msg, axis_linear_map,
				      scale_linear_map[which_map], "z");
  cmd_vel_msg.twist.angular.z = getVal(joy_msg, axis_angular_map,
				       scale_angular_map[which_map], "yaw");
  cmd_vel_msg.twist.angular.y = getVal(joy_msg, axis_angular_map,
				       scale_angular_map[which_map], "pitch");
  cmd_vel_msg.twist.angular.x = getVal(joy_msg, axis_angular_map,
				       scale_angular_map[which_map], "roll");

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::sendCmdVelMsgTwist(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                              const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map,
				scale_linear_map[which_map], "x");
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map,
				scale_linear_map[which_map], "y");
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map,
				scale_linear_map[which_map], "z");
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map,
				 scale_angular_map[which_map], "yaw");
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map,
				 scale_angular_map[which_map], "pitch");
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map,
				 scale_angular_map[which_map], "roll");

  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

// teleop_twist_joy will send piloting mode messages to Project 11 command_bridge_sender
// default rostopic: /send_command
// data: "piloting_mode autonomous"  or  "piloting_mode manual"
void TeleopTwistJoy::Impl::sendModeCmdMsg(const std::string& mode)
{
  std_msgs::String cmd_mode_msg;
  cmd_mode_msg.data = "piloting_mode " + mode;
  mode_cmd_pub.publish(cmd_mode_msg);
}

void TeleopTwistJoy::Impl::sendBlueArrowModeMsg()
{
  bluearrow_mode.header.stamp = ros::Time::now();
  bluearrow_mode_pub.publish(bluearrow_mode);
}

// teleop_twist_jow will send xci control enable commands to blue_arrow interface
// default rostopic: /xci_control_enable
// data: bool
void TeleopTwistJoy::Impl::sendXciControlEnable(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  std_msgs::Bool xci_control;
  xci_control.data  = (bool)joy_msg->buttons[xci_control_button];
  xci_control_pub.publish(xci_control);
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Always publish xci enable
  if (joy_msg->buttons.size() > xci_control_button)
  {
    sendXciControlEnable(joy_msg);
  }

  // Only publish when enabled 
  if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendCmdVelMsg(joy_msg, "normal");

    // Removing joy publication of mode
    /*sendModeCmdMsg("manual");
    
    // Enter manual mode
    if (joy_msg->buttons.size() > manual_button &&
	joy_msg->buttons[manual_button])
      {
	sendModeCmdMsg("manual");
      }
    // Enter autonomous mode
    else if (joy_msg->buttons.size() > auto_button &&
	joy_msg->buttons[auto_button])
      {
	//HACK
	//sendModeCmdMsg("autonomous");
      }
    */
    // Blue arrow command mode
    if (joy_msg->buttons.size() > station_keep_button &&
	joy_msg->buttons[station_keep_button])
      {
	bluearrow_mode.mode =
	  gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode::STATION_KEEP;
	sendBlueArrowModeMsg();
      }
    else if (joy_msg->buttons.size() > open_loop_button &&
	joy_msg->buttons[open_loop_button])
      {
	bluearrow_mode.mode =
	  gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode::OPEN_LOOP;
	sendBlueArrowModeMsg();
      }

    else if (joy_msg->buttons.size() > virtual_anchor_button &&
	     joy_msg->buttons[virtual_anchor_button])
      {
	bluearrow_mode.mode =
	  gfoe_j1939_blue_arrow_interface::BlueArrowHelmMode::VIRTUAL_ANCHOR;
	sendBlueArrowModeMsg();
      }
    

  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      if (joy_vel_output == "Twist")
      {
        geometry_msgs::Twist cmd_vel_msg;
	cmd_vel_msg.linear.x = 0;
	cmd_vel_msg.linear.y = 0;
	cmd_vel_msg.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel_msg);
      }
      else
      {
        geometry_msgs::TwistStamped cmd_vel_msg;
	cmd_vel_msg.header.stamp = ros::Time::now();
	cmd_vel_msg.header.frame_id = "teleop_twist_joy";
	cmd_vel_msg.twist.linear.x = 0;
	cmd_vel_msg.twist.linear.y = 0;
	cmd_vel_msg.twist.angular.z = 0;
        cmd_vel_pub.publish(cmd_vel_msg);
      }
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
