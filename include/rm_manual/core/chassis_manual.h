//
// Created by guanlin on 25-6-23.
//

#pragma once

#include <rm_msgs/ChassisCmd.h>
#include <rm_common/decision/command_sender.h>

#include "rm_manual/core/keyboard.h"

namespace rm_manual
{
class ChassisManual
{
public:
  ChassisManual(ros::NodeHandle& nh, KeyBoard* keyboard)
  {
    ros::NodeHandle chassis_nh(nh, "chassis");
    ros::NodeHandle vel_nh(nh, "vel");
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh);
    vel_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
      ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
    if (!vel_nh.getParam("gyro_rotate_reduction", gyro_rotate_reduction_))
      ROS_ERROR("Gyro rotate reduction no defined (namespace: %s)", nh.getNamespace().c_str());

    // Set up keyboard events
    keyboard->w_event_.setEdge(boost::bind(&ChassisManual::wPress, this), boost::bind(&ChassisManual::wRelease, this));
    keyboard->w_event_.setActiveHigh(boost::bind(&ChassisManual::wPressing, this));
    keyboard->s_event_.setEdge(boost::bind(&ChassisManual::sPress, this), boost::bind(&ChassisManual::sRelease, this));
    keyboard->s_event_.setActiveHigh(boost::bind(&ChassisManual::sPressing, this));
    keyboard->a_event_.setEdge(boost::bind(&ChassisManual::aPress, this), boost::bind(&ChassisManual::aRelease, this));
    keyboard->a_event_.setActiveHigh(boost::bind(&ChassisManual::aPressing, this));
    keyboard->d_event_.setEdge(boost::bind(&ChassisManual::dPress, this), boost::bind(&ChassisManual::dRelease, this));
    keyboard->d_event_.setActiveHigh(boost::bind(&ChassisManual::dPressing, this));
  }
  void wPress()
  {
    x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
  }
  void sPress()
  {
    x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
  }
  void aPress()
  {
    y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
  }
  void dPress()
  {
    y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
  }
  void wRelease()
  {
    x_scale_ = x_scale_ <= -1.0 ? -1.0 : x_scale_ - 1.0;
    vel_cmd_sender_->setLinearXVel(x_scale_);
  }
  void aRelease()
  {
    y_scale_ = y_scale_ <= -1.0 ? -1.0 : y_scale_ - 1.0;
    vel_cmd_sender_->setLinearYVel(y_scale_);
  }
  void sRelease()
  {
    x_scale_ = x_scale_ >= 1.0 ? 1.0 : x_scale_ + 1.0;
    vel_cmd_sender_->setLinearXVel(x_scale_);
  }
  void dRelease()
  {
    y_scale_ = y_scale_ >= 1.0 ? 1.0 : y_scale_ + 1.0;
    vel_cmd_sender_->setLinearYVel(y_scale_);
  }
  void wPressing()
  {
    double final_x_scale = x_scale_ * speed_change_scale_;
    vel_cmd_sender_->setLinearXVel(is_gyro_ ? final_x_scale * gyro_move_reduction_ : final_x_scale);
  }
  void aPressing()
  {
    double final_y_scale = y_scale_ * speed_change_scale_;
    vel_cmd_sender_->setLinearYVel(is_gyro_ ? final_y_scale * gyro_move_reduction_ : final_y_scale);
  }
  void sPressing()
  {
    double final_x_scale = x_scale_ * speed_change_scale_;
    vel_cmd_sender_->setLinearXVel(is_gyro_ ? final_x_scale * gyro_move_reduction_ : final_x_scale);
  }
  void dPressing()
  {
    double final_y_scale = y_scale_ * speed_change_scale_;
    vel_cmd_sender_->setLinearYVel(is_gyro_ ? final_y_scale * gyro_move_reduction_ : final_y_scale);
  }
  void sendCommand(const ros::Time& time)
  {
    chassis_cmd_sender_->sendChassisCommand(time, is_gyro_);
    vel_cmd_sender_->sendCommand(time);
  }
  void setRcVel(const rm_msgs::DbusData::ConstPtr& dbus_data)
  {
    vel_cmd_sender_->setLinearXVel(is_gyro_ ? dbus_data->ch_r_y * gyro_move_reduction_ : dbus_data->ch_r_y);
    vel_cmd_sender_->setLinearYVel(is_gyro_ ? -dbus_data->ch_r_x * gyro_move_reduction_ : -dbus_data->ch_r_x);
  }
  void setChassisMode(int mode)
  {
    switch (mode)
    {
      case rm_msgs::ChassisCmd::RAW:
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        is_gyro_ = true;
        if (x_scale_ != 0.0 || y_scale_ != 0.0)
          vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
        else
          vel_cmd_sender_->setAngularZVel(1.0);
        break;
      case rm_msgs::ChassisCmd::FOLLOW:
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
        is_gyro_ = false;
        vel_cmd_sender_->setAngularZVel(0.0);
        break;
      case rm_msgs::ChassisCmd::DEPLOY:
        chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
        is_gyro_ = true;
        vel_cmd_sender_->setAngularZVel(0.0);
        break;
    }
  }
  bool getIsStatic() const
  {
    return x_scale_ == 0.0 && y_scale_ == 0.0;
  }

  rm_common::ChassisCommandSender* chassis_cmd_sender_{};
  rm_common::Vel2DCommandSender* vel_cmd_sender_{};

  bool is_gyro_{ 0 };
  double speed_change_scale_{ 1. };
  double x_scale_{}, y_scale_{};
  double gyro_move_reduction_{ 1. }, gyro_rotate_reduction_{ 1. };
};
}  // namespace rm_manual
