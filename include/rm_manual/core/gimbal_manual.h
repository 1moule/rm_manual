//
// Created by guanlin on 25-6-23.
//

#pragma once

#include <rm_msgs/GimbalCmd.h>

#include "rm_manual/core/keyboard.h"

namespace rm_manual
{
class GimbalManual
{
public:
  GimbalManual(ros::NodeHandle& nh, KeyBoard* keyboard)
  {
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new rm_common::GimbalCommandSender(gimbal_nh);
    gimbal_scale_ = getParam(gimbal_nh, "gimbal_scale", 1.0);
    traj_scale_ = getParam(gimbal_nh, "traj_scale", 0.5);
  }
  void setRcRate(const rm_msgs::DbusData::ConstPtr& dbus_data)
  {
    gimbal_cmd_sender_->setRate(-dbus_data->ch_l_x, -dbus_data->ch_l_y);
  }
  void setPcRate(const rm_msgs::DbusData::ConstPtr& dbus_data)
  {
    gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_, -dbus_data->m_y * gimbal_scale_);
  }

  rm_common::GimbalCommandSender* gimbal_cmd_sender_{};
  double gimbal_scale_{ 1. }, traj_scale_{ 0.5 };
};
}  // namespace rm_manual
