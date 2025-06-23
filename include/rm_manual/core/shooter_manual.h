//
// Created by guanlin on 25-6-23.
//

#pragma once

#include <rm_msgs/ShootCmd.h>

#include "rm_manual/core/keyboard.h"

namespace rm_manual
{
class ShooterManual
{
public:
  ShooterManual(ros::NodeHandle& nh, KeyBoard* keyboard)
  {
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(shooter_nh);

    keyboard->ctrl_f_event_.setRising(boost::bind(&ShooterManual::ctrlFPress, this));
    keyboard->mouse_left_event_.setActiveHigh(boost::bind(&ShooterManual::mouseLeftPress, this));
    keyboard->mouse_left_event_.setFalling(boost::bind(&ShooterManual::mouseLeftRelease, this));
  }
  void ctrlFPress()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  }
  void mouseLeftPress()
  {
    if (shooter_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::STOP)
    {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
      prepare_shoot_ = false;
    }
    if (prepare_shoot_)
    {
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkError(ros::Time::now());
    }
  }
  void mouseLeftRelease()
  {
    shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    prepare_shoot_ = true;
  }

  rm_common::ShooterCommandSender* shooter_cmd_sender_{};

private:
  bool prepare_shoot_ = false;
};

}  // namespace rm_manual
