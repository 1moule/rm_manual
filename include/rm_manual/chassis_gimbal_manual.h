//
// Created by qiayuan on 5/22/21.
//

#ifndef RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
#define RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_

#include "rm_manual/common/manual_base.h"
namespace rm_manual {
class ChassisGimbalManual : public ManualBase {
 public:
  explicit ChassisGimbalManual(ros::NodeHandle &nh) : ManualBase(nh) {
    ros::NodeHandle chassis_nh(nh, "chassis");
    chassis_cmd_sender_ = new ChassisCommandSender(chassis_nh);
    ros::NodeHandle vel_nh(nh, "vel");
    vel_cmd_sender_ = new VelCommandSender(vel_nh);
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh, *data_.referee_);
  }
 protected:
  void rightSwitchMid() override {
    ManualBase::rightSwitchMid();
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
    vel_cmd_sender_->setXVel(data_.dbus_data_.ch_r_y);
    vel_cmd_sender_->setYVel(data_.dbus_data_.ch_r_x);
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    gimbal_cmd_sender_->setRate(-data_.dbus_data_.ch_l_x, -data_.dbus_data_.ch_l_y);
  }
  void sendCommand(const ros::Time &time) override {
    chassis_cmd_sender_->sendCommand(time);
    vel_cmd_sender_->sendCommand(time);
    gimbal_cmd_sender_->sendCommand(time);
  }
  void wPress() override { if (state_ == PC) vel_cmd_sender_->setXVel(1.); }
  void aPress() override { if (state_ == PC) vel_cmd_sender_->setYVel(1.); }
  void sPress() override { if (state_ == PC) vel_cmd_sender_->setXVel(-1.); }
  void dPress() override { if (state_ == PC) vel_cmd_sender_->setYVel(-1.); }
  ChassisCommandSender *chassis_cmd_sender_;
  VelCommandSender *vel_cmd_sender_;
  GimbalCommandSender *gimbal_cmd_sender_;
};
}

#endif //RM_MANUAL_CHASSIS_GIMBAL_MANUAL_H_
