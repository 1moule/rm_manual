//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_cover_manual.h"

namespace rm_manual
{
ChassisGimbalShooterCoverManual::ChassisGimbalShooterCoverManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ChassisGimbalShooterManual(nh, nh_referee)
{
  ros::NodeHandle cover_nh(nh, "cover");
  nh.param("supply_frame", supply_frame_, std::string("supply_frame"));
  cover_command_sender_ = new rm_common::JointPositionBinaryCommandSender(cover_nh);
  ros::NodeHandle buff_switch_nh(nh, "buff_switch");
  switch_buff_srv_ = new rm_common::SwitchDetectionCaller(buff_switch_nh);
  ros::NodeHandle buff_type_switch_nh(nh, "buff_type_switch");
  switch_buff_type_srv_ = new rm_common::SwitchDetectionCaller(buff_type_switch_nh);
  ros::NodeHandle exposure_switch_nh(nh, "exposure_switch");
  switch_exposure_srv_ = new rm_common::SwitchDetectionCaller(exposure_switch_nh);
  ros::NodeHandle chassis_nh(nh, "chassis");
  normal_speed_scale_ = chassis_nh.param("normal_speed_scale", 1);
  low_speed_scale_ = chassis_nh.param("low_speed_scale", 0.30);
  nh.param("exit_buff_mode_duration", exit_buff_mode_duration_, 0.5);

  ctrl_z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
                        boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));
  z_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterCoverManual::zPressing, this));
  z_event_.setFalling(boost::bind(&ChassisGimbalShooterCoverManual::zRelease, this));
}

void ChassisGimbalShooterCoverManual::changeSpeedMode(SpeedMode speed_mode)
{
  if (speed_mode == LOW)
  {
    speed_change_scale_ = low_speed_scale_;
  }
  else if (speed_mode == NORMAL)
  {
    speed_change_scale_ = normal_speed_scale_;
  }
}

void ChassisGimbalShooterCoverManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterManual::updatePc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_,
                              cover_command_sender_->getState() ? 0.0 : dbus_data->m_y * gimbal_scale_);
}

void ChassisGimbalShooterCoverManual::checkReferee()
{
  manual_to_referee_pub_data_.cover_state = cover_command_sender_->getState();
  if (switch_detection_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    manual_to_referee_pub_data_.det_target = switch_buff_type_srv_->getTarget();
  else
    manual_to_referee_pub_data_.det_target = switch_detection_srv_->getTarget();
  ChassisGimbalShooterManual::checkReferee();
}
void ChassisGimbalShooterCoverManual::checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterManual::checkKeyboard(dbus_data);
  ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);
  z_event_.update((!dbus_data->key_ctrl) & dbus_data->key_z);
}

void ChassisGimbalShooterCoverManual::sendCommand(const ros::Time& time)
{
  if (is_auto_)
  {
    if (track_data_.id == 0)
    {
      geometry_msgs::PointStamped point_in, point_out;
      try
      {
        point_in.header.frame_id = "yaw";
        point_in.point.z = tf_buffer_.lookupTransform("yaw", "pitch", ros::Time(0)).transform.translation.z +
                           (0.25 * sin(12 * M_PI * count_ / 100) - 0.1);
        tf2::doTransform(point_in, point_out, tf_buffer_.lookupTransform("odom", "yaw", ros::Time(0)));
        point_out.point.x = cos(2 * M_PI * count_ / 100) +
                            tf_buffer_.lookupTransform("odom", "yaw", ros::Time(0)).transform.translation.x;
        point_out.point.y = sin(2 * M_PI * count_ / 100) +
                            tf_buffer_.lookupTransform("odom", "yaw", ros::Time(0)).transform.translation.y;
        count_ = (count_ + 1) % 100;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
      }
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::DIRECT);
      gimbal_cmd_sender_->setPoint(point_out);
    }
    else
    {
      gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      gimbal_cmd_sender_->setBulletSpeed(shooter_cmd_sender_->getSpeed());
      shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_cmd_sender_->checkError(ros::Time::now());
    }
  }
  else
    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  ChassisGimbalShooterManual::sendCommand(time);
  cover_command_sender_->sendCommand(time);
}

void ChassisGimbalShooterCoverManual::rightSwitchDownRise()
{
  ChassisGimbalShooterManual::rightSwitchDownRise();
  supply_ = true;
}

void ChassisGimbalShooterCoverManual::rightSwitchMidRise()
{
  ChassisGimbalShooterManual::rightSwitchMidRise();
  supply_ = false;
}

void ChassisGimbalShooterCoverManual::rightSwitchUpRise()
{
  ChassisGimbalShooterManual::rightSwitchUpRise();
  supply_ = false;
}

void ChassisGimbalShooterCoverManual::rPress()
{
  if (!is_auto_)
  {
    count_ = 0;
    last_gyro_state_ = is_gyro_;
    is_auto_ = true;
    is_gyro_ = true;
    chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
  else
  {
    is_auto_ = false;
    if (!last_gyro_state_)
    {
      chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
      vel_cmd_sender_->setAngularZVel(0.0);
      is_gyro_ = false;
    }
  }
}

void ChassisGimbalShooterCoverManual::ePress()
{
  switch_buff_srv_->switchTargetType();
  switch_detection_srv_->switchTargetType();
  switch_buff_type_srv_->setTargetType(switch_buff_srv_->getTarget());
  switch_exposure_srv_->switchTargetType();
  switch_buff_srv_->callService();
  switch_detection_srv_->callService();
  switch_buff_type_srv_->callService();
  switch_exposure_srv_->callService();
}

void ChassisGimbalShooterCoverManual::zPressing()
{
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
}

void ChassisGimbalShooterCoverManual::zRelease()
{
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::LOW);
}

void ChassisGimbalShooterCoverManual::wPress()
{
  ChassisGimbalShooterManual::wPress();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    last_switch_time_ = ros::Time::now();
}

void ChassisGimbalShooterCoverManual::wPressing()
{
  ChassisGimbalShooterManual::wPressing();
  if ((ros::Time::now() - last_switch_time_).toSec() > exit_buff_mode_duration_ &&
      switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
  {
    switch_buff_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
    switch_detection_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
    switch_buff_type_srv_->setTargetType(switch_buff_srv_->getTarget());
    switch_exposure_srv_->setTargetType(rm_msgs::StatusChangeRequest::ARMOR);
    switch_buff_srv_->callService();
    switch_detection_srv_->callService();
    switch_buff_type_srv_->callService();
    switch_exposure_srv_->callService();
  }
}

void ChassisGimbalShooterCoverManual::ctrlZPress()
{
  if (!cover_command_sender_->getState())
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  else
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  supply_ = !cover_command_sender_->getState();
  if (supply_)
  {
    changeSpeedMode(LOW);
  }
  else
  {
    changeSpeedMode(NORMAL);
  }
}

}  // namespace rm_manual
