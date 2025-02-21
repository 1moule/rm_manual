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
  nh.param("gyro_speed_limit", gyro_speed_limit_, 6.0);
  ros::NodeHandle vel_nh(nh, "vel");
  sin_gyro_base_scale_ = vel_nh.param("sin_gyro_base_scale", 1.0);
  sin_gyro_amplitude_ = vel_nh.param("sin_gyro_amplitude", 0.0);
  sin_gyro_period_ = vel_nh.param("sin_gyro_period", 1.0);
  sin_gyro_phase_ = vel_nh.param("sin_gyro_phase", 0.0);

  last_time = ros::Time::now();
  current_velocity = 0.5;
  max_velocity = 8.0;
  vel_step = 0.5;
  angle = -M_PI;
  direction = 1;

  ctrl_z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::ctrlZPress, this),
                        boost::bind(&ChassisGimbalShooterCoverManual::ctrlZRelease, this));
  //  ctrl_r_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterCoverManual::ctrlRPressing, this));
  z_event_.setEdge(boost::bind(&ChassisGimbalShooterCoverManual::zPress, this),
                   boost::bind(&ChassisGimbalShooterCoverManual::zRelease, this));
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

double ChassisGimbalShooterCoverManual::getDynamicScale(const double base_scale, const double amplitude,
                                                        const double period, const double phase)
{
  ros::Time current_time = ros::Time::now();
  double t = current_time.toSec();
  double f = 2 * M_PI / period;
  double dynamic_scale = base_scale + amplitude * sin(f * t + phase);
  if (dynamic_scale < 0.0)
  {
    dynamic_scale = 0.0;
  }
  else if (dynamic_scale > 1.0)
  {
    dynamic_scale = 1.0;
  }
  return dynamic_scale;
}

void ChassisGimbalShooterCoverManual::changeGyroSpeedMode(SpeedMode speed_mode)
{
  if (speed_mode == LOW)
  {
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_, gyro_speed_limit_);
    else
      vel_cmd_sender_->setAngularZVel(1.0, gyro_speed_limit_);
  }
  else if (speed_mode == NORMAL)
  {
    if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(1.0);
  }
}

void ChassisGimbalShooterCoverManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ChassisGimbalShooterManual::updatePc(dbus_data);
  gimbal_cmd_sender_->setRate(-dbus_data->m_x * gimbal_scale_,
                              cover_command_sender_->getState() ? 0.0 : dbus_data->m_y * gimbal_scale_);
  if (is_gyro_)
  {
    if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
      if (x_scale_ != 0.0 || y_scale_ != 0.0)
        vel_cmd_sender_->setAngularZVel(gyro_rotate_reduction_, gyro_speed_limit_);
      else
        vel_cmd_sender_->setAngularZVel(1.0, gyro_speed_limit_);
    else if (x_scale_ != 0.0 || y_scale_ != 0.0)
      vel_cmd_sender_->setAngularZVel(
          getDynamicScale(sin_gyro_base_scale_, sin_gyro_amplitude_, sin_gyro_period_, sin_gyro_phase_) *
          gyro_rotate_reduction_);
    else
      vel_cmd_sender_->setAngularZVel(
          getDynamicScale(sin_gyro_base_scale_, sin_gyro_amplitude_, sin_gyro_period_, sin_gyro_phase_));
  }
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
  if (start_identify_)
  {
    ros::Time current_time = ros::Time::now();
    double elapsed_time = (current_time - last_time).toSec();
    last_time = current_time;
    angle += direction * current_velocity * elapsed_time;

    if (direction == 1 && angle >= M_PI)
    {
      angle = M_PI;
      direction = -1;
      ROS_INFO("Completed forward motion at velocity: %f", current_velocity);
    }
    else if (direction == -1 && angle <= -M_PI)
    {
      angle = -M_PI;
      direction = 1;
      double mean_forward = calculateMean(forward_efforts);
      double mean_backward = calculateMean(backward_efforts);
      double friction = (mean_forward - mean_backward) / 2.0;
      friction_results.push_back(std::make_pair(current_velocity, friction));
      forward_efforts.clear();
      backward_efforts.clear();
      ROS_INFO("Velocity: %f, Friction: %f", current_velocity, friction);
      if (current_velocity < max_velocity)
        current_velocity += vel_step;
      else
        start_identify_ = false;
    }

    gimbal_cmd_sender_->setGimbalTraj(0., 0.);
    vel_cmd_sender_->setAngularZVel(-direction * current_velocity);
  }
  else
    vel_cmd_sender_->setAngularZVel(0.);
  //    gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
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
  if (is_gyro_)
  {
    if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
      changeGyroSpeedMode(LOW);
    else
      changeGyroSpeedMode(NORMAL);
  }
}

void ChassisGimbalShooterCoverManual::cPress()
{
  if (is_gyro_)
  {
    setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  }
  else
  {
    setChassisMode(rm_msgs::ChassisCmd::RAW);
    chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
      changeGyroSpeedMode(LOW);
    else
      changeGyroSpeedMode(NORMAL);
  }
}

void ChassisGimbalShooterCoverManual::zPress()
{
  last_shoot_freq_ = shooter_cmd_sender_->getShootFrequency();
  shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
}

void ChassisGimbalShooterCoverManual::zRelease()
{
  shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
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
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::aPressing()
{
  ChassisGimbalShooterManual::aPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::sPressing()
{
  ChassisGimbalShooterManual::sPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::dPressing()
{
  ChassisGimbalShooterManual::dPressing();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? gyro_rotate_reduction_ : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::wRelease()
{
  ChassisGimbalShooterManual::wRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::aRelease()
{
  ChassisGimbalShooterManual::aRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::sRelease()
{
  ChassisGimbalShooterManual::sRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
}

void ChassisGimbalShooterCoverManual::dRelease()
{
  ChassisGimbalShooterManual::dRelease();
  if (switch_buff_srv_->getTarget() != rm_msgs::StatusChangeRequest::ARMOR)
    vel_cmd_sender_->setAngularZVel(is_gyro_ ? 1 : 0, gyro_speed_limit_);
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

void ChassisGimbalShooterCoverManual::ctrlRPress()
{
  start_identify_ = true;
  last_time = ros::Time::now();
  current_velocity = 0.5;
  max_velocity = 8.0;
  vel_step = 0.5;
  angle = -M_PI;
  direction = 1;
  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
  chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
}

void ChassisGimbalShooterCoverManual::ctrlRRelease()
{
  //  count_ = 0;
  //  start_identify_ = false;
  //  gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  //  shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterCoverManual::jointStateCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  if (start_identify_)
  {
    if (direction == 1)
      forward_efforts.push_back(data->effort[9]);
    else
      backward_efforts.push_back(data->effort[9]);
  }
}

double ChassisGimbalShooterCoverManual::calculateMean(const std::vector<double>& efforts)
{
  double sum = 0.0;
  for (size_t i = 0; i < efforts.size(); ++i)
  {
    sum += efforts[i];
  }
  return efforts.empty() ? 0.0 : sum / efforts.size();
}

}  // namespace rm_manual
