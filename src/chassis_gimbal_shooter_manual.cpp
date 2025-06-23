//
// Created by peter on 2021/7/22.
//

#include "rm_manual/chassis_gimbal_shooter_manual.h"

namespace rm_manual
{
ChassisGimbalShooterManual::ChassisGimbalShooterManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee)
  : ManualBase(nh, nh_referee)
{
  chassis_manual_ = std::make_unique<ChassisManual>(nh, keyboard_);
  gimbal_manual_ = std::make_unique<GimbalManual>(nh, keyboard_);
  shooter_manual_ = std::make_unique<ShooterManual>(nh, keyboard_);
  if (nh.hasParam("camera"))
  {
    ros::NodeHandle camera_nh(nh, "camera");
    camera_switch_cmd_sender_ = new rm_common::CameraSwitchCommandSender(camera_nh);
  }
  if (nh.hasParam("scope"))
  {
    ros::NodeHandle scope_nh(nh, "scope");
    scope_cmd_sender_ = new rm_common::JointPositionBinaryCommandSender(scope_nh);
  }
  if (nh.hasParam("image_transmission"))
  {
    ros::NodeHandle image_transmission_nh(nh, "image_transmission");
    image_transmission_cmd_sender_ = new rm_common::JointPositionBinaryCommandSender(image_transmission_nh);
    scale_ = getParam(image_transmission_nh, "position_scale", 1);
  }

  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  ros::NodeHandle armor_target_switch_nh(nh, "armor_target_switch");
  switch_armor_target_srv_ = new rm_common::SwitchDetectionCaller(armor_target_switch_nh);
  XmlRpc::XmlRpcValue rpc_value;
  nh.getParam("shooter_calibration", rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  nh.getParam("chassis_calibration", rpc_value);
  chassis_calibration_ = new rm_common::CalibrationQueue(rpc_value, nh, controller_manager_);
  shooter_power_on_event_.setRising(boost::bind(&ChassisGimbalShooterManual::shooterOutputOn, this));
  self_inspection_event_.setRising(boost::bind(&ChassisGimbalShooterManual::selfInspectionStart, this));
  game_start_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gameStart, this));
  left_switch_up_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchUpOn, this, _1));
  left_switch_up_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::leftSwitchUpFall, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::leftSwitchMidOn, this, _1));
  keyboard_->e_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::ePress, this),
                              boost::bind(&ChassisGimbalShooterManual::eRelease, this));
  keyboard_->c_event_.setRising(boost::bind(&ChassisGimbalShooterManual::cPress, this));
  keyboard_->b_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::bPress, this),
                              boost::bind(&ChassisGimbalShooterManual::bRelease, this));
  keyboard_->x_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::xPress, this),
                              boost::bind(&ChassisGimbalShooterManual::xRelease, this));
  keyboard_->r_event_.setRising(boost::bind(&ChassisGimbalShooterManual::rPress, this));
  keyboard_->g_event_.setRising(boost::bind(&ChassisGimbalShooterManual::gPress, this));
  keyboard_->v_event_.setRising(boost::bind(&ChassisGimbalShooterManual::vPress, this));
  keyboard_->z_event_.setRising(boost::bind(&ChassisGimbalShooterManual::zPress, this));
  keyboard_->ctrl_v_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlVPress, this));
  keyboard_->ctrl_b_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlBPress, this));
  keyboard_->ctrl_q_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlQPress, this));
  keyboard_->ctrl_z_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlZPress, this));
  keyboard_->ctrl_x_event_.setRising(boost::bind(&ChassisGimbalShooterManual::ctrlXPress, this));
  keyboard_->ctrl_r_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::ctrlRPress, this),
                                   boost::bind(&ChassisGimbalShooterManual::ctrlRRelease, this));
  keyboard_->shift_event_.setEdge(boost::bind(&ChassisGimbalShooterManual::shiftPress, this),
                                  boost::bind(&ChassisGimbalShooterManual::shiftRelease, this));
  keyboard_->mouse_right_event_.setActiveHigh(boost::bind(&ChassisGimbalShooterManual::mouseRightPress, this));
  keyboard_->mouse_right_event_.setFalling(boost::bind(&ChassisGimbalShooterManual::mouseRightRelease, this));
}

void ChassisGimbalShooterManual::run()
{
  ManualBase::run();
  chassis_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
  gimbal_calibration_->update(ros::Time::now());
}

void ChassisGimbalShooterManual::ecatReconnected()
{
  ManualBase::ecatReconnected();
  shooter_calibration_->reset();
  gimbal_calibration_->reset();
  up_change_position_ = false;
  low_change_position_ = false;
  need_change_position_ = false;
}

void ChassisGimbalShooterManual::checkReferee()
{
  manual_to_referee_pub_data_.power_limit_state = chassis_manual_->chassis_cmd_sender_->power_limit_->getState();
  manual_to_referee_pub_data_.start_burst_time =
      chassis_manual_->chassis_cmd_sender_->power_limit_->getStartBurstTime();
  manual_to_referee_pub_data_.shoot_frequency = shooter_manual_->shooter_cmd_sender_->getShootFrequency();
  manual_to_referee_pub_data_.gimbal_eject = gimbal_manual_->gimbal_cmd_sender_->getEject();
  manual_to_referee_pub_data_.det_armor_target = switch_armor_target_srv_->getArmorTarget();
  manual_to_referee_pub_data_.det_color = switch_detection_srv_->getColor();
  manual_to_referee_pub_data_.det_exposure = switch_detection_srv_->getExposureLevel();
  manual_to_referee_pub_data_.stamp = ros::Time::now();
  ManualBase::checkReferee();
}

void ChassisGimbalShooterManual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  chassis_manual_->chassis_cmd_sender_->updateGameRobotStatus(*data);
  shooter_manual_->shooter_cmd_sender_->updateGameRobotStatus(*data);
}

void ChassisGimbalShooterManual::powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
{
  ManualBase::powerHeatDataCallback(data);
  chassis_manual_->chassis_cmd_sender_->updatePowerHeatData(*data);
  shooter_manual_->shooter_cmd_sender_->updatePowerHeatData(*data);
}

void ChassisGimbalShooterManual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  chassis_manual_->chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
  shooter_manual_->shooter_cmd_sender_->updateRefereeStatus(referee_is_online_);
}

void ChassisGimbalShooterManual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  chassis_manual_->chassis_cmd_sender_->updateGameStatus(*data);
  self_inspection_event_.update(data->game_progress == 2);
  game_start_event_.update(data->game_progress == 4);
}

void ChassisGimbalShooterManual::gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data)
{
  ManualBase::gimbalDesErrorCallback(data);
  shooter_manual_->shooter_cmd_sender_->updateGimbalDesError(*data);
}

void ChassisGimbalShooterManual::capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData ::ConstPtr& data)
{
  ManualBase::capacityDataCallback(data);
  chassis_manual_->chassis_cmd_sender_->updateCapacityData(*data);
}

void ChassisGimbalShooterManual::shootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd ::ConstPtr& data)
{
  ManualBase::shootBeforehandCmdCallback(data);
  shooter_manual_->shooter_cmd_sender_->updateShootBeforehandCmd(*data);
}

void ChassisGimbalShooterManual::trackCallback(const rm_msgs::TrackData::ConstPtr& data)
{
  ManualBase::trackCallback(data);
  shooter_manual_->shooter_cmd_sender_->updateTrackData(*data);
  chassis_manual_->vel_cmd_sender_->updateTrackData(*data);
}

void ChassisGimbalShooterManual::suggestFireCallback(const std_msgs::Bool::ConstPtr& data)
{
  ManualBase::suggestFireCallback(data);
  shooter_manual_->shooter_cmd_sender_->updateSuggestFireData(*data);
}

void ChassisGimbalShooterManual::shootDataCallback(const rm_msgs::ShootData::ConstPtr& data)
{
  ManualBase::shootDataCallback(data);
  if (referee_is_online_ && !use_scope_)
    shooter_manual_->shooter_cmd_sender_->updateShootData(*data);
}

void ChassisGimbalShooterManual::sendCommand(const ros::Time& time)
{
  shooter_manual_->shooter_cmd_sender_->sendCommand(time);
  chassis_manual_->sendCommand(time);
  gimbal_manual_->gimbal_cmd_sender_->sendCommand(time);
  if (camera_switch_cmd_sender_)
    camera_switch_cmd_sender_->sendCommand(time);
  if (scope_cmd_sender_)
  {
    if (!use_scope_)
      scope_cmd_sender_->off();
    else
      scope_cmd_sender_->on();
    scope_cmd_sender_->sendCommand(time);
  }
  if (image_transmission_cmd_sender_)
  {
    if (!need_change_position_)
    {
      if (!adjust_image_transmission_)
        image_transmission_cmd_sender_->off();
      else
        image_transmission_cmd_sender_->on();
    }
    if (up_change_position_)
    {
      image_transmission_cmd_sender_->changePosition(scale_);
      up_change_position_ = false;
    }
    else if (low_change_position_)
    {
      image_transmission_cmd_sender_->changePosition(-scale_);
      low_change_position_ = false;
    }
    image_transmission_cmd_sender_->sendCommand(time);
  }
}

void ChassisGimbalShooterManual::remoteControlTurnOff()
{
  ManualBase::remoteControlTurnOff();
  shooter_manual_->shooter_cmd_sender_->setZero();
  shooter_calibration_->stop();
  gimbal_calibration_->stop();
  chassis_calibration_->stop();
  use_scope_ = false;
  adjust_image_transmission_ = false;
  up_change_position_ = false;
  low_change_position_ = false;
  need_change_position_ = false;
  deployed_ = false;
}

void ChassisGimbalShooterManual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  shooter_calibration_->stopController();
  gimbal_calibration_->stopController();
  chassis_calibration_->stopController();
  std::string robot_color = robot_id_ >= 100 ? "blue" : "red";
  switch_detection_srv_->setEnemyColor(robot_id_, robot_color);
}

void ChassisGimbalShooterManual::robotDie()
{
  ManualBase::robotDie();
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  use_scope_ = false;
  adjust_image_transmission_ = false;
  up_change_position_ = false;
  low_change_position_ = false;
  need_change_position_ = false;
  deployed_ = false;
}

void ChassisGimbalShooterManual::chassisOutputOn()
{
  ManualBase::chassisOutputOn();
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  chassis_calibration_->reset();
}

void ChassisGimbalShooterManual::shooterOutputOn()
{
  ManualBase::shooterOutputOn();
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
  shooter_calibration_->reset();
}

void ChassisGimbalShooterManual::gimbalOutputOn()
{
  ManualBase::gimbalOutputOn();
  gimbal_calibration_->reset();
}

void ChassisGimbalShooterManual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  gimbal_manual_->setRcRate(dbus_data);
  chassis_manual_->setRcVel(dbus_data);
  if (std::abs(dbus_data->wheel) > 0.01)
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::RAW);
  else
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_manual_->vel_cmd_sender_->setAngularZVel(
      (std::abs(dbus_data->ch_r_y) > 0.01 || std::abs(dbus_data->ch_r_x) > 0.01) ?
          dbus_data->wheel * chassis_manual_->gyro_rotate_reduction_ :
          dbus_data->wheel);

  if (shooter_manual_->shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP)
    gimbal_manual_->gimbal_cmd_sender_->setBulletSpeed(shooter_manual_->shooter_cmd_sender_->getSpeed());
}

void ChassisGimbalShooterManual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updatePc(dbus_data);
  gimbal_manual_->setPcRate(dbus_data);
  if (chassis_manual_->chassis_cmd_sender_->power_limit_->getState() != rm_common::PowerLimit::BURST &&
      !chassis_manual_->is_gyro_ && !is_balance_)
  {  // Capacitor enter fast charge when chassis stop.
    if (!dbus_data->key_shift && chassis_manual_->chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW &&
        std::sqrt(std::pow(chassis_manual_->vel_cmd_sender_->getMsg()->linear.x, 2) +
                  std::pow(chassis_manual_->vel_cmd_sender_->getMsg()->linear.y, 2)) > 0.0)
      chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    else if (chassis_power_ < 6.0 && chassis_manual_->chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::FOLLOW)
      chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  }
  if (gimbal_manual_->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRAJ && deployed_)
  {
    traj_yaw_ += gimbal_manual_->traj_scale_ * gimbal_manual_->gimbal_cmd_sender_->getMsg()->rate_yaw *
                 ros::Duration(0.01).toSec();
    traj_pitch_ += gimbal_manual_->traj_scale_ * gimbal_manual_->gimbal_cmd_sender_->getMsg()->rate_pitch *
                   ros::Duration(0.01).toSec();
    gimbal_manual_->gimbal_cmd_sender_->setGimbalTraj(traj_yaw_, traj_pitch_);
  }
  if ((gimbal_manual_->gimbal_cmd_sender_->getMsg()->mode == rm_msgs::GimbalCmd::TRAJ && deployed_) &&
      std::sqrt(std::pow(chassis_manual_->vel_cmd_sender_->getMsg()->linear.x, 2) +
                std::pow(chassis_manual_->vel_cmd_sender_->getMsg()->linear.y, 2)) > 0.0)
  {
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    deployed_ = false;
    shooter_manual_->shooter_cmd_sender_->setDeployState(false);
  }
}

void ChassisGimbalShooterManual::rightSwitchDownRise()
{
  ManualBase::rightSwitchDownRise();
  chassis_manual_->chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_manual_->vel_cmd_sender_->setZero();
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_manual_->gimbal_cmd_sender_->setZero();
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  chassis_manual_->chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_manual_->gimbal_cmd_sender_->setUseRc(true);
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
  chassis_manual_->chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
  chassis_manual_->vel_cmd_sender_->setZero();
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  gimbal_manual_->gimbal_cmd_sender_->setUseRc(false);
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchDownRise()
{
  ManualBase::leftSwitchDownRise();
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
}

void ChassisGimbalShooterManual::leftSwitchMidRise()
{
  ManualBase::leftSwitchMidRise();
  shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::leftSwitchMidOn(ros::Duration duration)
{
  if (track_data_.id == 0)
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
}

void ChassisGimbalShooterManual::leftSwitchUpRise()
{
  ManualBase::leftSwitchUpRise();
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  last_shoot_freq_ = shooter_manual_->shooter_cmd_sender_->getShootFrequency();
}

void ChassisGimbalShooterManual::leftSwitchUpFall()
{
  shooter_manual_->shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
}

void ChassisGimbalShooterManual::leftSwitchUpOn(ros::Duration duration)
{
  if (track_data_.id == 0)
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
  if (duration > ros::Duration(1.))
  {
    shooter_manual_->shooter_cmd_sender_->setShootFrequency(last_shoot_freq_);
    shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_manual_->shooter_cmd_sender_->checkError(ros::Time::now());
  }
  else if (duration < ros::Duration(0.02))
  {
    shooter_manual_->shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::MINIMAL);
    shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
    shooter_manual_->shooter_cmd_sender_->checkError(ros::Time::now());
  }
  else
    shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
}

void ChassisGimbalShooterManual::mouseRightPress()
{
  if (track_data_.id == 0)
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
  else
  {
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    gimbal_manual_->gimbal_cmd_sender_->setBulletSpeed(shooter_manual_->shooter_cmd_sender_->getSpeed());
  }
  if (switch_armor_target_srv_->getArmorTarget() == rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE)
  {
    if (shooter_manual_->shooter_cmd_sender_->getMsg()->mode != rm_msgs::ShootCmd::STOP)
    {
      shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      shooter_manual_->shooter_cmd_sender_->checkError(ros::Time::now());
    }
  }
}

void ChassisGimbalShooterManual::ePress()
{
  switch_armor_target_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE);
  switch_armor_target_srv_->callService();
  shooter_manual_->shooter_cmd_sender_->setArmorType(switch_armor_target_srv_->getArmorTarget());
}

void ChassisGimbalShooterManual::eRelease()
{
  switch_armor_target_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_ALL);
  switch_armor_target_srv_->callService();
  shooter_manual_->shooter_cmd_sender_->setArmorType(switch_armor_target_srv_->getArmorTarget());
}

void ChassisGimbalShooterManual::cPress()
{
  if (chassis_manual_->is_gyro_)
  {
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  }
  else
  {
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::RAW);
    chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
  }
}

void ChassisGimbalShooterManual::bPress()
{
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::bRelease()
{
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
}

void ChassisGimbalShooterManual::rPress()
{
  if (scope_cmd_sender_)
  {
    use_scope_ = !scope_cmd_sender_->getState();
    if (use_scope_)
      gimbal_manual_->gimbal_cmd_sender_->setEject(true);
    else
    {
      gimbal_manual_->gimbal_cmd_sender_->setEject(false);
      adjust_image_transmission_ = false;
    }
  }
}

void ChassisGimbalShooterManual::gPress()
{
  shooter_manual_->shooter_cmd_sender_->dropSpeed();
}

void ChassisGimbalShooterManual::xPress()
{
  double roll{}, pitch{}, yaw{};
  try
  {
    quatToRPY(tf_buffer_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  gimbal_manual_->gimbal_cmd_sender_->setGimbalTrajFrameId("base_link");
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
  gimbal_manual_->gimbal_cmd_sender_->setGimbalTraj(yaw + M_PI, pitch);
}

void ChassisGimbalShooterManual::xRelease()
{
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalShooterManual::vPress()
{
  shooter_manual_->shooter_cmd_sender_->raiseSpeed();
}

void ChassisGimbalShooterManual::zPress()
{
  if (chassis_manual_->chassis_cmd_sender_->getMsg()->mode != rm_msgs::ChassisCmd::RAW && !deployed_)
  {
    double roll{}, pitch{}, yaw{};
    try
    {
      quatToRPY(tf_buffer_.lookupTransform("base_link", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    gimbal_manual_->gimbal_cmd_sender_->setGimbalTrajFrameId("base_link");
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    traj_yaw_ = yaw, traj_pitch_ = -0.585;
    gimbal_manual_->gimbal_cmd_sender_->setGimbalTraj(traj_yaw_, traj_pitch_);
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::DEPLOY);
    shooter_manual_->shooter_cmd_sender_->setDeployState(true);
    deployed_ = true;
  }
  else
  {
    chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    shooter_manual_->shooter_cmd_sender_->setDeployState(false);
    deployed_ = false;
  }
}

void ChassisGimbalShooterManual::shiftPress()
{
  chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::BURST);
}

void ChassisGimbalShooterManual::shiftRelease()
{
  if (chassis_manual_->chassis_cmd_sender_->getMsg()->mode == rm_msgs::ChassisCmd::RAW ||
      std::sqrt(std::pow(chassis_manual_->vel_cmd_sender_->getMsg()->linear.x, 2) +
                std::pow(chassis_manual_->vel_cmd_sender_->getMsg()->linear.y, 2)) > 0.0)
    chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
  else
    chassis_manual_->chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::CHARGE);
}

void ChassisGimbalShooterManual::ctrlVPress()
{
  if (shooter_manual_->shooter_cmd_sender_->getShootFrequency() != rm_common::HeatLimit::LOW)
    shooter_manual_->shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::LOW);
  else
    shooter_manual_->shooter_cmd_sender_->setShootFrequency(rm_common::HeatLimit::HIGH);
}

void ChassisGimbalShooterManual::ctrlRPress()
{
  if (image_transmission_cmd_sender_)
    adjust_image_transmission_ = !image_transmission_cmd_sender_->getState();
  if (adjust_image_transmission_)
  {
    double roll{}, pitch{}, yaw{};
    try
    {
      quatToRPY(tf_buffer_.lookupTransform("odom", "yaw", ros::Time(0)).transform.rotation, roll, pitch, yaw);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRAJ);
    gimbal_manual_->gimbal_cmd_sender_->setGimbalTraj(yaw, -0.45);
  }
}

void ChassisGimbalShooterManual::ctrlRRelease()
{
  gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
}

void ChassisGimbalShooterManual::ctrlBPress()
{
  switch_detection_srv_->switchEnemyColor();
  switch_detection_srv_->callService();
}

void ChassisGimbalShooterManual::ctrlQPress()
{
  shooter_calibration_->reset();
  gimbal_calibration_->reset();
  adjust_image_transmission_ = false;
  up_change_position_ = false;
  low_change_position_ = false;
  need_change_position_ = false;
}

void ChassisGimbalShooterManual::ctrlZPress()
{
  up_change_position_ = true;
  need_change_position_ = true;
}

void ChassisGimbalShooterManual::ctrlXPress()
{
  low_change_position_ = true;
  need_change_position_ = true;
}

void ChassisGimbalShooterManual::robotRevive()
{
  chassis_manual_->setChassisMode(rm_msgs::ChassisCmd::FOLLOW);
  ManualBase::robotRevive();
}

}  // namespace rm_manual
