//
// Created by qiayuan on 5/22/21.
//

#pragma once

#include <angles/angles.h>
#include <rm_common/decision/calibration_queue.h>

#include "rm_manual/core/chassis_manual.h"
#include "rm_manual/core/gimbal_manual.h"
#include "rm_manual/core/shooter_manual.h"
#include "rm_manual/manual_base.h"

namespace rm_manual
{
class ChassisGimbalShooterManual : public ManualBase
{
public:
  ChassisGimbalShooterManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);
  void run() override;

protected:
  void ecatReconnected() override;
  void checkReferee() override;
  void updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data) override;
  void sendCommand(const ros::Time& time) override;
  void chassisOutputOn() override;
  void shooterOutputOn() override;
  void gimbalOutputOn() override;
  void selfInspectionStart()
  {
    shooter_calibration_->reset();
  };
  void gameStart()
  {
    shooter_calibration_->reset();
  };
  void remoteControlTurnOff() override;
  void remoteControlTurnOn() override;
  void robotDie() override;
  void robotRevive() override;
  void rightSwitchDownRise() override;
  void rightSwitchMidRise() override;
  void rightSwitchUpRise() override;
  void leftSwitchDownRise() override;
  void leftSwitchMidRise() override;
  void leftSwitchMidOn(ros::Duration duration);
  void leftSwitchUpRise() override;
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data) override;
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data) override;
  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data) override;
  void capacityDataCallback(const rm_msgs::PowerManagementSampleAndStatusData ::ConstPtr& data) override;
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data) override;
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr& data) override;
  void shootBeforehandCmdCallback(const rm_msgs::ShootBeforehandCmd ::ConstPtr& data) override;
  void suggestFireCallback(const std_msgs::Bool::ConstPtr& data) override;
  void trackCallback(const rm_msgs::TrackData::ConstPtr& data) override;
  void shootDataCallback(const rm_msgs::ShootData::ConstPtr& data) override;
  void leftSwitchUpOn(ros::Duration duration);
  void leftSwitchUpFall();
  void mouseRightPress();
  void mouseRightRelease()
  {
    gimbal_manual_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    if (shooter_manual_->shooter_cmd_sender_->getMsg()->mode == rm_msgs::ShootCmd::PUSH)
      shooter_manual_->shooter_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
  }
  virtual void gPress();
  virtual void zPress();
  virtual void vPress();
  virtual void xPress();
  virtual void ePress();
  virtual void eRelease();
  virtual void cPress();
  virtual void bPress();
  virtual void bRelease();
  virtual void xRelease();
  virtual void shiftPress();
  virtual void shiftRelease();
  virtual void rPress();
  void ctrlVPress();
  void ctrlBPress();
  void ctrlRPress();
  void ctrlZPress();
  void ctrlXPress();
  virtual void ctrlRRelease();
  virtual void ctrlQPress();

  InputEvent self_inspection_event_, game_start_event_;
  rm_common::CameraSwitchCommandSender* camera_switch_cmd_sender_{};
  rm_common::JointPositionBinaryCommandSender* scope_cmd_sender_{};
  rm_common::JointPositionBinaryCommandSender* image_transmission_cmd_sender_{};
  rm_common::SwitchDetectionCaller* switch_detection_srv_{};
  rm_common::SwitchDetectionCaller* switch_armor_target_srv_{};
  rm_common::CalibrationQueue* chassis_calibration_;
  rm_common::CalibrationQueue* shooter_calibration_;
  rm_common::CalibrationQueue* gimbal_calibration_;

  std::unique_ptr<ChassisManual> chassis_manual_;
  std::unique_ptr<GimbalManual> gimbal_manual_;
  std::unique_ptr<ShooterManual> shooter_manual_;

  geometry_msgs::PointStamped point_out_;
  uint8_t last_shoot_freq_{};

  bool is_balance_ = false, use_scope_ = false, adjust_image_transmission_ = false, up_change_position_ = false,
       low_change_position_ = false, need_change_position_ = false, deployed_ = false;
  double yaw_current_{};
  double traj_yaw_, traj_pitch_;
  double scale_;
};
}  // namespace rm_manual
