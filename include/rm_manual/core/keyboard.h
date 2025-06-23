//
// Created by guanlin on 25-5-19.
//

#pragma once

#include <rm_msgs/DbusData.h>
#include "rm_manual/core/input_event.h"

namespace rm_manual
{
class KeyBoard
{
public:
  KeyBoard() = default;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& dbus_data)
  {
    w_event_.update((!dbus_data->key_ctrl) && dbus_data->key_w);
    s_event_.update((!dbus_data->key_ctrl) && dbus_data->key_s);
    a_event_.update((!dbus_data->key_ctrl) && dbus_data->key_a);
    d_event_.update((!dbus_data->key_ctrl) && dbus_data->key_d);
    c_event_.update((!dbus_data->key_ctrl) & dbus_data->key_c);
    e_event_.update(dbus_data->key_e & !dbus_data->key_ctrl);
    g_event_.update(dbus_data->key_g & !dbus_data->key_ctrl);
    q_event_.update((!dbus_data->key_ctrl) & dbus_data->key_q);
    b_event_.update((!dbus_data->key_ctrl && !dbus_data->key_shift) & dbus_data->key_b);
    x_event_.update(dbus_data->key_x & !dbus_data->key_ctrl);
    r_event_.update((!dbus_data->key_ctrl) & dbus_data->key_r);
    v_event_.update((!dbus_data->key_ctrl) & dbus_data->key_v);
    z_event_.update((!dbus_data->key_ctrl) & dbus_data->key_z);
    ctrl_f_event_.update(dbus_data->key_f & dbus_data->key_ctrl);
    ctrl_v_event_.update(dbus_data->key_ctrl & dbus_data->key_v);
    ctrl_b_event_.update(dbus_data->key_ctrl & dbus_data->key_b & !dbus_data->key_shift);
    ctrl_q_event_.update(dbus_data->key_ctrl & dbus_data->key_q);
    ctrl_r_event_.update(dbus_data->key_ctrl & dbus_data->key_r);
    ctrl_z_event_.update(dbus_data->key_ctrl & dbus_data->key_z);
    ctrl_x_event_.update(dbus_data->key_ctrl & dbus_data->key_x);
    shift_event_.update(dbus_data->key_shift & !dbus_data->key_ctrl);
    ctrl_shift_b_event_.update(dbus_data->key_ctrl & dbus_data->key_shift & dbus_data->key_b);
    mouse_left_event_.update(dbus_data->p_l & !dbus_data->key_ctrl);
    mouse_right_event_.update(dbus_data->p_r & !dbus_data->key_ctrl);
  }

  InputEvent w_event_, s_event_, a_event_, d_event_, c_event_, e_event_, g_event_, q_event_, b_event_, x_event_,
      r_event_, v_event_, z_event_, ctrl_f_event_, ctrl_v_event_, ctrl_b_event_, ctrl_q_event_, ctrl_r_event_,
      ctrl_z_event_, ctrl_x_event_, shift_event_, ctrl_shift_b_event_, mouse_right_event_, mouse_left_event_;
};
}  // namespace rm_manual
