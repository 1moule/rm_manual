//
// Created by guanlin on 25-5-19.
//

#pragma once

#include "rm_manual/core/input_event.h"

class KeyBoard
{
public:
  KeyBoard() = default;
  void checkKeyboard(const rm_msgs::DbusData::ConstPtr& data){};
};
